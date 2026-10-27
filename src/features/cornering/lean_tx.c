#include "features/cornering/lean_tx.h"

#include "services/can_if.h"
#include "services/lean_health.h"
#include "services/vehicle_signals.h"

#include <stddef.h>
#include <string.h>

_Static_assert(LEAN_TX_FUTURE_MAX_MS == LEAN_HEALTH_FUTURE_MAX_MS,
               "0xFD03 RECORD_FRESH uses 0x020's future-stamp rule");
_Static_assert(PLATFORM_EKF_LEAN_CYCLE_TIME_MS > 0u, "a zero cycle time would send every pass");

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: called by app/comms (comms_open_lean) and the tests
bool lean_tx_open(lean_tx_t* tx, lean_link_t* link, const alive_counter_t* registered)
{
    lean_health_init(); /* 0xFD03: no record since boot; first, before any validation */
    if (tx == NULL) {
        return false;
    }
    (void)memset(tx, 0, sizeof *tx);
    vehicle_e2e_tx_init(&tx->e2e);
    if ((link == NULL) || (registered == NULL) || (registered != lean_link_alive(link))) {
        return false; /* fail-closed: no supervised EKF behind this link */
    }
    if (can_if_register_tx_dedicated(CAN_PORT_PLATFORM, PLATFORM_EKF_LEAN_FRAME_ID,
                                     PLATFORM_EKF_LEAN_IS_EXTENDED != 0) != CAN_IF_OK) {
        return false; /* never 0x020 through the Tx FIFO */
    }
    tx->link = link;
    tx->opened = true;
    return true;
}

void lean_tx_publish_speed(lean_tx_t* tx, uint32_t now_ms)
{
    if ((tx == NULL) || !tx->opened || snapshot_producer_fault(&tx->link->speed_snap)) {
        return; /* after a producer fault the write slot may be the EKF's front (MINOR-1) */
    }
    vehicle_signal_sample_t s;
    (void)memset(&s, 0, sizeof s); /* VEHICLE_SIGNAL_NONE */
    (void)vehicle_signals_get(VEHICLE_CL250_IDX_VEHICLE_SPEED, now_ms, &s);

    lean_link_t* const link = tx->link;
    lean_speed_t* const slot = &link->speed_slots[snapshot_write_index(&link->speed_snap)];
    slot->stamp_ms = s.timestamp_ms;
    if ((s.state == VEHICLE_SIGNAL_VALID) && platform_vehicle_speed_vehicle_speed_is_in_phys_range(s.physical)) {
        slot->speed_mps = s.physical / LEAN_KMH_PER_MPS;
        slot->state = LEAN_SPEED_VALID;
    } else {
        slot->speed_mps = 0.0f;
        slot->state = LEAN_SPEED_UNUSABLE;
    }
    snapshot_publish(&link->speed_snap);
}

bool lean_tx_speed_producer_fault(const lean_tx_t* tx)
{
    return (tx != NULL) && tx->opened && snapshot_producer_fault(&tx->link->speed_snap);
}

bool lean_tx_output_consumer_fault(const lean_tx_t* tx)
{
    return (tx != NULL) && tx->opened && snapshot_consumer_fault(&tx->link->out_snap);
}

void lean_tx_step(lean_tx_t* tx, uint32_t now_ms)
{
    if ((tx == NULL) || !tx->opened || !com_cycle_due(&tx->cycle, now_ms)) {
        return;
    }
    lean_link_t* const link = tx->link;
    uint8_t ri = 0u;
    const bool fresh = snapshot_read_index(&link->out_snap, &ri);
    /* DID 0xFD03 (D-067): copy the record out of the index this one read returned, now.
     * The UDS server reads this copy and never the snapshot, and no index outlives this
     * call, so 0x020 can never be starved of a record by the diagnostic. */
    const lean_record_t* const rec = &link->out_slots[ri];
    /* A record exists once one was published: the read said so, or the slot carries a
     * stamp (only the boot record has stamp 0, so a front slot that something else
     * consumed first still counts). */
    tx->have_record = tx->have_record || fresh || (rec->stamp_ms != 0u);
    lean_health_rec_t copy;
    copy.have_record = tx->have_record;
    copy.stamp_ms = rec->stamp_ms;
    copy.ekf = rec->health;
    copy.speed_consumer_fault = rec->fault;
    lean_health_set_record(&copy);
    const bool comms_fault =
        snapshot_consumer_fault(&link->out_snap) || snapshot_producer_fault(&link->speed_snap);

    struct platform_ekf_lean_t msg;
    vehicle_e2e_tx_state_t next;
    uint8_t data[PLATFORM_EKF_LEAN_LENGTH];
    lean_tx_msg(rec, comms_fault, now_ms, &msg);
    const bool built = lean_tx_frame(&msg, &tx->e2e, &next, data);
    can_port_status_t st = CAN_PORT_ERR_ARG;
    const bool finished = com_send(PLATFORM_EKF_LEAN_FRAME_ID, PLATFORM_EKF_LEAN_IS_EXTENDED != 0,
                                   built ? data : NULL, (uint8_t)PLATFORM_EKF_LEAN_LENGTH, &tx->stats, &st);
    if (st == CAN_PORT_OK) {
        tx->e2e = next; /* the counter advances with accepted frames only (D-056 item 5) */
    }
    if (finished) {
        com_cycle_done(&tx->cycle, now_ms, PLATFORM_EKF_LEAN_CYCLE_TIME_MS);
    }
}
