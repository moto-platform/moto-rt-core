#include "features/uds/uds_client.h"

#include "platform_uds.h"
#include "services/can_if.h"
#include "services/diag.h"
#include "services/timebase.h"
#include "services/vehicle_signals.h"
#include "vehicle_cl250.h"

#include <stddef.h>

_Static_assert(UDS_CLIENT_TX_BUF >= UDS_CLIENT_REQ_MAX, "request buffer too small");
_Static_assert(UDS_CLIENT_RX_BUF > ISOTP_SF_MAX_LEN, "rx buffer must exceed a Single Frame");

#define FALLBACK_IDS_EXTENDED (VEHICLE_CL250_FALLBACK_REQUEST_ID > CAN_PORT_STD_ID_MAX)
#define REQUEST_IDS_EXTENDED (VEHICLE_CL250_REQUEST_ID > CAN_PORT_STD_ID_MAX)

/* Frames seen on the ECU request IDs. The CAN controller does not receive its own
 * frames, so each one was sent by another tester (D-021). One vehicle link exists, so
 * one counter serves it (a second client cannot open the link). */
static uint32_t foreign_frames;

static void on_foreign_request(void* ctx, const can_frame_t* frame)
{
    (void)ctx;
    (void)frame;
    if (foreign_frames < UINT32_MAX) {
        foreign_frames++;
    }
}

static void latch(uds_client_t* client, uds_client_fault_t fault)
{
    if (client->fault == UDS_CLIENT_FAULT_NONE) {
        client->fault = fault;
    }
    uds_client_core_latch(&client->core);
}

isotp_status_t uds_client_open(uds_client_t* client)
{
    if (client == NULL) {
        return ISOTP_ERR_ARG;
    }
    if (client->open) {
        return ISOTP_ERR_BUSY;
    }
    const isotp_status_t st = isotp_link_open_vehicle_cl250(
        &client->link, client->rx_buf, UDS_CLIENT_RX_BUF, client->tx_buf, UDS_CLIENT_TX_BUF);
    if (st != ISOTP_OK) {
        return st;
    }
    if ((can_if_register_rx(CAN_PORT_VEHICLE, VEHICLE_CL250_REQUEST_ID, REQUEST_IDS_EXTENDED,
                            on_foreign_request, NULL) != CAN_IF_OK) ||
        (can_if_register_rx(CAN_PORT_VEHICLE, VEHICLE_CL250_FALLBACK_REQUEST_ID,
                            FALLBACK_IDS_EXTENDED, on_foreign_request, NULL) != CAN_IF_OK)) {
        return ISOTP_ERR_ARG; /* never poll without the second-tester watch */
    }
    /* Q-021/D-040: generic OBD dongles use functional addressing; watch those IDs too. */
    for (uint32_t i = 0u; i < VEHICLE_CL250_FUNCTIONAL_WATCH_COUNT; i++) {
        if (can_if_register_rx(CAN_PORT_VEHICLE, vehicle_cl250_functional_watch[i].id,
                               vehicle_cl250_functional_watch[i].extended, on_foreign_request,
                               NULL) != CAN_IF_OK) {
            return ISOTP_ERR_ARG;
        }
    }
    uds_client_core_init(&client->core, isotp_link_n_bs_ms(&client->link));
    client->guard_refused_seen = can_if_tx_refused_count(CAN_PORT_VEHICLE);
    client->foreign_seen = foreign_frames;
    client->fault = UDS_CLIENT_FAULT_NONE;
    client->open_ms = timebase_now_ms();
    client->absence_armed = false;
    client->open = true;
    return ISOTP_OK;
}

static uint8_t fault_code(uds_client_fault_t fault)
{
    uint8_t code;
    switch (fault) {
    case UDS_CLIENT_FAULT_GATE:
        code = (uint8_t)PLATFORM_UDS_VEHICLE_TESTER_STATUS_FAULT_GATE;
        break;
    case UDS_CLIENT_FAULT_GUARD:
        code = (uint8_t)PLATFORM_UDS_VEHICLE_TESTER_STATUS_FAULT_GUARD;
        break;
    case UDS_CLIENT_FAULT_FOREIGN_TESTER:
        code = (uint8_t)PLATFORM_UDS_VEHICLE_TESTER_STATUS_FAULT_FOREIGN_TESTER;
        break;
    default:
        code = (uint8_t)PLATFORM_UDS_VEHICLE_TESTER_STATUS_FAULT_NONE;
        break;
    }
    return code;
}

/* Level-triggered DTC results, the 0xFD00 status and the 0xFD02 step counters, every
 * pass, latched or not (services/diag). A 0x14 clear therefore hides an active
 * condition for one pass at most, and never releases the latch itself. The ECU gets
 * ECU_ABSENT_TIMEOUT_MS after open to answer before its absence counts. */
static void report_diag(uds_client_t* client, uint32_t now, bool present)
{
    const bool latched = uds_client_core_failed(&client->core);
    if (!client->absence_armed &&
        timebase_expired(now, client->open_ms, VEHICLE_CL250_ECU_ABSENT_TIMEOUT_MS)) {
        client->absence_armed = true; /* sticky: the ms counter wrap cannot disarm it */
    }
    const bool comm_lost = !present && client->absence_armed;
    diag_dtc_report(PLATFORM_UDS_DTC_IDX_VEHICLE_ECU_COMM_LOST, comm_lost);
    diag_dtc_report(PLATFORM_UDS_DTC_IDX_VEHICLE_TESTER_LATCHED, latched);
    const diag_vehicle_tester_t status = {present, uds_client_core_session_up(&client->core),
                                          latched, fault_code(client->fault)};
    diag_set_vehicle_tester(&status, now);
    const uds_client_stats_t* st = uds_client_core_stats(&client->core);
    diag_set_client_steps(st->step_overruns, st->step_gap_max_ms, now); /* 0xFD02, D-055 */
}

static void take_indication(uds_client_t* client, uint32_t now)
{
    isotp_n_result_t res = ISOTP_N_OK;
    uint16_t len = 0u;
    if (!isotp_link_take_rx(&client->link, &res, &len)) {
        return;
    }
    const uint8_t* data = (res == ISOTP_N_OK) ? isotp_link_rx_data(&client->link) : NULL;
    uds_client_sample_t s = {0u, 0u, 0.0f, 0u};
    if (uds_client_core_on_indication(&client->core, now, res, data, len, &s)) {
        (void)vehicle_signals_write(s.idx, s.raw, s.physical, s.stamp_ms); /* D-051 */
    }
    if (res == ISOTP_N_OK) {
        isotp_link_rx_release(&client->link);
    }
}

void uds_client_step(uds_client_t* client)
{
    if ((client == NULL) || !client->open) {
        return;
    }
    const uint32_t now = timebase_now_ms();
    vehicle_signals_expire(now);
    if (foreign_frames != client->foreign_seen) {
        client->foreign_seen = foreign_frames;
        latch(client, UDS_CLIENT_FAULT_FOREIGN_TESTER); /* before anything is sent */
    }
    take_indication(client, now);
    if (isotp_link_take_fc_withheld(&client->link)) {
        uds_client_core_on_fc_withheld(&client->core, now);
    }
    isotp_n_result_t conf = ISOTP_N_OK;
    (void)isotp_link_take_tx_confirm(&client->link, &conf); /* a Single Frame cannot fail */

    uint8_t req[UDS_CLIENT_REQ_MAX];
    uint16_t len = 0u;
    if (uds_client_core_poll(&client->core, now, isotp_link_rx_busy(&client->link),
                             isotp_link_tx_ready(&client->link), req, &len)) {
        const isotp_status_t st = isotp_link_send(&client->link, req, len);
        if (st == ISOTP_ERR_BUSY) {
            uds_client_core_not_sent(&client->core); /* its schedule is restored */
        } else if (st != ISOTP_OK) {
            latch(client, UDS_CLIENT_FAULT_GATE); /* D-020 gate or length: a bug */
        } else {
            /* sent */
        }
    }
    isotp_link_step(&client->link);

    /* The can_if guard refused a vehicle frame. The link's own refused counter is not
     * watched: a refused request is caught by isotp_link_send(), and a withheld Flow
     * Control is not a refusal (isotp_link_take_fc_withheld() above). */
    const uint32_t guard_refused = can_if_tx_refused_count(CAN_PORT_VEHICLE);
    if (guard_refused != client->guard_refused_seen) {
        client->guard_refused_seen = guard_refused;
        latch(client, UDS_CLIENT_FAULT_GUARD);
    }
    const bool present = uds_client_core_ecu_present(&client->core, now);
    vehicle_signals_set_ecu_present(present);
    report_diag(client, now, present);
}

bool uds_client_session_up(const uds_client_t* client)
{
    return (client != NULL) && client->open && uds_client_core_session_up(&client->core);
}

bool uds_client_ecu_present(const uds_client_t* client)
{
    return (client != NULL) && client->open &&
           uds_client_core_ecu_present(&client->core, timebase_now_ms());
}

bool uds_client_failed(const uds_client_t* client)
{
    return (client == NULL) || !client->open || uds_client_core_failed(&client->core);
}

uds_client_fault_t uds_client_fault(const uds_client_t* client)
{
    return (client == NULL) ? UDS_CLIENT_FAULT_NONE : client->fault;
}

const uds_client_stats_t* uds_client_stats(const uds_client_t* client)
{
    return (client == NULL) ? NULL : uds_client_core_stats(&client->core);
}
