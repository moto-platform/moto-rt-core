#include "features/heartbeat/heartbeat.h"

#include "platform_uds.h"
#include "services/can_if.h"
#include "services/can_sm.h"
#include "services/com.h"
#include "services/diag.h"
#include "services/timebase.h"

#include <stddef.h>
#include <string.h>

_Static_assert(PLATFORM_HEARTBEAT_RT_CORE_CYCLE_TIME_MS > 0u, "a zero cycle time would send every pass");

/* D-064 item 1: the DTC monitors' results, and their conditions read directly so that a
 * monitor that did not run (UDS server not stepped) never reads as healthy. */
static bool fault_active(uint32_t now)
{
    const diag_vehicle_tester_t tester = diag_vehicle_tester(now);
    const bool port_latched = can_sm_state_known(CAN_PORT_VEHICLE) &&
                              (can_sm_state(CAN_PORT_VEHICLE) == CAN_SM_LATCHED);
    return heartbeat_fault_active(
        diag_fault_active(), tester.latched,
        tester.fault == (uint8_t)PLATFORM_UDS_VEHICLE_TESTER_STATUS_FAULT_NOT_RUNNING, port_latched);
}

static void send(heartbeat_t* hb, uint32_t now)
{
    struct platform_heartbeat_rt_core_t msg;
    moto_e2e_tx_state_t next;
    uint8_t data[PLATFORM_HEARTBEAT_RT_CORE_LENGTH];

    heartbeat_msg(!hb->stepped, fault_active(now), diag_fault_onsets(), &hb->uptime, &msg);
    const bool built = heartbeat_frame(&msg, &hb->e2e, &next, data);
    can_port_status_t st = CAN_PORT_ERR_ARG;
    const bool finished = com_send(PLATFORM_HEARTBEAT_RT_CORE_FRAME_ID,
                                   PLATFORM_HEARTBEAT_RT_CORE_IS_EXTENDED != 0, built ? data : NULL,
                                   (uint8_t)PLATFORM_HEARTBEAT_RT_CORE_LENGTH, &hb->stats, &st);
    if (st == CAN_PORT_OK) {
        hb->e2e = next; /* the counter advances with accepted frames only */
    }
    if (finished) {
        com_cycle_done(&hb->cycle, now, PLATFORM_HEARTBEAT_RT_CORE_CYCLE_TIME_MS);
    }
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: called by the host program and the H7 setup
bool heartbeat_open(heartbeat_t* hb)
{
    if (hb == NULL) {
        return false;
    }
    (void)memset(hb, 0, sizeof *hb);
    moto_e2e_tx_init(&hb->e2e);
    heartbeat_uptime_init(&hb->uptime, timebase_now_ms());
    hb->opened = (can_if_register_tx_dedicated(CAN_PORT_PLATFORM, PLATFORM_HEARTBEAT_RT_CORE_FRAME_ID,
                                               PLATFORM_HEARTBEAT_RT_CORE_IS_EXTENDED != 0) == CAN_IF_OK);
    return hb->opened;
}

void heartbeat_step(heartbeat_t* hb)
{
    if ((hb == NULL) || !hb->opened) {
        return; /* never 0x081 without its dedicated buffer */
    }
    const uint32_t now = timebase_now_ms();
    heartbeat_uptime_update(&hb->uptime, now);
    if (com_cycle_due(&hb->cycle, now)) {
        send(hb, now);
    }
    hb->stepped = true;
}
