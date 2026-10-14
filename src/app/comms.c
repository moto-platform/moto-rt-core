#include "app/comms.h"

#include "platform.h"
#include "services/can_if.h"
#include "services/can_sm.h"
#include "services/timebase.h"

#include <stddef.h>
#include <string.h>

/* One late EKF cycle must not read as a stall, and the window must stay within the
 * receivers' 0x020 timeout (D-065 item 2). */
_Static_assert(COMMS_EKF_ALIVE_WINDOW_MS >= (2u * PLATFORM_EKF_LEAN_CYCLE_TIME_MS),
               "the EKF alive window must cover a late EKF cycle");
_Static_assert(COMMS_EKF_ALIVE_WINDOW_MS <= PLATFORM_EKF_LEAN_E2E_TIMEOUT_MS,
               "the EKF alive window must not exceed the 0x020 receiver timeout");

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: called by the host program and the H7 setup
void comms_ekf_init(comms_ekf_t* ekf)
{
    if (ekf != NULL) {
        (void)memset(ekf, 0, sizeof *ekf);
    }
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: called by the EKF setup (D-065 PR 3) and the tests
bool comms_ekf_register(comms_ekf_t* ekf, const alive_counter_t* counter)
{
    if ((ekf == NULL) || (counter == NULL) || (ekf->counter != NULL)) {
        return false;
    }
    /* monitor first, then the pointer that enables it (safety-reviewer MAJOR-1) */
    alive_monitor_init(&ekf->monitor, alive_count(counter));
    ekf->counter = counter;
    return true;
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: called by the 0x020 sender's open (D-065 PR 3) and the tests
bool comms_ekf_registered(const comms_ekf_t* ekf)
{
    return (ekf != NULL) && (ekf->counter != NULL);
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: called by the boot setup (host program, H7) and the tests
bool comms_open_lean(const comms_ekf_t* ekf, lean_tx_t* tx, lean_link_t* link)
{
    if (!comms_ekf_registered(ekf)) {
        if (tx != NULL) {
            (void)lean_tx_open(tx, NULL, NULL); /* resets tx closed */
        }
        return false;
    }
    return lean_tx_open(tx, link, ekf->counter);
}

/* D-064 item 4: sampled every pass, so the change stamp is at most one pass late. */
static bool ekf_stalled(comms_ekf_t* ekf)
{
    if (ekf == NULL) {
        return true; /* misuse reads as a stalled EKF, never as a healthy one */
    }
    if (ekf->counter == NULL) {
        return false; /* no EKF registered: nothing to attest (and no 0x020) */
    }
    return alive_monitor_stalled(&ekf->monitor, alive_count(ekf->counter), timebase_now_ms(),
                                 COMMS_EKF_ALIVE_WINDOW_MS);
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: called by the host program and the H7 task
bool comms_apply_filters(void)
{
    const can_port_status_t vehicle = can_if_apply_filters(CAN_PORT_VEHICLE);
    const can_port_status_t platform = can_if_apply_filters(CAN_PORT_PLATFORM);
    return (vehicle == CAN_PORT_OK) && (platform == CAN_PORT_OK);
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: called by the host program and the H7 task
void comms_pass(uds_client_t* client, uds_server_t* server, vehicle_republish_t* republisher,
                heartbeat_t* heartbeat, comms_ekf_t* ekf, lean_tx_t* lean)
{
    /* 0x020 runs only under the supervision it was opened with (D-065 amendment): an open
     * lean link with another (or no) EKF counter here stops 0x020 and reads as a stalled
     * EKF, never as "no EKF" (safety-reviewer MINOR-2 on 3b). */
    const bool lean_open = (lean != NULL) && lean->opened;
    const bool lean_supervised =
        lean_open && (ekf != NULL) && (ekf->counter == lean_link_alive(lean->link));
    lean_tx_t* const active = lean_supervised ? lean : NULL;

    can_sm_step(CAN_PORT_VEHICLE);
    (void)can_if_dispatch(CAN_PORT_VEHICLE, COMMS_RX_PER_PASS);
    uds_client_step(client);
    lean_tx_publish_speed(active, timebase_now_ms()); /* the sample the client wrote in this pass */

    can_sm_step(CAN_PORT_PLATFORM);
    (void)can_if_dispatch(CAN_PORT_PLATFORM, COMMS_RX_PER_PASS);
    vehicle_republish_step(republisher);
    const bool stalled = ekf_stalled(ekf); /* sampled every pass */
    heartbeat_step(heartbeat, stalled || (lean_open && !lean_supervised));
    lean_tx_step(active, timebase_now_ms());
    uds_server_step(server);
}
