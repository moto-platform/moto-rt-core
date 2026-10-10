#ifndef APP_COMMS_H
#define APP_COMMS_H

/*
 * The comms pass (Ç1, ISSUES D-2): the fixed order of rt-core's CAN work, shared by the
 * host program and the H7 comms task (one task runs both UDS roles):
 *   can_sm_step(VEHICLE)  -> can_if_dispatch(VEHICLE)  -> uds_client_step()
 *   can_sm_step(PLATFORM) -> can_if_dispatch(PLATFORM) -> vehicle_republish_step()
 *                                                      -> heartbeat_step()
 *                                                      -> uds_server_step()
 * The client steps before the server: the server reports what the client wrote in the
 * same pass (services/diag, vehicle_signals), and the vehicle tester's step period
 * (D-053, VEHICLE_CL250_CLIENT_STEP_MAX_MS) never waits for a platform-bus answer.
 * Each port's state manager runs before its dispatch, so a bus-off or an N_As abort is
 * seen by the links in the same pass. The republisher (D-056 item 4) sends the samples
 * the client wrote in this pass, sees the platform port's current state, and its E2E
 * frame never waits behind server work. The pass period therefore bounds 0x021's
 * freshness: the 20 ms EKF work (0x020) must not join this task (CLAUDE.md rule 4).
 * The heartbeat (0x081, D-064) follows for the same reasons; it sees the DTC results
 * the server's monitors reported in the previous pass, and reads their conditions
 * directly as well, so a server that does not step never makes it read healthy.
 * Before it, every pass samples the EKF task's alive counter (D-064 item 4, D-065 item
 * 2): this is where the heartbeat learns about the EKF, which it never includes. The EKF
 * runs in its own, higher-priority task and shares only lock-free data with this one
 * (services/alive, services/snapshot).
 *
 * Portable: services and feature headers only, never a direct hal/ include
 * (services/alive.h brings hal/hal_atomic.h transitively, like can_if.h brings
 * hal/can_types.h).
 */

#include "features/heartbeat/heartbeat.h"
#include "features/uds/uds_client.h"
#include "features/uds/uds_server.h"
#include "features/vehicle_republish/vehicle_republish.h"
#include "platform_e2e.h"
#include "services/alive.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Frames dispatched per port and pass at most (bounds the pass). */
#define COMMS_RX_PER_PASS 32u

/* EKF alive window (D-064 item 4, D-065 item 2): the EKF is stalled when its counter
 * did not change for longer than the receivers' 0x020 E2E timeout (gen/, 60 ms = three
 * 20 ms EKF cycles): by then its output would be too old for safety-node anyway. */
#define COMMS_EKF_ALIVE_WINDOW_MS PLATFORM_EKF_LEAN_E2E_TIMEOUT_MS

/* The EKF supervision of the comms pass. Zeroed (comms_ekf_init() or static storage):
 * no EKF registered, ekf_stalled stays false and NODE_MODE does not attest the EKF.
 * Owned by the comms task; only the counter is shared with the EKF task. */
typedef struct {
    const alive_counter_t* counter; /* NULL until comms_ekf_register() */
    alive_monitor_t monitor;
} comms_ekf_t;

/* No EKF registered. */
void comms_ekf_init(comms_ekf_t* ekf);

/* Boot only, after alive_init(counter) and BEFORE the scheduler starts the comms task
 * (safety-reviewer MAJOR-1): the comms pass reads ekf without a lock, so registering
 * from a running task (e.g. the EKF task's own init) is not allowed. From now on the heartbeat reads DEGRADED until the
 * counter changes (registered but never ran), and whenever it stops changing for longer
 * than COMMS_EKF_ALIVE_WINDOW_MS. The baseline is the count read now. False for NULL or
 * a second registration (the first stays). */
bool comms_ekf_register(comms_ekf_t* ekf, const alive_counter_t* counter);

/* True once an EKF counter is registered. 0x020's sender must refuse to open without it
 * (D-065 PR 3): a lean never goes out unsupervised. */
bool comms_ekf_registered(const comms_ekf_t* ekf);

/* Applies the acceptance filters (and the platform port's dedicated TX buffers) of both
 * ports (services/can_if) after the client, the server, the republisher and the heartbeat opened.
 * False if a port refused them. */
bool comms_apply_filters(void);

/* ekf: never NULL in a correct setup; NULL reads as a stalled EKF (DEGRADED, the safe
 * side), while a zeroed comms_ekf_t means "no EKF". */
void comms_pass(uds_client_t* client, uds_server_t* server, vehicle_republish_t* republisher,
                heartbeat_t* heartbeat, comms_ekf_t* ekf);

#ifdef __cplusplus
}
#endif

#endif /* APP_COMMS_H */
