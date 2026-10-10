#ifndef FEATURES_HEARTBEAT_H
#define FEATURES_HEARTBEAT_H

/*
 * rt-core heartbeat (D-005, D-026, D-042 item 3, D-064): the gen/ platform message 0x081
 * HeartbeatRtCore at its gen/ cycle time, E2E-protected, in a dedicated replace-on-new TX
 * buffer (D-054 item 6, D-056 item 6). Receivers (SAFETY, IO, CONN, LINUX) supervise
 * rt-core through it: safety-node uses rt-core's lean only while this frame passes its
 * E2E check and NODE_MODE is NORMAL (D-042 item 3).
 *
 * Open it before comms_apply_filters() (it registers its dedicated TX ID); step it every
 * comms pass, after the platform port's can_sm_step() and dispatch and before the UDS
 * server (app/comms). Main loop only. Static state owned by the caller, no heap.
 */

#include "features/heartbeat/heartbeat_core.h"
#include "services/com.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    com_cycle_t cycle;
    moto_e2e_tx_state_t e2e; /* committed: advanced only by accepted frames */
    heartbeat_uptime_t uptime;
    com_msg_stats_t stats;
    bool stepped; /* a step completed: frames built later are not INIT */
    bool opened;  /* 0x081's dedicated buffer was registered: the only state that sends */
} heartbeat_t;

/* Resets hb, anchors the uptime at the current timebase reading and registers 0x081's
 * dedicated TX buffer on the platform port. False if can_if refused it (sealed, already
 * listed, full): hb then stays closed and heartbeat_step() sends nothing, so 0x081 never
 * goes through the Tx FIFO; its receivers see an E2E timeout. */
bool heartbeat_open(heartbeat_t* hb);

/* Updates the uptime and, when the cycle is due, sends 0x081; nothing unless opened.
 * ekf_stalled: the EKF's alive verdict from app/comms (D-064 item 4), DEGRADED while true;
 * the heartbeat does not know the EKF itself. */
void heartbeat_step(heartbeat_t* hb, bool ekf_stalled);

#ifdef __cplusplus
}
#endif

#endif /* FEATURES_HEARTBEAT_H */
