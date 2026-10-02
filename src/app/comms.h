#ifndef APP_COMMS_H
#define APP_COMMS_H

/*
 * The comms pass (Ç1, ISSUES D-2): the fixed order of rt-core's CAN work, shared by the
 * host program and the H7 comms task (one task runs both UDS roles):
 *   can_sm_step(VEHICLE)  -> can_if_dispatch(VEHICLE)  -> uds_client_step()
 *   can_sm_step(PLATFORM) -> can_if_dispatch(PLATFORM) -> uds_server_step()
 * The client steps before the server: the server reports what the client wrote in the
 * same pass (services/diag, vehicle_signals), and the vehicle tester's step period
 * (D-053, VEHICLE_CL250_CLIENT_STEP_MAX_MS) never waits for a platform-bus answer.
 * Each port's state manager runs before its dispatch, so a bus-off or an N_As abort is
 * seen by the links in the same pass.
 *
 * Portable: services and feature headers only, never hal/.
 */

#include "features/uds/uds_client.h"
#include "features/uds/uds_server.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Frames dispatched per port and pass at most (bounds the pass). */
#define COMMS_RX_PER_PASS 32u

/* Applies the acceptance filters of both ports (services/can_if) after the client and
 * the server opened. False if a port refused them. */
bool comms_apply_filters(void);

void comms_pass(uds_client_t* client, uds_server_t* server);

#ifdef __cplusplus
}
#endif

#endif /* APP_COMMS_H */
