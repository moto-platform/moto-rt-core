#ifndef FEATURES_VEHICLE_REPUBLISH_H
#define FEATURES_VEHICLE_REPUBLISH_H

/*
 * Platform-bus republisher (ISSUES D-5, D-021, D-041, D-048, D-056): sends the samples of
 * services/vehicle_signals as the gen/ platform messages 0x021 VehicleSpeed (E2E, safety
 * range, a dedicated replace-on-new TX buffer) and 0x110 VehicleEngine (state range, Tx
 * FIFO), each at its gen/ cycle time. NONE, STALE and out-of-range samples go out as
 * INVALID. 0x021 is not a safety-node input (D-041, D-048): no safety decision may use it.
 *
 * Open it before comms_apply_filters() (it registers its dedicated TX ID); step it every
 * comms pass, after the platform port's can_sm_step() and dispatch (app/comms). Main loop
 * only, like services/vehicle_signals. Static state, no heap.
 */

#include "features/vehicle_republish/vehicle_republish_core.h"
#include "services/com.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* The shared send counters (services/com). */
typedef com_msg_stats_t vehicle_republish_msg_stats_t;

typedef struct {
    com_cycle_t speed_cycle;
    com_cycle_t engine_cycle;
    moto_e2e_tx_state_t speed_e2e; /* committed: advanced only by accepted frames */
    vehicle_republish_msg_stats_t speed;
    vehicle_republish_msg_stats_t engine;
    bool opened; /* 0x021's dedicated buffer was registered: the only state that sends */
} vehicle_republish_t;

/* Resets r and registers 0x021's dedicated TX buffer on the platform port. False if
 * can_if refused it (sealed, already listed, full): r then stays closed and
 * vehicle_republish_step() sends nothing, so 0x021 never goes through the Tx FIFO
 * (D-054 item 6, D-056 item 6; safety review MAJOR-1). */
bool vehicle_republish_open(vehicle_republish_t* r);

/* Sends every message whose cycle is due; nothing unless r was opened. */
void vehicle_republish_step(vehicle_republish_t* r);

#ifdef __cplusplus
}
#endif

#endif /* FEATURES_VEHICLE_REPUBLISH_H */
