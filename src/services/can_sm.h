#ifndef SERVICES_CAN_SM_H
#define SERVICES_CAN_SM_H

/*
 * CAN state manager (Ç1, ≈ AUTOSAR CanSM as a concept, D-006): runs services/can_sm_core
 * for each port on the controller state from hal/can_port.h and carries out its actions
 * (abort the pending TX, start a bus-off recovery). None of them sends a frame.
 *
 * Per port (D-054):
 *   CAN_PORT_VEHICLE   backoff VEHICLE_CL250_BUS_OFF_BACKOFF_INITIAL_MS .. _MAX_MS (gen/),
 *                      latched after CAN_SM_VEHICLE_BUS_OFF_LATCH bus-offs (D-030):
 *                      off the vehicle bus until reboot
 *   CAN_PORT_PLATFORM  the same backoff schedule (local, provisional), never latched:
 *                      the platform bus carries rt-core's own traffic (D-009)
 *   both               N_As = CAN_SM_TX_TIMEOUT_MS, timed from the first step that sees a
 *                      frame pending, so up to N_As plus one step
 *
 * services/can_if asks can_sm_tx_allowed() before every write and in can_if_tx_free(),
 * and exposes can_sm_abort_seq() to the ISO-TP links. can_if_init() calls can_sm_init().
 * Main loop only (not ISR-safe). Run can_sm_step(port) before can_if_dispatch(port).
 */

#include "hal/can_types.h"
#include "services/can_sm_core.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ISO 15765-2:2016 Table 16, N_As timeout default (a generic ISO constant, D-030). */
#define CAN_SM_TX_TIMEOUT_MS 1000u

/* D-030 (connectivity-node parity), D-054: the vehicle port latches on its 5th bus-off
 * since boot. A local constant until it moves to uds/vehicle_cl250.yaml. */
#define CAN_SM_VEHICLE_BUS_OFF_LATCH 5u

/* D-054, provisional: the platform port uses the vehicle bus-off schedule as local
 * values until the platform bus timing exists in moto-vehicle-defs. */
#define CAN_SM_PLATFORM_BACKOFF_INITIAL_MS 1000u
#define CAN_SM_PLATFORM_BACKOFF_MAX_MS 30000u

/* Resets both ports to ERROR_ACTIVE with their configuration. Boot only (through
 * can_if_init()): it also clears a vehicle latch, which by D-030 / D-054 lasts until
 * reboot, so no runtime path may call it again. */
void can_sm_init(void);

/* Reads the port's controller state and runs one step (no-op before can_sm_init()). If
 * the port cannot report its state (e.g. not bound), TX on it is refused and its pending
 * TX aborted once, until a step reads the state again (fail-closed). */
void can_sm_step(can_port_id_t port);

/* CAN_SM_LATCHED for an unknown port and before can_sm_init(). */
can_sm_state_t can_sm_state(can_port_id_t port);

/* True when can_sm_state() reports what the port really is (D-055, health DID): after
 * can_sm_init(), for a known port, once a step read a controller snapshot and while the
 * last read succeeded. A latch is rt-core's own state, so a latched port stays known
 * even if its controller cannot be read. False otherwise: the caller reports UNKNOWN,
 * never the last state (safety review MAJOR-1 on defs#32). */
bool can_sm_state_known(can_port_id_t port);

/* True in ERROR_ACTIVE / ERROR_PASSIVE. False for an unknown port and before can_sm_init()
 * (can_if_init() calls it), so nothing is sent fail-open. */
bool can_sm_tx_allowed(can_port_id_t port);

/* Changes on every TX abort of the port (wraps). */
uint32_t can_sm_abort_seq(can_port_id_t port);

/* NULL for an unknown port. */
const can_sm_stats_t* can_sm_stats(can_port_id_t port);

#ifdef __cplusplus
}
#endif

#endif /* SERVICES_CAN_SM_H */
