#ifndef SERVICES_CAN_IF_H
#define SERVICES_CAN_IF_H

/*
 * CAN interface service (≈ AUTOSAR CanIf, concept only, D-006): the one place that
 * reads the CAN ports. Features register a receiver per (port, ID, format);
 * can_if_dispatch() drains a port and hands each frame to its receiver. TX goes
 * to the port, except that every frame for CAN_PORT_VEHICLE must first pass the
 * vehicle-bus guard (below). Features include this header, never hal/.
 *
 * Vehicle-bus guard (D-020, CLAUDE.md safety rule 2): fixed and fail-closed, not
 * configurable. A frame goes out on CAN_PORT_VEHICLE only if it is a 29-bit frame on
 * VEHICLE_CL250_REQUEST_ID with DLC VEHICLE_CL250_FRAME_DLC whose bytes pass the
 * generated vehicle_cl250_frame_allowed() (Single Frame, allowed service) and whose
 * bytes after the payload are VEHICLE_CL250_PADDING_BYTE. Anything else returns
 * CAN_PORT_ERR_REFUSED and is counted.
 *
 * Port state (Ç1, services/can_sm): a port that is bus-off or latched takes no frame.
 * can_if_write() then returns CAN_PORT_ERR_IO and counts it in can_if_tx_blocked_count(),
 * never in the guard's refused counter (a bus-off is not a guard refusal), and
 * can_if_tx_free() is false. can_if_tx_allowed() stays the pure, stateless D-020 guard.
 *
 * Acceptance filters: once every receiver is registered, can_if_apply_filters(port) hands
 * the port exactly the registered (ID, format) pairs; later registrations on that port
 * are refused (CAN_IF_ERR_SEALED), so the receiver table and the hardware filters never
 * differ.
 *
 * Static table of CAN_IF_MAX_RECEIVERS entries, no heap. A frame with no receiver is
 * counted and dropped. Main loop only (not ISR-safe).
 */

#include "hal/can_types.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Vehicle: link RX + 2 physical + 2 functional request-ID watches (D-039, D-040);
 * platform: UDS server physical + functional; room for the SIL tester and growth. */
#define CAN_IF_MAX_RECEIVERS 12u

typedef void (*can_if_rx_fn)(void* ctx, const can_frame_t* frame);

typedef enum {
    CAN_IF_OK = 0,
    CAN_IF_ERR_ARG,   /* NULL callback, unknown port or ID out of range */
    CAN_IF_ERR_DUP,   /* (port, ID, format) already has a receiver */
    CAN_IF_ERR_FULL,  /* receiver table full */
    CAN_IF_ERR_SEALED /* the port's filters were applied: no new receiver */
} can_if_status_t;

/* Clears the receiver table, the counters and the filter seals, and resets services/can_sm.
 * Boot only: it would also clear a vehicle bus-off latch (D-030, D-054). */
void can_if_init(void);

can_if_status_t can_if_register_rx(can_port_id_t port, uint32_t id, bool extended,
                                   can_if_rx_fn fn, void* ctx);

/* Reads up to max_frames frames from the port and dispatches them. Returns the count read. */
uint32_t can_if_dispatch(can_port_id_t port, uint32_t max_frames);

/* CAN_PORT_ERR_REFUSED: the vehicle-bus guard refused the frame (not written).
 * CAN_PORT_ERR_IO: the port is bus-off or latched (services/can_sm; not written). */
can_port_status_t can_if_write(can_port_id_t port, const can_frame_t* frame);

/* True if the port may send (services/can_sm) and has a free TX slot. */
bool can_if_tx_free(can_port_id_t port);

/* Changes whenever the port's pending TX was aborted (N_As or bus-off, services/can_sm);
 * wraps. An ISO-TP link whose message was under way ends it with N_TIMEOUT_A. */
uint32_t can_if_tx_abort_count(can_port_id_t port);

/* Writes refused because the port was bus-off or latched (diagnostics). */
uint32_t can_if_tx_blocked_count(can_port_id_t port);

/* Sets the port's acceptance filters to the registered receivers of that port and seals
 * it. On CAN_PORT_PLATFORM the gen/ platform UDS IDs (diagnostics, 0x7xx) go to FIFO1,
 * every other ID to FIFO0. Returns the port's status (CAN_PORT_ERR_ARG for an unknown
 * port). The receiver table always fits the filters (static assertion). */
can_port_status_t can_if_apply_filters(can_port_id_t port);

/* Frames read from the port with no registered receiver (diagnostics). */
uint32_t can_if_unrouted_count(can_port_id_t port);

/* Frames the vehicle-bus guard refused on this port since init (diagnostics). */
uint32_t can_if_tx_refused_count(can_port_id_t port);

/* The guard itself, exposed for tests: true if the frame may go out on the port. */
bool can_if_tx_allowed(can_port_id_t port, const can_frame_t* frame);

#ifdef __cplusplus
}
#endif

#endif /* SERVICES_CAN_IF_H */
