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
 * Static table of CAN_IF_MAX_RECEIVERS entries, no heap. A frame with no receiver is
 * counted and dropped. Main loop only (not ISR-safe).
 */

#include "hal/can_types.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CAN_IF_MAX_RECEIVERS 8u

typedef void (*can_if_rx_fn)(void* ctx, const can_frame_t* frame);

typedef enum {
    CAN_IF_OK = 0,
    CAN_IF_ERR_ARG,   /* NULL callback, unknown port or ID out of range */
    CAN_IF_ERR_DUP,   /* (port, ID, format) already has a receiver */
    CAN_IF_ERR_FULL   /* receiver table full */
} can_if_status_t;

/* Clears the receiver table and counters. */
void can_if_init(void);

can_if_status_t can_if_register_rx(can_port_id_t port, uint32_t id, bool extended,
                                   can_if_rx_fn fn, void* ctx);

/* Reads up to max_frames frames from the port and dispatches them. Returns the count read. */
uint32_t can_if_dispatch(can_port_id_t port, uint32_t max_frames);

/* CAN_PORT_ERR_REFUSED: the vehicle-bus guard refused the frame (not written). */
can_port_status_t can_if_write(can_port_id_t port, const can_frame_t* frame);
bool can_if_tx_free(can_port_id_t port);

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
