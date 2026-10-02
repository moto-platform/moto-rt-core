#ifndef HAL_CAN_PORT_H
#define HAL_CAN_PORT_H

/*
 * CAN port interface (HAL-free). One implementation per platform:
 *  - hal/host/can_port_host.c : in-process virtual bus, or Linux SocketCAN (vcan/can)
 *  - hal/stm32/ (later)       : FDCAN1 (vehicle bus) / FDCAN2 (platform bus), once the
 *                               board (H743/H723, Q-019) and its CubeMX project exist
 * Only services/ (and the platform setup in app/) include this header. Features talk to
 * CAN through services/can_if, which also enforces the vehicle-bus guard (D-020); CI
 * fails if a file under src/features/ includes hal/.
 *
 * All calls are non-blocking and safe to call from the main loop only (not from an ISR).
 *
 * TX free: on CAN_PORT_VEHICLE, true only while no frame is pending (one TX buffer: the
 * tester has one request in flight, D-021), so a request never queues behind a stuck
 * one. On CAN_PORT_PLATFORM, true while no Tx FIFO element is pending: FDCAN2 requests
 * one FIFO element at a time (M_CAN erratum "Tx FIFO message sequence inversion",
 * hal/README.md), and a FIFO write while one is pending returns CAN_PORT_TX_FULL.
 *
 * Dedicated TX buffers (D-054 item 6, D-056): an ID handed to can_port_set_tx_dedicated()
 * has its own buffer instead of the Tx FIFO, with replace-on-new semantics, so its
 * frame never waits behind other traffic and is never older than one write:
 *  - can_port_write() of that ID puts the frame in its buffer if the buffer is empty
 *    (CAN_PORT_OK), whatever can_port_tx_free() says about the Tx FIFO.
 *  - If the buffer still holds an unsent frame, the write requests its cancellation
 *    (FDCAN: TXBCR for that buffer only) and returns CAN_PORT_TX_FULL; the new frame is
 *    not written. A later write is accepted once the cancellation completed (FDCAN: its
 *    TXBRP bit cleared). No wait loop. A frame already in arbitration may still go out.
 *  - Such a replace cancellation is not an abort: it does not count in tx_done and
 *    services/can_sm never sees it (no N_As, no abort sequence change).
 *  - can_port_tx_abort() cancels the dedicated buffers too; tx_pending counts them.
 * The dedicated buffers and the Tx FIFO head arbitrate by ID (ISO 11898-1).
 */

#include "hal/can_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Queues one frame for transmission. CAN_PORT_TX_FULL leaves the frame unsent. */
can_port_status_t can_port_write(can_port_id_t port, const can_frame_t* frame);

/* True if can_port_write() would accept a frame now. */
bool can_port_tx_free(can_port_id_t port);

/* Takes the oldest received frame, or returns CAN_PORT_EMPTY. */
can_port_status_t can_port_read(can_port_id_t port, can_frame_t* frame);

/*
 * Controller supervision (Ç1, services/can_sm only). None of these puts a frame on the
 * bus: the vehicle-bus boundary (D-020, D-037) is unchanged.
 */

/* Fills *state with the controller's error state and TX progress. */
can_port_status_t can_port_get_state(can_port_id_t port, can_port_state_t* state);

/* Starts the bus-off recovery: the controller leaves its init state and rejoins the bus
 * after 128 x 11 recessive bits (ISO 11898-1). FDCAN: clear CCCR.INIT. No-op unless the
 * port is bus-off. */
can_port_status_t can_port_recover(can_port_id_t port);

/* Cancels every frame accepted but not sent yet (FDCAN: TXBCR for all pending buffers),
 * so a stale frame never goes out late (N_As abort, bus-off). */
can_port_status_t can_port_tx_abort(can_port_id_t port);

/* Replaces the acceptance filters: the port then receives only frames whose (ID, format)
 * is in the list, each into its FIFO; every other frame and every remote frame is
 * rejected (FDCAN GFC: ANFS/ANFE reject, RRFS/RRFE reject). count <= CAN_PORT_MAX_FILTERS.
 * Until it is called, a port receives every data frame. */
can_port_status_t can_port_set_filters(can_port_id_t port, const can_port_filter_t* filters,
                                       uint32_t count);

/* Replaces the port's dedicated TX buffer list (see above): every ID gets one buffer,
 * every other ID uses the Tx FIFO. count <= CAN_PORT_MAX_TX_DEDICATED, no duplicates,
 * valid IDs. Written with the filters, while the controller is in its init state (FDCAN:
 * TXBC with CCCR.INIT = 1 and CCE = 1). CAN_PORT_VEHICLE takes count 0 only: the
 * tester's single TX buffer is fixed (D-021). The SocketCAN host backend has no
 * dedicated buffers and accepts the list without effect (development path only). */
can_port_status_t can_port_set_tx_dedicated(can_port_id_t port, const can_port_tx_id_t* ids,
                                            uint32_t count);

#ifdef __cplusplus
}
#endif

#endif /* HAL_CAN_PORT_H */
