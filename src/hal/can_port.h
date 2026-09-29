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

#ifdef __cplusplus
}
#endif

#endif /* HAL_CAN_PORT_H */
