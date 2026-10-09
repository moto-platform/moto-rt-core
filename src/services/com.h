#ifndef SERVICES_COM_H
#define SERVICES_COM_H

/*
 * Platform-bus send path of rt-core's cyclic senders (D-056 items 3, 5 and 7): one frame
 * through services/can_if on CAN_PORT_PLATFORM and the classification of the write
 * result, shared so that no feature includes another. The sender keeps its own E2E
 * state and commits it only when the status is CAN_PORT_OK (D-056 item 5). Main loop
 * only. Static state, no heap.
 */

#include "hal/can_types.h"
#include "services/com_core.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Saturating counters (a dead bus can refuse a write every pass). */
typedef struct {
    uint32_t sent;     /* frames the port accepted */
    uint32_t retried;  /* writes refused with CAN_PORT_TX_FULL, tried again next pass */
    uint32_t dropped;  /* cycles given up: port bus-off/latched or a packing failure */
} com_msg_stats_t;

/*
 * Writes (id, extended, data[0..dlc)) to the platform port; data NULL means the sender
 * could not build the frame, which is dropped like a bus-off (so is a dlc above
 * CAN_PORT_MAX_DLC). stats and status must be valid. *status receives the
 * write result (CAN_PORT_ERR_ARG when nothing was written). Returns true when the cycle
 * is finished (CAN_PORT_OK, or given up: bus-off, latched, not built) and false to retry
 * in the next pass (CAN_PORT_TX_FULL: a replace cancel pending or the Tx FIFO element in
 * use).
 */
bool com_send(uint32_t id, bool extended, const uint8_t* data, uint8_t dlc,
              com_msg_stats_t* stats, can_port_status_t* status);

#ifdef __cplusplus
}
#endif

#endif /* SERVICES_COM_H */
