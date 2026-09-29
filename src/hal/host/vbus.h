#ifndef HAL_HOST_VBUS_H
#define HAL_HOST_VBUS_H

/*
 * In-process virtual CAN bus for the host build (D-034): the SIL executable and the
 * mock-bus tests attach several nodes (rt-core ports, simulated ECUs, sniffers) to one
 * bus. A frame sent by a node is delivered to every other attached node, like CAN; the
 * sender does not receive its own frame. Static storage, no threads, no arbitration:
 * frames are delivered in send order the moment they are sent.
 *
 * Fault injection for tests: vbus_set_tx_blocked() makes a node's TX mailbox look
 * full; a full RX queue drops the frame for that node and counts an overrun.
 */

#include "hal/can_port.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VBUS_MAX_NODES 6u
#define VBUS_RX_DEPTH 64u /* frames queued per node until it reads them */

typedef struct {
    can_frame_t queue[VBUS_RX_DEPTH];
    uint16_t head;
    uint16_t count;
    uint32_t overruns;
    bool attached;
    bool tx_blocked;
} vbus_node_t;

typedef struct {
    vbus_node_t nodes[VBUS_MAX_NODES];
    uint32_t frame_count; /* frames put on the bus since init */
} vbus_t;

void vbus_init(vbus_t* bus);

/* Attaches a new node; false if the bus is full. */
bool vbus_attach(vbus_t* bus, uint8_t* node);

can_port_status_t vbus_send(vbus_t* bus, uint8_t node, const can_frame_t* frame);
can_port_status_t vbus_recv(vbus_t* bus, uint8_t node, can_frame_t* frame);
bool vbus_tx_free(const vbus_t* bus, uint8_t node);

void vbus_set_tx_blocked(vbus_t* bus, uint8_t node, bool blocked);
uint32_t vbus_overruns(const vbus_t* bus, uint8_t node);
uint32_t vbus_frame_count(const vbus_t* bus);

#ifdef __cplusplus
}
#endif

#endif /* HAL_HOST_VBUS_H */
