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
 * Controller faults (Ç1, services/can_sm):
 *  - TX stall (no ACK, lost arbitration forever): frames are accepted into the node's
 *    TX buffers (VBUS_TX_DEPTH) but not sent; un-stalling sends them late, in order,
 *    unless vbus_tx_abort() dropped them first.
 *  - Bus-off: the node sends and receives nothing, and its bus-off counter goes up;
 *    vbus_recover() (can_port_recover()) brings it back.
 *  - Error counters (TEC/REC) as the controller would report them.
 *  - Acceptance filters (can_port_set_filters()): exact (ID, format) matches; the FIFO
 *    is ignored (one RX queue).
 */

#include "hal/can_port.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VBUS_MAX_NODES 6u
#define VBUS_RX_DEPTH 64u /* frames queued per node until it reads them */
#define VBUS_TX_DEPTH 3u  /* TX buffers per node (like a bxCAN/FDCAN TX queue) */

typedef struct {
    can_frame_t queue[VBUS_RX_DEPTH];
    uint16_t head;
    uint16_t count;
    uint32_t overruns;
    can_frame_t tx_pending[VBUS_TX_DEPTH];
    uint32_t tx_pending_count;
    uint32_t tx_done;
    uint32_t bus_off_events;
    uint32_t recover_count;
    can_port_filter_t filters[CAN_PORT_MAX_FILTERS];
    uint32_t filter_count;
    uint8_t tec;
    uint8_t rec;
    bool filtered;
    bool attached;
    bool tx_blocked;
    bool tx_stalled;
    bool bus_off;
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

/* TX stall on: accepted frames wait in the TX buffers. Off: they are sent now, in order. */
void vbus_set_tx_stalled(vbus_t* bus, uint8_t node, bool stalled);

/* Drops the node's pending TX frames (can_port_tx_abort()). */
void vbus_tx_abort(vbus_t* bus, uint8_t node);

/* Bus-off on (counted once per entry) or off. */
void vbus_set_bus_off(vbus_t* bus, uint8_t node, bool bus_off);

/* Test hook: `count` bus-off entries the driver's ISR counted between two steps (the
 * port is back on the bus), e.g. to check saturating readers. */
void vbus_add_bus_off_events(vbus_t* bus, uint8_t node, uint32_t count);

/* can_port_recover(): leaves bus-off; counted, so tests see each attempt. */
void vbus_recover(vbus_t* bus, uint8_t node);
uint32_t vbus_recover_count(const vbus_t* bus, uint8_t node);

void vbus_set_error_counters(vbus_t* bus, uint8_t node, uint8_t tec, uint8_t rec);

/* CAN_PORT_ERR_ARG for count > CAN_PORT_MAX_FILTERS or a NULL list with count > 0. */
can_port_status_t vbus_set_filters(vbus_t* bus, uint8_t node, const can_port_filter_t* filters,
                                   uint32_t count);

/* The node's controller state, as can_port_get_state() reports it. */
can_port_status_t vbus_state(const vbus_t* bus, uint8_t node, can_port_state_t* state);
uint32_t vbus_overruns(const vbus_t* bus, uint8_t node);
uint32_t vbus_frame_count(const vbus_t* bus);

#ifdef __cplusplus
}
#endif

#endif /* HAL_HOST_VBUS_H */
