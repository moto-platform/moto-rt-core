#include "hal/host/vbus.h"

#include <stddef.h>

static bool node_ok(const vbus_t* bus, uint8_t node)
{
    return (bus != NULL) && (node < VBUS_MAX_NODES) && bus->nodes[node].attached;
}

void vbus_init(vbus_t* bus)
{
    if (bus == NULL) {
        return;
    }
    for (uint8_t i = 0u; i < VBUS_MAX_NODES; i++) {
        bus->nodes[i].head = 0u;
        bus->nodes[i].count = 0u;
        bus->nodes[i].overruns = 0u;
        bus->nodes[i].attached = false;
        bus->nodes[i].tx_blocked = false;
    }
    bus->frame_count = 0u;
}

bool vbus_attach(vbus_t* bus, uint8_t* node)
{
    if ((bus == NULL) || (node == NULL)) {
        return false;
    }
    for (uint8_t i = 0u; i < VBUS_MAX_NODES; i++) {
        if (!bus->nodes[i].attached) {
            bus->nodes[i].attached = true;
            *node = i;
            return true;
        }
    }
    return false;
}

can_port_status_t vbus_send(vbus_t* bus, uint8_t node, const can_frame_t* frame)
{
    if (!node_ok(bus, node) || !can_frame_valid(frame)) {
        return CAN_PORT_ERR_ARG;
    }
    if (bus->nodes[node].tx_blocked) {
        return CAN_PORT_TX_FULL;
    }
    for (uint8_t i = 0u; i < VBUS_MAX_NODES; i++) {
        vbus_node_t* dst = &bus->nodes[i];
        if ((i == node) || !dst->attached) {
            continue;
        }
        if (dst->count >= VBUS_RX_DEPTH) {
            dst->overruns++;
            continue;
        }
        uint16_t tail = (uint16_t)((dst->head + dst->count) % VBUS_RX_DEPTH);
        dst->queue[tail] = *frame;
        dst->count++;
    }
    bus->frame_count++;
    return CAN_PORT_OK;
}

can_port_status_t vbus_recv(vbus_t* bus, uint8_t node, can_frame_t* frame)
{
    if (!node_ok(bus, node) || (frame == NULL)) {
        return CAN_PORT_ERR_ARG;
    }
    vbus_node_t* n = &bus->nodes[node];
    if (n->count == 0u) {
        return CAN_PORT_EMPTY;
    }
    *frame = n->queue[n->head];
    n->head = (uint16_t)((n->head + 1u) % VBUS_RX_DEPTH);
    n->count--;
    return CAN_PORT_OK;
}

bool vbus_tx_free(const vbus_t* bus, uint8_t node)
{
    return node_ok(bus, node) && !bus->nodes[node].tx_blocked;
}

void vbus_set_tx_blocked(vbus_t* bus, uint8_t node, bool blocked)
{
    if (node_ok(bus, node)) {
        bus->nodes[node].tx_blocked = blocked;
    }
}

uint32_t vbus_overruns(const vbus_t* bus, uint8_t node)
{
    return node_ok(bus, node) ? bus->nodes[node].overruns : 0u;
}

uint32_t vbus_frame_count(const vbus_t* bus)
{
    return (bus != NULL) ? bus->frame_count : 0u;
}
