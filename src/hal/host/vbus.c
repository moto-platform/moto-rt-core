#include "hal/host/vbus.h"

#include <stddef.h>

/* Above this TEC or REC a controller is error passive (ISO 11898-1). */
#define ERROR_PASSIVE_LIMIT 127u

static bool node_ok(const vbus_t* bus, uint8_t node)
{
    return (bus != NULL) && (node < VBUS_MAX_NODES) && bus->nodes[node].attached;
}

static void reset_node(vbus_node_t* n)
{
    n->head = 0u;
    n->count = 0u;
    n->overruns = 0u;
    n->tx_pending_count = 0u;
    n->tx_ded_count = 0u;
    for (uint32_t i = 0u; i < VBUS_TX_DEDICATED; i++) {
        n->tx_ded_pending[i] = false;
    }
    n->tx_replaced = 0u;
    n->tx_cancel_late = false;
    n->tx_done = 0u;
    n->bus_off_events = 0u;
    n->recover_count = 0u;
    n->filter_count = 0u;
    n->tec = 0u;
    n->rec = 0u;
    n->filtered = false;
    n->attached = false;
    n->tx_blocked = false;
    n->tx_stalled = false;
    n->bus_off = false;
}

void vbus_init(vbus_t* bus)
{
    if (bus == NULL) {
        return;
    }
    for (uint8_t i = 0u; i < VBUS_MAX_NODES; i++) {
        reset_node(&bus->nodes[i]);
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
            reset_node(&bus->nodes[i]);
            bus->nodes[i].attached = true;
            *node = i;
            return true;
        }
    }
    return false;
}

static bool accepts(const vbus_node_t* n, const can_frame_t* frame)
{
    if (!n->filtered) {
        return true;
    }
    for (uint32_t i = 0u; i < n->filter_count; i++) {
        if ((n->filters[i].id == frame->id) && (n->filters[i].extended == frame->extended)) {
            return true;
        }
    }
    return false;
}

/* Puts one frame on the bus: every other attached, bus-on node that accepts it gets it. */
static void deliver(vbus_t* bus, uint8_t node, const can_frame_t* frame)
{
    for (uint8_t i = 0u; i < VBUS_MAX_NODES; i++) {
        vbus_node_t* dst = &bus->nodes[i];
        if ((i == node) || !dst->attached || dst->bus_off || !accepts(dst, frame)) {
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
    bus->nodes[node].tx_done++;
}

/* Index of the frame's dedicated slot, or VBUS_TX_DEDICATED if it uses the queue. */
static uint32_t dedicated_slot(const vbus_node_t* n, uint32_t id, bool extended)
{
    for (uint32_t i = 0u; i < n->tx_ded_count; i++) {
        if ((n->tx_ded_ids[i].id == id) && (n->tx_ded_ids[i].extended == extended)) {
            return i;
        }
    }
    return VBUS_TX_DEDICATED;
}

static uint32_t dedicated_pending(const vbus_node_t* n)
{
    uint32_t count = 0u;
    for (uint32_t i = 0u; i < n->tx_ded_count; i++) {
        if (n->tx_ded_pending[i]) {
            count++;
        }
    }
    return count;
}

/* ISO 11898-1 arbitration order: the 11-bit base ID first; a standard frame beats an
 * extended one with the same base (recessive SRR/IDE); then the 18-bit ID extension.
 * The key uses 30 bits (11 + 1 + 18), so the shifts stay inside uint32_t. */
static uint32_t arbitration_key(const can_frame_t* f)
{
    if (!f->extended) {
        return f->id << 19;
    }
    return ((f->id >> 18) << 19) | (1u << 18) | (f->id & 0x3FFFFu);
}

static can_port_status_t send_dedicated(vbus_t* bus, uint8_t node, uint32_t slot,
                                        const can_frame_t* frame)
{
    vbus_node_t* n = &bus->nodes[node];
    if (n->tx_ded_pending[slot]) {
        n->tx_ded_pending[slot] = false; /* replace: cancel the unsent frame */
        if (n->tx_cancel_late) {
            n->tx_cancel_late = false;
            deliver(bus, node, &n->tx_ded_frames[slot]); /* it won arbitration first */
        } else {
            n->tx_replaced++;
        }
        return CAN_PORT_TX_FULL;
    }
    if (n->tx_stalled) {
        n->tx_ded_frames[slot] = *frame;
        n->tx_ded_pending[slot] = true;
        return CAN_PORT_OK;
    }
    deliver(bus, node, frame);
    return CAN_PORT_OK;
}

can_port_status_t vbus_send(vbus_t* bus, uint8_t node, const can_frame_t* frame)
{
    if (!node_ok(bus, node) || !can_frame_valid(frame)) {
        return CAN_PORT_ERR_ARG;
    }
    vbus_node_t* n = &bus->nodes[node];
    if (n->bus_off) {
        return CAN_PORT_ERR_IO;
    }
    if (n->tx_blocked) {
        return CAN_PORT_TX_FULL;
    }
    const uint32_t slot = dedicated_slot(n, frame->id, frame->extended);
    if (slot < VBUS_TX_DEDICATED) {
        return send_dedicated(bus, node, slot, frame);
    }
    if (n->tx_stalled || (n->tx_pending_count > 0u)) {
        if (n->tx_pending_count >= VBUS_TX_DEPTH) {
            return CAN_PORT_TX_FULL;
        }
        n->tx_pending[n->tx_pending_count] = *frame;
        n->tx_pending_count++;
        return CAN_PORT_OK;
    }
    deliver(bus, node, frame);
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
    if (!node_ok(bus, node)) {
        return false;
    }
    const vbus_node_t* n = &bus->nodes[node];
    return !n->tx_blocked && !n->bus_off && (n->tx_pending_count < VBUS_TX_DEPTH);
}

void vbus_set_tx_blocked(vbus_t* bus, uint8_t node, bool blocked)
{
    if (node_ok(bus, node)) {
        bus->nodes[node].tx_blocked = blocked;
    }
}

/* Sends every pending frame: the dedicated slots and the queue head by arbitration, the
 * queue in its own order. */
static void flush_pending(vbus_t* bus, uint8_t node)
{
    vbus_node_t* n = &bus->nodes[node];
    if (n->bus_off) {
        return;
    }
    uint32_t head = 0u;
    const uint32_t total = n->tx_pending_count + dedicated_pending(n);
    for (uint32_t k = 0u; k < total; k++) {
        uint32_t best = VBUS_TX_DEDICATED; /* the queue head */
        const can_frame_t* winner = (head < n->tx_pending_count) ? &n->tx_pending[head] : NULL;
        for (uint32_t i = 0u; i < n->tx_ded_count; i++) {
            if (n->tx_ded_pending[i] &&
                ((winner == NULL) ||
                 (arbitration_key(&n->tx_ded_frames[i]) < arbitration_key(winner)))) {
                best = i;
                winner = &n->tx_ded_frames[i];
            }
        }
        if (best < VBUS_TX_DEDICATED) {
            n->tx_ded_pending[best] = false;
        } else {
            head++;
        }
        deliver(bus, node, winner);
    }
    n->tx_pending_count = 0u;
}

void vbus_set_tx_stalled(vbus_t* bus, uint8_t node, bool stalled)
{
    if (!node_ok(bus, node)) {
        return;
    }
    bus->nodes[node].tx_stalled = stalled;
    if (!stalled) {
        flush_pending(bus, node); /* the late frames a missing abort would let out */
    }
}

void vbus_tx_abort(vbus_t* bus, uint8_t node)
{
    if (!node_ok(bus, node)) {
        return;
    }
    vbus_node_t* n = &bus->nodes[node];
    n->tx_pending_count = 0u;
    for (uint32_t i = 0u; i < VBUS_TX_DEDICATED; i++) {
        n->tx_ded_pending[i] = false;
    }
}

void vbus_set_bus_off(vbus_t* bus, uint8_t node, bool bus_off)
{
    if (!node_ok(bus, node)) {
        return;
    }
    vbus_node_t* n = &bus->nodes[node];
    if (bus_off && !n->bus_off) {
        n->bus_off_events++;
    }
    n->bus_off = bus_off;
}

void vbus_add_bus_off_events(vbus_t* bus, uint8_t node, uint32_t count)
{
    if (node_ok(bus, node)) {
        bus->nodes[node].bus_off_events += count; /* wraps like the hardware counter */
    }
}

void vbus_recover(vbus_t* bus, uint8_t node)
{
    if (!node_ok(bus, node)) {
        return;
    }
    vbus_node_t* n = &bus->nodes[node];
    n->recover_count++;
    if (n->bus_off) {
        n->bus_off = false;
        n->tec = 0u; /* the controller restarts with cleared error counters */
        n->rec = 0u;
        if (!n->tx_stalled) {
            flush_pending(bus, node);
        }
    }
}

uint32_t vbus_recover_count(const vbus_t* bus, uint8_t node)
{
    return node_ok(bus, node) ? bus->nodes[node].recover_count : 0u;
}

void vbus_set_error_counters(vbus_t* bus, uint8_t node, uint8_t tec, uint8_t rec)
{
    if (node_ok(bus, node)) {
        bus->nodes[node].tec = tec;
        bus->nodes[node].rec = rec;
    }
}

can_port_status_t vbus_set_filters(vbus_t* bus, uint8_t node, const can_port_filter_t* filters,
                                   uint32_t count)
{
    if (!node_ok(bus, node) || (count > CAN_PORT_MAX_FILTERS) ||
        ((filters == NULL) && (count > 0u))) {
        return CAN_PORT_ERR_ARG;
    }
    vbus_node_t* n = &bus->nodes[node];
    for (uint32_t i = 0u; i < count; i++) {
        n->filters[i] = filters[i];
    }
    n->filter_count = count;
    n->filtered = true;
    return CAN_PORT_OK;
}

can_port_status_t vbus_set_tx_dedicated(vbus_t* bus, uint8_t node, const can_port_tx_id_t* ids,
                                        uint32_t count)
{
    if (!node_ok(bus, node) || (count > VBUS_TX_DEDICATED) || ((ids == NULL) && (count > 0u))) {
        return CAN_PORT_ERR_ARG;
    }
    for (uint32_t i = 0u; i < count; i++) {
        if (!can_id_valid(ids[i].id, ids[i].extended)) {
            return CAN_PORT_ERR_ARG;
        }
        for (uint32_t j = 0u; j < i; j++) {
            if ((ids[j].id == ids[i].id) && (ids[j].extended == ids[i].extended)) {
                return CAN_PORT_ERR_ARG;
            }
        }
    }
    vbus_node_t* n = &bus->nodes[node];
    for (uint32_t i = 0u; i < VBUS_TX_DEDICATED; i++) {
        n->tx_ded_pending[i] = false;
        if (i < count) {
            n->tx_ded_ids[i] = ids[i];
        }
    }
    n->tx_ded_count = count;
    return CAN_PORT_OK;
}

uint32_t vbus_tx_queue_pending(const vbus_t* bus, uint8_t node)
{
    return node_ok(bus, node) ? bus->nodes[node].tx_pending_count : 0u;
}

bool vbus_tx_is_dedicated(const vbus_t* bus, uint8_t node, const can_frame_t* frame)
{
    return node_ok(bus, node) && (frame != NULL) &&
           (dedicated_slot(&bus->nodes[node], frame->id, frame->extended) < VBUS_TX_DEDICATED);
}

void vbus_set_tx_cancel_late(vbus_t* bus, uint8_t node, bool late)
{
    if (node_ok(bus, node)) {
        bus->nodes[node].tx_cancel_late = late;
    }
}

uint32_t vbus_tx_replaced(const vbus_t* bus, uint8_t node)
{
    return node_ok(bus, node) ? bus->nodes[node].tx_replaced : 0u;
}

can_port_status_t vbus_state(const vbus_t* bus, uint8_t node, can_port_state_t* state)
{
    if (!node_ok(bus, node) || (state == NULL)) {
        return CAN_PORT_ERR_ARG;
    }
    const vbus_node_t* n = &bus->nodes[node];
    state->tec = n->tec;
    state->rec = n->rec;
    state->error_passive = (n->tec > ERROR_PASSIVE_LIMIT) || (n->rec > ERROR_PASSIVE_LIMIT);
    state->bus_off = n->bus_off;
    state->bus_off_events = n->bus_off_events;
    state->tx_pending = n->tx_pending_count + dedicated_pending(n);
    state->tx_done = n->tx_done;
    return CAN_PORT_OK;
}

uint32_t vbus_overruns(const vbus_t* bus, uint8_t node)
{
    return node_ok(bus, node) ? bus->nodes[node].overruns : 0u;
}

uint32_t vbus_frame_count(const vbus_t* bus)
{
    return (bus != NULL) ? bus->frame_count : 0u;
}
