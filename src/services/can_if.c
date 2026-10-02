#include "services/can_if.h"

#include "hal/can_port.h"
#include "platform_uds.h"
#include "services/can_sm.h"
#include "vehicle_cl250.h"

#include <stddef.h>

_Static_assert(CAN_IF_MAX_RECEIVERS <= CAN_PORT_MAX_FILTERS, "a port could need more filters");

#define SF_DL_MASK 0x0Fu /* ISO 15765-2 Single Frame data length (low nibble of PCI) */

typedef struct {
    bool used;
    can_port_id_t port;
    uint32_t id;
    bool extended;
    can_if_rx_fn fn;
    void* ctx;
} receiver_t;

static receiver_t receivers[CAN_IF_MAX_RECEIVERS];
static uint32_t unrouted[CAN_PORT_COUNT];
static uint32_t tx_refused[CAN_PORT_COUNT];
static uint32_t tx_blocked[CAN_PORT_COUNT];
static bool sealed[CAN_PORT_COUNT];
static can_port_tx_id_t tx_dedicated[CAN_IF_MAX_TX_DEDICATED]; /* CAN_PORT_PLATFORM only */
static uint32_t tx_dedicated_count;

static void inc_saturated(uint32_t* counter)
{
    if (*counter < UINT32_MAX) {
        (*counter)++;
    }
}

static bool port_ok(can_port_id_t port)
{
    return (uint32_t)port < (uint32_t)CAN_PORT_COUNT;
}

static const receiver_t* find(can_port_id_t port, uint32_t id, bool extended)
{
    for (uint32_t i = 0u; i < CAN_IF_MAX_RECEIVERS; i++) {
        const receiver_t* r = &receivers[i];
        if (r->used && (r->port == port) && (r->id == id) && (r->extended == extended)) {
            return r;
        }
    }
    return NULL;
}

void can_if_init(void)
{
    for (uint32_t i = 0u; i < CAN_IF_MAX_RECEIVERS; i++) {
        receivers[i].used = false;
        receivers[i].fn = NULL;
        receivers[i].ctx = NULL;
    }
    for (uint32_t p = 0u; p < (uint32_t)CAN_PORT_COUNT; p++) {
        unrouted[p] = 0u;
        tx_refused[p] = 0u;
        tx_blocked[p] = 0u;
        sealed[p] = false;
    }
    tx_dedicated_count = 0u;
    can_sm_init();
}

can_if_status_t can_if_register_rx(can_port_id_t port, uint32_t id, bool extended,
                                   can_if_rx_fn fn, void* ctx)
{
    if (!port_ok(port) || (fn == NULL) || !can_id_valid(id, extended)) {
        return CAN_IF_ERR_ARG;
    }
    if (sealed[port]) {
        return CAN_IF_ERR_SEALED; /* the hardware filters would drop it */
    }
    if (find(port, id, extended) != NULL) {
        return CAN_IF_ERR_DUP;
    }
    for (uint32_t i = 0u; i < CAN_IF_MAX_RECEIVERS; i++) {
        receiver_t* r = &receivers[i];
        if (!r->used) {
            r->used = true;
            r->port = port;
            r->id = id;
            r->extended = extended;
            r->fn = fn;
            r->ctx = ctx;
            return CAN_IF_OK;
        }
    }
    return CAN_IF_ERR_FULL;
}

can_if_status_t can_if_register_tx_dedicated(can_port_id_t port, uint32_t id, bool extended)
{
    if ((port != CAN_PORT_PLATFORM) || !can_id_valid(id, extended)) {
        return CAN_IF_ERR_ARG;
    }
    if (sealed[port]) {
        return CAN_IF_ERR_SEALED; /* the port's TX buffers are configured */
    }
    for (uint32_t i = 0u; i < tx_dedicated_count; i++) {
        if ((tx_dedicated[i].id == id) && (tx_dedicated[i].extended == extended)) {
            return CAN_IF_ERR_DUP;
        }
    }
    if (tx_dedicated_count >= CAN_IF_MAX_TX_DEDICATED) {
        return CAN_IF_ERR_FULL;
    }
    tx_dedicated[tx_dedicated_count].id = id;
    tx_dedicated[tx_dedicated_count].extended = extended;
    tx_dedicated_count++;
    return CAN_IF_OK;
}

uint32_t can_if_dispatch(can_port_id_t port, uint32_t max_frames)
{
    if (!port_ok(port)) {
        return 0u;
    }
    uint32_t n = 0u;
    while (n < max_frames) {
        can_frame_t frame;
        if (can_port_read(port, &frame) != CAN_PORT_OK) {
            break;
        }
        n++;
        const receiver_t* r = find(port, frame.id, frame.extended);
        if (r != NULL) {
            r->fn(r->ctx, &frame);
        } else {
            inc_saturated(&unrouted[port]);
        }
    }
    return n;
}

bool can_if_tx_allowed(can_port_id_t port, const can_frame_t* frame)
{
    /* The CL250 request ID decides the frame format of the tester (29-bit, D-019). */
    static const bool vehicle_tx_extended = (VEHICLE_CL250_REQUEST_ID > CAN_PORT_STD_ID_MAX);

    if (!port_ok(port) || !can_frame_valid(frame)) {
        return false;
    }
    if (port != CAN_PORT_VEHICLE) {
        return true;
    }
    if ((frame->extended != vehicle_tx_extended) || (frame->id != VEHICLE_CL250_REQUEST_ID) ||
        (frame->dlc != VEHICLE_CL250_FRAME_DLC) ||
        !vehicle_cl250_frame_allowed(frame->data, (size_t)frame->dlc)) {
        return false;
    }
    /* The gate accepted a Single Frame: every byte after its payload must be padding. */
    const uint8_t sf_len = (uint8_t)(frame->data[0] & SF_DL_MASK);
    bool padded = true;
    for (uint8_t i = (uint8_t)(sf_len + 1u); i < frame->dlc; i++) {
        padded = padded && (frame->data[i] == VEHICLE_CL250_PADDING_BYTE);
    }
    return padded;
}

can_port_status_t can_if_write(can_port_id_t port, const can_frame_t* frame)
{
    if (!port_ok(port) || !can_frame_valid(frame)) {
        return CAN_PORT_ERR_ARG;
    }
    if (!can_if_tx_allowed(port, frame)) {
        inc_saturated(&tx_refused[port]);
        return CAN_PORT_ERR_REFUSED;
    }
    if (!can_sm_tx_allowed(port)) {
        inc_saturated(&tx_blocked[port]); /* bus-off or latched: not the guard */
        return CAN_PORT_ERR_IO;
    }
    return can_port_write(port, frame);
}

bool can_if_tx_free(can_port_id_t port)
{
    return can_sm_tx_allowed(port) && can_port_tx_free(port);
}

uint32_t can_if_tx_abort_count(can_port_id_t port)
{
    return can_sm_abort_seq(port);
}

uint32_t can_if_tx_blocked_count(can_port_id_t port)
{
    return port_ok(port) ? tx_blocked[port] : 0u;
}

/* FDCAN2: the platform UDS IDs (gen/) go to FIFO1, away from the periodic traffic. */
static can_port_fifo_t fifo_of(can_port_id_t port, uint32_t id, bool extended)
{
    const bool diag = (port == CAN_PORT_PLATFORM) && !extended &&
                      ((id == PLATFORM_UDS_PHYS_REQUEST_ID) ||
                       (id == PLATFORM_UDS_FUNCTIONAL_REQUEST_ID) ||
                       (id == PLATFORM_UDS_PHYS_RESPONSE_ID));
    return diag ? CAN_PORT_FIFO1 : CAN_PORT_FIFO0;
}

can_port_status_t can_if_apply_filters(can_port_id_t port)
{
    if (!port_ok(port)) {
        return CAN_PORT_ERR_ARG;
    }
    const uint32_t dedicated = (port == CAN_PORT_PLATFORM) ? tx_dedicated_count : 0u;
    const can_port_status_t tx_st = can_port_set_tx_dedicated(port, tx_dedicated, dedicated);
    if (tx_st != CAN_PORT_OK) {
        return tx_st;
    }
    can_port_filter_t filters[CAN_PORT_MAX_FILTERS];
    uint32_t n = 0u;
    for (uint32_t i = 0u; i < CAN_IF_MAX_RECEIVERS; i++) {
        const receiver_t* r = &receivers[i];
        if (!r->used || (r->port != port)) {
            continue;
        }
        filters[n].id = r->id;
        filters[n].extended = r->extended;
        filters[n].fifo = fifo_of(port, r->id, r->extended);
        n++;
    }
    const can_port_status_t st = can_port_set_filters(port, filters, n);
    if (st == CAN_PORT_OK) {
        sealed[port] = true;
    }
    return st;
}

uint32_t can_if_unrouted_count(can_port_id_t port)
{
    return port_ok(port) ? unrouted[port] : 0u;
}

uint32_t can_if_tx_refused_count(can_port_id_t port)
{
    return port_ok(port) ? tx_refused[port] : 0u;
}
