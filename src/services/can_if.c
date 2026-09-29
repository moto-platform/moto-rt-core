#include "services/can_if.h"

#include "hal/can_port.h"
#include "vehicle_cl250.h"

#include <stddef.h>

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
    }
}

can_if_status_t can_if_register_rx(can_port_id_t port, uint32_t id, bool extended,
                                   can_if_rx_fn fn, void* ctx)
{
    if (!port_ok(port) || (fn == NULL) || !can_id_valid(id, extended)) {
        return CAN_IF_ERR_ARG;
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
    return can_port_write(port, frame);
}

bool can_if_tx_free(can_port_id_t port)
{
    return can_port_tx_free(port);
}

uint32_t can_if_unrouted_count(can_port_id_t port)
{
    return port_ok(port) ? unrouted[port] : 0u;
}

uint32_t can_if_tx_refused_count(can_port_id_t port)
{
    return port_ok(port) ? tx_refused[port] : 0u;
}
