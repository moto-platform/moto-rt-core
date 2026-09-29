#include "features/uds/isotp_link.h"

#include "services/timebase.h"
#include "vehicle_cl250.h"

#include <stddef.h>

_Static_assert(ISOTP_CAN_DL == CAN_PORT_MAX_DLC, "ISO-TP and CAN frame sizes differ");

static void link_count(uint32_t* counter)
{
    if (*counter < UINT32_MAX) {
        (*counter)++;
    }
}

static void on_rx(void* ctx, const can_frame_t* frame)
{
    // cppcheck-suppress misra-c2012-11.5 ; DEV-003: ctx is the link this file registered
    isotp_can_link_t* link = (isotp_can_link_t*)ctx;
    if ((link == NULL) || !link->open || (frame == NULL)) {
        return;
    }
    isotp_frame_t f;
    f.dlc = frame->dlc;
    for (uint8_t i = 0u; i < ISOTP_CAN_DL; i++) {
        f.data[i] = frame->data[i];
    }
    isotp_on_frame(&link->iso, &f, timebase_now_ms());
}

static isotp_status_t open_link(isotp_can_link_t* link, const isotp_link_addr_t* addr,
                                const isotp_config_t* cfg, bool vehicle, uint8_t* rx_buf,
                                uint16_t rx_cap, uint8_t* tx_buf, uint16_t tx_cap)
{
    if (!can_id_valid(addr->tx_id, addr->extended) || !can_id_valid(addr->rx_id, addr->extended) ||
        (addr->tx_id == addr->rx_id)) {
        return ISOTP_ERR_ARG;
    }
    isotp_status_t st = isotp_init(&link->iso, cfg, rx_buf, rx_cap, tx_buf, tx_cap);
    if (st != ISOTP_OK) {
        return st;
    }
    link->addr = *addr;
    link->rx_buf = rx_buf;
    link->tx_error_count = 0u;
    link->tx_refused_count = 0u;
    link->vehicle = vehicle;
    if (can_if_register_rx(addr->port, addr->rx_id, addr->extended, on_rx, link) != CAN_IF_OK) {
        return ISOTP_ERR_ARG;
    }
    link->open = true;
    return ISOTP_OK;
}

isotp_status_t isotp_link_open(isotp_can_link_t* link, const isotp_link_addr_t* addr,
                               const isotp_config_t* cfg, uint8_t* rx_buf, uint16_t rx_cap,
                               uint8_t* tx_buf, uint16_t tx_cap)
{
    if ((link == NULL) || (addr == NULL) || (addr->port == CAN_PORT_VEHICLE)) {
        return ISOTP_ERR_ARG;
    }
    if (link->open) {
        return ISOTP_ERR_BUSY;
    }
    return open_link(link, addr, cfg, false, rx_buf, rx_cap, tx_buf, tx_cap);
}

isotp_status_t isotp_link_open_vehicle_cl250(isotp_can_link_t* link, uint8_t* rx_buf,
                                             uint16_t rx_cap, uint8_t* tx_buf, uint16_t tx_cap)
{
    if (link == NULL) {
        return ISOTP_ERR_ARG;
    }
    if (link->open) {
        return ISOTP_ERR_BUSY;
    }
    isotp_link_addr_t addr;
    addr.port = CAN_PORT_VEHICLE;
    addr.tx_id = VEHICLE_CL250_REQUEST_ID;
    addr.rx_id = VEHICLE_CL250_RESPONSE_ID;
    addr.extended = VEHICLE_CL250_REQUEST_ID > CAN_PORT_STD_ID_MAX;
    isotp_config_t cfg;
    isotp_default_config(&cfg);
    cfg.padding_enabled = (VEHICLE_CL250_FRAME_DLC == ISOTP_CAN_DL);
    cfg.padding_byte = VEHICLE_CL250_PADDING_BYTE;
    return open_link(link, &addr, &cfg, true, rx_buf, rx_cap, tx_buf, tx_cap);
}

isotp_status_t isotp_link_send(isotp_can_link_t* link, const uint8_t* data, uint16_t len)
{
    if ((link == NULL) || !link->open || (data == NULL)) {
        return ISOTP_ERR_ARG;
    }
    if (link->vehicle) {
        if (!vehicle_cl250_request_allowed(data, (size_t)len)) {
            link_count(&link->tx_refused_count);
            return ISOTP_ERR_ARG;
        }
        if (len > ISOTP_SF_MAX_LEN) {
            link_count(&link->tx_refused_count); /* would need a First Frame (D-020 gate) */
            return ISOTP_ERR_LENGTH;
        }
    }
    return isotp_send(&link->iso, data, len);
}

/* Writes one frame the core produced; the core counts it as sent either way. */
static void transmit(isotp_can_link_t* link, const isotp_frame_t* f)
{
    if (link->vehicle && !vehicle_cl250_frame_allowed(f->data, (size_t)f->dlc)) {
        link_count(&link->tx_refused_count); /* e.g. an FC: the peer times out */
        return;
    }
    can_frame_t out;
    out.id = link->addr.tx_id;
    out.extended = link->addr.extended;
    out.dlc = f->dlc;
    for (uint8_t i = 0u; i < ISOTP_CAN_DL; i++) {
        out.data[i] = f->data[i];
    }
    if (can_if_write(link->addr.port, &out) != CAN_PORT_OK) {
        link_count(&link->tx_error_count);
    }
}

void isotp_link_step(isotp_can_link_t* link)
{
    if ((link == NULL) || !link->open) {
        return;
    }
    const uint32_t now = timebase_now_ms();
    bool more = true;
    for (uint32_t n = 0u; more && (n < ISOTP_LINK_MAX_TX_PER_STEP); n++) {
        isotp_frame_t f;
        more = can_if_tx_free(link->addr.port) && isotp_poll(&link->iso, now, &f);
        if (more) {
            transmit(link, &f);
        }
    }
}

bool isotp_link_take_rx(isotp_can_link_t* link, isotp_n_result_t* result, uint16_t* length)
{
    if ((link == NULL) || !link->open) {
        return false;
    }
    return isotp_take_rx_indication(&link->iso, result, length);
}

const uint8_t* isotp_link_rx_data(const isotp_can_link_t* link)
{
    return ((link != NULL) && link->open) ? link->rx_buf : NULL;
}

void isotp_link_rx_release(isotp_can_link_t* link)
{
    if ((link != NULL) && link->open) {
        isotp_rx_release(&link->iso);
    }
}

bool isotp_link_take_tx_confirm(isotp_can_link_t* link, isotp_n_result_t* result)
{
    if ((link == NULL) || !link->open) {
        return false;
    }
    return isotp_take_tx_confirm(&link->iso, result);
}

uint32_t isotp_link_tx_error_count(const isotp_can_link_t* link)
{
    return (link != NULL) ? link->tx_error_count : 0u;
}

uint32_t isotp_link_tx_refused_count(const isotp_can_link_t* link)
{
    return (link != NULL) ? link->tx_refused_count : 0u;
}

uint16_t isotp_link_rx_error_count(const isotp_can_link_t* link)
{
    return (link != NULL) ? isotp_rx_error_count(&link->iso) : 0u;
}
