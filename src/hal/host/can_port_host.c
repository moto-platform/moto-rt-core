#include "hal/host/can_port_host.h"

#include <stddef.h>

#if defined(__linux__)
#include <errno.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#define HAVE_SOCKETCAN 1
#else
#define HAVE_SOCKETCAN 0
#endif

/* Non-data frames skipped per read before reporting CAN_PORT_EMPTY. */
#define SOCKETCAN_SKIP_MAX 8u

typedef enum { BACKEND_NONE = 0, BACKEND_VBUS, BACKEND_SOCKETCAN } backend_t;

typedef struct {
    backend_t backend;
    vbus_t* bus;
    uint8_t node;
    int fd;
    uint32_t tx_done; /* SocketCAN: frames the socket took (no ACK information) */
} port_binding_t;

static port_binding_t bindings[CAN_PORT_COUNT];

static port_binding_t* binding_of(can_port_id_t port)
{
    return ((uint32_t)port < (uint32_t)CAN_PORT_COUNT) ? &bindings[port] : NULL;
}

bool can_port_host_bind_vbus(can_port_id_t port, vbus_t* bus, uint8_t node)
{
    port_binding_t* b = binding_of(port);
    if ((b == NULL) || (bus == NULL) || (node >= VBUS_MAX_NODES) || !bus->nodes[node].attached) {
        return false;
    }
    can_port_host_unbind(port);
    b->backend = BACKEND_VBUS;
    b->bus = bus;
    b->node = node;
    return true;
}

bool can_port_host_has_socketcan(void)
{
    return HAVE_SOCKETCAN != 0;
}

#if HAVE_SOCKETCAN

bool can_port_host_bind_socketcan(can_port_id_t port, const char* ifname)
{
    port_binding_t* b = binding_of(port);
    if ((b == NULL) || (ifname == NULL)) {
        return false;
    }
    unsigned int ifindex = if_nametoindex(ifname);
    if (ifindex == 0u) {
        return false;
    }
    int fd = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);
    if (fd < 0) {
        return false;
    }
    struct sockaddr_can addr;
    memset(&addr, 0, sizeof addr);
    addr.can_family = AF_CAN;
    addr.can_ifindex = (int)ifindex;
    if (bind(fd, (const struct sockaddr*)&addr, sizeof addr) != 0) {
        (void)close(fd);
        return false;
    }
    can_port_host_unbind(port);
    b->backend = BACKEND_SOCKETCAN;
    b->fd = fd;
    b->tx_done = 0u;
    return true;
}

static can_port_status_t socketcan_write(int fd, const can_frame_t* frame)
{
    struct can_frame raw;
    memset(&raw, 0, sizeof raw);
    raw.can_id = frame->extended ? (frame->id | CAN_EFF_FLAG) : frame->id;
    raw.can_dlc = frame->dlc;
    memcpy(raw.data, frame->data, frame->dlc);
    ssize_t n = write(fd, &raw, sizeof raw);
    if (n == (ssize_t)sizeof raw) {
        return CAN_PORT_OK;
    }
    if ((n < 0) && ((errno == EAGAIN) || (errno == EWOULDBLOCK) || (errno == ENOBUFS))) {
        return CAN_PORT_TX_FULL;
    }
    return CAN_PORT_ERR_IO;
}

/* Exact (ID, format) filters; an empty list receives nothing. */
static can_port_status_t socketcan_set_filters(int fd, const can_port_filter_t* filters,
                                               uint32_t count)
{
    struct can_filter raw[CAN_PORT_MAX_FILTERS];
    for (uint32_t i = 0u; i < count; i++) {
        raw[i].can_id = filters[i].extended ? (filters[i].id | CAN_EFF_FLAG) : filters[i].id;
        raw[i].can_mask = CAN_EFF_FLAG | CAN_RTR_FLAG |
                          (filters[i].extended ? CAN_EFF_MASK : CAN_SFF_MASK);
    }
    const int rc = setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, (count > 0u) ? raw : NULL,
                              (socklen_t)(count * sizeof raw[0]));
    return (rc == 0) ? CAN_PORT_OK : CAN_PORT_ERR_IO;
}

static bool socketcan_tx_free(int fd)
{
    struct pollfd p = {.fd = fd, .events = POLLOUT, .revents = 0};
    return (poll(&p, 1u, 0) == 1) && ((p.revents & POLLOUT) != 0);
}

static can_port_status_t socketcan_read(int fd, can_frame_t* frame)
{
    for (uint8_t i = 0u; i < SOCKETCAN_SKIP_MAX; i++) {
        struct can_frame raw;
        ssize_t n = read(fd, &raw, sizeof raw);
        if (n < 0) {
            return ((errno == EAGAIN) || (errno == EWOULDBLOCK)) ? CAN_PORT_EMPTY
                                                                  : CAN_PORT_ERR_IO;
        }
        if ((n != (ssize_t)sizeof raw) || ((raw.can_id & (CAN_ERR_FLAG | CAN_RTR_FLAG)) != 0u) ||
            (raw.can_dlc > CAN_PORT_MAX_DLC)) {
            continue; /* FD, error or remote frame: not ours */
        }
        frame->extended = (raw.can_id & CAN_EFF_FLAG) != 0u;
        frame->id = raw.can_id & (frame->extended ? CAN_EFF_MASK : CAN_SFF_MASK);
        frame->dlc = raw.can_dlc;
        memset(frame->data, 0, sizeof frame->data);
        memcpy(frame->data, raw.data, raw.can_dlc);
        return CAN_PORT_OK;
    }
    return CAN_PORT_EMPTY;
}

#else /* !HAVE_SOCKETCAN */

bool can_port_host_bind_socketcan(can_port_id_t port, const char* ifname)
{
    (void)port;
    (void)ifname;
    return false;
}

#endif

void can_port_host_unbind(can_port_id_t port)
{
    port_binding_t* b = binding_of(port);
    if (b == NULL) {
        return;
    }
#if HAVE_SOCKETCAN
    if (b->backend == BACKEND_SOCKETCAN) {
        (void)close(b->fd);
    }
#endif
    b->backend = BACKEND_NONE;
    b->bus = NULL;
    b->node = 0u;
    b->fd = -1;
    b->tx_done = 0u;
}

void can_port_host_unbind_all(void)
{
    for (uint32_t p = 0u; p < (uint32_t)CAN_PORT_COUNT; p++) {
        can_port_host_unbind((can_port_id_t)p);
    }
}

can_port_status_t can_port_write(can_port_id_t port, const can_frame_t* frame)
{
    const port_binding_t* b = binding_of(port);
    if ((b == NULL) || !can_frame_valid(frame)) {
        return CAN_PORT_ERR_ARG;
    }
    switch (b->backend) {
    case BACKEND_VBUS:
        return vbus_send(b->bus, b->node, frame);
#if HAVE_SOCKETCAN
    case BACKEND_SOCKETCAN: {
        const can_port_status_t st = socketcan_write(b->fd, frame);
        if (st == CAN_PORT_OK) {
            bindings[port].tx_done++;
        }
        return st;
    }
#endif
    default:
        return CAN_PORT_ERR_CLOSED;
    }
}

bool can_port_tx_free(can_port_id_t port)
{
    const port_binding_t* b = binding_of(port);
    if (b == NULL) {
        return false;
    }
    switch (b->backend) {
    case BACKEND_VBUS: {
        /* The H7 vehicle port has one TX buffer (hal/can_port.h): free only when empty. */
        can_port_state_t st;
        const bool empty = (port != CAN_PORT_VEHICLE) ||
                           ((vbus_state(b->bus, b->node, &st) == CAN_PORT_OK) &&
                            (st.tx_pending == 0u));
        return empty && vbus_tx_free(b->bus, b->node);
    }
#if HAVE_SOCKETCAN
    case BACKEND_SOCKETCAN:
        return socketcan_tx_free(b->fd);
#endif
    default:
        return false;
    }
}

can_port_status_t can_port_read(can_port_id_t port, can_frame_t* frame)
{
    const port_binding_t* b = binding_of(port);
    if ((b == NULL) || (frame == NULL)) {
        return CAN_PORT_ERR_ARG;
    }
    switch (b->backend) {
    case BACKEND_VBUS:
        return vbus_recv(b->bus, b->node, frame);
#if HAVE_SOCKETCAN
    case BACKEND_SOCKETCAN:
        return socketcan_read(b->fd, frame);
#endif
    default:
        return CAN_PORT_ERR_CLOSED;
    }
}

can_port_status_t can_port_get_state(can_port_id_t port, can_port_state_t* state)
{
    const port_binding_t* b = binding_of(port);
    if ((b == NULL) || (state == NULL)) {
        return CAN_PORT_ERR_ARG;
    }
    switch (b->backend) {
    case BACKEND_VBUS:
        return vbus_state(b->bus, b->node, state);
#if HAVE_SOCKETCAN
    case BACKEND_SOCKETCAN:
        /* A raw socket shows no controller state: report a healthy, idle controller. */
        state->tec = 0u;
        state->rec = 0u;
        state->error_passive = false;
        state->bus_off = false;
        state->bus_off_events = 0u;
        state->tx_pending = 0u;
        state->tx_done = b->tx_done;
        return CAN_PORT_OK;
#endif
    default:
        return CAN_PORT_ERR_CLOSED;
    }
}

can_port_status_t can_port_recover(can_port_id_t port)
{
    const port_binding_t* b = binding_of(port);
    if (b == NULL) {
        return CAN_PORT_ERR_ARG;
    }
    switch (b->backend) {
    case BACKEND_VBUS:
        vbus_recover(b->bus, b->node);
        return CAN_PORT_OK;
#if HAVE_SOCKETCAN
    case BACKEND_SOCKETCAN:
        return CAN_PORT_OK; /* the kernel driver restarts the controller (restart-ms) */
#endif
    default:
        return CAN_PORT_ERR_CLOSED;
    }
}

can_port_status_t can_port_tx_abort(can_port_id_t port)
{
    const port_binding_t* b = binding_of(port);
    if (b == NULL) {
        return CAN_PORT_ERR_ARG;
    }
    switch (b->backend) {
    case BACKEND_VBUS:
        vbus_tx_abort(b->bus, b->node);
        return CAN_PORT_OK;
#if HAVE_SOCKETCAN
    case BACKEND_SOCKETCAN:
        return CAN_PORT_OK; /* nothing is held back: pending is always 0 */
#endif
    default:
        return CAN_PORT_ERR_CLOSED;
    }
}

can_port_status_t can_port_set_filters(can_port_id_t port, const can_port_filter_t* filters,
                                       uint32_t count)
{
    const port_binding_t* b = binding_of(port);
    if ((b == NULL) || (count > CAN_PORT_MAX_FILTERS) || ((filters == NULL) && (count > 0u))) {
        return CAN_PORT_ERR_ARG;
    }
    switch (b->backend) {
    case BACKEND_VBUS:
        return vbus_set_filters(b->bus, b->node, filters, count);
#if HAVE_SOCKETCAN
    case BACKEND_SOCKETCAN:
        return socketcan_set_filters(b->fd, filters, count);
#endif
    default:
        return CAN_PORT_ERR_CLOSED;
    }
}
