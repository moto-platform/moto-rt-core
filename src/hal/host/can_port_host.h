#ifndef HAL_HOST_CAN_PORT_HOST_H
#define HAL_HOST_CAN_PORT_HOST_H

/*
 * Host implementation of hal/can_port.h (D-034). Each logical port is bound to one
 * backend before use:
 *  - a node of an in-process virtual bus (hal/host/vbus.h), on every host OS
 *  - a Linux SocketCAN interface (vcan0, can0, ...), non-blocking raw socket;
 *    classic frames only, RTR/error/FD frames are skipped
 * An unbound port returns CAN_PORT_ERR_CLOSED.
 */

#include "hal/can_port.h"
#include "hal/host/vbus.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

bool can_port_host_bind_vbus(can_port_id_t port, vbus_t* bus, uint8_t node);

/* Linux only; returns false on other systems or if the interface cannot be opened. */
bool can_port_host_bind_socketcan(can_port_id_t port, const char* ifname);

/* True if this build has the SocketCAN backend. */
bool can_port_host_has_socketcan(void);

/* Unbinds the port (closes a SocketCAN socket). */
void can_port_host_unbind(can_port_id_t port);

/* Unbinds every port. */
void can_port_host_unbind_all(void);

#ifdef __cplusplus
}
#endif

#endif /* HAL_HOST_CAN_PORT_HOST_H */
