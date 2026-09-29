#ifndef HAL_CAN_TYPES_H
#define HAL_CAN_TYPES_H

/*
 * CAN frame and port types shared by hal/ and services/ (HAL-free). Features see these
 * types through services/can_if.h; the port functions themselves (hal/can_port.h) are
 * visible to services/ and the platform setup in app/ only.
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CAN_PORT_MAX_DLC 8u             /* classic CAN */
#define CAN_PORT_STD_ID_MAX 0x7FFu      /* 11-bit */
#define CAN_PORT_EXT_ID_MAX 0x1FFFFFFFu /* 29-bit */

/* Logical ports. The vehicle bus has rt-core as its only tester (D-021). */
typedef enum {
    CAN_PORT_VEHICLE = 0, /* FDCAN1: CL250 DLC, UDS client, 500 kbit/s (D-019) */
    CAN_PORT_PLATFORM,    /* FDCAN2: platform bus, UDS server, 500 kbit/s (D-009) */
    CAN_PORT_COUNT
} can_port_id_t;

typedef struct {
    uint32_t id;     /* 11-bit or 29-bit identifier, without flag bits */
    bool extended;   /* true: 29-bit identifier */
    uint8_t dlc;     /* 0..8 */
    uint8_t data[CAN_PORT_MAX_DLC];
} can_frame_t;

typedef enum {
    CAN_PORT_OK = 0,
    CAN_PORT_EMPTY,       /* read: no frame waiting */
    CAN_PORT_TX_FULL,     /* write: no free TX mailbox, try again later */
    CAN_PORT_ERR_ARG,     /* NULL, unknown port, DLC > 8 or ID out of range */
    CAN_PORT_ERR_CLOSED,  /* port not bound to a bus */
    CAN_PORT_ERR_IO,      /* driver/bus error */
    CAN_PORT_ERR_REFUSED  /* services/can_if: vehicle-bus guard (D-020) refused the frame */
} can_port_status_t;

/* True if the ID fits its format (11-bit or 29-bit). */
bool can_id_valid(uint32_t id, bool extended);

/* True if a frame is well formed: DLC <= 8 and the ID fits its format. */
bool can_frame_valid(const can_frame_t* frame);

#ifdef __cplusplus
}
#endif

#endif /* HAL_CAN_TYPES_H */
