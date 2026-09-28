#ifndef ISOTP_LINK_H
#define ISOTP_LINK_H

/*
 * ISO-TP link glue (thesis deliverable Ç2): binds one isotp_core link to a CAN port
 * and a (TX ID, RX ID) pair through services/can_if, with time from services/timebase.
 * The same code runs on the host (SIL, mock bus) and on the H7 (D-034).
 *
 * Per main-loop pass:
 *   can_if_dispatch(port, n)      delivers received frames to the link (callback)
 *   isotp_link_step(link)          runs timers, transmits due frames while TX is free
 *   isotp_link_take_rx(...)        N_USData.indication; data via isotp_link_rx_data()
 *   isotp_link_take_tx_confirm()   N_USData.confirm
 *
 * Vehicle bus (D-020, safety rule 2): the only way to open a link on CAN_PORT_VEHICLE
 * is isotp_link_open_vehicle_cl250(); its addressing and padding come from gen/ and
 * cannot be passed in. Its requests must pass vehicle_cl250_request_allowed() and fit
 * in a Single Frame; its frames are checked with vehicle_cl250_frame_allowed() before
 * they are written. These are early rejects with proper errors. The hard control is
 * the fail-closed guard in services/can_if, which every vehicle frame passes as well.
 * The generated frame gate passes Single Frames only, so this link cannot send a Flow
 * Control and cannot receive segmented responses (see README).
 *
 * The link registers itself as a can_if receiver, so it must have static storage
 * duration. N_As/N_Ar need a TX-complete confirmation from the driver and are not
 * implemented yet: a frame accepted by can_if_write() counts as sent.
 */

#include "features/uds/isotp_core.h"
#include "services/can_if.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Frames transmitted per isotp_link_step() at most (bounds the loop; a real FDCAN
 * has 3 TX mailboxes, the host bus has no limit of its own). */
#define ISOTP_LINK_MAX_TX_PER_STEP 16u

typedef struct {
    can_port_id_t port;
    uint32_t tx_id;   /* ID of the frames we send (request ID for a client) */
    uint32_t rx_id;   /* ID of the frames we receive, including FC for our sends */
    bool extended;    /* 29-bit IDs (both) */
} isotp_link_addr_t;

/* Treat as opaque. */
typedef struct {
    isotp_link_addr_t addr;
    isotp_link_t iso;
    const uint8_t* rx_buf;
    uint32_t tx_error_count;
    uint32_t tx_refused_count;
    bool vehicle;     /* CL250 tester link: D-020 checks apply */
    bool open;
} isotp_can_link_t;

/*
 * Generic link on a port other than CAN_PORT_VEHICLE (ISOTP_ERR_ARG for the vehicle
 * port). Inits the core and registers the RX ID with can_if. ISOTP_ERR_ARG also covers
 * an RX ID that already has a receiver; ISOTP_ERR_BUSY a link that is already open.
 */
isotp_status_t isotp_link_open(isotp_can_link_t* link, const isotp_link_addr_t* addr,
                               const isotp_config_t* cfg, uint8_t* rx_buf, uint16_t rx_cap,
                               uint8_t* tx_buf, uint16_t tx_cap);

/*
 * The vehicle link to the CL250 engine ECU (D-019), everything from gen/vehicle_cl250.h:
 * vehicle port, VEHICLE_CL250_REQUEST_ID -> VEHICLE_CL250_RESPONSE_ID (29-bit), 8-byte
 * frames padded with VEHICLE_CL250_PADDING_BYTE, ISO default BS/STmin/N_Bs/N_Cr.
 */
isotp_status_t isotp_link_open_vehicle_cl250(isotp_can_link_t* link, uint8_t* rx_buf,
                                             uint16_t rx_cap, uint8_t* tx_buf, uint16_t tx_cap);

/*
 * N_USData.request. On the vehicle link: ISOTP_ERR_ARG if the request gate refuses the
 * service, ISOTP_ERR_LENGTH if it does not fit in a Single Frame (both counted as
 * refused, nothing is queued).
 */
isotp_status_t isotp_link_send(isotp_can_link_t* link, const uint8_t* data, uint16_t len);

void isotp_link_step(isotp_can_link_t* link);

bool isotp_link_take_rx(isotp_can_link_t* link, isotp_n_result_t* result, uint16_t* length);

/* The received message after an ISOTP_N_OK indication, valid until isotp_link_rx_release(). */
const uint8_t* isotp_link_rx_data(const isotp_can_link_t* link);

void isotp_link_rx_release(isotp_can_link_t* link);

bool isotp_link_take_tx_confirm(isotp_can_link_t* link, isotp_n_result_t* result);

/* Frames the core produced that the port refused or could not take (lost; the peer
 * then times out). Includes frames refused by the can_if vehicle guard. */
uint32_t isotp_link_tx_error_count(const isotp_can_link_t* link);

/* Vehicle link: requests and frames refused by the D-020 checks (never written). */
uint32_t isotp_link_tx_refused_count(const isotp_can_link_t* link);

uint16_t isotp_link_rx_error_count(const isotp_can_link_t* link);

#ifdef __cplusplus
}
#endif

#endif /* ISOTP_LINK_H */
