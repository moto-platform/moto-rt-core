#ifndef UDS_CLIENT_H
#define UDS_CLIENT_H

/*
 * UDS client glue (thesis deliverable Ç3): runs uds_client_core on the CL250 vehicle
 * ISO-TP link and stores every decoded DID in services/vehicle_signals. rt-core is the
 * single, read-only vehicle-bus tester (D-021, D-037).
 *
 * Per main-loop pass, after can_if_dispatch(CAN_PORT_VEHICLE, ...):
 *   uds_client_step(client)
 * It takes the link's indication, runs the core, hands at most one request to the
 * link and steps the link. Time comes from services/timebase.
 *
 * Fail-closed latch: the client sends nothing more until it is opened again when
 *   - the link refuses a request (D-020 request gate or Single Frame length): a bug,
 *     since the core builds its requests from gen/            -> UDS_CLIENT_FAULT_GATE
 *   - the can_if vehicle guard refuses any frame (counter up) -> UDS_CLIENT_FAULT_GUARD
 *   - a frame appears on the ECU request IDs (29-bit, or the 11-bit fallback): our own
 *     frames are never received back, so it is a second tester (D-021)
 *                                                             -> UDS_CLIENT_FAULT_FOREIGN_TESTER
 * Not a fault: a request the link cannot take because it is busy (dropped, retried on
 * schedule; the client waits for isotp_link_tx_ready() anyway), and the FC.CTS the link
 * drops for a segmented response (Q-020).
 * The values then go STALE in services/vehicle_signals; it expires them every step.
 *
 * Republishing the values on the platform bus is not part of this module: readers use
 * services/vehicle_signals.h.
 */

#include "features/uds/isotp_link.h"
#include "features/uds/uds_client_core.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Link buffers. Requests are at most UDS_CLIENT_REQ_MAX bytes. The receive buffer is
 * larger than a Single Frame, so a segmented response starts a reception that ends in
 * ISOTP_N_TIMEOUT_CR ("service unavailable") instead of being dropped silently. */
#define UDS_CLIENT_RX_BUF 64u
#define UDS_CLIENT_TX_BUF 8u

typedef enum {
    UDS_CLIENT_FAULT_NONE = 0,
    UDS_CLIENT_FAULT_GATE,           /* the link refused a request */
    UDS_CLIENT_FAULT_GUARD,          /* the can_if vehicle guard refused a frame */
    UDS_CLIENT_FAULT_FOREIGN_TESTER  /* a frame on the ECU request IDs: second tester */
} uds_client_fault_t;

/* Treat as opaque. Must have static storage duration (the link is a can_if receiver). */
typedef struct {
    isotp_can_link_t link;
    uint8_t rx_buf[UDS_CLIENT_RX_BUF];
    uint8_t tx_buf[UDS_CLIENT_TX_BUF];
    uds_client_core_t core;
    uint32_t guard_refused_seen;
    uint32_t foreign_seen;
    uds_client_fault_t fault;
    bool open;
} uds_client_t;

/* Opens the vehicle link (isotp_link_open_vehicle_cl250), registers the foreign-tester
 * watch on both ECU request IDs and resets the core. can_if_init() must have run.
 * Returns the link's status, or ISOTP_ERR_ARG if the watch cannot be registered (the
 * client then stays closed: it never polls without the watch). */
isotp_status_t uds_client_open(uds_client_t* client);

void uds_client_step(uds_client_t* client);

bool uds_client_session_up(const uds_client_t* client);
bool uds_client_ecu_present(const uds_client_t* client);

/* True once the client latched as failed (or when it is not open). */
bool uds_client_failed(const uds_client_t* client);

/* Why it latched (the first reason); UDS_CLIENT_FAULT_NONE while it runs. */
uds_client_fault_t uds_client_fault(const uds_client_t* client);

/* Counters of the core; NULL if client is NULL. */
const uds_client_stats_t* uds_client_stats(const uds_client_t* client);

#ifdef __cplusplus
}
#endif

#endif /* UDS_CLIENT_H */
