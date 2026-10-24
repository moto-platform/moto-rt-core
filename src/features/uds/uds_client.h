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
 *   - a frame appears on the ECU request IDs (29-bit, or the 11-bit fallback) or on the
 *     OBD functional request IDs (gen/ functional watch, Q-021/D-040): our own frames
 *     are never received back, so it is a second tester (D-021)
 *                                                             -> UDS_CLIENT_FAULT_FOREIGN_TESTER
 * D-080: after the DID reads, a 0x19 ReadDTCInformation run (count, list, 0x04/0x06 per
 * DTC) when a DTC is present, at key-on and at warm idle; the warm-idle condition is
 * evaluated here from services/vehicle_signals every step.
 * Not a fault: a request the link cannot take because it is busy (dropped, retried on
 * schedule; the client waits for isotp_link_tx_ready() anyway), and a Flow Control the
 * link withholds (a First Frame it may not answer, D-059).
 * The values then go STALE in services/vehicle_signals; it expires them every step.
 *
 * Diagnostics (D-040): every step reports the DTCs VEHICLE_ECU_COMM_LOST and
 * VEHICLE_TESTER_LATCHED (level-triggered) and the 0xFD00 status to services/diag, which
 * the platform UDS server reads. The server never includes this header.
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

/* Link buffers. Requests are at most UDS_CLIENT_REQ_MAX bytes. The receive buffer takes
 * the longest answer the tester ever asks for: a 0x19 answer of up to
 * VEHICLE_CL250_MAX_FF_DL bytes (D-080; at most 36 CFs). A First Frame above what the
 * pending request allows gets no Flow Control (withheld). */
#define UDS_CLIENT_RX_BUF VEHICLE_CL250_MAX_FF_DL
/* A session or DID read gets the Flow Control only for FF_DL up to this many bytes. This is
 * the D-059 client policy for table-DID reads (the old receive buffer size; no table DID
 * needs a segmented answer, so more is "service unavailable"), not a vehicle fact. */
#define UDS_CLIENT_READ_MAX_FF_DL 64u
#define UDS_CLIENT_TX_BUF 8u

typedef enum {
    UDS_CLIENT_FAULT_NONE = 0,
    UDS_CLIENT_FAULT_GATE,           /* the link refused a request */
    UDS_CLIENT_FAULT_GUARD,          /* the can_if vehicle guard refused a frame */
    UDS_CLIENT_FAULT_FOREIGN_TESTER  /* a frame on the ECU request IDs: second tester */
} uds_client_fault_t;

/*
 * D-080 item 5 / D-069 item 6: every positive 0x19 answer goes to the sink raw, with what
 * it answers, before the link's receive buffer is released. Contract: called from the
 * comms task inside uds_client_step(); it must not block, must not call back into the
 * client, and must copy the bytes (data is valid only during the call); len <=
 * VEHICLE_CL250_MAX_FF_DL. The record layouts are never decoded here.
 * H7 port: the sink runs inside the comms pass, whose step gaps D-053 watches
 * (stats.step_overruns), so it must be a bounded copy into a static ring that a lower-priority
 * task drains (flash or UART writes never run here). TODO: replaced by the services/ logging
 * manager when it exists. Without a sink the answers are not logged.
 */
typedef void (*uds_client_dtc_sink_t)(void* ctx, const uds_client_dtc_answer_t* info,
                                      const uint8_t* data, uint16_t len);

/* Treat as opaque. Must have static storage duration (the link is a can_if receiver). */
typedef struct {
    isotp_can_link_t link;
    uint8_t rx_buf[UDS_CLIENT_RX_BUF];
    uint8_t tx_buf[UDS_CLIENT_TX_BUF];
    uds_client_core_t core;
    uint32_t guard_refused_seen;
    uint32_t foreign_seen;
    uds_client_fault_t fault;
    uds_client_dtc_sink_t dtc_sink;
    void* dtc_sink_ctx;
    uint32_t open_ms;
    bool absence_armed; /* ECU_ABSENT_TIMEOUT_MS since open: absence is a DTC */
    bool open;
} uds_client_t;

/* Opens the vehicle link (isotp_link_open_vehicle_cl250), registers the foreign-tester
 * watch on both ECU request IDs and the functional watch IDs, and resets the core. can_if_init() must have run.
 * Returns the link's status, or ISOTP_ERR_ARG if the watch cannot be registered (the
 * client then stays closed: it never polls without the watch). */
isotp_status_t uds_client_open(uds_client_t* client);

/* Sets (or, with NULL, removes) the D-080 sink; see uds_client_dtc_sink_t. */
void uds_client_set_dtc_sink(uds_client_t* client, uds_client_dtc_sink_t sink, void* ctx);

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
