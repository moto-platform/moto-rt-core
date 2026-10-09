#ifndef UDS_CLIENT_CORE_H
#define UDS_CLIENT_CORE_H

/*
 * UDS client core (thesis deliverable Ç3): the CL250 vehicle poller as a pure state
 * machine. rt-core is the single vehicle-bus tester and only reads (D-021, D-037).
 *
 * It only ever produces three requests, all built from gen/vehicle_cl250.h:
 *   - DiagnosticSessionControl, extended session (VEHICLE_CL250_SESSION_SID/_SUBFUNCTION)
 *   - TesterPresent, response suppressed (VEHICLE_CL250_TESTER_PRESENT_SID/_SUBFUNCTION)
 *   - ReadDataByIdentifier for a DID of vehicle_cl250_dids[]
 * Every one must still pass vehicle_cl250_request_allowed() (D-020) in the link and the
 * can_if guard; this module adds no path around them.
 *
 * Every time and limit comes from gen/ (VEHICLE_CL250_*). The one other duration, the
 * wait after an aborted segmented response, is the link's N_Bs, passed to init.
 *
 * Behaviour (README.md has the full list):
 *   - Session: 0x10 03 every SESSION_RETRY_INTERVAL_MS until the positive response;
 *     no DID is read before that. Lost on NRC 0x7E/0x7F or when the ECU goes absent,
 *     then re-established the same way.
 *   - Tester present: 0x3E 80 every TESTER_PRESENT_PERIOD_MS, whatever the session
 *     state (verified legacy behaviour). No response is expected, so it never holds
 *     the in-flight slot.
 *   - Reads: of the DIDs that are due (poll_period_ms since their last request, or
 *     since their last timeout) and not in skip cooldown, the lowest gen/ priority value
 *     first, then table order (D-043). One request in flight. No starvation while each
 *     request holds the slot for at most ASSUMED_ROUND_TRIP_MS (the ECU's answer plus
 *     one poll step). A DID whose last read timed out competes in the normal class and
 *     gets no NRC 0x78 extension until it answers or is skipped (D-050), so after its
 *     first failing attempt it holds the slot for at most RESPONSE_TIMEOUT_BASE_MS
 *     (plus N_Cr if a segmented reception starts). A read answered more than its
 *     poll_period_ms after its sample stamp makes the DID faulty too, until an answer
 *     comes within the period or the DID is skipped (D-051, ISSUES E-7).
 *   - Sample stamp (D-051): the send time of the DID's oldest read since its last
 *     answer: the read that just ended, or the first read that timed out before it. A
 *     late answer to a timed-out read can be taken for the next read of the same DID;
 *     this stamp never makes the sample look younger than it is.
 *   - Response timeout RESPONSE_TIMEOUT_BASE_MS; NRC 0x78 for the pending SID restarts
 *     it doubled (not for a D-050 faulty read), up to RESPONSE_TIMEOUT_MAX_MS; a
 *     request never waits longer than RESPONSE_TIMEOUT_MAX_MS in total. The base
 *     timeout pauses while a segmented
 *     reception runs (it ends in N_Cr at the latest); the total cap does not.
 *   - MAX_CONSECUTIVE_TIMEOUTS timeouts in a row skip a DID for DID_SKIP_COOLDOWN_MS.
 *     Only an answer in time resets the count: a slow answer between two timeouts does
 *     not (D-052), so an ECU alternating them is skipped too.
 *   - Any other NRC for the pending SID ends the request (legacy behaviour): not a
 *     timeout, the DID keeps its schedule.
 *   - A failed reception (e.g. ISOTP_N_TIMEOUT_CR or a wrong sequence number) means
 *     "service unavailable": the DID goes into skip cooldown at once, and nothing is sent
 *     for the link's N_Bs, so the ECU has given up its segmented send first.
 *   - A complete segmented answer (longer than a Single Frame, D-059) to the pending
 *     read: "service unavailable" (skip cooldown), not a timeout, no hold.
 *   - A First Frame the link did not answer with a Flow Control
 *     (uds_client_core_on_fc_withheld(), Q-020 / D-059): the pending request ends
 *     without a timeout (a read is "service unavailable" as above), and nothing is sent
 *     for RESPONSE_TIMEOUT_MAX_MS, while the ECU waits for the Flow Control. A running
 *     hold is never shortened.
 *   - ECU present while any response came within ECU_ABSENT_TIMEOUT_MS. Absence also
 *     ends every DID's D-050/D-051 fault state: no read from before it is answered.
 *   - No request is produced while the link cannot take one at once (tx_ready false,
 *     e.g. no node ACKs and the mailbox stays full); the timers keep running.
 *   - Step gaps (D-053): every DID's poll_period_ms is at least RESPONSE_TIMEOUT_BASE_MS
 *     + CLIENT_STEP_MAX_MS (defs codegen), so an answer seen at the next step is in time
 *     and its DID is not due yet. That holds while two polls are at most
 *     CLIENT_STEP_MAX_MS apart; every larger gap between two polls is counted
 *     (stats.step_overruns), and the longest one is kept (stats.step_gap_max_ms). A
 *     step is every poll, also with rx_busy or while latched (the defs yaml definition;
 *     D-053's "rx_busy false" adds nothing under Q-020, see README). The core does not
 *     act on them.
 *   - A request the link could not take is dropped (uds_client_core_not_sent()); a read
 *     is due again at once. Only
 *     uds_client_core_latch() stops the core for good (a D-020 check refused a request,
 *     or a second tester was seen): it sends nothing more until init.
 *
 * Pure logic: no HAL, services or RTOS; time is passed in (ms, wrap-safe); static
 * storage, no heap, bounded loops.
 */

#include "features/uds/isotp_core.h"
#include "vehicle_cl250.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Longest request the core produces: ReadDataByIdentifier, SID + 16-bit DID. */
#define UDS_CLIENT_REQ_MAX 3u

typedef enum {
    UDS_CLIENT_REQ_NONE = 0,
    UDS_CLIENT_REQ_SESSION,
    UDS_CLIENT_REQ_READ
} uds_client_req_kind_t;

/* The request the last poll produced, for uds_client_core_not_sent() (internal). */
typedef enum {
    UDS_CLIENT_LAST_NONE = 0,
    UDS_CLIENT_LAST_SESSION,
    UDS_CLIENT_LAST_TESTER_PRESENT,
    UDS_CLIENT_LAST_READ
} uds_client_last_t;

typedef struct {
    uint32_t last_request_ms; /* start of the poll period: last request, or its timeout */
    uint32_t skip_start_ms;
    uint32_t unanswered_ms; /* send time of the first timed-out read since the last answer */
    uint8_t consecutive_timeouts; /* timeouts since the last answer in time: the skip count */
    bool requested;   /* last_request_ms is set */
    bool skipped;     /* in skip cooldown since skip_start_ms */
    bool slow;        /* last answer came more than poll_period_ms after its stamp (D-051) */
    bool unanswered;  /* a timed-out read is unanswered: unanswered_ms is the stamp (D-051) */
} uds_client_did_state_t;

/* Diagnostic counters (saturating). */
typedef struct {
    uint32_t requests;          /* requests the link took, all kinds (not a not_sent one) */
    uint32_t reads_ok;          /* DID samples produced */
    uint32_t timeouts;          /* requests that ended without an answer */
    uint32_t nrc;               /* negative responses, any SID and code */
    uint32_t response_pending;  /* NRC 0x78 for the pending SID (not extended if faulty, D-050) */
    uint32_t unavailable;       /* failed receptions (e.g. ISOTP_N_TIMEOUT_CR) */
    uint32_t did_skips;         /* DIDs put into skip cooldown */
    uint32_t unexpected;        /* responses that matched no pending request */
    uint32_t fc_withheld;       /* First Frames the link did not answer with an FC */
    uint32_t slow_answers;      /* answers later than poll_period_ms after their stamp (D-051) */
    uint32_t session_starts;    /* positive session responses */
    uint32_t session_losses;    /* session up -> down */
    uint32_t step_overruns;     /* poll gaps above VEHICLE_CL250_CLIENT_STEP_MAX_MS (D-053) */
    uint32_t step_gap_max_ms;   /* longest gap between two polls since init (D-029 bench) */
} uds_client_stats_t;

/* One decoded DID read, for the caller to store (services/vehicle_signals). */
typedef struct {
    uint32_t idx;       /* index into vehicle_cl250_dids[] */
    uint32_t raw;
    float physical;
    uint32_t stamp_ms;  /* the ECU took the sample no earlier than this ("Sample stamp") */
} uds_client_sample_t;

/* State. Treat as opaque: use the functions below. */
typedef struct {
    uds_client_did_state_t did[VEHICLE_CL250_DID_COUNT];
    uds_client_stats_t stats;
    uint32_t abort_hold_ms;

    bool session_up;
    bool session_tried;
    uint32_t session_tried_ms;
    bool tp_sent;
    uint32_t tp_sent_ms;

    uds_client_req_kind_t pending;
    uint8_t pending_sid;
    uint32_t pending_idx;
    uds_client_last_t last;   /* produced by the last poll, until not_sent or the next poll */
    uint32_t prev_request_ms; /* that request's schedule before it, for not_sent */
    bool prev_requested;
    uint32_t sent_ms;         /* start of the total cap */
    uint32_t wait_start_ms;   /* start of the current response timeout */
    uint32_t wait_ms;         /* current response timeout */

    bool hold;
    uint32_t hold_start_ms;
    uint32_t hold_ms;

    bool ecu_seen;
    uint32_t last_response_ms;

    uint32_t last_poll_ms;    /* the step gap check (D-053) */
    bool polled;              /* last_poll_ms is set */

    bool failed;
} uds_client_core_t;

/* Resets everything; abort_hold_ms is the link's N_Bs (isotp_link_n_bs_ms()). */
void uds_client_core_init(uds_client_core_t* c, uint32_t abort_hold_ms);

/*
 * Runs the timers and returns the next request, if one is due: *len bytes in out (room
 * for UDS_CLIENT_REQ_MAX). Call it once per step of the tester, after the step's
 * indication: the gap to the previous call is the step gap of D-053. The request counts
 * as sent; if the link does not take it, call uds_client_core_not_sent() or
 * uds_client_core_latch().
 *   rx_busy:  a segmented reception is running on the link (isotp_link_rx_busy());
 *             nothing is sent while it runs.
 *   tx_ready: the link can send at once (isotp_link_tx_ready()); nothing is sent
 *             while it cannot.
 */
bool uds_client_core_poll(uds_client_core_t* c, uint32_t now_ms, bool rx_busy, bool tx_ready,
                          uint8_t* out, uint16_t* len);

/* The request from the last poll did not go out (link busy): it is dropped without a
 * timeout and not counted, and its schedule is restored (a read's DID, the session
 * retry or tester present), so it is due again at once. Without a request from the
 * last poll it does nothing. */
void uds_client_core_not_sent(uds_client_core_t* c);

/* Stops the core for good (until init): no request, session down, slot free. */
void uds_client_core_latch(uds_client_core_t* c);

/* The link withheld the Flow Control for a First Frame (isotp_link_take_fc_withheld()):
 * no answer comes. Ends the pending request without a timeout (a read: skip cooldown,
 * counted unavailable) and sends nothing for RESPONSE_TIMEOUT_MAX_MS. */
void uds_client_core_on_fc_withheld(uds_client_core_t* c, uint32_t now_ms);

/*
 * N_USData.indication of the link. data/len are the message for ISOTP_N_OK (ignored
 * otherwise). Returns true and fills *sample when the message is the positive answer
 * to the pending read and decodes (gen/ vehicle_cl250_parse_response()).
 */
bool uds_client_core_on_indication(uds_client_core_t* c, uint32_t now_ms,
                                   isotp_n_result_t result, const uint8_t* data, uint16_t len,
                                   uds_client_sample_t* sample);

bool uds_client_core_session_up(const uds_client_core_t* c);
bool uds_client_core_ecu_present(const uds_client_core_t* c, uint32_t now_ms);
bool uds_client_core_failed(const uds_client_core_t* c);

/* True while DID index idx is in skip cooldown (false for an index out of range). */
bool uds_client_core_did_skipped(const uds_client_core_t* c, uint32_t idx, uint32_t now_ms);

/* Pending request kind (UDS_CLIENT_REQ_NONE when the slot is free). */
uds_client_req_kind_t uds_client_core_pending(const uds_client_core_t* c);

/* The positive response SID of the pending request (its SID + 0x40); 0 when none. */
uint8_t uds_client_core_pending_response_sid(const uds_client_core_t* c);

const uds_client_stats_t* uds_client_core_stats(const uds_client_core_t* c);

#ifdef __cplusplus
}
#endif

#endif /* UDS_CLIENT_CORE_H */
