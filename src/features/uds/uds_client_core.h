#ifndef UDS_CLIENT_CORE_H
#define UDS_CLIENT_CORE_H

/*
 * UDS client core (thesis deliverable Ç3): the CL250 vehicle poller as a pure state
 * machine. rt-core is the single vehicle-bus tester and only reads (D-021, D-037).
 *
 * It only ever produces four requests, all built from gen/vehicle_cl250.h or gen/
 * uds_iso14229.h:
 *   - DiagnosticSessionControl, extended session (VEHICLE_CL250_SESSION_SID/_SUBFUNCTION)
 *   - TesterPresent, response suppressed (VEHICLE_CL250_TESTER_PRESENT_SID/_SUBFUNCTION)
 *   - ReadDataByIdentifier for a DID of vehicle_cl250_dids[]
 *   - ReadDTCInformation 0x19, only the four gen/ sub-functions (D-080, dtc_snapshot_core)
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
 *     it doubled (not for a D-050 faulty read, nor for an unverified DID's read: D-068
 *     item 1), up to RESPONSE_TIMEOUT_MAX_MS; a
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
 *   - D-080: if the total cap fires while a 0x19 answer is still being received, the
 *     reception is cancelled (uds_client_core_take_rx_cancel()), 0x19 is off for the epoch
 *     and nothing is sent for the link's N_Bs.
 *   - DTC freeze frames (D-080, features/uds/dtc_snapshot_core): a 0x19 request goes out
 *     only when the session is up, nothing is pending or held, the link is free and no
 *     DID at all is due (verified or not), so a DID is never delayed by more than one 0x19
 *     slot. The wait is RESPONSE_TIMEOUT_BASE_MS, never extended by NRC 0x78. The run
 *     starts with the stop epoch (the confirmation that arms the unverified DIDs); a
 *     confirmation after NRC 0x7E/0x7F stays in the epoch. Its raw positive answers are
 *     handed out once by uds_client_core_take_dtc_answer() (D-069 item 6). A timeout, a
 *     failed, refused or withheld segmented reception and a transaction longer than
 *     RESPONSE_TIMEOUT_BASE_MS + CLIENT_STEP_MAX_MS turn 0x19 off for the epoch; no DID
 *     rule (timeout count, skip, quiet window) applies to it.
 *   - Unverified DIDs (D-068 item 2; vehicle_cl250_dids[idx].verified == false): each
 *     stop epoch arms them all (due at once). Until the epoch ends, one
 *     without a positive answer in this epoch is stopped (never due, no skip) after NRC
 *     0x31 on two separate reads (not one in the quiet window after a read timeout),
 *     MAX_CONSECUTIVE_TIMEOUTS timeouts in a row, or a First Frame answer. A decoded answer
 *     marks it answered: from then on it, like every verified DID, keeps the skip/retry
 *     above. Other NRCs never stop a DID (D-075 item 1 (f): an answered unverified
 *     DID is stopped by two NRC 0x7E/0x7F reads, counted apart). NRC 0x7E/0x7F drop the
 *     session and, on a read of a never-answered unverified DID, count toward its stop
 *     like NRC 0x31 (D-075 (provisional) item 1).
 *   - Stop epoch (D-075 (provisional) item 1): the stop state and the engine-start flag
 *     belong to an epoch. It starts only at the tester's start and at the confirmation
 *     after the ECU was lost (ECU_ABSENT_TIMEOUT_MS); that confirmation arms every
 *     unverified DID (due at once). A confirmation after NRC 0x7E/0x7F stays in the
 *     epoch: stops, answered flags and the engine-start flag are kept, and no DID is
 *     made due (each keeps its last request time and is due at its tier rate). A counted
 *     NRC 0x7E/0x7F (D-075 item 1 (amended, provisional)) requests the session at the next
 *     step; every other session loss waits SESSION_RETRY_INTERVAL_MS. The engine-start
 *     zero belongs to the epoch; the samples above 0 must be consecutive after a
 *     re-confirmation.
 *   - Engine-start re-ask (D-069 item 3): once per stop epoch, at the first engine
 *     start (two consecutive in-time ENGINE_SPEED samples > 0 after one == 0), every
 *     stopped DID is pollable again, due one poll_period_ms after its last request, and
 *     the NRC counts of every unverified DID start from zero (its timeout count is kept).
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

#include "features/uds/dtc_snapshot_core.h"
#include "features/uds/isotp_core.h"
#include "vehicle_cl250.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Longest request the core produces: ReadDTCInformation 0x04/0x06, SID + sub-function +
 * 3 DTC bytes + record (D-080). A ReadDataByIdentifier is SID + 16-bit DID. */
#define UDS_CLIENT_REQ_MAX DTC_SNAPSHOT_REQ_MAX

typedef enum {
    UDS_CLIENT_REQ_NONE = 0,
    UDS_CLIENT_REQ_SESSION,
    UDS_CLIENT_REQ_READ,
    UDS_CLIENT_REQ_DTC
} uds_client_req_kind_t;

/* The request the last poll produced, for uds_client_core_not_sent() (internal). */
typedef enum {
    UDS_CLIENT_LAST_NONE = 0,
    UDS_CLIENT_LAST_SESSION,
    UDS_CLIENT_LAST_TESTER_PRESENT,
    UDS_CLIENT_LAST_READ,
    UDS_CLIENT_LAST_DTC
} uds_client_last_t;

/* Why an unverified DID is stopped (D-068 item 2, D-075), for the field and the bench. */
typedef enum {
    UDS_CLIENT_STOP_NONE = 0,
    UDS_CLIENT_STOP_NRC_31,      /* NRC 0x31 on two separate reads, or a 0x31 and a 0x7E/0x7F */
    UDS_CLIENT_STOP_NRC_SESSION, /* NRC 0x7E/0x7F completed the count (D-075 item 1) */
    UDS_CLIENT_STOP_TIMEOUTS,    /* MAX_CONSECUTIVE_TIMEOUTS timeouts in a row */
    UDS_CLIENT_STOP_FIRST_FRAME  /* a First Frame answer, a withheld FC or a failed reception */
} uds_client_stop_reason_t;

typedef struct {
    uint32_t last_request_ms; /* start of the poll period: last request, or its timeout */
    uint32_t skip_start_ms;
    uint32_t unanswered_ms; /* send time of the first timed-out read since the last answer */
    /* Placed after the three uint32_t fields so the state is 24 B whether the compiler
     * makes enums 1 B (arm-none-eabi-gcc short enums) or 4 B (host); asserted below. */
    uds_client_stop_reason_t stop_reason; /* NONE: not stopped (D-068 item 2, never due) */
    uint8_t consecutive_timeouts; /* timeouts since the last answer in time: the skip count */
    uint8_t stop_nrc_count; /* NRC 0x31/0x7E/0x7F reads of a never-answered unverified DID in
                               this epoch since the re-ask; a positive answer clears it
                               (D-068, D-075) */
    uint8_t session_nrc_count; /* NRC 0x7E/0x7F reads of an unverified DID in this epoch since
                                  the re-ask; a positive answer does not clear it (D-075
                                  item 1 (f)) */
    bool requested;   /* last_request_ms is set */
    bool skipped;     /* in skip cooldown since skip_start_ms */
    bool slow;        /* last answer came more than poll_period_ms after its stamp (D-051) */
    bool unanswered;  /* a timed-out read is unanswered: unanswered_ms is the stamp (D-051) */
    bool answered;    /* a positive answer in this epoch: the stop rule no longer applies */
} uds_client_did_state_t;

_Static_assert(sizeof(uds_client_did_state_t) <= 24u,
               "the per-DID state must stay 24 B with 1 B and with 4 B enums");

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
    uint32_t did_stops;         /* unverified DIDs stopped (D-068 item 2) */
    uint32_t rearms;            /* stopped DIDs made pollable again at an engine start (D-069) */
} uds_client_stats_t;

/* One decoded DID read, for the caller to store (services/vehicle_signals). */
typedef struct {
    uint32_t idx;       /* index into vehicle_cl250_dids[] */
    uint32_t raw;
    float physical;
    uint32_t stamp_ms;  /* the ECU took the sample no earlier than this ("Sample stamp") */
} uds_client_sample_t;

/* What a positive 0x19 answer was an answer to (D-080 item 5), for the log header. */
typedef struct {
    dtc_snapshot_step_t step;
    uint8_t sub_function;  /* gen/ UDS_READ_DTC_REPORT_* */
    uint8_t dtc[DTC_SNAPSHOT_DTC_BYTES]; /* 0x04/0x06: the DTC of the request, else zeros */
    dtc_snapshot_trigger_t trigger;
    uint8_t run;           /* run number in the epoch, from 1 */
} uds_client_dtc_answer_t;

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

    bool timeout_seen;        /* a read timed out at last_timeout_ms: its quiet window (D-068) */
    uint32_t last_timeout_ms;
    bool pending_quiet;       /* the pending read was sent inside that quiet window */
    bool rpm_zero_seen;       /* D-069: a valid ENGINE_SPEED == 0 in this epoch (kept across a 0x7E/0x7F) */
    uint8_t rpm_run;          /* consecutive valid ENGINE_SPEED samples > 0 after that zero */
    bool engine_rearmed;      /* the engine-start re-ask happened in this stop epoch */
    bool epoch_open;          /* D-075 item 1: a stop epoch is running (a confirmation armed it) */
    uint16_t epoch_counter;   /* D-087 item 2: bumped where an epoch opens; wraps */

    uint32_t last_poll_ms;    /* the step gap check (D-053) */
    bool polled;              /* last_poll_ms is set */

    dtc_snapshot_core_t dtc;  /* D-080 */
    uds_client_dtc_answer_t dtc_answer;
    bool dtc_answer_ready;    /* a positive 0x19 answer waits for take_dtc_answer() */
    bool dtc_slow;            /* ... and it came later than one slot: halt SLOW after it */
    uint32_t rx_cancel_ms;    /* when that cancel happened: the hold starts there */
    bool rx_cancel_due;       /* the glue must cancel the running reception (0x19 at the cap) */

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
 * counted unavailable) and sends nothing for RESPONSE_TIMEOUT_MAX_MS. first_byte is the
 * First Frame's first data byte (isotp_link_fc_withheld_first_byte(), 0 if unknown): the
 * positive 0x19 SID with no 0x19 pending is a late or duplicate 0x19 answer; it is nobody's
 * answer to a pending read (no consequence for the DID) and turns 0x19 off for the epoch. */
void uds_client_core_on_fc_withheld(uds_client_core_t* c, uint32_t now_ms, uint8_t first_byte);

/*
 * D-080: true once after the total cap fired during the reception of a 0x19 answer: the glue
 * must cancel the link's reception (isotp_link_rx_cancel()). The core has recorded a
 * segmented failure (0x19 off for the epoch) and holds for the link's N_Bs.
 */
bool uds_client_core_take_rx_cancel(uds_client_core_t* c);

/*
 * After take_rx_cancel(): lengthens the hold that started at the cancel to ms (a longer hold
 * is kept, a shorter never replaces it). The ECU does not stop at the cancel: with BS 0 it
 * sends its remaining CFs, each up to N_Cr apart, so the glue passes the time they need.
 */
void uds_client_core_extend_hold(uds_client_core_t* c, uint32_t ms);

/*
 * N_USData.indication of the link. data/len are the message for ISOTP_N_OK (ignored
 * otherwise). Returns true and fills *sample when the message is the positive answer
 * to the pending read and decodes (gen/ vehicle_cl250_parse_response()).
 */
bool uds_client_core_on_indication(uds_client_core_t* c, uint32_t now_ms,
                                   isotp_n_result_t result, const uint8_t* data, uint16_t len,
                                   uds_client_sample_t* sample);

/*
 * D-080: the positive 0x19 answer the last indication completed, once (false when none).
 * Call it right after uds_client_core_on_indication(), before the link's buffer is
 * released: the caller logs the raw bytes. A slow answer (longer than RESPONSE_TIMEOUT_BASE_MS
 * + CLIENT_STEP_MAX_MS) turns 0x19 off for the epoch when it is taken (or at the next
 * poll), so the answer is always handed out first.
 */
bool uds_client_core_take_dtc_answer(uds_client_core_t* c, uds_client_dtc_answer_t* info);

/*
 * D-080 item 1, once per step: holds = every warm-idle condition is met by a valid
 * sample (the caller evaluates vehicle_cl250_dtc_warm_idle[]). See
 * dtc_snapshot_core_warm_idle_step().
 */
bool uds_client_core_dtc_warm_idle(uds_client_core_t* c, uint32_t now_ms, bool holds);

/* The DTC run state, for the glue's Flow Control check and for diagnostics. */
const dtc_snapshot_core_t* uds_client_core_dtc(const uds_client_core_t* c);

bool uds_client_core_session_up(const uds_client_core_t* c);
bool uds_client_core_ecu_present(const uds_client_core_t* c, uint32_t now_ms);
bool uds_client_core_failed(const uds_client_core_t* c);

/* True while DID index idx is in skip cooldown (false for an index out of range). */
bool uds_client_core_did_skipped(const uds_client_core_t* c, uint32_t idx, uint32_t now_ms);

/* True while an unverified DID is stopped (D-068 item 2; false for an index out of range). */
bool uds_client_core_did_stopped(const uds_client_core_t* c, uint32_t idx);

/* Why DID index idx is stopped; UDS_CLIENT_STOP_NONE when it is not (or idx is out of range). */
uds_client_stop_reason_t uds_client_core_did_stop_reason(const uds_client_core_t* c, uint32_t idx);

/* Pending request kind (UDS_CLIENT_REQ_NONE when the slot is free). */
uds_client_req_kind_t uds_client_core_pending(const uds_client_core_t* c);

/* The positive response SID of the pending request (its SID + 0x40); 0 when none. */
uint8_t uds_client_core_pending_response_sid(const uds_client_core_t* c);

const uds_client_stats_t* uds_client_core_stats(const uds_client_core_t* c);

/* The stop epoch for the ride boundary of the derived signals (D-087 item 2): true while an
 * epoch is running; *counter (may be NULL) changes at every epoch start. */
bool uds_client_core_epoch(const uds_client_core_t* c, uint16_t* counter);

#ifdef __cplusplus
}
#endif

#endif /* UDS_CLIENT_CORE_H */
