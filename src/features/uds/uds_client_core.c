#include "features/uds/uds_client_core.h"

#include "uds_iso14229.h"

#include <math.h>
#include <stddef.h>

/* ReadDataByIdentifier positive response: [SID + 0x40][DID hi][DID lo][data...]. */
#define READ_RESPONSE_HEADER 3u

/* D-068 item 2: NRC 0x31 on this many separate reads stops an unverified DID. */
#define STOP_NRC_READS 2u

/* D-069 item 3: consecutive valid ENGINE_SPEED samples > 0 (after a 0) = engine start. */
#define ENGINE_START_SAMPLES 2u

static void count(uint32_t* counter)
{
    if (*counter < UINT32_MAX) {
        (*counter)++;
    }
}

/* Wrap-safe: true once period_ms have passed since start_ms. */
static bool expired(uint32_t now_ms, uint32_t start_ms, uint32_t period_ms)
{
    return (uint32_t)(now_ms - start_ms) >= period_ms;
}

void uds_client_core_init(uds_client_core_t* c, uint32_t abort_hold_ms)
{
    static const uds_client_core_t zero_core = {0};
    if (c == NULL) {
        return;
    }
    *c = zero_core;
    c->abort_hold_ms = abort_hold_ms;
}

/* Sends nothing for ms from now_ms; a hold that ends later is kept. */
static void hold_for(uds_client_core_t* c, uint32_t now_ms, uint32_t ms)
{
    if (c->hold) {
        const uint32_t elapsed = now_ms - c->hold_start_ms;
        if ((elapsed < c->hold_ms) && ((c->hold_ms - elapsed) >= ms)) {
            return;
        }
    }
    c->hold = true;
    c->hold_start_ms = now_ms;
    c->hold_ms = ms;
}

static void start_request(uds_client_core_t* c, uds_client_req_kind_t kind, uint8_t sid,
                          uint32_t idx, uint32_t now_ms)
{
    c->pending = kind;
    c->pending_sid = sid;
    c->pending_idx = idx;
    c->sent_ms = now_ms;
    c->wait_start_ms = now_ms;
    c->wait_ms = VEHICLE_CL250_RESPONSE_TIMEOUT_BASE_MS;
    count(&c->stats.requests);
}

/*
 * D-068 item 2: the stop rule applies to an unverified DID that has had no positive
 * answer in this epoch. A verified DID, and one that has answered, keep the
 * D-050..D-053 skip and retry whatever the ECU says.
 */
static bool stoppable(const uds_client_core_t* c, uint32_t idx)
{
    return !vehicle_cl250_dids[idx].verified && !c->did[idx].answered;
}

static void stop_did(uds_client_core_t* c, uint32_t idx, uds_client_stop_reason_t reason)
{
    uds_client_did_state_t* d = &c->did[idx];
    if (!d->stopped) {
        d->stopped = true;
        d->stop_reason = reason;
        count(&c->stats.did_stops);
    }
    d->consecutive_timeouts = 0u; /* the stop rule starts again from zero when it is lifted */
    d->stop_nrc_count = 0u;
    d->slow = false;
    d->unanswered = false;
}

static void skip_did(uds_client_core_t* c, uint32_t idx, uint32_t now_ms)
{
    uds_client_did_state_t* d = &c->did[idx];
    d->skipped = true;
    d->skip_start_ms = now_ms;
    d->consecutive_timeouts = 0u;
    d->slow = false; /* the fresh attempt after the cooldown gets its priority back */
    d->unanswered = false;
    count(&c->stats.did_skips);
}

/* D-069 item 3: a read of ENGINE_SPEED that ends without an in-time decoded sample breaks the run. */
static void break_rpm_run(uds_client_core_t* c, uint32_t idx)
{
    if (idx == (uint32_t)VEHICLE_CL250_IDX_ENGINE_SPEED) {
        c->rpm_run = 0u;
    }
}

/* Unverified DID that never answered in this epoch: stopped. Otherwise: the skip cooldown. */
static void stop_or_skip(uds_client_core_t* c, uint32_t idx, uint32_t now_ms,
                         uds_client_stop_reason_t reason)
{
    if (stoppable(c, idx)) {
        stop_did(c, idx, reason);
    } else {
        skip_did(c, idx, now_ms);
    }
}

/*
 * D-068 item 2: an NRC carries no DID, so within RESPONSE_TIMEOUT_MAX_MS of a read
 * timeout it may be the timed-out read's answer. True while that quiet window runs.
 */
static bool in_quiet_window(const uds_client_core_t* c, uint32_t now_ms)
{
    return c->timeout_seen &&
           !expired(now_ms, c->last_timeout_ms, VEHICLE_CL250_RESPONSE_TIMEOUT_MAX_MS);
}

static void end_with_timeout(uds_client_core_t* c, uint32_t now_ms)
{
    count(&c->stats.timeouts);
    if (c->pending == UDS_CLIENT_REQ_READ) {
        uds_client_did_state_t* d = &c->did[c->pending_idx];
        /* The period restarts now: a silent DID whose period is not longer than the
         * timeout would otherwise be due again at once and, served first, hold the slot
         * until it is skipped. */
        d->last_request_ms = now_ms;
        if (!d->unanswered) {
            d->unanswered = true;
            d->unanswered_ms = c->sent_ms; /* its answer may still come (D-051) */
        }
        d->consecutive_timeouts++;
        c->timeout_seen = true;
        c->last_timeout_ms = now_ms;
        break_rpm_run(c, c->pending_idx);
        if (d->consecutive_timeouts >= VEHICLE_CL250_MAX_CONSECUTIVE_TIMEOUTS) {
            stop_or_skip(c, c->pending_idx, now_ms, UDS_CLIENT_STOP_TIMEOUTS); /* D-068 item 2 */
        }
    }
    c->pending = UDS_CLIENT_REQ_NONE;
}

/*
 * The ECU went absent for longer than RESPONSE_TIMEOUT_MAX_MS, so it will not answer a
 * read from before: no answer is outstanding and no DID is faulty or slow (D-051).
 * Otherwise every sample after key-on would be stamped before the absence.
 */
static void forget_reads(uds_client_core_t* c)
{
    for (uint32_t idx = 0u; idx < VEHICLE_CL250_DID_COUNT; idx++) {
        c->did[idx].consecutive_timeouts = 0u;
        c->did[idx].slow = false;
        c->did[idx].unanswered = false;
    }
}

/*
 * D-053: the poll period margin covers one step of at most CLIENT_STEP_MAX_MS. A longer
 * gap lets an answer just inside the base timeout be seen at or after its DID is due
 * again (slow, or read again back to back): counted here for the D-029 measurement and
 * the field, not acted on. Every poll counts, also while latched or waiting.
 */
static void check_step_gap(uds_client_core_t* c, uint32_t now_ms)
{
    if (c->polled) {
        const uint32_t gap = now_ms - c->last_poll_ms; /* wrap-safe */
        if (gap > VEHICLE_CL250_CLIENT_STEP_MAX_MS) {
            count(&c->stats.step_overruns);
        }
        if (gap > c->stats.step_gap_max_ms) {
            c->stats.step_gap_max_ms = gap;
        }
    }
    c->polled = true;
    c->last_poll_ms = now_ms;
}

static void lose_session(uds_client_core_t* c)
{
    if (c->session_up) {
        c->session_up = false;
        count(&c->stats.session_losses);
    }
    c->rpm_run = 0u;
}

/*
 * D-068 item 2 / D-075 (provisional) item 1: a stop epoch starts. Every unverified DID is
 * due at once, with its stop state, NRC 0x31/0x7E/0x7F count and answered flag cleared (an
 * ECU that serves a DID only in the extended session refused it before). The verified
 * DIDs keep their schedule. D-069 item 3: the epoch may re-ask once, at its first engine
 * start. Called only at the first confirmation after init and at the first one after the
 * ECU was lost; a confirmation after NRC 0x7E/0x7F stays in the epoch and calls nothing.
 */
static void arm_unverified_reads(uds_client_core_t* c)
{
    for (uint32_t idx = 0u; idx < VEHICLE_CL250_DID_COUNT; idx++) {
        uds_client_did_state_t* d = &c->did[idx];
        if (!vehicle_cl250_dids[idx].verified) {
            d->requested = false;
            d->stopped = false;
            d->answered = false;
            d->stop_nrc_count = 0u;
            d->stop_reason = UDS_CLIENT_STOP_NONE;
            d->skipped = false;
            d->consecutive_timeouts = 0u;
        }
    }
    c->rpm_zero_seen = false;
    c->rpm_run = 0u;
    c->engine_rearmed = false;
}

/*
 * D-069 item 3: the first engine start of the stop epoch. Every stopped DID is
 * pollable again with the stop rule from zero. It keeps last_request_ms (and requested),
 * so it is due one poll_period_ms after its last request (not forced due: at once only if
 * that period has already passed), never faster than its tier: its tier rate
 * and the D-068 polling budget are not exceeded.
 */
static void rearm_stopped_reads(uds_client_core_t* c)
{
    for (uint32_t idx = 0u; idx < VEHICLE_CL250_DID_COUNT; idx++) {
        uds_client_did_state_t* d = &c->did[idx];
        if (d->stopped) {
            d->stopped = false;
            d->stop_reason = UDS_CLIENT_STOP_NONE;
            d->stop_nrc_count = 0u;
            d->consecutive_timeouts = 0u;
            count(&c->stats.rearms);
        }
    }
}

/* A decoded, in-time ENGINE_SPEED answer to the pending read (D-069 item 3). */
static void watch_engine_start(uds_client_core_t* c, float rpm)
{
    if (!isfinite(rpm)) {
        c->rpm_run = 0u; /* a NaN is neither a running engine nor a valid 0 */
    } else if (rpm <= 0.0f) {
        c->rpm_zero_seen = true;
        c->rpm_run = 0u;
    } else if (c->rpm_zero_seen && !c->engine_rearmed) {
        c->rpm_run++;
        if (c->rpm_run >= ENGINE_START_SAMPLES) {
            c->engine_rearmed = true; /* once per stop epoch, not at every restart */
            rearm_stopped_reads(c);
        }
    } else {
        /* an RPM > 0 without a zero before it, or after the re-ask: nothing to do */
    }
}

/*
 * D-050 (E-5): a DID whose last read timed out is faulty until it answers in time
 * (D-052: a slow answer keeps the timeout count) or is skipped. It competes in the
 * normal class whatever its gen/ priority, and its read gets no NRC 0x78 extension,
 * so it holds the slot for at most RESPONSE_TIMEOUT_BASE_MS.
 * Priority alone cannot help: one request is in flight, so the hold time starves the
 * others, not the order.
 * D-051 (E-7): a DID whose last answer came more than its poll_period_ms after its
 * stamp is faulty the same way, until an answer comes within the period or the skip.
 */
static bool faulty(const uds_client_core_t* c, uint32_t idx)
{
    return (c->did[idx].consecutive_timeouts > 0u) || c->did[idx].slow;
}

/*
 * D-051 (E-7): the earliest time the ECU can have taken the sample that answers the
 * pending read. An answer carries the DID but no request reference, so after a timeout
 * it may answer that earlier read: the first one since the last answer.
 */
static uint32_t sample_stamp(const uds_client_core_t* c)
{
    const uds_client_did_state_t* d = &c->did[c->pending_idx];
    return d->unanswered ? d->unanswered_ms : c->sent_ms;
}

/*
 * The pending read was answered (decoded, or an NRC other than 0x78). It ends the stamp
 * chain. D-052 (E-8 (3)): only an answer in time resets the skip count, so an ECU that
 * alternates timeouts with slow answers is still skipped after MAX_CONSECUTIVE_TIMEOUTS
 * timeouts instead of holding the slot with no skip.
 */
static void read_answered(uds_client_core_t* c, uint32_t now_ms)
{
    uds_client_did_state_t* d = &c->did[c->pending_idx];
    d->slow = (uint32_t)(now_ms - sample_stamp(c)) >
              (uint32_t)vehicle_cl250_dids[c->pending_idx].poll_period_ms;
    d->unanswered = false;
    if (d->slow) {
        count(&c->stats.slow_answers);
    } else {
        d->consecutive_timeouts = 0u;
    }
}

static uint8_t poll_priority(const uds_client_core_t* c, uint32_t idx)
{
    const uint8_t prio = vehicle_cl250_dids[idx].priority;
    return (faulty(c, idx) && (prio < VEHICLE_CL250_PRIORITY_NORMAL))
               ? (uint8_t)VEHICLE_CL250_PRIORITY_NORMAL
               : prio;
}

/*
 * Next due DID, or false (D-043): the lowest priority value first (poll_priority(),
 * D-050), then table order. The scan always covers the whole table; a strict < keeps
 * the first entry of a class. No starvation while every request holds the slot for at
 * most ASSUMED_ROUND_TRIP_MS (ECU answer + one step): the defs codegen (v0.3.2 on)
 * and test_the_gen_table_meets_its_gap_bound bound every DID's sample gap by its
 * stale_after_ms for this order. A faulty DID holds it for up to the base timeout
 * after its first failing attempt (README, "Reads"). A stopped unverified DID (D-068 item
 * 2) is never due; an unverified DID's read gets no NRC 0x78 extension (item 1), so one
 * blocks the verified DIDs, which come first in the table, for at most one such read.
 */
static bool next_read(uds_client_core_t* c, uint32_t now_ms, uint32_t* idx_out)
{
    bool found = false;
    uint32_t best = 0u;
    for (uint32_t idx = 0u; idx < VEHICLE_CL250_DID_COUNT; idx++) {
        uds_client_did_state_t* d = &c->did[idx];
        if (d->skipped && expired(now_ms, d->skip_start_ms, VEHICLE_CL250_DID_SKIP_COOLDOWN_MS)) {
            d->skipped = false;
        }
        const bool due = !d->skipped && !d->stopped &&
                         (!d->requested ||
                          expired(now_ms, d->last_request_ms,
                                  (uint32_t)vehicle_cl250_dids[idx].poll_period_ms));
        if (due && (!found || (poll_priority(c, idx) < poll_priority(c, best)))) {
            best = idx;
            found = true;
        }
    }
    *idx_out = best;
    return found;
}

bool uds_client_core_poll(uds_client_core_t* c, uint32_t now_ms, bool rx_busy, bool tx_ready,
                          uint8_t* out, uint16_t* len)
{
    if ((c == NULL) || (out == NULL) || (len == NULL)) {
        return false;
    }
    *len = 0u;
    c->last = UDS_CLIENT_LAST_NONE;
    check_step_gap(c, now_ms);
    if (c->timeout_seen && !in_quiet_window(c, now_ms)) {
        c->timeout_seen = false; /* the window is over: clear it, also across a counter wrap */
    }
    if (c->ecu_seen && !uds_client_core_ecu_present(c, now_ms)) {
        c->ecu_seen = false; /* absent: presence needs a new answer, also across a wrap */
        lose_session(c);     /* an ECU that came back is in its default session */
        forget_reads(c);
        c->epoch_open = false; /* D-075 item 1: its next confirmation starts a new stop epoch */
    }
    if (c->failed) {
        return false; /* presence still ages out above, also while latched */
    }

    if (c->pending != UDS_CLIENT_REQ_NONE) {
        const bool cap = expired(now_ms, c->sent_ms, VEHICLE_CL250_RESPONSE_TIMEOUT_MAX_MS);
        const bool base = !rx_busy && expired(now_ms, c->wait_start_ms, c->wait_ms);
        if (cap || base) {
            end_with_timeout(c, now_ms);
        }
    }
    if ((c->pending != UDS_CLIENT_REQ_NONE) || rx_busy || !tx_ready) {
        return false; /* one request in flight; the ECU is mid-transfer; TX not free */
    }
    if (c->hold) {
        if (!expired(now_ms, c->hold_start_ms, c->hold_ms)) {
            return false;
        }
        c->hold = false;
    }

    if (!c->session_up &&
        (!c->session_tried ||
         expired(now_ms, c->session_tried_ms, VEHICLE_CL250_SESSION_RETRY_INTERVAL_MS))) {
        out[0] = (uint8_t)VEHICLE_CL250_SESSION_SID;
        out[1] = (uint8_t)VEHICLE_CL250_SESSION_SUBFUNCTION;
        *len = 2u;
        c->last = UDS_CLIENT_LAST_SESSION;
        c->prev_request_ms = c->session_tried_ms;
        c->prev_requested = c->session_tried;
        c->session_tried = true;
        c->session_tried_ms = now_ms;
        start_request(c, UDS_CLIENT_REQ_SESSION, out[0], 0u, now_ms);
        return true;
    }
    if (!c->tp_sent || expired(now_ms, c->tp_sent_ms, VEHICLE_CL250_TESTER_PRESENT_PERIOD_MS)) {
        out[0] = (uint8_t)VEHICLE_CL250_TESTER_PRESENT_SID;
        out[1] = (uint8_t)VEHICLE_CL250_TESTER_PRESENT_SUBFUNCTION;
        *len = 2u;
        c->last = UDS_CLIENT_LAST_TESTER_PRESENT;
        c->prev_request_ms = c->tp_sent_ms;
        c->prev_requested = c->tp_sent;
        c->tp_sent = true;
        c->tp_sent_ms = now_ms;
        count(&c->stats.requests); /* response suppressed: nothing to wait for */
        return true;
    }
    if (!c->session_up) {
        return false;
    }

    uint32_t idx = 0u;
    if (!next_read(c, now_ms, &idx)) {
        return false;
    }
    const uint16_t did = vehicle_cl250_dids[idx].did;
    out[0] = (uint8_t)UDS_SID_READ_DATA_BY_IDENTIFIER;
    out[1] = (uint8_t)(did >> 8u);
    out[2] = (uint8_t)(did & 0xFFu);
    *len = 3u;
    c->last = UDS_CLIENT_LAST_READ;
    c->prev_request_ms = c->did[idx].last_request_ms;
    c->prev_requested = c->did[idx].requested;
    c->did[idx].requested = true;
    c->did[idx].last_request_ms = now_ms;
    start_request(c, UDS_CLIENT_REQ_READ, out[0], idx, now_ms);
    c->pending_quiet = in_quiet_window(c, now_ms);
    return true;
}

void uds_client_core_not_sent(uds_client_core_t* c)
{
    if ((c != NULL) && (c->last != UDS_CLIENT_LAST_NONE)) {
        /* Nothing went out: no period may run from it, and it is not counted (E-6). */
        if (c->last == UDS_CLIENT_LAST_SESSION) {
            c->session_tried_ms = c->prev_request_ms;
            c->session_tried = c->prev_requested;
        } else if (c->last == UDS_CLIENT_LAST_TESTER_PRESENT) {
            c->tp_sent_ms = c->prev_request_ms;
            c->tp_sent = c->prev_requested;
        } else {
            c->did[c->pending_idx].last_request_ms = c->prev_request_ms;
            c->did[c->pending_idx].requested = c->prev_requested;
        }
        if (c->stats.requests < UINT32_MAX) {
            c->stats.requests--; /* saturated stays saturated (may over-count by one) */
        }
        c->pending = UDS_CLIENT_REQ_NONE; /* tester present never held it */
        c->last = UDS_CLIENT_LAST_NONE;
    }
}

void uds_client_core_latch(uds_client_core_t* c)
{
    if (c != NULL) {
        c->failed = true;
        c->pending = UDS_CLIENT_REQ_NONE;
        c->last = UDS_CLIENT_LAST_NONE; /* nothing left for not_sent to undo */
        lose_session(c);
    }
}

static void on_negative(uds_client_core_t* c, uint32_t now_ms, uint8_t sid, uint8_t nrc)
{
    count(&c->stats.nrc);
    if ((nrc == UDS_NRC_SUBFUNCTION_NOT_SUPPORTED_IN_ACTIVE_SESSION) ||
        (nrc == UDS_NRC_SERVICE_NOT_SUPPORTED_IN_ACTIVE_SESSION)) {
        lose_session(c);
    }
    if ((c->pending == UDS_CLIENT_REQ_NONE) || (sid != c->pending_sid)) {
        return; /* e.g. an NRC for tester present: never ends another request */
    }
    if (nrc == UDS_NRC_RESPONSE_PENDING) {
        /* D-050: a faulty DID's read still ends at the base timeout from its request.
         * D-068 item 1: so does an unverified DID's (its read holds the slot for at most
         * the base timeout + one step, the premise of the verified DIDs' gap bounds). */
        if ((c->pending != UDS_CLIENT_REQ_READ) ||
            (!faulty(c, c->pending_idx) && vehicle_cl250_dids[c->pending_idx].verified)) {
            c->wait_ms = (c->wait_ms > (VEHICLE_CL250_RESPONSE_TIMEOUT_MAX_MS / 2u))
                             ? VEHICLE_CL250_RESPONSE_TIMEOUT_MAX_MS
                             : (c->wait_ms * 2u);
            c->wait_start_ms = now_ms;
        }
        count(&c->stats.response_pending);
        return;
    }
    if (c->pending == UDS_CLIENT_REQ_READ) {
        const uint32_t idx = c->pending_idx;
        const bool refusal = (nrc == UDS_NRC_REQUEST_OUT_OF_RANGE) ||
                             (nrc == UDS_NRC_SUBFUNCTION_NOT_SUPPORTED_IN_ACTIVE_SESSION) ||
                             (nrc == UDS_NRC_SERVICE_NOT_SUPPORTED_IN_ACTIVE_SESSION);
        if (refusal && stoppable(c, idx) && !c->pending_quiet && !in_quiet_window(c, now_ms)) {
            /* D-068 item 2: DID not supported. D-075 item 1: an NRC 0x7E/0x7F answer to a
             * never-answered unverified DID counts the same way (the ECU fell back to the
             * default session on it). Two separate reads (requests go to both response
             * IDs, and one NRC may trail an earlier answer); never in the quiet window
             * after a timeout, where this NRC may be the earlier read's. */
            c->did[idx].stop_nrc_count++;
            if (c->did[idx].stop_nrc_count >= STOP_NRC_READS) {
                stop_did(c, idx,
                         (nrc == UDS_NRC_REQUEST_OUT_OF_RANGE) ? UDS_CLIENT_STOP_NRC_31
                                                              : UDS_CLIENT_STOP_NRC_SESSION);
            }
            if (nrc != UDS_NRC_REQUEST_OUT_OF_RANGE) {
                /* D-075 item 1 (amended, provisional): a counted 0x7E/0x7F asks for the
                 * session again at the next step, not after SESSION_RETRY_INTERVAL_MS,
                 * so it does not black out the verified DIDs. Every other session loss
                 * keeps the retry interval. Same 0x10 03, inside tester_policy. */
                c->session_tried = false;
            }
        }
        break_rpm_run(c, idx); /* D-069 item 3: an NRC breaks the run */
        read_answered(c, now_ms); /* answered, not a timeout */
    }
    c->pending = UDS_CLIENT_REQ_NONE;
}

/*
 * The pending read's positive response. A message of at most ISOTP_SF_MAX_LEN bytes can
 * only have arrived as a Single Frame (the core ignores an FF_DL below 8), so it is put
 * back into that frame for the generated parser, which checks SID, DID echo and length
 * and decodes with the gen/ formula and range.
 */
static bool parse_read(const uds_client_core_t* c, const uint8_t* data, uint16_t len,
                       uds_client_sample_t* sample)
{
    const vehicle_cl250_did_t* e = &vehicle_cl250_dids[c->pending_idx];
    uint8_t frame[ISOTP_CAN_DL];
    float physical = 0.0f;
    if (len > ISOTP_SF_MAX_LEN) {
        return false;
    }
    frame[0] = (uint8_t)len;
    for (uint16_t i = 0u; i < len; i++) {
        frame[i + 1u] = data[i];
    }
    if (!vehicle_cl250_parse_response(e->did, frame, (size_t)len + 1u, &physical)) {
        return false;
    }
    uint32_t raw = 0u;
    for (uint8_t i = 0u; i < e->length; i++) { /* parse checked the length (<= 4 bytes) */
        raw = (raw << 8u) | (uint32_t)data[READ_RESPONSE_HEADER + i];
    }
    sample->idx = c->pending_idx;
    sample->raw = raw;
    sample->physical = physical;
    return true;
}

void uds_client_core_on_fc_withheld(uds_client_core_t* c, uint32_t now_ms)
{
    if (c == NULL) {
        return;
    }
    count(&c->stats.fc_withheld);
    if (c->pending != UDS_CLIENT_REQ_NONE) {
        count(&c->stats.unavailable); /* not a timeout: the ECU answered */
        if (c->pending == UDS_CLIENT_REQ_READ) {
            break_rpm_run(c, c->pending_idx);
            stop_or_skip(c, c->pending_idx, now_ms, UDS_CLIENT_STOP_FIRST_FRAME); /* D-068 item 2 */
        }
        c->pending = UDS_CLIENT_REQ_NONE;
    }
    hold_for(c, now_ms, VEHICLE_CL250_RESPONSE_TIMEOUT_MAX_MS); /* the ECU waits its N_Bs */
}

bool uds_client_core_on_indication(uds_client_core_t* c, uint32_t now_ms,
                                   isotp_n_result_t result, const uint8_t* data, uint16_t len,
                                   uds_client_sample_t* sample)
{
    if ((c == NULL) || (sample == NULL)) {
        return false;
    }
    if (result != ISOTP_N_OK) {
        /* A segmented reception failed (N_Cr, sequence): the ECU may still be sending. */
        count(&c->stats.unavailable);
        if (c->pending == UDS_CLIENT_REQ_READ) {
            break_rpm_run(c, c->pending_idx);
            /* D-068 item 2: a First Frame whose reception then failed is a First Frame
             * answer too: stop a never-answered unverified DID, skip any other. */
            stop_or_skip(c, c->pending_idx, now_ms, UDS_CLIENT_STOP_FIRST_FRAME);
        }
        c->pending = UDS_CLIENT_REQ_NONE;
        hold_for(c, now_ms, c->abort_hold_ms);
        return false;
    }
    if ((data == NULL) || (len == 0u)) {
        count(&c->stats.unexpected);
        return false;
    }
    c->ecu_seen = true;
    c->last_response_ms = now_ms;

    if ((len >= UDS_NEGATIVE_RESPONSE_LEN) && (data[0] == UDS_SID_NEGATIVE_RESPONSE)) {
        on_negative(c, now_ms, data[1], data[2]);
        return false;
    }
    if ((c->pending == UDS_CLIENT_REQ_READ) && (len > ISOTP_SF_MAX_LEN)) {
        /* D-059: a table DID answers in a Single Frame. A segmented answer is "service
         * unavailable", not a timeout; the ECU has finished, so no hold. */
        count(&c->stats.unavailable);
        break_rpm_run(c, c->pending_idx);
        stop_or_skip(c, c->pending_idx, now_ms, UDS_CLIENT_STOP_FIRST_FRAME); /* no table DID needs one */
        c->pending = UDS_CLIENT_REQ_NONE;
        return false;
    }
    if ((c->pending == UDS_CLIENT_REQ_SESSION) && (len >= 2u) &&
        (data[0] == (uint8_t)VEHICLE_CL250_SESSION_POSITIVE_SID) &&
        (data[1] == (uint8_t)VEHICLE_CL250_SESSION_SUBFUNCTION)) {
        c->session_up = true;
        c->pending = UDS_CLIENT_REQ_NONE;
        count(&c->stats.session_starts);
        if (!c->epoch_open) {
            c->epoch_open = true;
            arm_unverified_reads(c);
        }
        return false;
    }
    if ((c->pending == UDS_CLIENT_REQ_READ) && parse_read(c, data, len, sample)) {
        sample->stamp_ms = sample_stamp(c);
        read_answered(c, now_ms);
        uds_client_did_state_t* d = &c->did[c->pending_idx];
        d->answered = true; /* D-068 item 2: a positive answer undoes a stop and ends the rule */
        d->stopped = false;
        d->stop_reason = UDS_CLIENT_STOP_NONE;
        d->stop_nrc_count = 0u;
        if (c->pending_idx == (uint32_t)VEHICLE_CL250_IDX_ENGINE_SPEED) {
            if (d->slow) {
                c->rpm_run = 0u; /* D-051: a slow answer is not a fresh sample */
            } else {
                watch_engine_start(c, sample->physical);
            }
        }
        c->pending = UDS_CLIENT_REQ_NONE;
        count(&c->stats.reads_ok);
        return true;
    }
    count(&c->stats.unexpected); /* late, other DID, out of range: the request times out */
    return false;
}

bool uds_client_core_session_up(const uds_client_core_t* c)
{
    return (c != NULL) && c->session_up;
}

bool uds_client_core_ecu_present(const uds_client_core_t* c, uint32_t now_ms)
{
    return (c != NULL) && c->ecu_seen &&
           !expired(now_ms, c->last_response_ms, VEHICLE_CL250_ECU_ABSENT_TIMEOUT_MS);
}

bool uds_client_core_failed(const uds_client_core_t* c)
{
    return (c == NULL) || c->failed;
}

bool uds_client_core_did_skipped(const uds_client_core_t* c, uint32_t idx, uint32_t now_ms)
{
    if ((c == NULL) || (idx >= VEHICLE_CL250_DID_COUNT)) {
        return false;
    }
    const uds_client_did_state_t* d = &c->did[idx];
    return d->skipped && !expired(now_ms, d->skip_start_ms, VEHICLE_CL250_DID_SKIP_COOLDOWN_MS);
}

uds_client_stop_reason_t uds_client_core_did_stop_reason(const uds_client_core_t* c, uint32_t idx)
{
    return ((c != NULL) && (idx < VEHICLE_CL250_DID_COUNT) && c->did[idx].stopped)
               ? c->did[idx].stop_reason
               : UDS_CLIENT_STOP_NONE;
}

bool uds_client_core_did_stopped(const uds_client_core_t* c, uint32_t idx)
{
    return (c != NULL) && (idx < VEHICLE_CL250_DID_COUNT) && c->did[idx].stopped;
}

uds_client_req_kind_t uds_client_core_pending(const uds_client_core_t* c)
{
    return (c == NULL) ? UDS_CLIENT_REQ_NONE : c->pending;
}

uint8_t uds_client_core_pending_response_sid(const uds_client_core_t* c)
{
    return ((c == NULL) || (c->pending == UDS_CLIENT_REQ_NONE))
               ? 0u
               : (uint8_t)(c->pending_sid + UDS_POSITIVE_RESPONSE_OFFSET);
}

const uds_client_stats_t* uds_client_core_stats(const uds_client_core_t* c)
{
    return (c == NULL) ? NULL : &c->stats;
}
