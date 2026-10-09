#include "features/uds/uds_client_core.h"

#include "uds_iso14229.h"

#include <stddef.h>

/* ReadDataByIdentifier positive response: [SID + 0x40][DID hi][DID lo][data...]. */
#define READ_RESPONSE_HEADER 3u

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
        if (d->consecutive_timeouts >= VEHICLE_CL250_MAX_CONSECUTIVE_TIMEOUTS) {
            skip_did(c, c->pending_idx, now_ms);
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
 * after its first failing attempt (README, "Reads").
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
        const bool due = !d->skipped &&
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
    if (c->ecu_seen && !uds_client_core_ecu_present(c, now_ms)) {
        c->ecu_seen = false; /* absent: presence needs a new answer, also across a wrap */
        lose_session(c);     /* an ECU that came back is in its default session */
        forget_reads(c);
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
        /* D-050: a faulty DID's read still ends at the base timeout from its request. */
        if ((c->pending != UDS_CLIENT_REQ_READ) || !faulty(c, c->pending_idx)) {
            c->wait_ms = (c->wait_ms > (VEHICLE_CL250_RESPONSE_TIMEOUT_MAX_MS / 2u))
                             ? VEHICLE_CL250_RESPONSE_TIMEOUT_MAX_MS
                             : (c->wait_ms * 2u);
            c->wait_start_ms = now_ms;
        }
        count(&c->stats.response_pending);
        return;
    }
    if (c->pending == UDS_CLIENT_REQ_READ) {
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
            skip_did(c, c->pending_idx, now_ms);
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
            skip_did(c, c->pending_idx, now_ms);
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
    if ((c->pending == UDS_CLIENT_REQ_SESSION) && (len >= 2u) &&
        (data[0] == (uint8_t)VEHICLE_CL250_SESSION_POSITIVE_SID) &&
        (data[1] == (uint8_t)VEHICLE_CL250_SESSION_SUBFUNCTION)) {
        c->session_up = true;
        c->pending = UDS_CLIENT_REQ_NONE;
        count(&c->stats.session_starts);
        return false;
    }
    if ((c->pending == UDS_CLIENT_REQ_READ) && parse_read(c, data, len, sample)) {
        sample->stamp_ms = sample_stamp(c);
        read_answered(c, now_ms);
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

uds_client_req_kind_t uds_client_core_pending(const uds_client_core_t* c)
{
    return (c == NULL) ? UDS_CLIENT_REQ_NONE : c->pending;
}

const uds_client_stats_t* uds_client_core_stats(const uds_client_core_t* c)
{
    return (c == NULL) ? NULL : &c->stats;
}
