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
    count(&c->stats.did_skips);
}

static void end_with_timeout(uds_client_core_t* c, uint32_t now_ms)
{
    count(&c->stats.timeouts);
    if (c->pending == UDS_CLIENT_REQ_READ) {
        uds_client_did_state_t* d = &c->did[c->pending_idx];
        d->consecutive_timeouts++;
        if (d->consecutive_timeouts >= VEHICLE_CL250_MAX_CONSECUTIVE_TIMEOUTS) {
            skip_did(c, c->pending_idx, now_ms);
        }
    }
    c->pending = UDS_CLIENT_REQ_NONE;
}

static void lose_session(uds_client_core_t* c)
{
    if (c->session_up) {
        c->session_up = false;
        count(&c->stats.session_losses);
    }
}

/* Next due DID (round-robin from rr_next), or false. */
static bool next_read(uds_client_core_t* c, uint32_t now_ms, uint32_t* idx_out)
{
    for (uint32_t n = 0u; n < VEHICLE_CL250_DID_COUNT; n++) {
        const uint32_t idx = (c->rr_next + n) % VEHICLE_CL250_DID_COUNT;
        uds_client_did_state_t* d = &c->did[idx];
        if (d->skipped && !expired(now_ms, d->skip_start_ms, VEHICLE_CL250_DID_SKIP_COOLDOWN_MS)) {
            continue;
        }
        d->skipped = false;
        if (!d->requested ||
            expired(now_ms, d->last_request_ms, (uint32_t)vehicle_cl250_dids[idx].poll_period_ms)) {
            *idx_out = idx;
            return true;
        }
    }
    return false;
}

bool uds_client_core_poll(uds_client_core_t* c, uint32_t now_ms, bool rx_busy, bool tx_ready,
                          uint8_t* out, uint16_t* len)
{
    if ((c == NULL) || (out == NULL) || (len == NULL)) {
        return false;
    }
    *len = 0u;
    if (c->ecu_seen && !uds_client_core_ecu_present(c, now_ms)) {
        c->ecu_seen = false; /* absent: presence needs a new answer, also across a wrap */
        lose_session(c);     /* an ECU that came back is in its default session */
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
        if (!expired(now_ms, c->hold_start_ms, c->abort_hold_ms)) {
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
        c->session_tried = true;
        c->session_tried_ms = now_ms;
        start_request(c, UDS_CLIENT_REQ_SESSION, out[0], 0u, now_ms);
        return true;
    }
    if (!c->tp_sent || expired(now_ms, c->tp_sent_ms, VEHICLE_CL250_TESTER_PRESENT_PERIOD_MS)) {
        out[0] = (uint8_t)VEHICLE_CL250_TESTER_PRESENT_SID;
        out[1] = (uint8_t)VEHICLE_CL250_TESTER_PRESENT_SUBFUNCTION;
        *len = 2u;
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
    c->did[idx].requested = true;
    c->did[idx].last_request_ms = now_ms;
    c->rr_next = (idx + 1u) % VEHICLE_CL250_DID_COUNT;
    start_request(c, UDS_CLIENT_REQ_READ, out[0], idx, now_ms);
    return true;
}

void uds_client_core_not_sent(uds_client_core_t* c)
{
    if (c != NULL) {
        c->pending = UDS_CLIENT_REQ_NONE;
    }
}

void uds_client_core_latch(uds_client_core_t* c)
{
    if (c != NULL) {
        c->failed = true;
        c->pending = UDS_CLIENT_REQ_NONE;
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
        c->wait_ms = (c->wait_ms > (VEHICLE_CL250_RESPONSE_TIMEOUT_MAX_MS / 2u))
                         ? VEHICLE_CL250_RESPONSE_TIMEOUT_MAX_MS
                         : (c->wait_ms * 2u);
        c->wait_start_ms = now_ms;
        count(&c->stats.response_pending);
        return;
    }
    if (c->pending == UDS_CLIENT_REQ_READ) {
        c->did[c->pending_idx].consecutive_timeouts = 0u; /* answered, not a timeout */
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

bool uds_client_core_on_indication(uds_client_core_t* c, uint32_t now_ms,
                                   isotp_n_result_t result, const uint8_t* data, uint16_t len,
                                   uds_client_sample_t* sample)
{
    if ((c == NULL) || (sample == NULL)) {
        return false;
    }
    if (result != ISOTP_N_OK) {
        /* Q-020: a segmented response cannot be received (no FC on the vehicle bus). */
        count(&c->stats.unavailable);
        if (c->pending == UDS_CLIENT_REQ_READ) {
            skip_did(c, c->pending_idx, now_ms);
        }
        c->pending = UDS_CLIENT_REQ_NONE;
        c->hold = true;
        c->hold_start_ms = now_ms;
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
        c->did[c->pending_idx].consecutive_timeouts = 0u;
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
