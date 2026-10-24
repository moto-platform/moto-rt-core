#include "features/uds/dtc_snapshot_core.h"

#include "uds_iso14229.h"

#include <math.h>

/* Wrap-safe: true once period_ms have passed since start_ms. */
static bool hold_elapsed(uint32_t now_ms, uint32_t start_ms, uint32_t period_ms)
{
    return (uint32_t)(now_ms - start_ms) >= period_ms;
}

static void sat_inc(uint32_t* counter)
{
    if (*counter < UINT32_MAX) {
        (*counter)++;
    }
}

void dtc_snapshot_core_init(dtc_snapshot_core_t* c)
{
    static const dtc_snapshot_core_t zero_core = {0};
    if (c != NULL) {
        *c = zero_core;
    }
}

uint8_t dtc_snapshot_core_sub_function(dtc_snapshot_step_t step)
{
    uint8_t sub = 0u;
    switch (step) {
    case DTC_SNAPSHOT_STEP_COUNT:
        sub = (uint8_t)UDS_READ_DTC_REPORT_NUMBER_OF_DTC_BY_STATUS_MASK;
        break;
    case DTC_SNAPSHOT_STEP_LIST:
        sub = (uint8_t)UDS_READ_DTC_REPORT_DTC_BY_STATUS_MASK;
        break;
    case DTC_SNAPSHOT_STEP_SNAPSHOT:
        sub = (uint8_t)UDS_READ_DTC_REPORT_DTC_SNAPSHOT_RECORD_BY_DTC_NUMBER;
        break;
    case DTC_SNAPSHOT_STEP_EXT_DATA:
        sub = (uint8_t)UDS_READ_DTC_REPORT_DTC_EXT_DATA_RECORD_BY_DTC_NUMBER;
        break;
    default:
        break;
    }
    return sub;
}

static bool is_record_step(dtc_snapshot_step_t step)
{
    return (step == DTC_SNAPSHOT_STEP_SNAPSHOT) || (step == DTC_SNAPSHOT_STEP_EXT_DATA);
}

/* The run ends. The reason is kept only if a run was active (a new epoch ends silently). */
static void end_run(dtc_snapshot_core_t* c, dtc_snapshot_end_t why)
{
    if (c->run_active) {
        c->last_end = why;
    }
    c->run_active = false;
    c->in_flight = false;
    c->step = DTC_SNAPSHOT_STEP_NONE;
}

static void start_run(dtc_snapshot_core_t* c, dtc_snapshot_trigger_t trigger)
{
    c->run_active = true;
    c->in_flight = false;
    c->trigger = trigger;
    c->step = DTC_SNAPSHOT_STEP_COUNT;
    c->last_end = DTC_SNAPSHOT_END_NONE;
    c->requests_in_run = 0u;
    c->dtc_count = 0u;
    c->dtc_index = 0u;
    c->runs_started++;
    sat_inc(&c->stats.runs);
}

/* A halt that came while a request was in flight ends the run once it resolved. */
static void settle(dtc_snapshot_core_t* c)
{
    if ((c->halt != DTC_SNAPSHOT_HALT_NONE) && c->run_active && !c->in_flight) {
        end_run(c, DTC_SNAPSHOT_END_HALTED);
    }
}

/* 0x19 is off for the rest of the epoch; the first reason stays. */
static void halt_epoch(dtc_snapshot_core_t* c, dtc_snapshot_halt_t reason)
{
    end_run(c, DTC_SNAPSHOT_END_HALTED);
    if (c->halt == DTC_SNAPSHOT_HALT_NONE) {
        c->halt = reason;
        sat_inc(&c->stats.halts);
    }
}

void dtc_snapshot_core_start_epoch(dtc_snapshot_core_t* c)
{
    if (c == NULL) {
        return;
    }
    c->epoch_started = true;
    c->halt = DTC_SNAPSHOT_HALT_NONE;
    c->runs_started = 0u;
    c->warm_idle_used = false;
    c->warm_idle_timing = false;
    c->session_refusals = 0u;
    c->requests_in_epoch = 0u;
    c->run_active = false; /* silently: the new epoch forgets the old run */
    end_run(c, DTC_SNAPSHOT_END_NONE);
    start_run(c, DTC_SNAPSHOT_TRIGGER_KEY_ON);
}

/* Once per epoch, also if refused. While the key-on run is active nothing is used up. */
static bool arm_warm_idle(dtc_snapshot_core_t* c)
{
    bool started = false;
    if (!c->warm_idle_used && !c->run_active) {
        c->warm_idle_used = true;
        if ((c->halt == DTC_SNAPSHOT_HALT_NONE) &&
            (c->runs_started < DTC_SNAPSHOT_RUNS_PER_EPOCH)) {
            start_run(c, DTC_SNAPSHOT_TRIGGER_WARM_IDLE);
            started = true;
        }
    }
    return started;
}

bool dtc_snapshot_core_warm_idle_step(dtc_snapshot_core_t* c, uint32_t now_ms, bool holds)
{
    if (c == NULL) {
        return false;
    }
    if (!holds || !c->epoch_started) {
        c->warm_idle_timing = false; /* any break restarts the hold */
        return false;
    }
    if (!c->warm_idle_timing) {
        c->warm_idle_timing = true;
        c->warm_idle_since_ms = now_ms;
    }
    if (!hold_elapsed(now_ms, c->warm_idle_since_ms, VEHICLE_CL250_DTC_WARM_IDLE_MIN_DURATION_MS)) {
        return false;
    }
    return arm_warm_idle(c);
}

bool dtc_snapshot_core_condition_met(uint8_t op, float value, float threshold)
{
    bool met = false;
    if (isfinite(value) && isfinite(threshold)) {
        switch (op) {
        case VEHICLE_CL250_DTC_OP_LT:
            met = value < threshold;
            break;
        case VEHICLE_CL250_DTC_OP_LE:
            met = value <= threshold;
            break;
        case VEHICLE_CL250_DTC_OP_GT:
            met = value > threshold;
            break;
        case VEHICLE_CL250_DTC_OP_GE:
            met = value >= threshold;
            break;
        case VEHICLE_CL250_DTC_OP_EQ:
            met = value == threshold; /* exact, on purpose: see the glue */
            break;
        default:
            break; /* unknown op: not met */
        }
    }
    return met;
}

bool dtc_snapshot_core_due(const dtc_snapshot_core_t* c)
{
    return (c != NULL) && c->run_active && (c->halt == DTC_SNAPSHOT_HALT_NONE) && !c->in_flight;
}

bool dtc_snapshot_core_build_request(dtc_snapshot_core_t* c, uint8_t* out, uint16_t* len)
{
    if ((len != NULL)) {
        *len = 0u;
    }
    if ((out == NULL) || (len == NULL) || !dtc_snapshot_core_due(c)) {
        return false;
    }
    if (c->requests_in_run >= DTC_SNAPSHOT_MAX_REQUESTS_PER_RUN) {
        end_run(c, DTC_SNAPSHOT_END_BUDGET);
        return false;
    }
    uint16_t n = 0u;
    out[n] = (uint8_t)UDS_SID_READ_DTC_INFORMATION;
    n++;
    out[n] = dtc_snapshot_core_sub_function(c->step);
    n++;
    if (is_record_step(c->step)) {
        for (uint8_t k = 0u; k < DTC_SNAPSHOT_DTC_BYTES; k++) {
            out[n] = c->dtcs[c->dtc_index][k];
            n++;
        }
        out[n] = (c->step == DTC_SNAPSHOT_STEP_SNAPSHOT)
                     ? (uint8_t)VEHICLE_CL250_DTC_SNAPSHOT_RECORD
                     : (uint8_t)VEHICLE_CL250_DTC_EXT_DATA_RECORD;
    } else {
        out[n] = (uint8_t)VEHICLE_CL250_DTC_STATUS_MASK;
    }
    n++;
    *len = n;
    c->in_flight = true;
    c->requests_in_run++;
    c->requests_in_epoch++;
    sat_inc(&c->stats.requests);
    return true;
}

void dtc_snapshot_core_unsend(dtc_snapshot_core_t* c)
{
    if ((c == NULL) || !c->in_flight) {
        return;
    }
    c->in_flight = false;
    c->requests_in_run--; /* build_request() counted it */
    c->requests_in_epoch--;
    if (c->stats.requests < UINT32_MAX) {
        c->stats.requests--; /* saturated stays saturated (may over-count by one) */
    }
}

bool dtc_snapshot_core_match_positive(const dtc_snapshot_core_t* c, const uint8_t* msg,
                                      uint16_t len)
{
    if ((c == NULL) || (msg == NULL) || !c->in_flight || (len < 2u) ||
        (msg[0] != (uint8_t)(UDS_SID_READ_DTC_INFORMATION + UDS_POSITIVE_RESPONSE_OFFSET)) ||
        (msg[1] != dtc_snapshot_core_sub_function(c->step))) {
        return false;
    }
    if (is_record_step(c->step)) {
        if (len < (2u + DTC_SNAPSHOT_DTC_BYTES)) {
            return false;
        }
        for (uint8_t k = 0u; k < DTC_SNAPSHOT_DTC_BYTES; k++) {
            if (msg[2u + k] != c->dtcs[c->dtc_index][k]) {
                return false;
            }
        }
    }
    return true;
}

bool dtc_snapshot_core_expected_prefix(const dtc_snapshot_core_t* c, uint8_t* out, uint8_t* len)
{
    if ((c == NULL) || (out == NULL) || (len == NULL) || !c->in_flight) {
        return false;
    }
    uint8_t n = 0u;
    out[n] = (uint8_t)(UDS_SID_READ_DTC_INFORMATION + UDS_POSITIVE_RESPONSE_OFFSET);
    n++;
    out[n] = dtc_snapshot_core_sub_function(c->step);
    n++;
    if (is_record_step(c->step)) {
        for (uint8_t k = 0u; k < DTC_SNAPSHOT_DTC_BYTES; k++) {
            out[n] = c->dtcs[c->dtc_index][k];
            n++;
        }
    }
    *len = n;
    return true;
}

/* 0x01: [59][01][availability][format][count hi][count lo]. */
static void advance_count(dtc_snapshot_core_t* c, const uint8_t* msg, uint16_t len)
{
    if (len != DTC_SNAPSHOT_COUNT_ANSWER_LEN) {
        end_run(c, DTC_SNAPSHOT_END_MALFORMED);
    } else if ((msg[4] == 0u) && (msg[5] == 0u)) {
        end_run(c, DTC_SNAPSHOT_END_NO_DTC);
    } else {
        c->step = DTC_SNAPSHOT_STEP_LIST;
    }
}

/* 0x02: [59][02][availability] then <DTC 3 bytes><status> records. The DTCs are stored
 * only after the length matched the record layout. */
static void advance_list(dtc_snapshot_core_t* c, const uint8_t* msg, uint16_t len)
{
    if ((len < DTC_SNAPSHOT_LIST_HEADER_LEN) ||
        (((uint16_t)(len - DTC_SNAPSHOT_LIST_HEADER_LEN) % DTC_SNAPSHOT_LIST_RECORD_LEN) != 0u)) {
        end_run(c, DTC_SNAPSHOT_END_MALFORMED);
        return;
    }
    for (uint16_t off = DTC_SNAPSHOT_LIST_HEADER_LEN;
         (off < len) && (c->dtc_count < VEHICLE_CL250_DTC_MAX_PER_RUN);
         off = (uint16_t)(off + DTC_SNAPSHOT_LIST_RECORD_LEN)) {
        /* the ECU filters by the mask; checked again here */
        if ((msg[off + DTC_SNAPSHOT_DTC_BYTES] & VEHICLE_CL250_DTC_STATUS_MASK) != 0u) {
            for (uint8_t k = 0u; k < DTC_SNAPSHOT_DTC_BYTES; k++) {
                c->dtcs[c->dtc_count][k] = msg[off + k];
            }
            c->dtc_count++;
        }
    }
    if (c->dtc_count == 0u) {
        end_run(c, DTC_SNAPSHOT_END_NO_MATCH);
    } else {
        c->dtc_index = 0u;
        c->step = DTC_SNAPSHOT_STEP_SNAPSHOT;
    }
}

/* 0x04 / 0x06: [59][sub][DTC 3][status] and the record bytes, whose layout is not decoded. */
static void advance_record(dtc_snapshot_core_t* c, uint16_t len)
{
    if (len < DTC_SNAPSHOT_RECORD_HEADER_LEN) {
        end_run(c, DTC_SNAPSHOT_END_MALFORMED);
    } else if (c->step == DTC_SNAPSHOT_STEP_SNAPSHOT) {
        c->step = DTC_SNAPSHOT_STEP_EXT_DATA;
    } else {
        c->dtc_index++;
        if (c->dtc_index >= c->dtc_count) {
            end_run(c, DTC_SNAPSHOT_END_DONE);
        } else {
            c->step = DTC_SNAPSHOT_STEP_SNAPSHOT;
        }
    }
}

bool dtc_snapshot_core_on_positive(dtc_snapshot_core_t* c, const uint8_t* msg, uint16_t len)
{
    if (!dtc_snapshot_core_match_positive(c, msg, len)) {
        return false;
    }
    c->in_flight = false;
    sat_inc(&c->stats.answers);
    switch (c->step) {
    case DTC_SNAPSHOT_STEP_COUNT:
        advance_count(c, msg, len);
        break;
    case DTC_SNAPSHOT_STEP_LIST:
        advance_list(c, msg, len);
        break;
    case DTC_SNAPSHOT_STEP_SNAPSHOT:
    case DTC_SNAPSHOT_STEP_EXT_DATA:
        advance_record(c, len);
        break;
    default:
        end_run(c, DTC_SNAPSHOT_END_MALFORMED);
        break;
    }
    settle(c);
    return true;
}

dtc_snapshot_nrc_action_t dtc_snapshot_core_on_nrc(dtc_snapshot_core_t* c, uint8_t nrc)
{
    dtc_snapshot_nrc_action_t action = DTC_SNAPSHOT_NRC_ENDED;
    if ((c == NULL) || !c->in_flight) {
        return action;
    }
    if (nrc == UDS_NRC_RESPONSE_PENDING) {
        return DTC_SNAPSHOT_NRC_WAIT; /* stays in flight; the base timeout ends it */
    }
    c->in_flight = false;
    if ((nrc == UDS_NRC_SERVICE_NOT_SUPPORTED) || (nrc == UDS_NRC_SUBFUNCTION_NOT_SUPPORTED) ||
        (nrc == UDS_NRC_REQUEST_OUT_OF_RANGE)) {
        halt_epoch(c, DTC_SNAPSHOT_HALT_NOT_SERVED);
    } else if ((nrc == UDS_NRC_SERVICE_NOT_SUPPORTED_IN_ACTIVE_SESSION) ||
               (nrc == UDS_NRC_SUBFUNCTION_NOT_SUPPORTED_IN_ACTIVE_SESSION)) {
        /* The same step is due again once the caller has the session back. */
        c->session_refusals++;
        if (c->session_refusals >= DTC_SNAPSHOT_SESSION_REFUSALS_MAX) {
            halt_epoch(c, DTC_SNAPSHOT_HALT_SESSION_REFUSED);
        }
        settle(c);
        action = DTC_SNAPSHOT_NRC_SESSION_LOST;
    } else {
        end_run(c, DTC_SNAPSHOT_END_REFUSED);
    }
    return action;
}

void dtc_snapshot_core_on_timeout(dtc_snapshot_core_t* c)
{
    if ((c != NULL) && c->in_flight) {
        c->in_flight = false;
        halt_epoch(c, DTC_SNAPSHOT_HALT_NO_ANSWER);
    }
}

void dtc_snapshot_core_on_segmented_failure(dtc_snapshot_core_t* c)
{
    if ((c != NULL) && c->in_flight) {
        c->in_flight = false;
        halt_epoch(c, DTC_SNAPSHOT_HALT_SEGMENTED);
    }
}

void dtc_snapshot_core_halt_after_answer(dtc_snapshot_core_t* c, dtc_snapshot_halt_t reason)
{
    if ((c == NULL) || (c->halt != DTC_SNAPSHOT_HALT_NONE) ||
        (reason == DTC_SNAPSHOT_HALT_NONE)) {
        return;
    }
    c->halt = reason;
    sat_inc(&c->stats.halts);
    settle(c);
}

dtc_snapshot_step_t dtc_snapshot_core_step(const dtc_snapshot_core_t* c)
{
    return (c == NULL) ? DTC_SNAPSHOT_STEP_NONE : c->step;
}

dtc_snapshot_trigger_t dtc_snapshot_core_trigger(const dtc_snapshot_core_t* c)
{
    return (c == NULL) ? DTC_SNAPSHOT_TRIGGER_KEY_ON : c->trigger;
}

uint8_t dtc_snapshot_core_runs_started(const dtc_snapshot_core_t* c)
{
    return (c == NULL) ? 0u : c->runs_started;
}
