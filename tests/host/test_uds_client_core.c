/*
 * L0 tests for the UDS client core (features/uds/uds_client_core, Ç3): the CL250
 * vehicle poller state machine, pure, with a manual time value. Every limit comes from
 * gen/vehicle_cl250.h; boundaries are tested at limit - 1 and at the limit.
 *
 * Every request the core produces is checked against the generated D-020 request gate
 * (vehicle_cl250_request_allowed) and must be one of: extended session, tester present,
 * or 0x22 for a DID of the gen/ table.
 * Section numbers refer to ISO 14229-1:2020. No requirement IDs yet (Q-006).
 */
#include "features/uds/uds_client_core.h"
#include "uds_iso14229.h"
#include "vehicle_cl250.h"

#include <string.h>
#include <unity.h>

#define POS_OFFSET UDS_POSITIVE_RESPONSE_OFFSET
#define HOLD_MS ISOTP_DEFAULT_N_BS_MS  /* the vehicle link's N_Bs (ISO default) */
#define LOG_MAX 4096u

typedef enum { K_NONE = 0, K_SESSION, K_TP, K_READ } kind_t;

static uds_client_core_t c;
static uint8_t req[UDS_CLIENT_REQ_MAX];
static uint16_t req_len;
static uds_client_sample_t smp;

typedef struct {
    uint32_t t;
    kind_t k;
    uint32_t idx;
} log_entry_t;
static log_entry_t log_buf[LOG_MAX];
static uint32_t log_n;
static uint32_t silent_mask; /* DIDs (bit per index) the fake ECU leaves unanswered */
static bool ecu_silent;      /* the fake ECU answers nothing */

void setUp(void)
{
    memset(&c, 0, sizeof c);
    memset(&smp, 0, sizeof smp);
    log_n = 0u;
    silent_mask = 0u;
    ecu_silent = false;
}

void tearDown(void) {}

static const uds_client_stats_t* stats(void)
{
    return uds_client_core_stats(&c);
}

static uint32_t did_index(uint16_t did)
{
    for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
        if (vehicle_cl250_dids[i].did == did) {
            return i;
        }
    }
    TEST_FAIL_MESSAGE("request for a DID outside the gen/ table");
    return 0u;
}

static kind_t classify(void)
{
    if ((req_len == 2u) && (req[0] == VEHICLE_CL250_SESSION_SID) &&
        (req[1] == VEHICLE_CL250_SESSION_SUBFUNCTION)) {
        return K_SESSION;
    }
    if ((req_len == 2u) && (req[0] == VEHICLE_CL250_TESTER_PRESENT_SID) &&
        (req[1] == VEHICLE_CL250_TESTER_PRESENT_SUBFUNCTION)) {
        return K_TP;
    }
    if ((req_len == 3u) && (req[0] == UDS_SID_READ_DATA_BY_IDENTIFIER)) {
        (void)did_index((uint16_t)(((uint16_t)req[1] << 8u) | req[2]));
        return K_READ;
    }
    TEST_FAIL_MESSAGE("the core produced a request that is not session, TP or a gen/ DID read");
    return K_NONE;
}

static kind_t poll_full(uint32_t t, bool busy, bool tx_ready)
{
    req_len = 0xFFFFu;
    if (!uds_client_core_poll(&c, t, busy, tx_ready, req, &req_len)) {
        TEST_ASSERT_EQUAL_UINT16(0u, req_len);
        return K_NONE;
    }
    TEST_ASSERT_TRUE_MESSAGE(vehicle_cl250_request_allowed(req, req_len), "D-020 gate refused");
    return classify();
}

static kind_t poll_busy(uint32_t t, bool busy)
{
    return poll_full(t, busy, true);
}

static kind_t poll_at(uint32_t t)
{
    return poll_busy(t, false);
}

static uint32_t req_idx(void)
{
    return did_index((uint16_t)(((uint16_t)req[1] << 8u) | req[2]));
}

static bool indicate(uint32_t t, const uint8_t* d, uint16_t n)
{
    return uds_client_core_on_indication(&c, t, ISOTP_N_OK, d, n, &smp);
}

static void answer_session(uint32_t t)
{
    const uint8_t d[6] = {VEHICLE_CL250_SESSION_POSITIVE_SID, VEHICLE_CL250_SESSION_SUBFUNCTION,
                          0x00u, 0x32u, 0x01u, 0xF4u};
    TEST_ASSERT_FALSE(indicate(t, d, 6u));
}

static bool answer_read(uint32_t t, uint32_t idx, uint32_t raw)
{
    const vehicle_cl250_did_t* e = &vehicle_cl250_dids[idx];
    uint8_t d[8];
    d[0] = (uint8_t)(UDS_SID_READ_DATA_BY_IDENTIFIER + POS_OFFSET);
    d[1] = (uint8_t)(e->did >> 8u);
    d[2] = (uint8_t)(e->did & 0xFFu);
    for (uint8_t i = 0u; i < e->length; i++) {
        d[3u + i] = (uint8_t)(raw >> (8u * (e->length - 1u - i)));
    }
    return indicate(t, d, (uint16_t)(3u + e->length));
}

static void nrc(uint32_t t, uint8_t sid, uint8_t code)
{
    const uint8_t d[3] = {UDS_SID_NEGATIVE_RESPONSE, sid, code};
    TEST_ASSERT_FALSE(indicate(t, d, 3u));
}

/* Init at t, extended session confirmed at t; the first tester present is out too. */
static void bring_up(uint32_t t)
{
    uds_client_core_init(&c, HOLD_MS);
    TEST_ASSERT_EQUAL(K_SESSION, poll_at(t));
    answer_session(t);
    TEST_ASSERT_TRUE(uds_client_core_session_up(&c));
    TEST_ASSERT_EQUAL(K_TP, poll_at(t));
}

/* Polls every ms in [from, to) and plays a fake ECU that answers at once, unless it is
 * silent (all, or for the DIDs in silent_mask). Every request is logged. */
static void drive(uint32_t from, uint32_t to)
{
    for (uint32_t t = from; t != to; t++) {
        kind_t k;
        while ((k = poll_at(t)) != K_NONE) {
            const uint32_t idx = (k == K_READ) ? req_idx() : 0u;
            if (log_n < LOG_MAX) {
                log_buf[log_n].t = t;
                log_buf[log_n].k = k;
                log_buf[log_n].idx = idx;
                log_n++;
            }
            if (ecu_silent) {
                continue;
            }
            if (k == K_SESSION) {
                answer_session(t);
            } else if ((k == K_READ) && ((silent_mask & (1u << idx)) == 0u)) {
                TEST_ASSERT_TRUE(answer_read(t, idx, 1u));
            } else {
                /* tester present: response suppressed */
            }
        }
    }
}

static uint32_t count_kind(kind_t k, uint32_t from_t, uint32_t to_t)
{
    uint32_t n = 0u;
    for (uint32_t i = 0u; i < log_n; i++) {
        if ((log_buf[i].k == k) && (log_buf[i].t >= from_t) && (log_buf[i].t < to_t)) {
            n++;
        }
    }
    return n;
}

static uint32_t count_did_reads(uint32_t idx, uint32_t from_t, uint32_t to_t)
{
    uint32_t n = 0u;
    for (uint32_t i = 0u; i < log_n; i++) {
        if ((log_buf[i].k == K_READ) && (log_buf[i].idx == idx) && (log_buf[i].t >= from_t) &&
            (log_buf[i].t < to_t)) {
            n++;
        }
    }
    return n;
}

/* Between two session attempts only tester present may go out. */
static void assert_no_session_or_read(uint32_t from, uint32_t to)
{
    for (uint32_t t = from; t < to; t++) {
        const kind_t k = poll_at(t);
        TEST_ASSERT_TRUE_MESSAGE((k == K_NONE) || (k == K_TP), "only TP before the retry");
    }
}

/* ------------------------------------------------------------------------- */
/* Session (0x10 03) and tester present (0x3E 80), D-020 / gen/ values        */
/* ------------------------------------------------------------------------- */

static void test_first_request_is_the_extended_session_from_gen(void)
{
    uds_client_core_init(&c, HOLD_MS);
    TEST_ASSERT_EQUAL(K_SESSION, poll_at(0u));
    TEST_ASSERT_EQUAL_UINT16(2u, req_len);
    TEST_ASSERT_EQUAL_HEX8(VEHICLE_CL250_SESSION_SID, req[0]);
    TEST_ASSERT_EQUAL_HEX8(VEHICLE_CL250_SESSION_SUBFUNCTION, req[1]);
    TEST_ASSERT_EQUAL(UDS_CLIENT_REQ_SESSION, uds_client_core_pending(&c));
    TEST_ASSERT_FALSE(uds_client_core_session_up(&c));
    TEST_ASSERT_EQUAL(K_NONE, poll_at(0u)); /* one request in flight */
}

static void test_session_is_retried_at_the_gen_interval_until_positive(void)
{
    const uint32_t retry = VEHICLE_CL250_SESSION_RETRY_INTERVAL_MS;
    const uint32_t base = VEHICLE_CL250_RESPONSE_TIMEOUT_BASE_MS;
    uds_client_core_init(&c, HOLD_MS);
    TEST_ASSERT_EQUAL(K_SESSION, poll_at(0u));
    TEST_ASSERT_EQUAL(K_NONE, poll_at(base - 1u));
    /* No answer: the session request times out, and tester present goes out (legacy:
     * unconditionally, also before the session is confirmed). No DID read yet. */
    TEST_ASSERT_EQUAL(K_TP, poll_at(base));
    TEST_ASSERT_EQUAL_UINT32(1u, stats()->timeouts);
    assert_no_session_or_read(base + 1u, retry);
    TEST_ASSERT_EQUAL(K_SESSION, poll_at(retry));
    answer_session(retry + 5u);
    TEST_ASSERT_TRUE(uds_client_core_session_up(&c));
    TEST_ASSERT_EQUAL_UINT32(1u, stats()->session_starts);
    /* Session up: DID reads start at once, the high-priority one first (D-043). TP stays
     * on its period. */
    TEST_ASSERT_EQUAL(K_READ, poll_at(retry + 5u));
    TEST_ASSERT_EQUAL_UINT32(VEHICLE_CL250_IDX_VEHICLE_SPEED, req_idx());
}

static void test_no_did_is_read_while_the_session_is_down(void)
{
    uds_client_core_init(&c, HOLD_MS);
    ecu_silent = true;
    drive(0u, 10000u);
    TEST_ASSERT_EQUAL_UINT32(0u, count_kind(K_READ, 0u, 10000u));
    /* Retries every SESSION_RETRY_INTERVAL_MS: 0, 2000, 4000, 6000, 8000. */
    TEST_ASSERT_EQUAL_UINT32(10000u / VEHICLE_CL250_SESSION_RETRY_INTERVAL_MS,
                             count_kind(K_SESSION, 0u, 10000u));
    TEST_ASSERT_FALSE(uds_client_core_ecu_present(&c, 10000u));
}

static void test_session_nrc_keeps_it_down_until_the_next_retry(void)
{
    uds_client_core_init(&c, HOLD_MS);
    TEST_ASSERT_EQUAL(K_SESSION, poll_at(0u));
    nrc(3u, VEHICLE_CL250_SESSION_SID, 0x22u); /* conditionsNotCorrect */
    TEST_ASSERT_EQUAL(UDS_CLIENT_REQ_NONE, uds_client_core_pending(&c));
    TEST_ASSERT_FALSE(uds_client_core_session_up(&c));
    TEST_ASSERT_EQUAL_UINT32(0u, stats()->timeouts);
    TEST_ASSERT_TRUE(uds_client_core_ecu_present(&c, 3u)); /* any NRC proves the ECU */
    TEST_ASSERT_EQUAL(K_TP, poll_at(3u));
    assert_no_session_or_read(4u, VEHICLE_CL250_SESSION_RETRY_INTERVAL_MS);
    TEST_ASSERT_EQUAL(K_SESSION, poll_at(VEHICLE_CL250_SESSION_RETRY_INTERVAL_MS));
}

static void test_session_answer_with_another_subfunction_is_not_accepted(void)
{
    uds_client_core_init(&c, HOLD_MS);
    TEST_ASSERT_EQUAL(K_SESSION, poll_at(0u));
    const uint8_t d[2] = {VEHICLE_CL250_SESSION_POSITIVE_SID, 0x01u};
    TEST_ASSERT_FALSE(indicate(1u, d, 2u));
    const uint8_t short_d[1] = {VEHICLE_CL250_SESSION_POSITIVE_SID};
    TEST_ASSERT_FALSE(indicate(1u, short_d, 1u));
    TEST_ASSERT_FALSE(uds_client_core_session_up(&c));
    TEST_ASSERT_EQUAL(UDS_CLIENT_REQ_SESSION, uds_client_core_pending(&c));
    TEST_ASSERT_EQUAL_UINT32(2u, stats()->unexpected);
}

static void test_session_response_pending_extends_the_session_request(void)
{
    uds_client_core_init(&c, HOLD_MS);
    TEST_ASSERT_EQUAL(K_SESSION, poll_at(0u));
    nrc(90u, VEHICLE_CL250_SESSION_SID, UDS_NRC_RESPONSE_PENDING);
    TEST_ASSERT_EQUAL(K_NONE, poll_at(150u));
    TEST_ASSERT_EQUAL(UDS_CLIENT_REQ_SESSION, uds_client_core_pending(&c));
    answer_session(200u);
    TEST_ASSERT_TRUE(uds_client_core_session_up(&c));
    TEST_ASSERT_EQUAL_UINT32(0u, stats()->timeouts);
}

static void test_tester_present_every_gen_period_without_holding_the_slot(void)
{
    const uint32_t tp = VEHICLE_CL250_TESTER_PRESENT_PERIOD_MS;
    bring_up(0u);
    /* TP never becomes the pending request: the read goes out right after it. */
    TEST_ASSERT_EQUAL(UDS_CLIENT_REQ_NONE, uds_client_core_pending(&c));
    TEST_ASSERT_EQUAL(K_READ, poll_at(0u));
    TEST_ASSERT_TRUE(answer_read(0u, req_idx(), 1u));
    drive(1u, 10u * tp + 1u);
    TEST_ASSERT_EQUAL_UINT32(10u, count_kind(K_TP, 1u, 10u * tp + 1u));
    uint32_t last = 0u;
    for (uint32_t i = 0u; i < log_n; i++) {
        if (log_buf[i].k == K_TP) {
            TEST_ASSERT_EQUAL_UINT32(tp, log_buf[i].t - last); /* exactly on period */
            last = log_buf[i].t;
        }
    }
}

/* ------------------------------------------------------------------------- */
/* DID polling                                                                */
/* ------------------------------------------------------------------------- */

/* D-043 order: the lowest gen/ priority value first, then table order. */
static uint32_t expected_order(uint32_t* order)
{
    uint32_t n = 0u;
    for (uint8_t prio = 0u; prio <= VEHICLE_CL250_PRIORITY_NORMAL; prio++) {
        for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
            if (vehicle_cl250_dids[i].priority == prio) {
                order[n] = i;
                n++;
            }
        }
    }
    return n;
}

static void test_speed_is_the_only_high_priority_did_in_gen(void)
{
    /* D-048 item 4: 0xF40D high, every other DID normal. */
    for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
        TEST_ASSERT_EQUAL_UINT8((i == VEHICLE_CL250_IDX_VEHICLE_SPEED) ? VEHICLE_CL250_PRIORITY_HIGH
                                                                       : VEHICLE_CL250_PRIORITY_NORMAL,
                                vehicle_cl250_dids[i].priority);
    }
}

static void test_reads_are_priority_first_then_table_order_when_all_are_due(void)
{
    uint32_t order[VEHICLE_CL250_DID_COUNT];
    TEST_ASSERT_EQUAL_UINT32(VEHICLE_CL250_DID_COUNT, expected_order(order));
    TEST_ASSERT_EQUAL_UINT32(VEHICLE_CL250_IDX_VEHICLE_SPEED, order[0]);
    bring_up(0u);
    for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
        TEST_ASSERT_EQUAL(K_READ, poll_at(0u));
        TEST_ASSERT_EQUAL_UINT32(order[i], req_idx());
        TEST_ASSERT_EQUAL(UDS_CLIENT_REQ_READ, uds_client_core_pending(&c));
        TEST_ASSERT_TRUE(answer_read(0u, order[i], 1u));
    }
    TEST_ASSERT_EQUAL(K_NONE, poll_at(0u)); /* nothing due until the first period ends */
}

/* Marks every DID as requested at t, so none is due before its period. */
static void all_requested_at(uint32_t t)
{
    for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
        c.did[i].requested = true;
        c.did[i].last_request_ms = t;
    }
}

/* The next request at t that is not tester present. */
static kind_t poll_skip_tp(uint32_t t)
{
    kind_t k;
    do {
        k = poll_at(t);
    } while (k == K_TP);
    return k;
}

static void test_a_just_due_high_did_goes_before_long_overdue_normal_ones(void)
{
    const uint32_t speed = VEHICLE_CL250_IDX_VEHICLE_SPEED;
    const uint32_t period = vehicle_cl250_dids[speed].poll_period_ms;
    const uint32_t t0 = 1000u; /* < ECU_ABSENT_TIMEOUT_MS: the session stays up */
    bring_up(0u);
    all_requested_at(0u); /* the normal DIDs are overdue long before speed is due */
    c.did[speed].last_request_ms = t0;
    TEST_ASSERT_EQUAL(K_READ, poll_skip_tp(t0));
    TEST_ASSERT_TRUE(req_idx() != speed); /* not due yet: a normal one goes */
    TEST_ASSERT_TRUE(answer_read(t0, req_idx(), 1u));
    all_requested_at(0u);
    c.did[speed].last_request_ms = t0;
    TEST_ASSERT_EQUAL(K_READ, poll_skip_tp(t0 + period));
    TEST_ASSERT_EQUAL_UINT32(speed, req_idx());
}

static void test_within_a_class_the_first_due_in_table_order_goes(void)
{
    bring_up(0u);
    all_requested_at(0u);
    /* Two normal DIDs due, the later table entry overdue for longer: table order wins. */
    const uint32_t a = VEHICLE_CL250_IDX_ENGINE_SPEED;
    const uint32_t b = VEHICLE_CL250_IDX_BATTERY_VOLTAGE;
    TEST_ASSERT_LESS_THAN_UINT32(b, a);
    c.did[a].requested = false;
    c.did[b].requested = false;
    c.did[b].last_request_ms = 0u;
    TEST_ASSERT_EQUAL(K_READ, poll_at(1u));
    TEST_ASSERT_EQUAL_UINT32(a, req_idx());
    TEST_ASSERT_TRUE(answer_read(1u, a, 1u));
    TEST_ASSERT_EQUAL(K_READ, poll_at(1u));
    TEST_ASSERT_EQUAL_UINT32(b, req_idx());
}

/*
 * The defs codegen bound (yaml_checks.did_sample_gap_bounds, D-043), recomputed from
 * gen/: non-preemptive fixed priority in the poller's order, one request in flight,
 * each holding the slot for C = ASSUMED_ROUND_TRIP_MS.
 *   w = C + sum over the DIDs ordered before idx of (w / P_j + 1) * C
 *   worst sample gap = P_idx + w + C
 */
static uint32_t gap_bound(uint32_t idx)
{
    const uint32_t rtt = VEHICLE_CL250_ASSUMED_ROUND_TRIP_MS;
    const vehicle_cl250_did_t* e = &vehicle_cl250_dids[idx];
    uint32_t w = rtt;
    for (uint32_t iter = 0u; iter < 1000u; iter++) {
        uint32_t next = rtt;
        for (uint32_t j = 0u; j < VEHICLE_CL250_DID_COUNT; j++) {
            const vehicle_cl250_did_t* h = &vehicle_cl250_dids[j];
            if ((h->priority < e->priority) || ((h->priority == e->priority) && (j < idx))) {
                next += ((w / h->poll_period_ms) + 1u) * rtt;
            }
        }
        if (next == w) {
            return (uint32_t)e->poll_period_ms + w + rtt;
        }
        w = next;
    }
    TEST_FAIL_MESSAGE("bound did not converge");
    return 0u;
}

static void test_the_gen_table_meets_its_gap_bound(void)
{
    for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
        TEST_ASSERT_LESS_OR_EQUAL_UINT32(vehicle_cl250_dids[i].stale_after_ms, gap_bound(i));
    }
}

static uint32_t rng_state;
static uint32_t rng_next(void)
{
    rng_state = (rng_state * 1103515245u) + 12345u;
    return rng_state >> 8u;
}

/*
 * D-043: no starvation. A fake ECU answers every read after a round trip in
 * [rtt_min, rtt_max] (seeded); one request is in flight. Every DID's sample gap must
 * stay within the analytical bound (<= its stale_after_ms). With random_phase, each DID
 * starts as if requested at a random time within its last period (and answered then).
 */
static void run_with_round_trip(uint32_t rtt_min, uint32_t rtt_max, bool random_phase,
                                uint32_t duration_ms, uint32_t step_ms)
{
    const uint32_t t0 = 1000u;
    uint32_t last_sample[VEHICLE_CL250_DID_COUNT];
    uint32_t bound[VEHICLE_CL250_DID_COUNT];
    bool answer_due = false;
    uint32_t answer_t = 0u;
    uint32_t answer_idx = 0u;
    uint32_t request_t = 0u;
    bring_up(t0);
    for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
        bound[i] = gap_bound(i);
        last_sample[i] = t0;
        if (random_phase) {
            last_sample[i] = t0 - (rng_next() % vehicle_cl250_dids[i].poll_period_ms);
            c.did[i].requested = true;
            c.did[i].last_request_ms = last_sample[i];
        }
    }
    for (uint32_t t = t0; t < (t0 + duration_ms); t++) {
        if (((t - t0) % step_ms) != 0u) {
            /* between two steps of the poller: an answer waits for the next one */
        } else {
            if (answer_due && (t >= answer_t)) {
                TEST_ASSERT_TRUE(answer_read(t, answer_idx, 1u));
                TEST_ASSERT_EQUAL_UINT32(answer_idx, smp.idx);
                TEST_ASSERT_LESS_OR_EQUAL_UINT32_MESSAGE(VEHICLE_CL250_ASSUMED_ROUND_TRIP_MS,
                                                         t - request_t,
                                                         "held the slot past the model's C");
                last_sample[answer_idx] = t;
                answer_due = false;
            }
            kind_t k;
            while ((k = poll_at(t)) != K_NONE) {
                TEST_ASSERT_TRUE_MESSAGE(k != K_SESSION, "session lost");
                if (k == K_READ) {
                    TEST_ASSERT_FALSE_MESSAGE(answer_due, "two requests in flight");
                    answer_due = true;
                    answer_idx = req_idx();
                    request_t = t;
                    answer_t = t + rtt_min + (rng_next() % (rtt_max - rtt_min + 1u));
                }
            }
        }
        for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
            TEST_ASSERT_LESS_OR_EQUAL_UINT32_MESSAGE(bound[i], t - last_sample[i],
                                                     "a DID exceeded its D-043 gap bound");
        }
    }
    TEST_ASSERT_EQUAL_UINT32(0u, stats()->timeouts);
}

static void test_no_did_starves_with_answers_after_the_gen_assumed_round_trip(void)
{
    const uint32_t rtt = VEHICLE_CL250_ASSUMED_ROUND_TRIP_MS;
    run_with_round_trip(rtt, rtt, false, 20000u, 1u);
}

static void test_no_did_starves_with_answers_after_one_ms(void)
{
    run_with_round_trip(1u, 1u, false, 20000u, 1u);
}

/*
 * ASSUMED_ROUND_TRIP_MS = the ECU's answer + one uds_client_step() period (E-6 n4): with
 * a 10 ms step, an answer that arrives just after a step (round trip ASSUMED - 10 + 1)
 * is seen one step later and holds the slot for exactly ASSUMED_ROUND_TRIP_MS.
 */
static void test_no_did_starves_when_the_answer_is_seen_one_step_later(void)
{
    const uint32_t step = 10u;
    const uint32_t ecu_rtt = VEHICLE_CL250_ASSUMED_ROUND_TRIP_MS - step + 1u;
    for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
        TEST_ASSERT_EQUAL_UINT32(0u, vehicle_cl250_dids[i].poll_period_ms % step);
    }
    run_with_round_trip(ecu_rtt, ecu_rtt, false, 20000u, step);
    for (uint32_t seed = 1u; seed <= 20u; seed++) {
        setUp();
        rng_state = seed;
        run_with_round_trip(1u, ecu_rtt, true, 5000u, step);
    }
}

static void test_no_did_starves_with_random_phases_and_round_trips(void)
{
    for (uint32_t seed = 1u; seed <= 40u; seed++) {
        setUp();
        rng_state = seed;
        run_with_round_trip(1u, VEHICLE_CL250_ASSUMED_ROUND_TRIP_MS, true, 5000u, 1u);
    }
}

static void test_each_did_is_polled_at_its_gen_period(void)
{
    bring_up(0u);
    drive(0u, 10000u);
    for (uint32_t idx = 0u; idx < VEHICLE_CL250_DID_COUNT; idx++) {
        const uint32_t period = vehicle_cl250_dids[idx].poll_period_ms;
        uint32_t n = 0u;
        uint32_t last = 0u;
        for (uint32_t i = 0u; i < log_n; i++) {
            if ((log_buf[i].k == K_READ) && (log_buf[i].idx == idx)) {
                if (n > 0u) {
                    TEST_ASSERT_EQUAL_UINT32(period, log_buf[i].t - last);
                }
                last = log_buf[i].t;
                n++;
            }
        }
        TEST_ASSERT_EQUAL_UINT32((10000u + period - 1u) / period, n);
    }
    TEST_ASSERT_EQUAL_UINT32(0u, stats()->timeouts);
}

static void test_positive_response_gives_raw_and_physical_from_gen(void)
{
    struct {
        uint32_t idx;
        uint32_t raw;
        float physical;
    } cases[] = {
        {VEHICLE_CL250_IDX_ENGINE_SPEED, 0x1F40u, 2000.0f},  /* (A*256+B)/4 */
        {VEHICLE_CL250_IDX_VEHICLE_SPEED, 0x00u, 0.0f},      /* A, lower bound */
        {VEHICLE_CL250_IDX_THROTTLE_POS, 0xFFu, 100.0f},     /* A*100/255, upper bound */
        {VEHICLE_CL250_IDX_COOLANT_TEMP, 0x00u, -40.0f},     /* A-40, lower bound */
        {VEHICLE_CL250_IDX_BATTERY_VOLTAGE, 0xFFFFu, 65.535f}, /* (A*256+B)/1000, max */
    };
    uint32_t order[VEHICLE_CL250_DID_COUNT];
    TEST_ASSERT_EQUAL_UINT32(sizeof cases / sizeof cases[0], expected_order(order));
    bring_up(0u);
    for (uint32_t n = 0u; n < VEHICLE_CL250_DID_COUNT; n++) {
        const uint32_t i = order[n];
        TEST_ASSERT_EQUAL_UINT32(i, cases[i].idx); /* cases are in table order */
        TEST_ASSERT_EQUAL(K_READ, poll_at(0u));
        TEST_ASSERT_EQUAL_UINT32(cases[i].idx, req_idx());
        TEST_ASSERT_TRUE(answer_read(0u, cases[i].idx, cases[i].raw));
        TEST_ASSERT_EQUAL_UINT32(cases[i].idx, smp.idx);
        TEST_ASSERT_EQUAL_UINT32(cases[i].raw, smp.raw);
        TEST_ASSERT_FLOAT_WITHIN(0.001f, cases[i].physical, smp.physical);
    }
    TEST_ASSERT_EQUAL_UINT32(VEHICLE_CL250_DID_COUNT, stats()->reads_ok);
}

static void test_response_for_another_did_or_malformed_is_ignored(void)
{
    bring_up(0u);
    TEST_ASSERT_EQUAL(K_READ, poll_at(0u));
    TEST_ASSERT_EQUAL_UINT32(VEHICLE_CL250_IDX_VEHICLE_SPEED, req_idx());
    TEST_ASSERT_FALSE(answer_read(1u, VEHICLE_CL250_IDX_ENGINE_SPEED, 1u)); /* late/other */
    const uint8_t too_short[3] = {(uint8_t)(UDS_SID_READ_DATA_BY_IDENTIFIER + POS_OFFSET),
                                  (uint8_t)(VEHICLE_CL250_DID_VEHICLE_SPEED >> 8u),
                                  (uint8_t)(VEHICLE_CL250_DID_VEHICLE_SPEED & 0xFFu)};
    TEST_ASSERT_FALSE(indicate(1u, too_short, 3u));
    const uint8_t too_long[8] = {0};
    TEST_ASSERT_FALSE(indicate(1u, too_long, 8u)); /* more than a Single Frame */
    TEST_ASSERT_FALSE(indicate(1u, NULL, 0u));
    TEST_ASSERT_EQUAL_UINT32(4u, stats()->unexpected);
    TEST_ASSERT_EQUAL(UDS_CLIENT_REQ_READ, uds_client_core_pending(&c));
    TEST_ASSERT_EQUAL(K_NONE, poll_at(VEHICLE_CL250_RESPONSE_TIMEOUT_BASE_MS - 1u));
    TEST_ASSERT_EQUAL(K_READ, poll_at(VEHICLE_CL250_RESPONSE_TIMEOUT_BASE_MS));
    TEST_ASSERT_EQUAL_UINT32(1u, stats()->timeouts);
    TEST_ASSERT_EQUAL_UINT32(0u, stats()->reads_ok);
}

/* ------------------------------------------------------------------------- */
/* Response timeout, NRC 0x78, other NRCs                                     */
/* ------------------------------------------------------------------------- */

static void test_response_timeout_is_the_gen_base(void)
{
    const uint32_t base = VEHICLE_CL250_RESPONSE_TIMEOUT_BASE_MS;
    bring_up(0u);
    TEST_ASSERT_EQUAL(K_READ, poll_at(10u));
    TEST_ASSERT_EQUAL_UINT32(VEHICLE_CL250_IDX_VEHICLE_SPEED, req_idx());
    TEST_ASSERT_EQUAL(K_NONE, poll_at(10u + base - 1u));
    TEST_ASSERT_EQUAL_UINT32(0u, stats()->timeouts);
    /* 0xF40D timed out: its period restarts, so the next DID goes at once. */
    TEST_ASSERT_EQUAL(K_READ, poll_at(10u + base));
    TEST_ASSERT_EQUAL_UINT32(1u, stats()->timeouts);
    TEST_ASSERT_EQUAL_UINT32(VEHICLE_CL250_IDX_ENGINE_SPEED, req_idx());
}

/*
 * A silent 0xF40D (period = base timeout) must not hold the slot until it is skipped:
 * after each timeout its period restarts, and the normal DIDs are read in between.
 */
static void test_silent_speed_restarts_its_period_and_the_normal_dids_are_still_read(void)
{
    const uint32_t speed = VEHICLE_CL250_IDX_VEHICLE_SPEED;
    const uint32_t base = VEHICLE_CL250_RESPONSE_TIMEOUT_BASE_MS;
    const uint32_t period = vehicle_cl250_dids[speed].poll_period_ms;
    bring_up(0u);
    silent_mask = 1u << speed;
    uint32_t t = 0u;
    while (stats()->did_skips == 0u) {
        drive(t, t + 1u);
        t++;
        TEST_ASSERT_LESS_THAN_UINT32(5000u, t);
    }
    TEST_ASSERT_EQUAL_UINT32(VEHICLE_CL250_MAX_CONSECUTIVE_TIMEOUTS, stats()->timeouts);
    uint32_t prev = 0u;
    uint32_t n = 0u;
    for (uint32_t i = 0u; i < log_n; i++) {
        if ((log_buf[i].k != K_READ) || (log_buf[i].idx != speed)) {
            continue;
        }
        if (n > 0u) {
            /* timed out at prev + base, due again one period later */
            TEST_ASSERT_EQUAL_UINT32(prev + base + period, log_buf[i].t);
            for (uint32_t j = 0u; j < VEHICLE_CL250_DID_COUNT; j++) {
                /* every normal DID with a period up to the gap was read in it */
                if ((j != speed) && (vehicle_cl250_dids[j].poll_period_ms <= period)) {
                    TEST_ASSERT_GREATER_THAN_UINT32(0u, count_did_reads(j, prev + base,
                                                                        log_buf[i].t));
                }
            }
        }
        prev = log_buf[i].t;
        n++;
    }
    TEST_ASSERT_EQUAL_UINT32(VEHICLE_CL250_MAX_CONSECUTIVE_TIMEOUTS, n);
    TEST_ASSERT_TRUE(uds_client_core_did_skipped(&c, speed, t));
    TEST_ASSERT_TRUE(uds_client_core_session_up(&c));
}

/*
 * 0xF40D answering NRC 0x78 forever holds the single slot up to RESPONSE_TIMEOUT_MAX_MS
 * (nothing else is read meanwhile; their samples age to STALE in vehicle_signals).
 * After the cap the normal DIDs go first; 0xF40D is due again one period after it.
 */
static void test_endless_0x78_on_speed_holds_the_slot_up_to_the_cap_then_the_others_go(void)
{
    const uint32_t speed = VEHICLE_CL250_IDX_VEHICLE_SPEED;
    const uint32_t max = VEHICLE_CL250_RESPONSE_TIMEOUT_MAX_MS;
    const uint32_t period = vehicle_cl250_dids[speed].poll_period_ms;
    bring_up(0u);
    TEST_ASSERT_EQUAL(K_READ, poll_at(0u));
    TEST_ASSERT_EQUAL_UINT32(speed, req_idx());
    for (uint32_t t = 50u; t < max; t += 50u) {
        nrc(t, UDS_SID_READ_DATA_BY_IDENTIFIER, UDS_NRC_RESPONSE_PENDING);
        TEST_ASSERT_EQUAL(K_NONE, poll_skip_tp(t));
    }
    TEST_ASSERT_EQUAL(K_NONE, poll_skip_tp(max - 1u));
    TEST_ASSERT_EQUAL(K_READ, poll_skip_tp(max)); /* capped: a normal DID at once */
    TEST_ASSERT_EQUAL_UINT32(1u, stats()->timeouts);
    TEST_ASSERT_TRUE(req_idx() != speed);
    TEST_ASSERT_TRUE(answer_read(max, req_idx(), 1u));
    drive(max, max + period);
    TEST_ASSERT_EQUAL_UINT32(0u, count_did_reads(speed, max, max + period));
    drive(max + period, max + period + 1u);
    TEST_ASSERT_EQUAL_UINT32(1u, count_did_reads(speed, max + period, max + period + 1u));
}

/* ------------------------------------------------------------------------- */
/* D-050 (E-5): fault-mode fairness                                           */
/* ------------------------------------------------------------------------- */

static void test_a_timed_out_high_did_competes_in_the_normal_class_until_it_answers(void)
{
    const uint32_t speed = VEHICLE_CL250_IDX_VEHICLE_SPEED;
    const uint32_t rpm = VEHICLE_CL250_IDX_ENGINE_SPEED; /* first normal DID in table order */
    TEST_ASSERT_LESS_THAN_UINT32(speed, rpm);
    bring_up(0u);
    all_requested_at(0u);
    c.did[speed].consecutive_timeouts = 1u;
    c.did[speed].requested = false;
    c.did[rpm].requested = false; /* both due */
    TEST_ASSERT_EQUAL(K_READ, poll_at(0u));
    TEST_ASSERT_EQUAL_UINT32(rpm, req_idx()); /* faulty speed: normal class, table order */
    TEST_ASSERT_TRUE(answer_read(1u, rpm, 1u));
    TEST_ASSERT_EQUAL(K_READ, poll_at(1u));
    TEST_ASSERT_EQUAL_UINT32(speed, req_idx());
    TEST_ASSERT_TRUE(answer_read(2u, speed, 1u)); /* answered: gen/ priority again */
    TEST_ASSERT_EQUAL_UINT8(0u, c.did[speed].consecutive_timeouts);
    all_requested_at(0u);
    c.did[speed].requested = false;
    c.did[rpm].requested = false;
    TEST_ASSERT_EQUAL(K_READ, poll_at(2u));
    TEST_ASSERT_EQUAL_UINT32(speed, req_idx());
}

static void test_a_skip_gives_a_faulty_did_its_gen_priority_back(void)
{
    const uint32_t speed = VEHICLE_CL250_IDX_VEHICLE_SPEED;
    const uint32_t base = VEHICLE_CL250_RESPONSE_TIMEOUT_BASE_MS;
    bring_up(0u);
    all_requested_at(0u);
    c.did[speed].consecutive_timeouts = (uint8_t)(VEHICLE_CL250_MAX_CONSECUTIVE_TIMEOUTS - 1u);
    c.did[speed].requested = false;
    TEST_ASSERT_EQUAL(K_READ, poll_at(0u));
    TEST_ASSERT_EQUAL_UINT32(speed, req_idx());
    (void)poll_skip_tp(base); /* the last timeout skips it */
    TEST_ASSERT_EQUAL_UINT32(1u, stats()->did_skips);
    TEST_ASSERT_EQUAL_UINT8(0u, c.did[speed].consecutive_timeouts);
    /* After the cooldown it goes before a normal DID that is due at the same time. */
    const uint32_t back = base + VEHICLE_CL250_DID_SKIP_COOLDOWN_MS;
    c.last_response_ms = back; /* the other DIDs answered meanwhile: the ECU is present */
    all_requested_at(back);
    c.did[speed].requested = false;
    c.did[VEHICLE_CL250_IDX_ENGINE_SPEED].requested = false;
    TEST_ASSERT_EQUAL(K_READ, poll_skip_tp(back));
    TEST_ASSERT_EQUAL_UINT32(speed, req_idx());
}

static void test_nrc_0x78_does_not_extend_a_faulty_did_read(void)
{
    const uint32_t speed = VEHICLE_CL250_IDX_VEHICLE_SPEED;
    const uint32_t base = VEHICLE_CL250_RESPONSE_TIMEOUT_BASE_MS;
    bring_up(0u);
    all_requested_at(0u);
    c.did[speed].consecutive_timeouts = 1u;
    c.did[speed].requested = false; /* only speed is due */
    TEST_ASSERT_EQUAL(K_READ, poll_at(0u));
    TEST_ASSERT_EQUAL_UINT32(speed, req_idx());
    nrc(base / 2u, UDS_SID_READ_DATA_BY_IDENTIFIER, UDS_NRC_RESPONSE_PENDING);
    nrc(base - 1u, UDS_SID_READ_DATA_BY_IDENTIFIER, UDS_NRC_RESPONSE_PENDING);
    TEST_ASSERT_EQUAL_UINT32(2u, stats()->response_pending); /* still counted */
    TEST_ASSERT_EQUAL_UINT32(base, c.wait_ms);
    TEST_ASSERT_EQUAL(K_NONE, poll_skip_tp(base - 1u));
    TEST_ASSERT_EQUAL(UDS_CLIENT_REQ_READ, uds_client_core_pending(&c));
    (void)poll_skip_tp(base); /* ends at the base timeout from the request */
    TEST_ASSERT_EQUAL_UINT32(1u, stats()->timeouts);
    TEST_ASSERT_EQUAL_UINT8(2u, c.did[speed].consecutive_timeouts);
}

static void test_an_answer_after_0x78_on_a_faulty_did_within_the_base_is_accepted(void)
{
    const uint32_t speed = VEHICLE_CL250_IDX_VEHICLE_SPEED;
    const uint32_t base = VEHICLE_CL250_RESPONSE_TIMEOUT_BASE_MS;
    bring_up(0u);
    all_requested_at(0u);
    c.did[speed].consecutive_timeouts = 1u;
    c.did[speed].requested = false;
    TEST_ASSERT_EQUAL(K_READ, poll_at(0u));
    nrc(10u, UDS_SID_READ_DATA_BY_IDENTIFIER, UDS_NRC_RESPONSE_PENDING);
    TEST_ASSERT_TRUE(answer_read(base - 1u, speed, 50u));
    TEST_ASSERT_EQUAL_UINT8(0u, c.did[speed].consecutive_timeouts);
    TEST_ASSERT_EQUAL_UINT32(0u, stats()->timeouts);
}

/*
 * Known residual (README "Reads"): an NRC carries the SID only, so 0x78 that the ECU
 * keeps sending for a timed-out 0xF40D extends the next 0x22 read. That read is still
 * capped at RESPONSE_TIMEOUT_MAX_MS from its request; its DID is then faulty and its
 * next read gets no extension.
 */
static void test_a_late_0x78_extends_the_next_read_only_up_to_the_cap(void)
{
    const uint32_t speed = VEHICLE_CL250_IDX_VEHICLE_SPEED;
    const uint32_t rpm = VEHICLE_CL250_IDX_ENGINE_SPEED;
    const uint32_t base = VEHICLE_CL250_RESPONSE_TIMEOUT_BASE_MS;
    const uint32_t max = VEHICLE_CL250_RESPONSE_TIMEOUT_MAX_MS;
    const uint8_t sid = UDS_SID_READ_DATA_BY_IDENTIFIER;
    bring_up(0u);
    all_requested_at(0u);
    c.did[speed].consecutive_timeouts = 1u;
    c.did[speed].requested = false; /* only speed is due */
    TEST_ASSERT_EQUAL(K_READ, poll_at(0u));
    TEST_ASSERT_EQUAL_UINT32(speed, req_idx());
    c.did[rpm].requested = false; /* due when the speed read ends */
    TEST_ASSERT_EQUAL(K_READ, poll_skip_tp(base));
    TEST_ASSERT_EQUAL_UINT32(rpm, req_idx());
    for (uint32_t t = base + 10u; t < (base + max); t += 50u) {
        nrc(t, sid, UDS_NRC_RESPONSE_PENDING); /* the ECU's clock, meant for speed */
        TEST_ASSERT_EQUAL(K_NONE, poll_skip_tp(t));
    }
    TEST_ASSERT_EQUAL(UDS_CLIENT_REQ_READ, uds_client_core_pending(&c));
    TEST_ASSERT_EQUAL(K_NONE, poll_skip_tp(base + max - 1u));
    all_requested_at(base + max); /* nothing else due when the RPM read ends */
    TEST_ASSERT_EQUAL(K_NONE, poll_skip_tp(base + max)); /* capped from the RPM request */
    TEST_ASSERT_EQUAL_UINT32(2u, stats()->timeouts);
    TEST_ASSERT_EQUAL_UINT8(1u, c.did[rpm].consecutive_timeouts);
    c.did[rpm].requested = false; /* only RPM is due */
    const uint32_t t1 = base + max + 1u;
    TEST_ASSERT_EQUAL(K_READ, poll_skip_tp(t1));
    TEST_ASSERT_EQUAL_UINT32(rpm, req_idx());
    nrc(t1 + 10u, sid, UDS_NRC_RESPONSE_PENDING);
    TEST_ASSERT_EQUAL_UINT32(base, c.wait_ms); /* faulty now: no extension */
    (void)poll_skip_tp(t1 + base);
    TEST_ASSERT_EQUAL_UINT8(2u, c.did[rpm].consecutive_timeouts);
}

static void test_0x78_on_the_session_request_still_extends_while_a_did_is_faulty(void)
{
    const uint32_t base = VEHICLE_CL250_RESPONSE_TIMEOUT_BASE_MS;
    uds_client_core_init(&c, HOLD_MS);
    c.did[0].consecutive_timeouts = 1u; /* pending_idx is 0 for a session request */
    TEST_ASSERT_EQUAL(K_SESSION, poll_at(0u));
    nrc(10u, VEHICLE_CL250_SESSION_SID, UDS_NRC_RESPONSE_PENDING);
    TEST_ASSERT_EQUAL_UINT32(2u * base, c.wait_ms);
    TEST_ASSERT_EQUAL(UDS_CLIENT_REQ_SESSION, uds_client_core_pending(&c));
}

/*
 * D-050: 0xF40D is silent (endless_0x78 false) or answers NRC 0x78 every 50 ms while
 * its read is pending (true); every other DID answers after a round trip in
 * [rtt_min, rtt_max] (seeded). The run covers several skip cycles. Every other DID's
 * sample gap must stay within its stale_after_ms plus the round-trip jitter, except
 * across a fresh attempt (0xF40D read with no timeout before it: the first failing
 * attempt, and the first after each skip cooldown), which may hold the slot up to
 * RESPONSE_TIMEOUT_MAX_MS. A faulty attempt holds it at most RESPONSE_TIMEOUT_BASE_MS.
 */
static void run_with_faulty_speed(bool endless_0x78, uint32_t rtt_min, uint32_t rtt_max,
                                  uint32_t duration_ms)
{
    const uint32_t speed = VEHICLE_CL250_IDX_VEHICLE_SPEED;
    const uint32_t base = VEHICLE_CL250_RESPONSE_TIMEOUT_BASE_MS;
    const uint32_t t0 = 1000u;
    uint32_t last_sample[VEHICLE_CL250_DID_COUNT];
    bool answer_due = false;
    uint32_t answer_t = 0u;
    uint32_t answer_idx = 0u;
    bool attempt = false;      /* a 0xF40D read is pending */
    bool attempt_fresh = false;
    uint32_t attempt_t = 0u;
    uint32_t fresh_end = 0u;   /* end of the latest fresh attempt */
    uint32_t faulty_attempts = 0u;
    bring_up(t0);
    for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
        last_sample[i] = t0;
    }
    for (uint32_t t = t0; t < (t0 + duration_ms); t++) {
        if (answer_due && (t == answer_t)) {
            TEST_ASSERT_TRUE(answer_read(t, answer_idx, 1u));
            last_sample[answer_idx] = t;
            answer_due = false;
        }
        if (endless_0x78 && attempt && (t != attempt_t) && (((t - attempt_t) % 50u) == 0u)) {
            nrc(t, UDS_SID_READ_DATA_BY_IDENTIFIER, UDS_NRC_RESPONSE_PENDING);
        }
        kind_t k;
        while ((k = poll_at(t)) != K_NONE) {
            TEST_ASSERT_TRUE_MESSAGE(k != K_SESSION, "session lost");
            if (attempt && ((uds_client_core_pending(&c) != UDS_CLIENT_REQ_READ) ||
                            (c.pending_idx != speed) || (k == K_READ))) {
                attempt = false; /* ended in this poll (timeout) */
                if (attempt_fresh) {
                    fresh_end = t;
                    TEST_ASSERT_LESS_OR_EQUAL_UINT32(VEHICLE_CL250_RESPONSE_TIMEOUT_MAX_MS,
                                                     t - attempt_t);
                } else {
                    faulty_attempts++;
                    TEST_ASSERT_LESS_OR_EQUAL_UINT32_MESSAGE(base, t - attempt_t,
                                                             "a faulty read held the slot");
                }
            }
            if (k == K_READ) {
                TEST_ASSERT_FALSE_MESSAGE(answer_due, "two requests in flight");
                if (req_idx() == speed) {
                    attempt = true;
                    attempt_fresh = (c.did[speed].consecutive_timeouts == 0u);
                    attempt_t = t;
                } else {
                    answer_due = true;
                    answer_idx = req_idx();
                    answer_t = t + rtt_min + (rng_next() % (rtt_max - rtt_min + 1u));
                }
            }
        }
        for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
            const bool across_fresh =
                (attempt && attempt_fresh) || (fresh_end > last_sample[i]);
            if ((i != speed) && !across_fresh) {
                TEST_ASSERT_LESS_OR_EQUAL_UINT32_MESSAGE(
                    vehicle_cl250_dids[i].stale_after_ms + (rtt_max - rtt_min),
                    t - last_sample[i], "a DID went STALE while 0xF40D was faulty");
            }
        }
    }
    TEST_ASSERT_GREATER_THAN_UINT32(1u, stats()->did_skips); /* several skip cycles */
    TEST_ASSERT_GREATER_THAN_UINT32(0u, faulty_attempts);
}

static void test_silent_speed_keeps_the_others_within_stale_after_ms(void)
{
    const uint32_t rtt = VEHICLE_CL250_ASSUMED_ROUND_TRIP_MS;
    run_with_faulty_speed(false, rtt, rtt, 30000u);
    setUp();
    run_with_faulty_speed(false, 1u, 1u, 30000u);
}

static void test_endless_0x78_on_speed_keeps_the_others_within_stale_after_ms(void)
{
    const uint32_t rtt = VEHICLE_CL250_ASSUMED_ROUND_TRIP_MS;
    run_with_faulty_speed(true, rtt, rtt, 30000u);
    setUp();
    run_with_faulty_speed(true, 1u, 1u, 30000u);
}

static void test_faulty_speed_with_random_round_trips_adds_only_the_jitter(void)
{
    for (uint32_t seed = 1u; seed <= 20u; seed++) {
        setUp();
        rng_state = seed;
        run_with_faulty_speed((seed % 2u) == 0u, 1u, VEHICLE_CL250_ASSUMED_ROUND_TRIP_MS,
                              15000u);
    }
}

/* ------------------------------------------------------------------------- */
/* D-051 (E-7): slow answers and the sample stamp                             */
/* ------------------------------------------------------------------------- */

static void test_an_answer_is_stamped_with_its_request_send_time(void)
{
    const uint32_t speed = VEHICLE_CL250_IDX_VEHICLE_SPEED;
    bring_up(0u);
    TEST_ASSERT_EQUAL(K_READ, poll_at(10u));
    TEST_ASSERT_EQUAL_UINT32(speed, req_idx());
    TEST_ASSERT_TRUE(answer_read(30u, speed, 60u));
    TEST_ASSERT_EQUAL_UINT32(10u, smp.stamp_ms); /* the send time, not the arrival */
    TEST_ASSERT_FALSE(c.did[speed].slow);
}

/*
 * 0xF40D answered after a 0x78, 180 ms after its request (period 100 ms): faulty for
 * one round. Its next read competes in the normal class and gets no 0x78 extension;
 * an answer within the period gives it its gen/ priority and the extension back.
 */
static void test_a_read_answered_after_its_period_makes_the_did_faulty_for_one_round(void)
{
    const uint32_t speed = VEHICLE_CL250_IDX_VEHICLE_SPEED;
    const uint32_t rpm = VEHICLE_CL250_IDX_ENGINE_SPEED;
    const uint32_t base = VEHICLE_CL250_RESPONSE_TIMEOUT_BASE_MS;
    const uint32_t period = vehicle_cl250_dids[speed].poll_period_ms;
    bring_up(0u);
    all_requested_at(0u);
    c.did[speed].requested = false; /* only speed is due */
    TEST_ASSERT_EQUAL(K_READ, poll_at(0u));
    TEST_ASSERT_EQUAL_UINT32(speed, req_idx());
    nrc(10u, UDS_SID_READ_DATA_BY_IDENTIFIER, UDS_NRC_RESPONSE_PENDING);
    TEST_ASSERT_EQUAL_UINT32(2u * base, c.wait_ms); /* not faulty yet: extended */
    TEST_ASSERT_TRUE(answer_read(period + 80u, speed, 60u)); /* held past its period */
    TEST_ASSERT_EQUAL_UINT32(0u, smp.stamp_ms);
    TEST_ASSERT_TRUE(c.did[speed].slow);
    /* Both due: speed now competes in the normal class, RPM goes first. */
    const uint32_t t1 = 2u * period;
    all_requested_at(t1);
    c.did[speed].last_request_ms = 0u;
    c.did[rpm].requested = false;
    TEST_ASSERT_EQUAL(K_READ, poll_skip_tp(t1));
    TEST_ASSERT_EQUAL_UINT32(rpm, req_idx());
    TEST_ASSERT_TRUE(answer_read(t1 + 1u, rpm, 1u));
    TEST_ASSERT_EQUAL(K_READ, poll_skip_tp(t1 + 1u));
    TEST_ASSERT_EQUAL_UINT32(speed, req_idx());
    nrc(t1 + 10u, UDS_SID_READ_DATA_BY_IDENTIFIER, UDS_NRC_RESPONSE_PENDING);
    TEST_ASSERT_EQUAL_UINT32(base, c.wait_ms); /* faulty: no extension */
    TEST_ASSERT_TRUE(answer_read(t1 + 21u, speed, 61u)); /* within the period */
    TEST_ASSERT_EQUAL_UINT32(t1 + 1u, smp.stamp_ms);
    TEST_ASSERT_FALSE(c.did[speed].slow);
    /* Its gen/ priority is back. */
    all_requested_at(t1 + 21u);
    c.did[speed].requested = false;
    c.did[rpm].requested = false;
    TEST_ASSERT_EQUAL(K_READ, poll_skip_tp(t1 + 21u));
    TEST_ASSERT_EQUAL_UINT32(speed, req_idx());
    TEST_ASSERT_EQUAL_UINT32(0u, stats()->timeouts);
}

/* An NRC other than 0x78 also ends the read with an answer: the same rule applies. */
static void test_an_nrc_after_the_period_makes_the_did_faulty_too(void)
{
    const uint32_t speed = VEHICLE_CL250_IDX_VEHICLE_SPEED;
    const uint32_t period = vehicle_cl250_dids[speed].poll_period_ms;
    bring_up(0u);
    all_requested_at(0u);
    c.did[speed].requested = false;
    TEST_ASSERT_EQUAL(K_READ, poll_at(0u));
    nrc(10u, UDS_SID_READ_DATA_BY_IDENTIFIER, UDS_NRC_RESPONSE_PENDING);
    nrc(period, UDS_SID_READ_DATA_BY_IDENTIFIER, UDS_NRC_REQUEST_OUT_OF_RANGE);
    TEST_ASSERT_FALSE(c.did[speed].slow); /* exactly the period is in time */
    all_requested_at(period);
    c.did[speed].requested = false;
    TEST_ASSERT_EQUAL(K_READ, poll_skip_tp(period));
    nrc(period + 10u, UDS_SID_READ_DATA_BY_IDENTIFIER, UDS_NRC_RESPONSE_PENDING);
    nrc(2u * period + 1u, UDS_SID_READ_DATA_BY_IDENTIFIER, UDS_NRC_REQUEST_OUT_OF_RANGE);
    TEST_ASSERT_TRUE(c.did[speed].slow); /* one ms past the period */
    TEST_ASSERT_EQUAL(UDS_CLIENT_REQ_NONE, uds_client_core_pending(&c));
}

/*
 * A late answer to a timed-out read is taken for the next read of the same DID (it
 * carries no request reference). It is stamped with the first timed-out read since the
 * last answer, so its age is never underestimated; it also counts as slow.
 */
static void test_a_late_answer_to_a_timed_out_read_is_stamped_with_that_read(void)
{
    const uint32_t speed = VEHICLE_CL250_IDX_VEHICLE_SPEED;
    const uint32_t base = VEHICLE_CL250_RESPONSE_TIMEOUT_BASE_MS;
    const uint32_t period = vehicle_cl250_dids[speed].poll_period_ms;
    const uint32_t gap = base + period; /* timeout, then the period from it */
    bring_up(0u);
    all_requested_at(0u);
    c.did[speed].requested = false;
    TEST_ASSERT_EQUAL(K_READ, poll_at(0u));
    for (uint32_t n = 1u; n <= 2u; n++) {
        const uint32_t sent = (n - 1u) * gap;
        all_requested_at(sent + base); /* the others are not due */
        TEST_ASSERT_EQUAL(K_NONE, poll_skip_tp(sent + base)); /* times out */
        TEST_ASSERT_EQUAL_UINT8((uint8_t)n, c.did[speed].consecutive_timeouts);
        TEST_ASSERT_EQUAL_UINT32(0u, c.did[speed].unanswered_ms); /* the first one */
        all_requested_at(n * gap);
        c.did[speed].last_request_ms = sent + base; /* its period restarted at the timeout */
        TEST_ASSERT_EQUAL(K_READ, poll_skip_tp(n * gap));
        TEST_ASSERT_EQUAL_UINT32(speed, req_idx());
    }
    TEST_ASSERT_TRUE(answer_read((2u * gap) + 10u, speed, 60u)); /* meant for the read at 0 */
    TEST_ASSERT_EQUAL_UINT32(0u, smp.stamp_ms);
    TEST_ASSERT_TRUE(c.did[speed].slow);
    TEST_ASSERT_EQUAL_UINT8(0u, c.did[speed].consecutive_timeouts);
    /* The next answer within the period is stamped with its own read again. */
    const uint32_t t1 = (2u * gap) + period;
    all_requested_at(t1);
    c.did[speed].last_request_ms = 2u * gap;
    TEST_ASSERT_EQUAL(K_READ, poll_skip_tp(t1));
    TEST_ASSERT_EQUAL_UINT32(speed, req_idx());
    TEST_ASSERT_TRUE(answer_read(t1 + 5u, speed, 61u));
    TEST_ASSERT_EQUAL_UINT32(t1, smp.stamp_ms);
    TEST_ASSERT_FALSE(c.did[speed].slow);
}

static void test_a_skip_clears_the_slow_state(void)
{
    const uint32_t speed = VEHICLE_CL250_IDX_VEHICLE_SPEED;
    const uint32_t base = VEHICLE_CL250_RESPONSE_TIMEOUT_BASE_MS;
    bring_up(0u);
    all_requested_at(0u);
    c.did[speed].slow = true;
    c.did[speed].consecutive_timeouts = (uint8_t)(VEHICLE_CL250_MAX_CONSECUTIVE_TIMEOUTS - 1u);
    c.did[speed].requested = false;
    TEST_ASSERT_EQUAL(K_READ, poll_at(0u));
    (void)poll_skip_tp(base);
    TEST_ASSERT_EQUAL_UINT32(1u, stats()->did_skips);
    TEST_ASSERT_FALSE(c.did[speed].slow); /* the fresh attempt gets the extension back */
}

/*
 * D-051 runs. Every DID but 0xF40D answers after a round trip in [rtt_min, rtt_max].
 * 0xF40D, by mode:
 *   SLOW_ABANDONS: NRC 0x78 10 ms after each request (and every 50 ms while that read
 *                  is pending), then the answer 150-250 ms after the request (seeded).
 *                  A new 0xF40D request drops the one before (like app/host/sim_ecu).
 *   SLOW_KEEPS:    the same, but the ECU still answers a read after the next one came,
 *                  so a late answer may be taken for the next 0xF40D read.
 *   ALTERNATING:   odd requests answered after rtt_min, even ones NRC 0x78 every 50 ms
 *                  while pending, never answered.
 * Checks in every mode: one request in flight; a 0xF40D read that was faulty when sent
 * holds the slot at most the base timeout (a fresh one up to the cap); the other DIDs
 * stay within stale_after_ms plus the jitter except across a fresh attempt; the first
 * answer after a timeout is stamped no later than the read the ECU answered.
 * SLOW_ABANDONS and ALTERNATING: no sample looks younger than it is, and fresh attempts
 * come only at the start and after a skip. SLOW_KEEPS is the README residual: the own
 * answer of a read that a late answer ended can land on the next read, look younger
 * and clear the slow state; it is checked to keep RPM STALE under 5 %.
 */
typedef enum { SPEED_SLOW_ABANDONS = 0, SPEED_SLOW_KEEPS, SPEED_ALTERNATING } speed_mode_t;

#define LATE_MAX 8u


static void run_with_slow_speed(speed_mode_t mode, uint32_t rtt_min, uint32_t rtt_max,
                                uint32_t duration_ms)
{
    const uint32_t speed = VEHICLE_CL250_IDX_VEHICLE_SPEED;
    const uint32_t base = VEHICLE_CL250_RESPONSE_TIMEOUT_BASE_MS;
    const uint32_t t0 = 1000u;
    uint32_t last_sample[VEHICLE_CL250_DID_COUNT];
    uint32_t late_t[LATE_MAX];    /* pending 0xF40D answers: arrival */
    uint32_t late_sent[LATE_MAX]; /* and the send time of the read they answer */
    uint32_t late_n = 0u;
    bool answer_due = false;
    uint32_t answer_t = 0u;
    uint32_t answer_idx = 0u;
    bool attempt = false;
    bool attempt_fresh = false;
    uint32_t attempt_t = 0u;
    uint32_t fresh_end = 0u;
    uint32_t fresh_attempts = 0u;
    uint32_t speed_requests = 0u;
    uint32_t speed_samples = 0u;
    uint32_t rpm_stale_ms = 0u;
    uint32_t chained = 0u; /* speed samples stamped later than the read they answer */
    bring_up(t0);
    for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
        last_sample[i] = t0;
    }
    for (uint32_t t = t0; t < (t0 + duration_ms); t++) {
        if (answer_due && (t == answer_t)) {
            TEST_ASSERT_TRUE(answer_read(t, answer_idx, 1u));
            last_sample[answer_idx] = t;
            answer_due = false;
        }
        for (uint32_t n = 0u; n < late_n; n++) {
            if (late_t[n] == t) {
                const bool after_timeout = (c.did[speed].consecutive_timeouts > 0u);
                if (answer_read(t, speed, 1u)) {
                    speed_samples++;
                    if (after_timeout) {
                        TEST_ASSERT_LESS_OR_EQUAL_UINT32_MESSAGE(
                            late_sent[n], smp.stamp_ms, "a speed sample looks younger than it is");
                    } else if (smp.stamp_ms > late_sent[n]) {
                        /* Residual (README): the own answer of a read that a late answer
                         * ended lands on the next read. */
                        chained++;
                    } else {
                        /* stamped no later than its read */
                    }
                }
                late_t[n] = late_t[late_n - 1u];
                late_sent[n] = late_sent[late_n - 1u];
                late_n--;
                n--;
            }
        }
        if (attempt && (t != attempt_t) && (((t - attempt_t) % 50u) == 10u) &&
            (uds_client_core_pending(&c) == UDS_CLIENT_REQ_READ) && (c.pending_idx == speed)) {
            nrc(t, UDS_SID_READ_DATA_BY_IDENTIFIER, UDS_NRC_RESPONSE_PENDING);
        }
        kind_t k;
        while ((k = poll_at(t)) != K_NONE) {
            TEST_ASSERT_TRUE_MESSAGE(k != K_SESSION, "session lost");
            if (attempt && ((uds_client_core_pending(&c) != UDS_CLIENT_REQ_READ) ||
                            (c.pending_idx != speed) || (k == K_READ))) {
                attempt = false; /* ended in this poll (timeout) */
            }
            if (k == K_READ) {
                TEST_ASSERT_FALSE_MESSAGE(answer_due, "two requests in flight");
                if (req_idx() == speed) {
                    attempt = true;
                    attempt_fresh = !c.did[speed].slow && (c.did[speed].consecutive_timeouts == 0u);
                    attempt_t = t;
                    speed_requests++;
                    if (attempt_fresh) {
                        fresh_attempts++;
                    }
                    const bool slow = (mode != SPEED_ALTERNATING);
                    if (mode == SPEED_SLOW_ABANDONS) {
                        late_n = 0u; /* the ECU drops a read when the next one comes */
                    }
                    if (slow || ((speed_requests % 2u) == 1u)) {
                        TEST_ASSERT_LESS_THAN_UINT32(LATE_MAX, late_n);
                        late_t[late_n] = t + (slow ? (150u + (rng_next() % 101u)) : rtt_min);
                        late_sent[late_n] = t;
                        late_n++;
                    }
                } else {
                    answer_due = true;
                    answer_idx = req_idx();
                    answer_t = t + rtt_min + (rng_next() % (rtt_max - rtt_min + 1u));
                }
            }
        }
        if (attempt && (uds_client_core_pending(&c) == UDS_CLIENT_REQ_READ) &&
            (c.pending_idx == speed)) {
            TEST_ASSERT_LESS_OR_EQUAL_UINT32_MESSAGE(
                attempt_fresh ? VEHICLE_CL250_RESPONSE_TIMEOUT_MAX_MS : base, t - attempt_t,
                "a speed read held the slot too long");
        } else if (attempt) {
            attempt = false; /* answered */
        } else {
            /* no speed read pending */
        }
        if (attempt_fresh && !attempt && (fresh_end < attempt_t)) {
            fresh_end = t;
        }
        if ((t - last_sample[VEHICLE_CL250_IDX_ENGINE_SPEED]) >
            vehicle_cl250_dids[VEHICLE_CL250_IDX_ENGINE_SPEED].stale_after_ms) {
            rpm_stale_ms++;
        }
        for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
            const bool across_fresh = (attempt && attempt_fresh) || (fresh_end > last_sample[i]);
            if ((i != speed) && !across_fresh) {
                TEST_ASSERT_LESS_OR_EQUAL_UINT32_MESSAGE(
                    vehicle_cl250_dids[i].stale_after_ms + (rtt_max - rtt_min),
                    t - last_sample[i], "a DID went STALE while 0xF40D was slow");
            }
        }
    }
    TEST_ASSERT_GREATER_THAN_UINT32(0u, speed_samples);
    if (mode == SPEED_SLOW_KEEPS) {
        /* The residual (README): shifted answers may look younger and clear the slow
         * state, so fresh attempts recur; each holds the slot only until its answer. */
        TEST_ASSERT_LESS_OR_EQUAL_UINT32(duration_ms / 20u, rpm_stale_ms);
    } else {
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(0u, chained, "a speed sample looks younger than it is");
        TEST_ASSERT_LESS_OR_EQUAL_UINT32(
            stats()->did_skips + ((mode == SPEED_SLOW_ABANDONS) ? 1u : 2u), fresh_attempts);
    }
}

static void test_speed_answering_150_to_250_ms_after_0x78_keeps_the_others_fresh(void)
{
    const uint32_t rtt = VEHICLE_CL250_ASSUMED_ROUND_TRIP_MS;
    for (uint32_t seed = 1u; seed <= 10u; seed++) {
        for (uint32_t m = (uint32_t)SPEED_SLOW_ABANDONS; m <= (uint32_t)SPEED_SLOW_KEEPS; m++) {
            setUp();
            rng_state = seed;
            run_with_slow_speed((speed_mode_t)m, rtt, rtt, 20000u);
            setUp();
            rng_state = seed;
            run_with_slow_speed((speed_mode_t)m, 1u, rtt, 20000u);
        }
    }
}

static void test_speed_alternating_answer_and_endless_0x78_keeps_the_others_fresh(void)
{
    const uint32_t rtt = VEHICLE_CL250_ASSUMED_ROUND_TRIP_MS;
    run_with_slow_speed(SPEED_ALTERNATING, rtt, rtt, 20000u);
    setUp();
    run_with_slow_speed(SPEED_ALTERNATING, 1u, 1u, 20000u);
    for (uint32_t seed = 1u; seed <= 10u; seed++) {
        setUp();
        rng_state = seed;
        run_with_slow_speed(SPEED_ALTERNATING, 1u, rtt, 15000u);
    }
}

static void test_speed_answer_within_its_gen_period_keeps_the_period_from_the_request(void)
{
    const uint32_t speed = VEHICLE_CL250_IDX_VEHICLE_SPEED;
    const uint32_t period = vehicle_cl250_dids[speed].poll_period_ms;
    bring_up(0u);
    TEST_ASSERT_EQUAL(K_READ, poll_at(0u));
    TEST_ASSERT_EQUAL_UINT32(speed, req_idx());
    TEST_ASSERT_TRUE(answer_read(VEHICLE_CL250_ASSUMED_ROUND_TRIP_MS, speed, 60u));
    TEST_ASSERT_EQUAL_FLOAT(60.0f, smp.physical);
    all_requested_at(period - 1u); /* the normal DIDs are not due in this window */
    c.did[speed].last_request_ms = 0u;
    TEST_ASSERT_EQUAL(K_NONE, poll_at(period - 1u));
    TEST_ASSERT_EQUAL(K_READ, poll_at(period));
    TEST_ASSERT_EQUAL_UINT32(speed, req_idx());
}

static void test_nrc_0x78_doubles_the_timeout_up_to_the_gen_max_in_total(void)
{
    const uint32_t max = VEHICLE_CL250_RESPONSE_TIMEOUT_MAX_MS;
    const uint8_t sid = UDS_SID_READ_DATA_BY_IDENTIFIER;
    bring_up(0u);
    TEST_ASSERT_EQUAL(K_READ, poll_at(0u)); /* sent at 0, wait 100 */
    nrc(99u, sid, UDS_NRC_RESPONSE_PENDING);  /* wait 200 from 99 */
    TEST_ASSERT_EQUAL(K_NONE, poll_at(298u));
    nrc(298u, sid, UDS_NRC_RESPONSE_PENDING); /* wait 400 */
    TEST_ASSERT_EQUAL(K_NONE, poll_at(697u));
    nrc(697u, sid, UDS_NRC_RESPONSE_PENDING); /* wait 800 */
    TEST_ASSERT_EQUAL(K_NONE, poll_at(1496u));
    nrc(1496u, sid, UDS_NRC_RESPONSE_PENDING); /* wait 1600, but the total cap is 2000 */
    TEST_ASSERT_EQUAL_UINT32(4u, stats()->response_pending);
    TEST_ASSERT_EQUAL(K_NONE, poll_at(max - 1u));
    TEST_ASSERT_EQUAL(UDS_CLIENT_REQ_READ, uds_client_core_pending(&c));
    TEST_ASSERT_NOT_EQUAL(K_NONE, poll_at(max)); /* ECU answering 0x78 forever: ends */
    TEST_ASSERT_EQUAL_UINT32(1u, stats()->timeouts);
}

static void test_nrc_0x78_timeout_saturates_at_the_gen_max(void)
{
    bring_up(0u);
    TEST_ASSERT_EQUAL(K_READ, poll_at(0u));
    for (uint32_t i = 0u; i < 8u; i++) {
        nrc(i, UDS_SID_READ_DATA_BY_IDENTIFIER, UDS_NRC_RESPONSE_PENDING);
    }
    TEST_ASSERT_EQUAL_UINT32(VEHICLE_CL250_RESPONSE_TIMEOUT_MAX_MS, c.wait_ms);
    TEST_ASSERT_EQUAL(K_NONE, poll_at(VEHICLE_CL250_RESPONSE_TIMEOUT_MAX_MS - 1u));
}

static void test_answer_after_response_pending_is_accepted(void)
{
    bring_up(0u);
    TEST_ASSERT_EQUAL(K_READ, poll_at(0u));
    TEST_ASSERT_EQUAL_UINT32(VEHICLE_CL250_IDX_VEHICLE_SPEED, req_idx());
    nrc(50u, UDS_SID_READ_DATA_BY_IDENTIFIER, UDS_NRC_RESPONSE_PENDING); /* wait 200 from 50 */
    TEST_ASSERT_EQUAL(K_NONE, poll_at(149u)); /* the base would have ended here */
    TEST_ASSERT_EQUAL(K_NONE, poll_at(249u)); /* one in flight: no normal DID meanwhile */
    TEST_ASSERT_TRUE(answer_read(249u, VEHICLE_CL250_IDX_VEHICLE_SPEED, 100u));
    TEST_ASSERT_EQUAL_UINT32(VEHICLE_CL250_IDX_VEHICLE_SPEED, smp.idx);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 100.0f, smp.physical);
    TEST_ASSERT_EQUAL_UINT32(1u, stats()->response_pending);
    TEST_ASSERT_EQUAL_UINT32(0u, stats()->timeouts);
}

static void test_nrc_0x78_for_another_sid_does_not_extend_the_read(void)
{
    bring_up(0u);
    TEST_ASSERT_EQUAL(K_READ, poll_at(0u));
    nrc(50u, VEHICLE_CL250_TESTER_PRESENT_SID, UDS_NRC_RESPONSE_PENDING);
    TEST_ASSERT_EQUAL_UINT32(0u, stats()->response_pending);
    TEST_ASSERT_EQUAL(UDS_CLIENT_REQ_READ, uds_client_core_pending(&c)); /* not ended either */
    TEST_ASSERT_NOT_EQUAL(K_NONE, poll_at(VEHICLE_CL250_RESPONSE_TIMEOUT_BASE_MS));
    TEST_ASSERT_EQUAL_UINT32(1u, stats()->timeouts);
}

static void test_other_nrc_ends_the_read_without_a_timeout(void)
{
    bring_up(0u);
    TEST_ASSERT_EQUAL(K_READ, poll_at(0u));
    nrc(5u, UDS_SID_READ_DATA_BY_IDENTIFIER, 0x31u); /* requestOutOfRange */
    TEST_ASSERT_EQUAL(UDS_CLIENT_REQ_NONE, uds_client_core_pending(&c));
    TEST_ASSERT_EQUAL_UINT32(1u, stats()->nrc);
    TEST_ASSERT_EQUAL_UINT32(0u, stats()->timeouts);
    TEST_ASSERT_TRUE(uds_client_core_ecu_present(&c, 5u));
    TEST_ASSERT_TRUE(uds_client_core_session_up(&c));
    /* The DID keeps its schedule (legacy): next request is the next DID. */
    TEST_ASSERT_EQUAL(K_READ, poll_at(5u));
    TEST_ASSERT_EQUAL_UINT32(VEHICLE_CL250_IDX_ENGINE_SPEED, req_idx());
}

static void test_session_nrcs_drop_the_session_and_it_is_reestablished(void)
{
    const uint8_t codes[2] = {UDS_NRC_SUBFUNCTION_NOT_SUPPORTED_IN_ACTIVE_SESSION,
                              UDS_NRC_SERVICE_NOT_SUPPORTED_IN_ACTIVE_SESSION};
    for (uint32_t i = 0u; i < 2u; i++) {
        setUp();
        bring_up(0u);
        TEST_ASSERT_EQUAL(K_READ, poll_at(0u));
        nrc(5u, UDS_SID_READ_DATA_BY_IDENTIFIER, codes[i]);
        TEST_ASSERT_FALSE(uds_client_core_session_up(&c));
        TEST_ASSERT_EQUAL_UINT32(1u, stats()->session_losses);
        /* The last session request was at 0: the retry is due at the gen interval. */
        assert_no_session_or_read(5u, VEHICLE_CL250_SESSION_RETRY_INTERVAL_MS);
        TEST_ASSERT_EQUAL(K_SESSION, poll_at(VEHICLE_CL250_SESSION_RETRY_INTERVAL_MS));
        answer_session(VEHICLE_CL250_SESSION_RETRY_INTERVAL_MS);
        TEST_ASSERT_TRUE(uds_client_core_session_up(&c));
    }
}

static void test_nrc_for_tester_present_never_ends_a_pending_read(void)
{
    bring_up(0u);
    TEST_ASSERT_EQUAL(K_READ, poll_at(0u));
    nrc(3u, VEHICLE_CL250_TESTER_PRESENT_SID, 0x12u); /* subFunctionNotSupported */
    TEST_ASSERT_EQUAL(UDS_CLIENT_REQ_READ, uds_client_core_pending(&c));
    TEST_ASSERT_TRUE(answer_read(4u, VEHICLE_CL250_IDX_VEHICLE_SPEED, 1u));
}

/* ------------------------------------------------------------------------- */
/* Consecutive timeouts, skip cooldown, ECU absent                            */
/* ------------------------------------------------------------------------- */

static void test_did_is_skipped_after_gen_max_consecutive_timeouts_then_resumes(void)
{
    const uint32_t max_to = VEHICLE_CL250_MAX_CONSECUTIVE_TIMEOUTS;
    const uint32_t cooldown = VEHICLE_CL250_DID_SKIP_COOLDOWN_MS;
    bring_up(0u);
    silent_mask = 1u << VEHICLE_CL250_IDX_ENGINE_SPEED;
    uint32_t t = 0u;
    uint32_t skipped_at = 0u;
    while (stats()->did_skips == 0u) {
        TEST_ASSERT_TRUE(uds_client_core_ecu_present(&c, t)); /* the others answer */
        TEST_ASSERT_LESS_THAN_UINT32(max_to, c.did[VEHICLE_CL250_IDX_ENGINE_SPEED].consecutive_timeouts);
        drive(t, t + 1u);
        skipped_at = t;
        t++;
        TEST_ASSERT_LESS_THAN_UINT32(5000u, t);
    }
    TEST_ASSERT_EQUAL_UINT32(max_to, stats()->timeouts);
    TEST_ASSERT_TRUE(uds_client_core_did_skipped(&c, VEHICLE_CL250_IDX_ENGINE_SPEED, skipped_at));
    TEST_ASSERT_TRUE(uds_client_core_did_skipped(&c, VEHICLE_CL250_IDX_ENGINE_SPEED,
                                                 skipped_at + cooldown - 1u));
    TEST_ASSERT_FALSE(uds_client_core_did_skipped(&c, VEHICLE_CL250_IDX_ENGINE_SPEED,
                                                  skipped_at + cooldown));
    const uint32_t mark = log_n;
    drive(t, skipped_at + cooldown);
    for (uint32_t i = mark; i < log_n; i++) {
        TEST_ASSERT_FALSE((log_buf[i].k == K_READ) &&
                          (log_buf[i].idx == VEHICLE_CL250_IDX_ENGINE_SPEED));
    }
    TEST_ASSERT_GREATER_THAN_UINT32(0u, count_kind(K_READ, t, skipped_at + cooldown));
    silent_mask = 0u;
    const uint32_t mark2 = log_n;
    drive(skipped_at + cooldown, skipped_at + cooldown + 5u);
    bool resumed = false;
    for (uint32_t i = mark2; i < log_n; i++) {
        resumed = resumed || ((log_buf[i].k == K_READ) &&
                              (log_buf[i].idx == VEHICLE_CL250_IDX_ENGINE_SPEED));
    }
    TEST_ASSERT_TRUE(resumed);
}

static void test_a_success_resets_the_consecutive_timeouts(void)
{
    bring_up(0u);
    const uint32_t idx = VEHICLE_CL250_IDX_COOLANT_TEMP;
    for (uint32_t i = 0u; i + 1u < VEHICLE_CL250_MAX_CONSECUTIVE_TIMEOUTS; i++) {
        c.did[idx].consecutive_timeouts++;
    }
    all_requested_at(0u);
    c.did[idx].requested = false; /* only this DID is due */
    TEST_ASSERT_EQUAL(K_READ, poll_at(0u));
    TEST_ASSERT_EQUAL_UINT32(idx, req_idx());
    TEST_ASSERT_TRUE(answer_read(1u, idx, 100u));
    TEST_ASSERT_EQUAL_UINT8(0u, c.did[idx].consecutive_timeouts);
    TEST_ASSERT_EQUAL_UINT32(0u, stats()->did_skips);
}

static void test_ecu_absent_after_the_gen_timeout_drops_the_session(void)
{
    const uint32_t absent = VEHICLE_CL250_ECU_ABSENT_TIMEOUT_MS;
    bring_up(0u);
    drive(0u, 1000u);
    const uint32_t last = c.last_response_ms; /* the fake ECU's last answer */
    ecu_silent = true;
    drive(1000u, last + absent);
    TEST_ASSERT_TRUE(uds_client_core_ecu_present(&c, last + absent - 1u));
    TEST_ASSERT_TRUE(uds_client_core_session_up(&c));
    TEST_ASSERT_FALSE(uds_client_core_ecu_present(&c, last + absent));
    const uint32_t mark = log_n;
    drive(last + absent, last + absent + VEHICLE_CL250_RESPONSE_TIMEOUT_MAX_MS);
    TEST_ASSERT_FALSE(uds_client_core_session_up(&c));
    TEST_ASSERT_EQUAL_UINT32(1u, stats()->session_losses);
    /* The first request after the loss is the session request, and no DID is read. */
    uint32_t first = mark;
    while ((first < log_n) && (log_buf[first].k == K_TP)) {
        first++;
    }
    TEST_ASSERT_LESS_THAN_UINT32(log_n, first);
    TEST_ASSERT_EQUAL(K_SESSION, log_buf[first].k);
    TEST_ASSERT_EQUAL_UINT32(0u, count_kind(K_READ, log_buf[first].t, 0xFFFFFFFFu));
}

/* ------------------------------------------------------------------------- */
/* Q-020: failed reception = service unavailable                              */
/* ------------------------------------------------------------------------- */

static void test_timeout_cr_is_service_unavailable_with_cooldown_and_n_bs_hold(void)
{
    bring_up(0u);
    TEST_ASSERT_EQUAL(K_READ, poll_at(0u));
    const uint32_t idx = req_idx();
    TEST_ASSERT_FALSE(uds_client_core_on_indication(&c, 1000u, ISOTP_N_TIMEOUT_CR, NULL, 0u, &smp));
    TEST_ASSERT_EQUAL_UINT32(1u, stats()->unavailable);
    TEST_ASSERT_EQUAL_UINT32(1u, stats()->did_skips);
    TEST_ASSERT_TRUE(uds_client_core_did_skipped(&c, idx, 1000u));
    TEST_ASSERT_EQUAL(UDS_CLIENT_REQ_NONE, uds_client_core_pending(&c));
    TEST_ASSERT_EQUAL(K_NONE, poll_at(1000u + HOLD_MS - 1u)); /* the ECU may still wait for FC */
    TEST_ASSERT_EQUAL(K_TP, poll_at(1000u + HOLD_MS));
    TEST_ASSERT_EQUAL(K_READ, poll_at(1000u + HOLD_MS));
    TEST_ASSERT_NOT_EQUAL(idx, req_idx());
}

static void test_failed_reception_without_a_pending_read_still_holds(void)
{
    bring_up(0u);
    TEST_ASSERT_FALSE(uds_client_core_on_indication(&c, 10u, ISOTP_N_WRONG_SN, NULL, 0u, &smp));
    TEST_ASSERT_EQUAL_UINT32(0u, stats()->did_skips);
    TEST_ASSERT_EQUAL(K_NONE, poll_at(10u + HOLD_MS - 1u));
    TEST_ASSERT_EQUAL(K_TP, poll_at(10u + HOLD_MS)); /* due since 1000, held */
    TEST_ASSERT_EQUAL(K_READ, poll_at(10u + HOLD_MS));
}

static void test_failed_reception_on_the_session_request_retries_later(void)
{
    uds_client_core_init(&c, HOLD_MS);
    TEST_ASSERT_EQUAL(K_SESSION, poll_at(0u));
    TEST_ASSERT_FALSE(uds_client_core_on_indication(&c, 5u, ISOTP_N_UNEXP_PDU, NULL, 0u, &smp));
    TEST_ASSERT_EQUAL(UDS_CLIENT_REQ_NONE, uds_client_core_pending(&c));
    TEST_ASSERT_EQUAL_UINT32(0u, stats()->did_skips);
    TEST_ASSERT_EQUAL(K_NONE, poll_at(5u + HOLD_MS - 1u));
    TEST_ASSERT_EQUAL(K_TP, poll_at(5u + HOLD_MS));
    TEST_ASSERT_EQUAL(K_SESSION, poll_at(VEHICLE_CL250_SESSION_RETRY_INTERVAL_MS));
}

static void test_base_timeout_pauses_during_a_segmented_reception_but_not_the_cap(void)
{
    const uint32_t max = VEHICLE_CL250_RESPONSE_TIMEOUT_MAX_MS;
    bring_up(0u);
    TEST_ASSERT_EQUAL(K_READ, poll_at(0u));
    for (uint32_t t = VEHICLE_CL250_RESPONSE_TIMEOUT_BASE_MS; t < max; t += 100u) {
        TEST_ASSERT_EQUAL(K_NONE, poll_busy(t, true));
    }
    TEST_ASSERT_EQUAL_UINT32(0u, stats()->timeouts);
    TEST_ASSERT_EQUAL(UDS_CLIENT_REQ_READ, uds_client_core_pending(&c));
    TEST_ASSERT_EQUAL(K_NONE, poll_busy(max, true)); /* cap: ended, but still nothing sent */
    TEST_ASSERT_EQUAL_UINT32(1u, stats()->timeouts);
    TEST_ASSERT_EQUAL(UDS_CLIENT_REQ_NONE, uds_client_core_pending(&c));
    TEST_ASSERT_NOT_EQUAL(K_NONE, poll_busy(max, false));
}

/* ------------------------------------------------------------------------- */
/* Fail-closed, time wrap, arguments                                          */
/* ------------------------------------------------------------------------- */

static void test_latch_stops_the_core_until_init(void)
{
    bring_up(0u);
    TEST_ASSERT_EQUAL(K_READ, poll_at(0u));
    TEST_ASSERT_FALSE(uds_client_core_failed(&c));
    uds_client_core_latch(&c);
    TEST_ASSERT_TRUE(uds_client_core_failed(&c));
    TEST_ASSERT_FALSE(uds_client_core_session_up(&c)); /* a dead client is not "up" */
    TEST_ASSERT_EQUAL(UDS_CLIENT_REQ_NONE, uds_client_core_pending(&c));
    for (uint32_t t = 0u; t < 20000u; t += 7u) {
        TEST_ASSERT_EQUAL(K_NONE, poll_at(t));
    }
    /* Presence still ages out while latched, so the 2^32 wrap cannot bring it back. */
    TEST_ASSERT_FALSE(uds_client_core_ecu_present(&c, 0u));
    uds_client_core_init(&c, HOLD_MS); /* only init clears it */
    TEST_ASSERT_FALSE(uds_client_core_failed(&c));
}

static void test_nothing_is_sent_while_tx_is_not_ready_but_timers_run(void)
{
    bring_up(0u);
    TEST_ASSERT_EQUAL(K_READ, poll_at(0u));
    /* The mailbox stays full (no node ACKs): the pending read still times out, and no
     * new request queues up behind the stuck frame. */
    for (uint32_t t = 1u; t < 5000u; t++) {
        TEST_ASSERT_EQUAL(K_NONE, poll_full(t, false, false));
    }
    TEST_ASSERT_EQUAL_UINT32(1u, stats()->timeouts);
    TEST_ASSERT_FALSE(uds_client_core_ecu_present(&c, 5000u));
    TEST_ASSERT_FALSE(uds_client_core_session_up(&c)); /* absent meanwhile */
    TEST_ASSERT_EQUAL(K_SESSION, poll_full(5000u, false, true)); /* TX free again */
    TEST_ASSERT_FALSE(uds_client_core_failed(&c));
}

static void test_a_request_the_link_could_not_take_is_dropped_without_a_latch(void)
{
    bring_up(0u);
    TEST_ASSERT_EQUAL(K_READ, poll_at(0u));
    const uint32_t idx = req_idx();
    const uint32_t sent = stats()->requests;
    uds_client_core_not_sent(&c);
    TEST_ASSERT_FALSE(uds_client_core_failed(&c));
    TEST_ASSERT_EQUAL(UDS_CLIENT_REQ_NONE, uds_client_core_pending(&c));
    TEST_ASSERT_EQUAL_UINT32(0u, stats()->timeouts);
    TEST_ASSERT_EQUAL_UINT32(sent - 1u, stats()->requests); /* E-6 n1: not counted */
    /* Nothing went out, so the DID's period was not started: it goes again at once. */
    TEST_ASSERT_EQUAL(K_READ, poll_at(1u));
    TEST_ASSERT_EQUAL_UINT32(idx, req_idx());
    TEST_ASSERT_TRUE(answer_read(2u, idx, 1u));
    /* A dropped read of a DID requested before restores that earlier schedule. */
    const uint32_t period = vehicle_cl250_dids[idx].poll_period_ms;
    all_requested_at(1u);
    TEST_ASSERT_EQUAL(K_READ, poll_skip_tp(1u + period));
    TEST_ASSERT_EQUAL_UINT32(idx, req_idx());
    uds_client_core_not_sent(&c);
    TEST_ASSERT_TRUE(c.did[idx].requested);
    TEST_ASSERT_EQUAL_UINT32(1u, c.did[idx].last_request_ms);
    const uint32_t after = stats()->requests;
    uds_client_core_not_sent(&c); /* nothing produced since: harmless */
    TEST_ASSERT_EQUAL_UINT32(1u, c.did[idx].last_request_ms);
    TEST_ASSERT_EQUAL_UINT32(after, stats()->requests);
}

/* A read times out and, in the same poll, the next DID's read is refused: the timeout
 * stands, and only the refused read is undone. */
static void test_not_sent_after_a_timeout_in_the_same_poll_keeps_the_timeout(void)
{
    const uint32_t speed = VEHICLE_CL250_IDX_VEHICLE_SPEED;
    const uint32_t rpm = VEHICLE_CL250_IDX_ENGINE_SPEED;
    const uint32_t base = VEHICLE_CL250_RESPONSE_TIMEOUT_BASE_MS;
    bring_up(0u);
    all_requested_at(0u);
    c.did[speed].requested = false;
    TEST_ASSERT_EQUAL(K_READ, poll_at(0u));
    TEST_ASSERT_EQUAL_UINT32(speed, req_idx());
    all_requested_at(base);
    c.did[speed].last_request_ms = 0u;
    c.did[rpm].last_request_ms = 7u; /* due at base, with an earlier schedule */
    TEST_ASSERT_EQUAL(K_READ, poll_skip_tp(base));
    TEST_ASSERT_EQUAL_UINT32(rpm, req_idx());
    uds_client_core_not_sent(&c);
    TEST_ASSERT_EQUAL_UINT32(base, c.did[speed].last_request_ms);
    TEST_ASSERT_EQUAL_UINT8(1u, c.did[speed].consecutive_timeouts);
    TEST_ASSERT_EQUAL_UINT32(7u, c.did[rpm].last_request_ms);
}

static void test_a_refused_session_retry_keeps_the_earlier_attempt(void)
{
    const uint32_t retry = VEHICLE_CL250_SESSION_RETRY_INTERVAL_MS;
    uds_client_core_init(&c, HOLD_MS);
    TEST_ASSERT_EQUAL(K_SESSION, poll_at(0u));
    nrc(1u, VEHICLE_CL250_SESSION_SID, 0x22u); /* conditionsNotCorrect: still down */
    TEST_ASSERT_EQUAL(K_SESSION, poll_skip_tp(retry));
    uds_client_core_not_sent(&c);
    TEST_ASSERT_TRUE(c.session_tried);
    TEST_ASSERT_EQUAL_UINT32(0u, c.session_tried_ms); /* due again at once, not at 2x */
    TEST_ASSERT_EQUAL(K_SESSION, poll_skip_tp(retry + 1u));
}

static void test_not_sent_after_a_latch_does_nothing(void)
{
    bring_up(0u);
    TEST_ASSERT_EQUAL(K_READ, poll_at(0u));
    const uint32_t idx = req_idx();
    const uint32_t sent = stats()->requests;
    uds_client_core_latch(&c);
    uds_client_core_not_sent(&c);
    TEST_ASSERT_EQUAL_UINT32(sent, stats()->requests);
    TEST_ASSERT_EQUAL_UINT32(0u, c.did[idx].last_request_ms);
    TEST_ASSERT_TRUE(c.did[idx].requested);
}

/* E-6 n2: a dropped tester present keeps its period, like a read (m7). */
static void test_a_tester_present_the_link_could_not_take_is_due_again_at_once(void)
{
    const uint32_t tp = VEHICLE_CL250_TESTER_PRESENT_PERIOD_MS;
    bring_up(0u);
    all_requested_at(tp); /* no read due at tp */
    TEST_ASSERT_EQUAL(K_TP, poll_at(tp));
    const uint32_t sent = stats()->requests;
    uds_client_core_not_sent(&c);
    TEST_ASSERT_EQUAL_UINT32(sent - 1u, stats()->requests);
    TEST_ASSERT_EQUAL_UINT32(0u, c.tp_sent_ms); /* the period runs from the last one sent */
    TEST_ASSERT_EQUAL(K_TP, poll_at(tp + 1u));
    TEST_ASSERT_EQUAL(K_NONE, poll_at(tp + 1u));
}

static void test_a_session_request_the_link_could_not_take_is_retried_at_once(void)
{
    uds_client_core_init(&c, HOLD_MS);
    TEST_ASSERT_EQUAL(K_SESSION, poll_at(0u));
    uds_client_core_not_sent(&c);
    TEST_ASSERT_EQUAL_UINT32(0u, stats()->requests);
    TEST_ASSERT_EQUAL(UDS_CLIENT_REQ_NONE, uds_client_core_pending(&c));
    TEST_ASSERT_FALSE(c.session_tried); /* never tried: no retry interval to wait */
    TEST_ASSERT_EQUAL(K_SESSION, poll_at(1u));
    TEST_ASSERT_EQUAL_UINT32(1u, stats()->requests);
}

static void test_ecu_presence_needs_a_new_answer_after_absence(void)
{
    const uint32_t absent = VEHICLE_CL250_ECU_ABSENT_TIMEOUT_MS;
    bring_up(0u);
    TEST_ASSERT_EQUAL(K_NONE, poll_full(absent, false, false));
    TEST_ASSERT_FALSE(uds_client_core_ecu_present(&c, absent));
    /* 2^32 ms later the old answer time would look recent again: it must not count. */
    TEST_ASSERT_FALSE(uds_client_core_ecu_present(&c, 0u));
    TEST_ASSERT_FALSE(uds_client_core_ecu_present(&c, 1u));
}

static void test_schedule_is_wrap_safe(void)
{
    const uint32_t start = 0xFFFFFFFFu - 1500u;
    bring_up(start);
    drive(start, start + 3000u); /* crosses 2^32 */
    TEST_ASSERT_EQUAL_UINT32(0u, stats()->timeouts);
    TEST_ASSERT_TRUE(uds_client_core_session_up(&c));
    TEST_ASSERT_TRUE(uds_client_core_ecu_present(&c, start + 3000u));
    uint32_t last = start;
    uint32_t n = 0u;
    for (uint32_t i = 0u; i < log_n; i++) {
        if ((log_buf[i].k == K_READ) && (log_buf[i].idx == VEHICLE_CL250_IDX_ENGINE_SPEED)) {
            if (n > 0u) {
                TEST_ASSERT_EQUAL_UINT32(vehicle_cl250_dids[0].poll_period_ms, log_buf[i].t - last);
            }
            last = log_buf[i].t;
            n++;
        }
    }
    TEST_ASSERT_EQUAL_UINT32(3000u / vehicle_cl250_dids[0].poll_period_ms, n);
}

static void test_null_arguments_are_harmless(void)
{
    uint8_t out[UDS_CLIENT_REQ_MAX];
    uint16_t len = 0u;
    uds_client_core_init(NULL, 0u);
    uds_client_core_init(&c, HOLD_MS);
    TEST_ASSERT_FALSE(uds_client_core_poll(NULL, 0u, false, true, out, &len));
    TEST_ASSERT_FALSE(uds_client_core_poll(&c, 0u, false, true, NULL, &len));
    TEST_ASSERT_FALSE(uds_client_core_poll(&c, 0u, false, true, out, NULL));
    TEST_ASSERT_FALSE(uds_client_core_on_indication(NULL, 0u, ISOTP_N_OK, out, 1u, &smp));
    TEST_ASSERT_FALSE(uds_client_core_on_indication(&c, 0u, ISOTP_N_OK, out, 1u, NULL));
    uds_client_core_latch(NULL);
    uds_client_core_not_sent(NULL);
    TEST_ASSERT_FALSE(uds_client_core_session_up(NULL));
    TEST_ASSERT_FALSE(uds_client_core_ecu_present(NULL, 0u));
    TEST_ASSERT_TRUE(uds_client_core_failed(NULL));
    TEST_ASSERT_FALSE(uds_client_core_did_skipped(NULL, 0u, 0u));
    TEST_ASSERT_FALSE(uds_client_core_did_skipped(&c, VEHICLE_CL250_DID_COUNT, 0u));
    TEST_ASSERT_EQUAL(UDS_CLIENT_REQ_NONE, uds_client_core_pending(NULL));
    TEST_ASSERT_NULL(uds_client_core_stats(NULL));
    TEST_ASSERT_FALSE(uds_client_core_ecu_present(&c, 0u)); /* nothing seen yet */
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_first_request_is_the_extended_session_from_gen);
    RUN_TEST(test_session_is_retried_at_the_gen_interval_until_positive);
    RUN_TEST(test_no_did_is_read_while_the_session_is_down);
    RUN_TEST(test_session_nrc_keeps_it_down_until_the_next_retry);
    RUN_TEST(test_session_answer_with_another_subfunction_is_not_accepted);
    RUN_TEST(test_session_response_pending_extends_the_session_request);
    RUN_TEST(test_tester_present_every_gen_period_without_holding_the_slot);
    RUN_TEST(test_speed_is_the_only_high_priority_did_in_gen);
    RUN_TEST(test_reads_are_priority_first_then_table_order_when_all_are_due);
    RUN_TEST(test_a_just_due_high_did_goes_before_long_overdue_normal_ones);
    RUN_TEST(test_within_a_class_the_first_due_in_table_order_goes);
    RUN_TEST(test_no_did_starves_with_answers_after_the_gen_assumed_round_trip);
    RUN_TEST(test_no_did_starves_with_answers_after_one_ms);
    RUN_TEST(test_no_did_starves_when_the_answer_is_seen_one_step_later);
    RUN_TEST(test_the_gen_table_meets_its_gap_bound);
    RUN_TEST(test_no_did_starves_with_random_phases_and_round_trips);
    RUN_TEST(test_each_did_is_polled_at_its_gen_period);
    RUN_TEST(test_positive_response_gives_raw_and_physical_from_gen);
    RUN_TEST(test_response_for_another_did_or_malformed_is_ignored);
    RUN_TEST(test_response_timeout_is_the_gen_base);
    RUN_TEST(test_silent_speed_restarts_its_period_and_the_normal_dids_are_still_read);
    RUN_TEST(test_an_answer_is_stamped_with_its_request_send_time);
    RUN_TEST(test_a_read_answered_after_its_period_makes_the_did_faulty_for_one_round);
    RUN_TEST(test_an_nrc_after_the_period_makes_the_did_faulty_too);
    RUN_TEST(test_a_late_answer_to_a_timed_out_read_is_stamped_with_that_read);
    RUN_TEST(test_a_skip_clears_the_slow_state);
    RUN_TEST(test_speed_answering_150_to_250_ms_after_0x78_keeps_the_others_fresh);
    RUN_TEST(test_speed_alternating_answer_and_endless_0x78_keeps_the_others_fresh);
    RUN_TEST(test_speed_answer_within_its_gen_period_keeps_the_period_from_the_request);
    RUN_TEST(test_a_timed_out_high_did_competes_in_the_normal_class_until_it_answers);
    RUN_TEST(test_a_skip_gives_a_faulty_did_its_gen_priority_back);
    RUN_TEST(test_nrc_0x78_does_not_extend_a_faulty_did_read);
    RUN_TEST(test_an_answer_after_0x78_on_a_faulty_did_within_the_base_is_accepted);
    RUN_TEST(test_a_late_0x78_extends_the_next_read_only_up_to_the_cap);
    RUN_TEST(test_0x78_on_the_session_request_still_extends_while_a_did_is_faulty);
    RUN_TEST(test_silent_speed_keeps_the_others_within_stale_after_ms);
    RUN_TEST(test_endless_0x78_on_speed_keeps_the_others_within_stale_after_ms);
    RUN_TEST(test_faulty_speed_with_random_round_trips_adds_only_the_jitter);
    RUN_TEST(test_endless_0x78_on_speed_holds_the_slot_up_to_the_cap_then_the_others_go);
    RUN_TEST(test_nrc_0x78_doubles_the_timeout_up_to_the_gen_max_in_total);
    RUN_TEST(test_nrc_0x78_timeout_saturates_at_the_gen_max);
    RUN_TEST(test_answer_after_response_pending_is_accepted);
    RUN_TEST(test_nrc_0x78_for_another_sid_does_not_extend_the_read);
    RUN_TEST(test_other_nrc_ends_the_read_without_a_timeout);
    RUN_TEST(test_session_nrcs_drop_the_session_and_it_is_reestablished);
    RUN_TEST(test_nrc_for_tester_present_never_ends_a_pending_read);
    RUN_TEST(test_did_is_skipped_after_gen_max_consecutive_timeouts_then_resumes);
    RUN_TEST(test_a_success_resets_the_consecutive_timeouts);
    RUN_TEST(test_ecu_absent_after_the_gen_timeout_drops_the_session);
    RUN_TEST(test_timeout_cr_is_service_unavailable_with_cooldown_and_n_bs_hold);
    RUN_TEST(test_failed_reception_without_a_pending_read_still_holds);
    RUN_TEST(test_failed_reception_on_the_session_request_retries_later);
    RUN_TEST(test_base_timeout_pauses_during_a_segmented_reception_but_not_the_cap);
    RUN_TEST(test_latch_stops_the_core_until_init);
    RUN_TEST(test_nothing_is_sent_while_tx_is_not_ready_but_timers_run);
    RUN_TEST(test_a_request_the_link_could_not_take_is_dropped_without_a_latch);
    RUN_TEST(test_a_tester_present_the_link_could_not_take_is_due_again_at_once);
    RUN_TEST(test_a_session_request_the_link_could_not_take_is_retried_at_once);
    RUN_TEST(test_not_sent_after_a_timeout_in_the_same_poll_keeps_the_timeout);
    RUN_TEST(test_a_refused_session_retry_keeps_the_earlier_attempt);
    RUN_TEST(test_not_sent_after_a_latch_does_nothing);
    RUN_TEST(test_ecu_presence_needs_a_new_answer_after_absence);
    RUN_TEST(test_schedule_is_wrap_safe);
    RUN_TEST(test_null_arguments_are_harmless);
    return UNITY_END();
}
