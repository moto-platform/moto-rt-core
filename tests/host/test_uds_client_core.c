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
#include "features/uds/uds_iso14229.h"
#include "vehicle_cl250.h"

#include <string.h>
#include <unity.h>

#define POS_OFFSET 0x40u               /* positive response SID = request SID + 0x40 */
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
    /* Session up: DID reads start at once, in table order. TP stays on its period. */
    TEST_ASSERT_EQUAL(K_READ, poll_at(retry + 5u));
    TEST_ASSERT_EQUAL_UINT32(VEHICLE_CL250_IDX_ENGINE_SPEED, req_idx());
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

static void test_reads_are_round_robin_in_table_order_when_all_are_due(void)
{
    bring_up(0u);
    for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
        TEST_ASSERT_EQUAL(K_READ, poll_at(0u));
        TEST_ASSERT_EQUAL_UINT32(i, req_idx());
        TEST_ASSERT_EQUAL(UDS_CLIENT_REQ_READ, uds_client_core_pending(&c));
        TEST_ASSERT_TRUE(answer_read(0u, i, 1u));
    }
    TEST_ASSERT_EQUAL(K_NONE, poll_at(0u)); /* nothing due until the first period ends */
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
    bring_up(0u);
    for (uint32_t i = 0u; i < sizeof cases / sizeof cases[0]; i++) {
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
    TEST_ASSERT_EQUAL_UINT32(VEHICLE_CL250_IDX_ENGINE_SPEED, req_idx());
    TEST_ASSERT_FALSE(answer_read(1u, VEHICLE_CL250_IDX_VEHICLE_SPEED, 1u)); /* late/other */
    const uint8_t too_short[3] = {(uint8_t)(UDS_SID_READ_DATA_BY_IDENTIFIER + POS_OFFSET),
                                  (uint8_t)(VEHICLE_CL250_DID_ENGINE_SPEED >> 8u),
                                  (uint8_t)(VEHICLE_CL250_DID_ENGINE_SPEED & 0xFFu)};
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
    TEST_ASSERT_EQUAL(K_NONE, poll_at(10u + base - 1u));
    TEST_ASSERT_EQUAL_UINT32(0u, stats()->timeouts);
    TEST_ASSERT_EQUAL(K_READ, poll_at(10u + base)); /* timed out, next DID at once */
    TEST_ASSERT_EQUAL_UINT32(1u, stats()->timeouts);
    TEST_ASSERT_EQUAL_UINT32(VEHICLE_CL250_IDX_VEHICLE_SPEED, req_idx());
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
    nrc(50u, UDS_SID_READ_DATA_BY_IDENTIFIER, UDS_NRC_RESPONSE_PENDING);
    TEST_ASSERT_EQUAL(K_NONE, poll_at(149u));
    TEST_ASSERT_TRUE(answer_read(240u, VEHICLE_CL250_IDX_ENGINE_SPEED, 400u));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 100.0f, smp.physical);
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
    TEST_ASSERT_EQUAL_UINT32(VEHICLE_CL250_IDX_VEHICLE_SPEED, req_idx());
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
    TEST_ASSERT_TRUE(answer_read(4u, VEHICLE_CL250_IDX_ENGINE_SPEED, 1u));
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
    c.rr_next = idx;
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
    uds_client_core_not_sent(&c);
    TEST_ASSERT_FALSE(uds_client_core_failed(&c));
    TEST_ASSERT_EQUAL(UDS_CLIENT_REQ_NONE, uds_client_core_pending(&c));
    TEST_ASSERT_EQUAL_UINT32(0u, stats()->timeouts);
    TEST_ASSERT_EQUAL(K_READ, poll_at(1u)); /* the next DID; the dropped one keeps its period */
    TEST_ASSERT_NOT_EQUAL(idx, req_idx());
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
    RUN_TEST(test_reads_are_round_robin_in_table_order_when_all_are_due);
    RUN_TEST(test_each_did_is_polled_at_its_gen_period);
    RUN_TEST(test_positive_response_gives_raw_and_physical_from_gen);
    RUN_TEST(test_response_for_another_did_or_malformed_is_ignored);
    RUN_TEST(test_response_timeout_is_the_gen_base);
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
    RUN_TEST(test_ecu_presence_needs_a_new_answer_after_absence);
    RUN_TEST(test_schedule_is_wrap_safe);
    RUN_TEST(test_null_arguments_are_harmless);
    return UNITY_END();
}
