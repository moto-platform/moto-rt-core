/*
 * L0 tests for the D-080 DTC freeze-frame state machine (features/uds/dtc_snapshot_core):
 * pure, with a manual time value. Sub-functions, mask, records and the cap come from gen/;
 * the DTC numbers and statuses below are synthetic test data. ISO 14229-1 clause 12.3.
 */
#include "features/uds/dtc_snapshot_core.h"
#include "uds_iso14229.h"
#include "vehicle_cl250.h"

#include <math.h>
#include <string.h>
#include <unity.h>

#define POS_SID ((uint8_t)(UDS_SID_READ_DTC_INFORMATION + UDS_POSITIVE_RESPONSE_OFFSET))
#define MASK ((uint8_t)VEHICLE_CL250_DTC_STATUS_MASK)
#define CAP VEHICLE_CL250_DTC_MAX_PER_RUN
#define STATUS_MATCH ((uint8_t)MASK) /* a status with exactly the mask's bits set */
#define STATUS_NONE 0x00u
#define IDLE_MS VEHICLE_CL250_DTC_WARM_IDLE_MIN_DURATION_MS

static dtc_snapshot_core_t d;
static uint8_t req[DTC_SNAPSHOT_REQ_MAX];
static uint16_t req_len;

void setUp(void)
{
    dtc_snapshot_core_init(&d);
    memset(req, 0, sizeof req);
    req_len = 0xFFFFu;
}

void tearDown(void) {}

static void dtc_number(uint8_t i, uint8_t* out)
{
    out[0] = 0x12u; /* synthetic */
    out[1] = 0x34u;
    out[2] = (uint8_t)(0x50u + i);
}

static bool build(void)
{
    const bool ok = dtc_snapshot_core_build_request(&d, req, &req_len);
    if (ok) {
        TEST_ASSERT_TRUE_MESSAGE(vehicle_cl250_request_allowed(req, req_len), "D-020 gate refused");
        TEST_ASSERT_EQUAL_HEX8(UDS_SID_READ_DTC_INFORMATION, req[0]);
        TEST_ASSERT_TRUE(dtc_snapshot_core_in_flight(&d));
    } else {
        TEST_ASSERT_EQUAL_UINT16(0u, req_len);
    }
    return ok;
}

static void expect_count_request(void)
{
    TEST_ASSERT_TRUE(build());
    const uint8_t e[3] = {UDS_SID_READ_DTC_INFORMATION,
                          UDS_READ_DTC_REPORT_NUMBER_OF_DTC_BY_STATUS_MASK, MASK};
    TEST_ASSERT_EQUAL_UINT16(3u, req_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(e, req, 3u);
}

static void expect_list_request(void)
{
    TEST_ASSERT_TRUE(build());
    const uint8_t e[3] = {UDS_SID_READ_DTC_INFORMATION, UDS_READ_DTC_REPORT_DTC_BY_STATUS_MASK,
                          MASK};
    TEST_ASSERT_EQUAL_UINT16(3u, req_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(e, req, 3u);
}

static void expect_record_request(uint8_t sub, uint8_t dtc_i)
{
    TEST_ASSERT_TRUE(build());
    uint8_t e[6] = {UDS_SID_READ_DTC_INFORMATION, sub, 0u, 0u, 0u, 0u};
    dtc_number(dtc_i, &e[2]);
    e[5] = (sub == UDS_READ_DTC_REPORT_DTC_SNAPSHOT_RECORD_BY_DTC_NUMBER)
               ? (uint8_t)VEHICLE_CL250_DTC_SNAPSHOT_RECORD
               : (uint8_t)VEHICLE_CL250_DTC_EXT_DATA_RECORD;
    TEST_ASSERT_EQUAL_UINT16(6u, req_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(e, req, 6u);
}

static bool answer_count(uint16_t n)
{
    const uint8_t m[6] = {POS_SID, UDS_READ_DTC_REPORT_NUMBER_OF_DTC_BY_STATUS_MASK, 0xFFu, 0x01u,
                          (uint8_t)(n >> 8u), (uint8_t)(n & 0xFFu)};
    TEST_ASSERT_TRUE(dtc_snapshot_core_match_positive(&d, m, sizeof m));
    return dtc_snapshot_core_on_positive(&d, m, sizeof m);
}

/* A 0x02 answer with n records, all with the given status. */
static uint16_t build_list(uint8_t* m, uint8_t n, uint8_t status)
{
    uint16_t len = 0u;
    m[len++] = POS_SID;
    m[len++] = UDS_READ_DTC_REPORT_DTC_BY_STATUS_MASK;
    m[len++] = 0xFFu;
    for (uint8_t i = 0u; i < n; i++) {
        dtc_number(i, &m[len]);
        len = (uint16_t)(len + 3u);
        m[len++] = status;
    }
    return len;
}

static void answer_list(uint8_t n, uint8_t status)
{
    uint8_t m[3u + (4u * 16u)];
    const uint16_t len = build_list(m, n, status);
    TEST_ASSERT_TRUE(dtc_snapshot_core_match_positive(&d, m, len));
    TEST_ASSERT_TRUE(dtc_snapshot_core_on_positive(&d, m, len));
}

static void answer_record(uint8_t sub, uint8_t dtc_i, uint16_t total)
{
    uint8_t m[40] = {0};
    m[0] = POS_SID;
    m[1] = sub;
    dtc_number(dtc_i, &m[2]);
    m[5] = STATUS_MATCH;
    TEST_ASSERT_TRUE(dtc_snapshot_core_on_positive(&d, m, total));
}

/* A whole run for n_dtc listed DTCs; returns the requests it took. */
static void run_to_the_end(uint8_t n_dtc)
{
    expect_count_request();
    TEST_ASSERT_TRUE(answer_count(n_dtc));
    expect_list_request();
    answer_list(n_dtc, STATUS_MATCH);
    const uint8_t read = (n_dtc > CAP) ? (uint8_t)CAP : n_dtc;
    for (uint8_t i = 0u; i < read; i++) {
        expect_record_request(UDS_READ_DTC_REPORT_DTC_SNAPSHOT_RECORD_BY_DTC_NUMBER, i);
        answer_record(UDS_READ_DTC_REPORT_DTC_SNAPSHOT_RECORD_BY_DTC_NUMBER, i, 12u);
        expect_record_request(UDS_READ_DTC_REPORT_DTC_EXT_DATA_RECORD_BY_DTC_NUMBER, i);
        answer_record(UDS_READ_DTC_REPORT_DTC_EXT_DATA_RECORD_BY_DTC_NUMBER, i, 9u);
    }
}

/* ------------------------------------------------------------------------- */
/* Constants and the run                                                      */
/* ------------------------------------------------------------------------- */

static void test_D080_item2_the_budget_constants_follow_gen(void)
{
    TEST_ASSERT_EQUAL_UINT32(2u + (2u * VEHICLE_CL250_DTC_MAX_PER_RUN),
                             DTC_SNAPSHOT_MAX_REQUESTS_PER_RUN);
    TEST_ASSERT_EQUAL_UINT32(18u, DTC_SNAPSHOT_MAX_REQUESTS_PER_RUN);
    TEST_ASSERT_EQUAL_UINT32(2u, DTC_SNAPSHOT_RUNS_PER_EPOCH);
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(64u, offsetof(dtc_snapshot_core_t, stats));
}

static void test_D080_item2_sub_functions_are_the_four_gen_values(void)
{
    TEST_ASSERT_EQUAL_HEX8(UDS_READ_DTC_REPORT_NUMBER_OF_DTC_BY_STATUS_MASK,
                           dtc_snapshot_core_sub_function(DTC_SNAPSHOT_STEP_COUNT));
    TEST_ASSERT_EQUAL_HEX8(UDS_READ_DTC_REPORT_DTC_BY_STATUS_MASK,
                           dtc_snapshot_core_sub_function(DTC_SNAPSHOT_STEP_LIST));
    TEST_ASSERT_EQUAL_HEX8(UDS_READ_DTC_REPORT_DTC_SNAPSHOT_RECORD_BY_DTC_NUMBER,
                           dtc_snapshot_core_sub_function(DTC_SNAPSHOT_STEP_SNAPSHOT));
    TEST_ASSERT_EQUAL_HEX8(UDS_READ_DTC_REPORT_DTC_EXT_DATA_RECORD_BY_DTC_NUMBER,
                           dtc_snapshot_core_sub_function(DTC_SNAPSHOT_STEP_EXT_DATA));
    TEST_ASSERT_EQUAL_HEX8(0u, dtc_snapshot_core_sub_function(DTC_SNAPSHOT_STEP_NONE));
}

static void test_D080_nothing_is_due_before_the_first_epoch(void)
{
    TEST_ASSERT_FALSE(dtc_snapshot_core_due(&d));
    TEST_ASSERT_FALSE(build());
    TEST_ASSERT_FALSE(dtc_snapshot_core_run_active(&d));
    TEST_ASSERT_EQUAL(DTC_SNAPSHOT_STEP_NONE, dtc_snapshot_core_step(&d));
    TEST_ASSERT_EQUAL(DTC_SNAPSHOT_HALT_NONE, dtc_snapshot_core_halt(&d));
    TEST_ASSERT_EQUAL(DTC_SNAPSHOT_END_NONE, dtc_snapshot_core_last_end(&d));
}

static void test_D080_item2_a_nominal_run_reads_count_list_then_both_records_per_dtc(void)
{
    dtc_snapshot_core_start_epoch(&d);
    TEST_ASSERT_TRUE(dtc_snapshot_core_due(&d));
    TEST_ASSERT_EQUAL(DTC_SNAPSHOT_TRIGGER_KEY_ON, dtc_snapshot_core_trigger(&d));
    TEST_ASSERT_EQUAL_UINT8(1u, dtc_snapshot_core_runs_started(&d));
    run_to_the_end(2u);
    TEST_ASSERT_FALSE(dtc_snapshot_core_run_active(&d));
    TEST_ASSERT_EQUAL(DTC_SNAPSHOT_END_DONE, dtc_snapshot_core_last_end(&d));
    TEST_ASSERT_EQUAL_UINT8(6u, dtc_snapshot_core_requests_in_run(&d));
    TEST_ASSERT_EQUAL_UINT8(6u, dtc_snapshot_core_requests_in_epoch(&d));
    TEST_ASSERT_FALSE(dtc_snapshot_core_due(&d));
    TEST_ASSERT_FALSE(build());
    TEST_ASSERT_EQUAL(DTC_SNAPSHOT_HALT_NONE, dtc_snapshot_core_halt(&d));
    TEST_ASSERT_EQUAL_UINT32(1u, dtc_snapshot_core_stats(&d)->runs);
    TEST_ASSERT_EQUAL_UINT32(6u, dtc_snapshot_core_stats(&d)->requests);
    TEST_ASSERT_EQUAL_UINT32(6u, dtc_snapshot_core_stats(&d)->answers);
    TEST_ASSERT_EQUAL_UINT32(0u, dtc_snapshot_core_stats(&d)->halts);
}

static void test_D080_item2_one_request_is_in_flight_at_a_time(void)
{
    dtc_snapshot_core_start_epoch(&d);
    expect_count_request();
    TEST_ASSERT_FALSE(dtc_snapshot_core_due(&d));
    TEST_ASSERT_FALSE(build());
}

static void test_D080_item2_a_count_of_zero_ends_the_run_without_a_list_request(void)
{
    dtc_snapshot_core_start_epoch(&d);
    expect_count_request();
    TEST_ASSERT_TRUE(answer_count(0u));
    TEST_ASSERT_EQUAL(DTC_SNAPSHOT_END_NO_DTC, dtc_snapshot_core_last_end(&d));
    TEST_ASSERT_FALSE(build());
}

static void test_D080_item2_a_count_with_only_the_low_byte_set_continues(void)
{
    dtc_snapshot_core_start_epoch(&d);
    expect_count_request();
    TEST_ASSERT_TRUE(answer_count(0x0100u)); /* high byte alone is a non-zero count */
    TEST_ASSERT_EQUAL(DTC_SNAPSHOT_STEP_LIST, dtc_snapshot_core_step(&d));
}

static void test_D080_item2_a_list_without_a_matching_status_ends_the_run(void)
{
    dtc_snapshot_core_start_epoch(&d);
    expect_count_request();
    TEST_ASSERT_TRUE(answer_count(2u));
    expect_list_request();
    answer_list(2u, STATUS_NONE);
    TEST_ASSERT_EQUAL(DTC_SNAPSHOT_END_NO_MATCH, dtc_snapshot_core_last_end(&d));
    TEST_ASSERT_EQUAL_UINT8(0u, dtc_snapshot_core_dtc_count(&d));
    TEST_ASSERT_FALSE(build());
}

static void test_D080_item2_only_dtcs_with_a_matching_status_are_read(void)
{
    dtc_snapshot_core_start_epoch(&d);
    expect_count_request();
    TEST_ASSERT_TRUE(answer_count(3u));
    expect_list_request();
    uint8_t m[3u + 12u];
    const uint16_t len = build_list(m, 3u, STATUS_NONE);
    m[3u + 3u] = STATUS_MATCH;        /* DTC 0 */
    m[3u + 8u + 3u] = (uint8_t)(MASK & 0x04u); /* DTC 2: one mask bit is enough */
    TEST_ASSERT_TRUE(dtc_snapshot_core_on_positive(&d, m, len));
    TEST_ASSERT_EQUAL_UINT8(2u, dtc_snapshot_core_dtc_count(&d));
    expect_record_request(UDS_READ_DTC_REPORT_DTC_SNAPSHOT_RECORD_BY_DTC_NUMBER, 0u);
    answer_record(UDS_READ_DTC_REPORT_DTC_SNAPSHOT_RECORD_BY_DTC_NUMBER, 0u, 6u);
    expect_record_request(UDS_READ_DTC_REPORT_DTC_EXT_DATA_RECORD_BY_DTC_NUMBER, 0u);
    answer_record(UDS_READ_DTC_REPORT_DTC_EXT_DATA_RECORD_BY_DTC_NUMBER, 0u, 6u);
    expect_record_request(UDS_READ_DTC_REPORT_DTC_SNAPSHOT_RECORD_BY_DTC_NUMBER, 2u);
    TEST_ASSERT_NOT_NULL(dtc_snapshot_core_dtc(&d, 1u));
    TEST_ASSERT_NULL(dtc_snapshot_core_dtc(&d, 2u));
}

static void test_D080_item2_more_dtcs_than_the_cap_are_cut_at_the_cap_and_the_run_stays_in_budget(void)
{
    dtc_snapshot_core_start_epoch(&d);
    run_to_the_end((uint8_t)(CAP + 4u));
    TEST_ASSERT_EQUAL(DTC_SNAPSHOT_END_DONE, dtc_snapshot_core_last_end(&d));
    TEST_ASSERT_EQUAL_UINT8(CAP, dtc_snapshot_core_dtc_count(&d));
    TEST_ASSERT_EQUAL_UINT8(DTC_SNAPSHOT_MAX_REQUESTS_PER_RUN, dtc_snapshot_core_requests_in_run(&d));
    TEST_ASSERT_FALSE(build());
}

static void test_D080_item2_a_re_send_after_a_session_refusal_counts_against_the_budget(void)
{
    dtc_snapshot_core_start_epoch(&d);
    expect_count_request();
    TEST_ASSERT_EQUAL(DTC_SNAPSHOT_NRC_SESSION_LOST,
                      dtc_snapshot_core_on_nrc(&d, UDS_NRC_SERVICE_NOT_SUPPORTED_IN_ACTIVE_SESSION));
    /* the run now needs 19 requests; the 19th is never built */
    expect_count_request();
    TEST_ASSERT_TRUE(answer_count(CAP));
    expect_list_request();
    answer_list(CAP, STATUS_MATCH);
    for (uint8_t i = 0u; i < CAP; i++) {
        expect_record_request(UDS_READ_DTC_REPORT_DTC_SNAPSHOT_RECORD_BY_DTC_NUMBER, i);
        answer_record(UDS_READ_DTC_REPORT_DTC_SNAPSHOT_RECORD_BY_DTC_NUMBER, i, 12u);
        if (i < (CAP - 1u)) {
            expect_record_request(UDS_READ_DTC_REPORT_DTC_EXT_DATA_RECORD_BY_DTC_NUMBER, i);
            answer_record(UDS_READ_DTC_REPORT_DTC_EXT_DATA_RECORD_BY_DTC_NUMBER, i, 9u);
        }
    }
    TEST_ASSERT_EQUAL_UINT8(DTC_SNAPSHOT_MAX_REQUESTS_PER_RUN, dtc_snapshot_core_requests_in_run(&d));
    TEST_ASSERT_TRUE(dtc_snapshot_core_due(&d));
    TEST_ASSERT_FALSE(build());
    TEST_ASSERT_EQUAL(DTC_SNAPSHOT_END_BUDGET, dtc_snapshot_core_last_end(&d));
    TEST_ASSERT_FALSE(dtc_snapshot_core_run_active(&d));
    TEST_ASSERT_EQUAL(DTC_SNAPSHOT_HALT_NONE, dtc_snapshot_core_halt(&d));
}

/* ------------------------------------------------------------------------- */
/* Malformed answers                                                          */
/* ------------------------------------------------------------------------- */

static void malformed_count(uint16_t len)
{
    setUp();
    dtc_snapshot_core_start_epoch(&d);
    expect_count_request();
    const uint8_t m[8] = {POS_SID, UDS_READ_DTC_REPORT_NUMBER_OF_DTC_BY_STATUS_MASK, 0xFFu, 0x01u,
                          0x00u, 0x02u, 0x00u, 0x00u};
    TEST_ASSERT_TRUE(dtc_snapshot_core_on_positive(&d, m, len));
    TEST_ASSERT_EQUAL(DTC_SNAPSHOT_END_MALFORMED, dtc_snapshot_core_last_end(&d));
    TEST_ASSERT_FALSE(build());
    TEST_ASSERT_EQUAL(DTC_SNAPSHOT_HALT_NONE, dtc_snapshot_core_halt(&d)); /* the warm-idle run may still come */
}

static void test_D080_item2_a_count_answer_of_the_wrong_length_ends_the_run(void)
{
    malformed_count(5u);
    malformed_count(7u);
    malformed_count(2u + 0u + 2u);
}

static void malformed_list(uint16_t len)
{
    setUp();
    dtc_snapshot_core_start_epoch(&d);
    expect_count_request();
    TEST_ASSERT_TRUE(answer_count(2u));
    expect_list_request();
    uint8_t m[3u + 16u];
    (void)build_list(m, 4u, STATUS_MATCH);
    TEST_ASSERT_TRUE(dtc_snapshot_core_on_positive(&d, m, len));
    TEST_ASSERT_EQUAL(DTC_SNAPSHOT_END_MALFORMED, dtc_snapshot_core_last_end(&d));
    /* a malformed list never yields a 0x04/0x06 request: nothing from it was stored */
    TEST_ASSERT_EQUAL_UINT8(0u, dtc_snapshot_core_dtc_count(&d));
    TEST_ASSERT_FALSE(dtc_snapshot_core_due(&d));
    TEST_ASSERT_FALSE(build());
}

static void test_D080_item2_a_list_answer_that_is_not_whole_records_ends_the_run_and_stores_nothing(void)
{
    malformed_list(2u);  /* shorter than the header */
    malformed_list(4u);  /* a partial record */
    malformed_list(3u + 4u + 3u);
    malformed_list(3u + 4u + 5u);
}

static void test_D080_item2_a_list_header_alone_matches_no_dtc(void)
{
    dtc_snapshot_core_start_epoch(&d);
    expect_count_request();
    TEST_ASSERT_TRUE(answer_count(1u));
    expect_list_request();
    answer_list(0u, STATUS_MATCH);
    TEST_ASSERT_EQUAL(DTC_SNAPSHOT_END_NO_MATCH, dtc_snapshot_core_last_end(&d));
}

static void test_D080_item2_a_record_answer_shorter_than_its_header_ends_the_run(void)
{
    for (uint8_t sub = 0u; sub < 2u; sub++) {
        setUp();
        dtc_snapshot_core_start_epoch(&d);
        expect_count_request();
        TEST_ASSERT_TRUE(answer_count(1u));
        expect_list_request();
        answer_list(1u, STATUS_MATCH);
        if (sub == 1u) {
            expect_record_request(UDS_READ_DTC_REPORT_DTC_SNAPSHOT_RECORD_BY_DTC_NUMBER, 0u);
            answer_record(UDS_READ_DTC_REPORT_DTC_SNAPSHOT_RECORD_BY_DTC_NUMBER, 0u, 6u);
            expect_record_request(UDS_READ_DTC_REPORT_DTC_EXT_DATA_RECORD_BY_DTC_NUMBER, 0u);
            answer_record(UDS_READ_DTC_REPORT_DTC_EXT_DATA_RECORD_BY_DTC_NUMBER, 0u, 5u);
        } else {
            expect_record_request(UDS_READ_DTC_REPORT_DTC_SNAPSHOT_RECORD_BY_DTC_NUMBER, 0u);
            answer_record(UDS_READ_DTC_REPORT_DTC_SNAPSHOT_RECORD_BY_DTC_NUMBER, 0u, 5u);
        }
        TEST_ASSERT_EQUAL(DTC_SNAPSHOT_END_MALFORMED, dtc_snapshot_core_last_end(&d));
        TEST_ASSERT_FALSE(build());
    }
}

static void test_D080_item2_match_positive_checks_sid_sub_function_and_the_dtc_echo(void)
{
    dtc_snapshot_core_start_epoch(&d);
    const uint8_t no_fc[2] = {POS_SID, UDS_READ_DTC_REPORT_NUMBER_OF_DTC_BY_STATUS_MASK};
    TEST_ASSERT_FALSE(dtc_snapshot_core_match_positive(&d, no_fc, 2u)); /* nothing in flight */
    expect_count_request();
    TEST_ASSERT_TRUE(dtc_snapshot_core_match_positive(&d, no_fc, 2u)); /* a First Frame's bytes */
    const uint8_t wrong_sid[2] = {0x62u, UDS_READ_DTC_REPORT_NUMBER_OF_DTC_BY_STATUS_MASK};
    const uint8_t wrong_sub[2] = {POS_SID, UDS_READ_DTC_REPORT_DTC_BY_STATUS_MASK};
    TEST_ASSERT_FALSE(dtc_snapshot_core_match_positive(&d, wrong_sid, 2u));
    TEST_ASSERT_FALSE(dtc_snapshot_core_match_positive(&d, wrong_sub, 2u));
    TEST_ASSERT_FALSE(dtc_snapshot_core_match_positive(&d, no_fc, 1u));
    TEST_ASSERT_FALSE(dtc_snapshot_core_match_positive(&d, NULL, 2u));
    TEST_ASSERT_FALSE(dtc_snapshot_core_on_positive(&d, wrong_sub, 2u)); /* not accepted */
    TEST_ASSERT_TRUE(dtc_snapshot_core_in_flight(&d));
    TEST_ASSERT_TRUE(answer_count(1u));
    expect_list_request();
    answer_list(1u, STATUS_MATCH);
    expect_record_request(UDS_READ_DTC_REPORT_DTC_SNAPSHOT_RECORD_BY_DTC_NUMBER, 0u);
    uint8_t m[8] = {POS_SID, UDS_READ_DTC_REPORT_DTC_SNAPSHOT_RECORD_BY_DTC_NUMBER, 0u, 0u, 0u, 0u};
    dtc_number(0u, &m[2]);
    TEST_ASSERT_TRUE(dtc_snapshot_core_match_positive(&d, m, 5u)); /* FF bytes: SID, sub, DTC */
    TEST_ASSERT_FALSE(dtc_snapshot_core_match_positive(&d, m, 4u)); /* DTC cut short */
    for (uint8_t k = 2u; k < 5u; k++) {
        m[k] ^= 0x01u; /* a wrong echo in any DTC byte */
        TEST_ASSERT_FALSE(dtc_snapshot_core_match_positive(&d, m, 6u));
        TEST_ASSERT_FALSE(dtc_snapshot_core_on_positive(&d, m, 6u));
        m[k] ^= 0x01u;
    }
    TEST_ASSERT_TRUE(dtc_snapshot_core_in_flight(&d));
}

static void test_D080_item2_expected_prefix_is_the_positive_sid_sub_function_and_for_records_the_dtc(void)
{
    uint8_t p[DTC_SNAPSHOT_PREFIX_MAX];
    uint8_t n = 0xFFu;
    dtc_snapshot_core_start_epoch(&d);
    TEST_ASSERT_FALSE(dtc_snapshot_core_expected_prefix(&d, p, &n)); /* nothing in flight */
    expect_count_request();
    TEST_ASSERT_TRUE(dtc_snapshot_core_expected_prefix(&d, p, &n));
    TEST_ASSERT_EQUAL_UINT8(2u, n);
    TEST_ASSERT_EQUAL_HEX8(POS_SID, p[0]);
    TEST_ASSERT_EQUAL_HEX8(UDS_READ_DTC_REPORT_NUMBER_OF_DTC_BY_STATUS_MASK, p[1]);
    TEST_ASSERT_TRUE(answer_count(1u));
    expect_list_request();
    TEST_ASSERT_TRUE(dtc_snapshot_core_expected_prefix(&d, p, &n));
    TEST_ASSERT_EQUAL_UINT8(2u, n);
    TEST_ASSERT_EQUAL_HEX8(UDS_READ_DTC_REPORT_DTC_BY_STATUS_MASK, p[1]);
    answer_list(1u, STATUS_MATCH);
    expect_record_request(UDS_READ_DTC_REPORT_DTC_SNAPSHOT_RECORD_BY_DTC_NUMBER, 0u);
    TEST_ASSERT_TRUE(dtc_snapshot_core_expected_prefix(&d, p, &n));
    TEST_ASSERT_EQUAL_UINT8(DTC_SNAPSHOT_PREFIX_MAX, n);
    uint8_t e[3];
    dtc_number(0u, e);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(e, &p[2], 3u);
    TEST_ASSERT_FALSE(dtc_snapshot_core_expected_prefix(&d, NULL, &n));
    TEST_ASSERT_FALSE(dtc_snapshot_core_expected_prefix(&d, p, NULL));
    TEST_ASSERT_FALSE(dtc_snapshot_core_expected_prefix(NULL, p, &n));
}

/* ------------------------------------------------------------------------- */
/* NRCs, timeouts, halts                                                      */
/* ------------------------------------------------------------------------- */

static void test_D080_item2_nrc_0x11_0x12_0x31_mean_not_served_for_the_epoch(void)
{
    const uint8_t codes[3] = {UDS_NRC_SERVICE_NOT_SUPPORTED, UDS_NRC_SUBFUNCTION_NOT_SUPPORTED,
                              UDS_NRC_REQUEST_OUT_OF_RANGE};
    for (uint32_t i = 0u; i < 3u; i++) {
        setUp();
        dtc_snapshot_core_start_epoch(&d);
        expect_count_request();
        TEST_ASSERT_EQUAL(DTC_SNAPSHOT_NRC_ENDED, dtc_snapshot_core_on_nrc(&d, codes[i]));
        TEST_ASSERT_EQUAL(DTC_SNAPSHOT_HALT_NOT_SERVED, dtc_snapshot_core_halt(&d));
        TEST_ASSERT_EQUAL(DTC_SNAPSHOT_END_HALTED, dtc_snapshot_core_last_end(&d));
        TEST_ASSERT_FALSE(dtc_snapshot_core_due(&d));
        TEST_ASSERT_FALSE(build());
        TEST_ASSERT_FALSE(dtc_snapshot_core_warm_idle_step(&d, 0u, true)); /* and the warm-idle run neither */
        TEST_ASSERT_FALSE(dtc_snapshot_core_warm_idle_step(&d, IDLE_MS, true));
        TEST_ASSERT_EQUAL_UINT8(1u, dtc_snapshot_core_runs_started(&d));
        TEST_ASSERT_EQUAL_UINT32(1u, dtc_snapshot_core_stats(&d)->halts);
    }
}

static void test_D080_item2_another_nrc_only_ends_the_run_and_the_warm_idle_run_may_come(void)
{
    dtc_snapshot_core_start_epoch(&d);
    expect_count_request();
    TEST_ASSERT_EQUAL(DTC_SNAPSHOT_NRC_ENDED,
                      dtc_snapshot_core_on_nrc(&d, UDS_NRC_CONDITIONS_NOT_CORRECT));
    TEST_ASSERT_EQUAL(DTC_SNAPSHOT_END_REFUSED, dtc_snapshot_core_last_end(&d));
    TEST_ASSERT_EQUAL(DTC_SNAPSHOT_HALT_NONE, dtc_snapshot_core_halt(&d));
    TEST_ASSERT_FALSE(build());
    TEST_ASSERT_FALSE(dtc_snapshot_core_warm_idle_step(&d, 0u, true));
    TEST_ASSERT_TRUE(dtc_snapshot_core_warm_idle_step(&d, IDLE_MS, true));
    TEST_ASSERT_EQUAL(DTC_SNAPSHOT_TRIGGER_WARM_IDLE, dtc_snapshot_core_trigger(&d));
    expect_count_request();
}

static void test_D080_item4_nrc_0x78_waits_and_keeps_the_request_in_flight(void)
{
    dtc_snapshot_core_start_epoch(&d);
    expect_count_request();
    TEST_ASSERT_EQUAL(DTC_SNAPSHOT_NRC_WAIT, dtc_snapshot_core_on_nrc(&d, UDS_NRC_RESPONSE_PENDING));
    TEST_ASSERT_TRUE(dtc_snapshot_core_in_flight(&d));
    TEST_ASSERT_FALSE(build());
    TEST_ASSERT_EQUAL_UINT8(1u, dtc_snapshot_core_requests_in_run(&d));
    TEST_ASSERT_TRUE(answer_count(0u)); /* the answer still resolves it */
}

static void test_D080_item2_an_nrc_without_a_request_in_flight_does_nothing(void)
{
    dtc_snapshot_core_start_epoch(&d);
    TEST_ASSERT_EQUAL(DTC_SNAPSHOT_NRC_ENDED, dtc_snapshot_core_on_nrc(&d, UDS_NRC_SERVICE_NOT_SUPPORTED));
    TEST_ASSERT_EQUAL(DTC_SNAPSHOT_HALT_NONE, dtc_snapshot_core_halt(&d));
    TEST_ASSERT_EQUAL(DTC_SNAPSHOT_NRC_ENDED, dtc_snapshot_core_on_nrc(NULL, UDS_NRC_SERVICE_NOT_SUPPORTED));
}

static void session_refusal_once_then_the_same_request(uint8_t code)
{
    setUp();
    dtc_snapshot_core_start_epoch(&d);
    expect_count_request();
    TEST_ASSERT_TRUE(answer_count(1u));
    expect_list_request();
    TEST_ASSERT_EQUAL(DTC_SNAPSHOT_NRC_SESSION_LOST, dtc_snapshot_core_on_nrc(&d, code));
    TEST_ASSERT_EQUAL(DTC_SNAPSHOT_HALT_NONE, dtc_snapshot_core_halt(&d));
    TEST_ASSERT_TRUE(dtc_snapshot_core_run_active(&d));
    expect_list_request(); /* the same step again once the session is back */
    TEST_ASSERT_EQUAL_UINT8(3u, dtc_snapshot_core_requests_in_run(&d)); /* and it counts */
    answer_list(1u, STATUS_MATCH);
}

static void test_D080_item2_nrc_0x7e_and_0x7f_once_resend_the_same_request(void)
{
    session_refusal_once_then_the_same_request(UDS_NRC_SUBFUNCTION_NOT_SUPPORTED_IN_ACTIVE_SESSION);
    session_refusal_once_then_the_same_request(UDS_NRC_SERVICE_NOT_SUPPORTED_IN_ACTIVE_SESSION);
}

static void test_D080_item2_the_second_session_refusal_in_the_epoch_halts(void)
{
    dtc_snapshot_core_start_epoch(&d);
    expect_count_request();
    TEST_ASSERT_EQUAL(DTC_SNAPSHOT_NRC_SESSION_LOST,
                      dtc_snapshot_core_on_nrc(&d, UDS_NRC_SERVICE_NOT_SUPPORTED_IN_ACTIVE_SESSION));
    expect_count_request();
    TEST_ASSERT_EQUAL(DTC_SNAPSHOT_NRC_SESSION_LOST,
                      dtc_snapshot_core_on_nrc(&d, UDS_NRC_SUBFUNCTION_NOT_SUPPORTED_IN_ACTIVE_SESSION));
    TEST_ASSERT_EQUAL(DTC_SNAPSHOT_HALT_SESSION_REFUSED, dtc_snapshot_core_halt(&d));
    TEST_ASSERT_FALSE(dtc_snapshot_core_run_active(&d));
    TEST_ASSERT_FALSE(build());
}

static void test_D080_item3_a_timeout_halts_the_epoch_with_no_answer(void)
{
    dtc_snapshot_core_start_epoch(&d);
    dtc_snapshot_core_on_timeout(&d); /* nothing in flight: nothing */
    TEST_ASSERT_EQUAL(DTC_SNAPSHOT_HALT_NONE, dtc_snapshot_core_halt(&d));
    expect_count_request();
    dtc_snapshot_core_on_timeout(&d);
    TEST_ASSERT_EQUAL(DTC_SNAPSHOT_HALT_NO_ANSWER, dtc_snapshot_core_halt(&d));
    TEST_ASSERT_EQUAL(DTC_SNAPSHOT_END_HALTED, dtc_snapshot_core_last_end(&d));
    TEST_ASSERT_FALSE(dtc_snapshot_core_in_flight(&d));
    TEST_ASSERT_FALSE(build());
}

static void test_D080_item3_a_failed_segmented_reception_halts_the_epoch(void)
{
    dtc_snapshot_core_start_epoch(&d);
    dtc_snapshot_core_on_segmented_failure(&d);
    TEST_ASSERT_EQUAL(DTC_SNAPSHOT_HALT_NONE, dtc_snapshot_core_halt(&d));
    expect_count_request();
    dtc_snapshot_core_on_segmented_failure(&d);
    TEST_ASSERT_EQUAL(DTC_SNAPSHOT_HALT_SEGMENTED, dtc_snapshot_core_halt(&d));
    TEST_ASSERT_FALSE(build());
    TEST_ASSERT_EQUAL_UINT32(1u, dtc_snapshot_core_stats(&d)->halts);
}

static void test_D080_item3_the_first_halt_reason_stays(void)
{
    dtc_snapshot_core_start_epoch(&d);
    expect_count_request();
    dtc_snapshot_core_on_timeout(&d);
    dtc_snapshot_core_halt_after_answer(&d, DTC_SNAPSHOT_HALT_SLOW);
    TEST_ASSERT_EQUAL(DTC_SNAPSHOT_HALT_NO_ANSWER, dtc_snapshot_core_halt(&d));
    TEST_ASSERT_EQUAL_UINT32(1u, dtc_snapshot_core_stats(&d)->halts);
}

static void test_D080_item3_a_halt_after_the_answer_lets_the_request_in_flight_resolve(void)
{
    dtc_snapshot_core_start_epoch(&d);
    expect_count_request();
    dtc_snapshot_core_halt_after_answer(&d, DTC_SNAPSHOT_HALT_SLOW);
    TEST_ASSERT_EQUAL(DTC_SNAPSHOT_HALT_SLOW, dtc_snapshot_core_halt(&d));
    TEST_ASSERT_TRUE(dtc_snapshot_core_in_flight(&d));
    TEST_ASSERT_TRUE(dtc_snapshot_core_run_active(&d)); /* still waiting for its answer */
    TEST_ASSERT_TRUE(answer_count(3u));                 /* resolves normally ... */
    TEST_ASSERT_FALSE(dtc_snapshot_core_run_active(&d)); /* ... then the run ends */
    TEST_ASSERT_EQUAL(DTC_SNAPSHOT_END_HALTED, dtc_snapshot_core_last_end(&d));
    TEST_ASSERT_FALSE(build());
}

static void test_D080_item3_a_halt_with_nothing_in_flight_ends_the_run_at_once(void)
{
    dtc_snapshot_core_start_epoch(&d);
    dtc_snapshot_core_halt_after_answer(&d, DTC_SNAPSHOT_HALT_LATE_FF);
    TEST_ASSERT_EQUAL(DTC_SNAPSHOT_HALT_LATE_FF, dtc_snapshot_core_halt(&d));
    TEST_ASSERT_FALSE(dtc_snapshot_core_run_active(&d));
    dtc_snapshot_core_halt_after_answer(&d, DTC_SNAPSHOT_HALT_NONE);
    dtc_snapshot_core_halt_after_answer(NULL, DTC_SNAPSHOT_HALT_SLOW);
    TEST_ASSERT_EQUAL(DTC_SNAPSHOT_HALT_LATE_FF, dtc_snapshot_core_halt(&d));
}

static void test_D080_item3_a_halt_with_the_nrc_session_refusal_pending_ends_the_run(void)
{
    dtc_snapshot_core_start_epoch(&d);
    expect_count_request();
    dtc_snapshot_core_halt_after_answer(&d, DTC_SNAPSHOT_HALT_SLOW);
    TEST_ASSERT_EQUAL(DTC_SNAPSHOT_NRC_SESSION_LOST,
                      dtc_snapshot_core_on_nrc(&d, UDS_NRC_SERVICE_NOT_SUPPORTED_IN_ACTIVE_SESSION));
    TEST_ASSERT_FALSE(dtc_snapshot_core_run_active(&d));
    TEST_ASSERT_FALSE(build());
}

/* ------------------------------------------------------------------------- */
/* Runs per epoch, warm idle                                                  */
/* ------------------------------------------------------------------------- */

static void test_D080_item1_the_warm_idle_run_follows_a_finished_key_on_run(void)
{
    dtc_snapshot_core_start_epoch(&d);
    run_to_the_end(1u);
    TEST_ASSERT_FALSE(dtc_snapshot_core_warm_idle_step(&d, 1000u, true));
    TEST_ASSERT_TRUE(dtc_snapshot_core_warm_idle_step(&d, 1000u + IDLE_MS, true));
    TEST_ASSERT_EQUAL_UINT8(2u, dtc_snapshot_core_runs_started(&d));
    TEST_ASSERT_EQUAL(DTC_SNAPSHOT_TRIGGER_WARM_IDLE, dtc_snapshot_core_trigger(&d));
    TEST_ASSERT_EQUAL_UINT8(0u, dtc_snapshot_core_requests_in_run(&d));
    TEST_ASSERT_EQUAL_UINT8(4u, dtc_snapshot_core_requests_in_epoch(&d)); /* the epoch keeps counting */
    expect_count_request();
    TEST_ASSERT_TRUE(answer_count(0u));
    /* the third run never comes */
    TEST_ASSERT_FALSE(dtc_snapshot_core_warm_idle_step(&d, 1000u + IDLE_MS + 1u, true));
    TEST_ASSERT_FALSE(dtc_snapshot_core_warm_idle_step(&d, 1000u + (3u * IDLE_MS), true));
    TEST_ASSERT_EQUAL_UINT8(2u, dtc_snapshot_core_runs_started(&d));
    TEST_ASSERT_FALSE(dtc_snapshot_core_due(&d));
    TEST_ASSERT_EQUAL_UINT32(2u, dtc_snapshot_core_stats(&d)->runs);
}

static void test_D080_item1_a_warm_idle_while_the_key_on_run_is_active_is_not_used_up(void)
{
    dtc_snapshot_core_start_epoch(&d);
    TEST_ASSERT_FALSE(dtc_snapshot_core_warm_idle_step(&d, 0u, true));
    TEST_ASSERT_FALSE(dtc_snapshot_core_warm_idle_step(&d, IDLE_MS, true)); /* key-on still active */
    TEST_ASSERT_FALSE(dtc_snapshot_core_warm_idle_step(&d, IDLE_MS + 5u, true));
    TEST_ASSERT_EQUAL_UINT8(1u, dtc_snapshot_core_runs_started(&d));
    run_to_the_end(1u);
    /* the hold has lasted all along, so the next step arms it */
    TEST_ASSERT_TRUE(dtc_snapshot_core_warm_idle_step(&d, IDLE_MS + 10u, true));
    TEST_ASSERT_EQUAL_UINT8(2u, dtc_snapshot_core_runs_started(&d));
}

static void test_D080_item1_the_warm_idle_run_after_a_halt_is_refused_and_used_up(void)
{
    dtc_snapshot_core_start_epoch(&d);
    expect_count_request();
    dtc_snapshot_core_on_timeout(&d);
    TEST_ASSERT_FALSE(dtc_snapshot_core_warm_idle_step(&d, 0u, true));
    TEST_ASSERT_FALSE(dtc_snapshot_core_warm_idle_step(&d, IDLE_MS, true));
    TEST_ASSERT_EQUAL_UINT8(1u, dtc_snapshot_core_runs_started(&d));
    TEST_ASSERT_FALSE(dtc_snapshot_core_run_active(&d));
}

static void test_D080_item1_the_hold_must_last_the_gen_duration_exactly(void)
{
    dtc_snapshot_core_start_epoch(&d);
    expect_count_request();
    TEST_ASSERT_TRUE(answer_count(0u));
    TEST_ASSERT_FALSE(dtc_snapshot_core_warm_idle_step(&d, 500u, true));
    TEST_ASSERT_FALSE(dtc_snapshot_core_warm_idle_step(&d, 500u + IDLE_MS - 1u, true));
    TEST_ASSERT_TRUE(dtc_snapshot_core_warm_idle_step(&d, 500u + IDLE_MS, true));
}

static void test_D080_item1_any_break_restarts_the_hold(void)
{
    dtc_snapshot_core_start_epoch(&d);
    expect_count_request();
    TEST_ASSERT_TRUE(answer_count(0u));
    TEST_ASSERT_FALSE(dtc_snapshot_core_warm_idle_step(&d, 0u, true));
    TEST_ASSERT_FALSE(dtc_snapshot_core_warm_idle_step(&d, IDLE_MS - 1u, true));
    TEST_ASSERT_FALSE(dtc_snapshot_core_warm_idle_step(&d, IDLE_MS, false)); /* one false step */
    TEST_ASSERT_FALSE(dtc_snapshot_core_warm_idle_step(&d, IDLE_MS + 1u, true));
    TEST_ASSERT_FALSE(dtc_snapshot_core_warm_idle_step(&d, IDLE_MS + IDLE_MS, true));
    TEST_ASSERT_TRUE(dtc_snapshot_core_warm_idle_step(&d, IDLE_MS + 1u + IDLE_MS, true));
}

static void test_D080_item1_the_hold_timer_is_wrap_safe(void)
{
    dtc_snapshot_core_start_epoch(&d);
    expect_count_request();
    TEST_ASSERT_TRUE(answer_count(0u));
    const uint32_t t0 = UINT32_MAX - 5000u;
    TEST_ASSERT_FALSE(dtc_snapshot_core_warm_idle_step(&d, t0, true));
    TEST_ASSERT_FALSE(dtc_snapshot_core_warm_idle_step(&d, t0 + IDLE_MS - 1u, true)); /* wraps past 0 */
    TEST_ASSERT_TRUE(dtc_snapshot_core_warm_idle_step(&d, t0 + IDLE_MS, true));
}

static void test_D080_item1_nothing_runs_without_an_epoch_and_a_new_epoch_resets_the_timer(void)
{
    TEST_ASSERT_FALSE(dtc_snapshot_core_warm_idle_step(&d, 0u, true));
    TEST_ASSERT_FALSE(dtc_snapshot_core_warm_idle_step(&d, IDLE_MS, true)); /* no epoch: no run */
    TEST_ASSERT_EQUAL_UINT8(0u, dtc_snapshot_core_runs_started(&d));
    dtc_snapshot_core_start_epoch(&d);
    expect_count_request();
    TEST_ASSERT_TRUE(answer_count(0u));
    TEST_ASSERT_FALSE(dtc_snapshot_core_warm_idle_step(&d, IDLE_MS + 1u, true)); /* the hold starts here */
    dtc_snapshot_core_start_epoch(&d);                                          /* ... and is forgotten */
    expect_count_request();
    TEST_ASSERT_TRUE(answer_count(0u));
    TEST_ASSERT_FALSE(dtc_snapshot_core_warm_idle_step(&d, (2u * IDLE_MS) + 1u, true));
    TEST_ASSERT_TRUE(dtc_snapshot_core_warm_idle_step(&d, (3u * IDLE_MS) + 1u, true));
    TEST_ASSERT_FALSE(dtc_snapshot_core_warm_idle_step(NULL, 0u, true));
}

static void test_D080_item1_a_new_epoch_forgets_halts_and_counts_and_restarts_the_key_on_run(void)
{
    dtc_snapshot_core_start_epoch(&d);
    expect_count_request();
    TEST_ASSERT_EQUAL(DTC_SNAPSHOT_NRC_ENDED,
                      dtc_snapshot_core_on_nrc(&d, UDS_NRC_SERVICE_NOT_SUPPORTED));
    TEST_ASSERT_EQUAL(DTC_SNAPSHOT_HALT_NOT_SERVED, dtc_snapshot_core_halt(&d));
    dtc_snapshot_core_start_epoch(&d);
    TEST_ASSERT_EQUAL(DTC_SNAPSHOT_HALT_NONE, dtc_snapshot_core_halt(&d));
    TEST_ASSERT_EQUAL_UINT8(1u, dtc_snapshot_core_runs_started(&d));
    TEST_ASSERT_EQUAL_UINT8(0u, dtc_snapshot_core_requests_in_epoch(&d));
    TEST_ASSERT_EQUAL(DTC_SNAPSHOT_TRIGGER_KEY_ON, dtc_snapshot_core_trigger(&d));
    run_to_the_end(1u);
    TEST_ASSERT_FALSE(dtc_snapshot_core_warm_idle_step(&d, 0u, true));
    TEST_ASSERT_TRUE(dtc_snapshot_core_warm_idle_step(&d, IDLE_MS, true));
    TEST_ASSERT_EQUAL_UINT8(2u, dtc_snapshot_core_runs_started(&d));
    /* a session refusal count of the old epoch is gone too */
    dtc_snapshot_core_start_epoch(&d);
    expect_count_request();
    TEST_ASSERT_EQUAL(DTC_SNAPSHOT_NRC_SESSION_LOST,
                      dtc_snapshot_core_on_nrc(&d, UDS_NRC_SERVICE_NOT_SUPPORTED_IN_ACTIVE_SESSION));
    TEST_ASSERT_EQUAL(DTC_SNAPSHOT_HALT_NONE, dtc_snapshot_core_halt(&d));
    /* a new epoch in the middle of a request in flight drops it silently */
    expect_count_request();
    dtc_snapshot_core_start_epoch(&d);
    TEST_ASSERT_FALSE(dtc_snapshot_core_in_flight(&d));
    TEST_ASSERT_EQUAL(DTC_SNAPSHOT_END_NONE, dtc_snapshot_core_last_end(&d));
    expect_count_request();
}

static void test_D080_unsend_restores_the_budgets_and_the_request_is_due_again(void)
{
    dtc_snapshot_core_start_epoch(&d);
    dtc_snapshot_core_unsend(&d); /* nothing in flight */
    TEST_ASSERT_EQUAL_UINT8(0u, dtc_snapshot_core_requests_in_run(&d));
    expect_count_request();
    TEST_ASSERT_EQUAL_UINT8(1u, dtc_snapshot_core_requests_in_epoch(&d));
    dtc_snapshot_core_unsend(&d);
    TEST_ASSERT_FALSE(dtc_snapshot_core_in_flight(&d));
    TEST_ASSERT_EQUAL_UINT8(0u, dtc_snapshot_core_requests_in_run(&d));
    TEST_ASSERT_EQUAL_UINT8(0u, dtc_snapshot_core_requests_in_epoch(&d));
    TEST_ASSERT_EQUAL_UINT32(0u, dtc_snapshot_core_stats(&d)->requests);
    expect_count_request();
    dtc_snapshot_core_unsend(NULL);
}

/* ------------------------------------------------------------------------- */
/* Warm-idle conditions                                                       */
/* ------------------------------------------------------------------------- */

static void test_D080_item1_every_comparison_operator_and_its_boundary(void)
{
    const float t = 80.0f;
    const float below = 79.9f;
    const float above = 80.1f;
    TEST_ASSERT_TRUE(dtc_snapshot_core_condition_met(VEHICLE_CL250_DTC_OP_LT, below, t));
    TEST_ASSERT_FALSE(dtc_snapshot_core_condition_met(VEHICLE_CL250_DTC_OP_LT, t, t));
    TEST_ASSERT_FALSE(dtc_snapshot_core_condition_met(VEHICLE_CL250_DTC_OP_LT, above, t));
    TEST_ASSERT_TRUE(dtc_snapshot_core_condition_met(VEHICLE_CL250_DTC_OP_LE, below, t));
    TEST_ASSERT_TRUE(dtc_snapshot_core_condition_met(VEHICLE_CL250_DTC_OP_LE, t, t));
    TEST_ASSERT_FALSE(dtc_snapshot_core_condition_met(VEHICLE_CL250_DTC_OP_LE, above, t));
    TEST_ASSERT_FALSE(dtc_snapshot_core_condition_met(VEHICLE_CL250_DTC_OP_GT, below, t));
    TEST_ASSERT_FALSE(dtc_snapshot_core_condition_met(VEHICLE_CL250_DTC_OP_GT, t, t));
    TEST_ASSERT_TRUE(dtc_snapshot_core_condition_met(VEHICLE_CL250_DTC_OP_GT, above, t));
    TEST_ASSERT_FALSE(dtc_snapshot_core_condition_met(VEHICLE_CL250_DTC_OP_GE, below, t));
    TEST_ASSERT_TRUE(dtc_snapshot_core_condition_met(VEHICLE_CL250_DTC_OP_GE, t, t));
    TEST_ASSERT_TRUE(dtc_snapshot_core_condition_met(VEHICLE_CL250_DTC_OP_GE, above, t));
    TEST_ASSERT_FALSE(dtc_snapshot_core_condition_met(VEHICLE_CL250_DTC_OP_EQ, below, t));
    TEST_ASSERT_TRUE(dtc_snapshot_core_condition_met(VEHICLE_CL250_DTC_OP_EQ, t, t));
    TEST_ASSERT_FALSE(dtc_snapshot_core_condition_met(VEHICLE_CL250_DTC_OP_EQ, above, t));
    TEST_ASSERT_TRUE(dtc_snapshot_core_condition_met(VEHICLE_CL250_DTC_OP_EQ, 0.0f, 0.0f));
    TEST_ASSERT_TRUE(dtc_snapshot_core_condition_met(VEHICLE_CL250_DTC_OP_EQ, -0.0f, 0.0f));
}

static void test_D080_item1_nan_infinity_and_an_unknown_operator_are_never_met(void)
{
    const uint8_t ops[5] = {VEHICLE_CL250_DTC_OP_LT, VEHICLE_CL250_DTC_OP_LE,
                            VEHICLE_CL250_DTC_OP_GT, VEHICLE_CL250_DTC_OP_GE,
                            VEHICLE_CL250_DTC_OP_EQ};
    for (uint32_t i = 0u; i < 5u; i++) {
        TEST_ASSERT_FALSE(dtc_snapshot_core_condition_met(ops[i], NAN, 1.0f));
        TEST_ASSERT_FALSE(dtc_snapshot_core_condition_met(ops[i], NAN, NAN));
        TEST_ASSERT_FALSE(dtc_snapshot_core_condition_met(ops[i], 1.0f, NAN));
        TEST_ASSERT_FALSE(dtc_snapshot_core_condition_met(ops[i], INFINITY, 1.0f));
        TEST_ASSERT_FALSE(dtc_snapshot_core_condition_met(ops[i], -INFINITY, 1.0f));
    }
    TEST_ASSERT_FALSE(dtc_snapshot_core_condition_met(VEHICLE_CL250_DTC_OP_EQ + 1u, 1.0f, 1.0f));
    TEST_ASSERT_FALSE(dtc_snapshot_core_condition_met(0xFFu, 1.0f, 1.0f));
}

static void test_D080_item1_the_gen_warm_idle_table_is_well_formed(void)
{
    for (uint32_t i = 0u; i < VEHICLE_CL250_DTC_WARM_IDLE_CONDITION_COUNT; i++) {
        TEST_ASSERT_LESS_THAN_UINT32(VEHICLE_CL250_DID_COUNT, vehicle_cl250_dtc_warm_idle[i].did_index);
        TEST_ASSERT_LESS_OR_EQUAL_UINT8(VEHICLE_CL250_DTC_OP_EQ, vehicle_cl250_dtc_warm_idle[i].op);
        TEST_ASSERT_TRUE(isfinite(vehicle_cl250_dtc_warm_idle[i].value));
    }
}

/* ------------------------------------------------------------------------- */
/* Misc                                                                       */
/* ------------------------------------------------------------------------- */

static void test_D080_every_request_of_a_full_run_passes_the_gen_gate_and_is_never_0x14(void)
{
    dtc_snapshot_core_start_epoch(&d);
    expect_count_request();
    TEST_ASSERT_TRUE(answer_count(CAP));
    expect_list_request();
    answer_list(CAP, STATUS_MATCH);
    for (uint8_t i = 0u; i < CAP; i++) {
        for (uint8_t k = 0u; k < 2u; k++) {
            TEST_ASSERT_TRUE(build());
            TEST_ASSERT_TRUE(vehicle_cl250_request_allowed(req, req_len));
            TEST_ASSERT_NOT_EQUAL_UINT8(0x14u, req[0]);
            TEST_ASSERT_EQUAL_HEX8(UDS_SID_READ_DTC_INFORMATION, req[0]);
            answer_record(req[1], i, 8u);
        }
    }
    TEST_ASSERT_EQUAL(DTC_SNAPSHOT_END_DONE, dtc_snapshot_core_last_end(&d));
}

static void test_D080_null_arguments_are_harmless(void)
{
    dtc_snapshot_core_init(NULL);
    dtc_snapshot_core_start_epoch(NULL);
    dtc_snapshot_core_unsend(NULL);
    dtc_snapshot_core_on_timeout(NULL);
    dtc_snapshot_core_on_segmented_failure(NULL);
    TEST_ASSERT_FALSE(dtc_snapshot_core_due(NULL));
    TEST_ASSERT_FALSE(dtc_snapshot_core_build_request(NULL, req, &req_len));
    dtc_snapshot_core_start_epoch(&d);
    TEST_ASSERT_FALSE(dtc_snapshot_core_build_request(&d, NULL, &req_len));
    TEST_ASSERT_FALSE(dtc_snapshot_core_build_request(&d, req, NULL));
    TEST_ASSERT_FALSE(dtc_snapshot_core_match_positive(NULL, req, 2u));
    TEST_ASSERT_FALSE(dtc_snapshot_core_on_positive(NULL, req, 2u));
    TEST_ASSERT_FALSE(dtc_snapshot_core_run_active(NULL));
    TEST_ASSERT_FALSE(dtc_snapshot_core_in_flight(NULL));
    TEST_ASSERT_EQUAL(DTC_SNAPSHOT_STEP_NONE, dtc_snapshot_core_step(NULL));
    TEST_ASSERT_EQUAL(DTC_SNAPSHOT_HALT_NONE, dtc_snapshot_core_halt(NULL));
    TEST_ASSERT_EQUAL(DTC_SNAPSHOT_END_NONE, dtc_snapshot_core_last_end(NULL));
    TEST_ASSERT_EQUAL(DTC_SNAPSHOT_TRIGGER_KEY_ON, dtc_snapshot_core_trigger(NULL));
    TEST_ASSERT_EQUAL_UINT8(0u, dtc_snapshot_core_runs_started(NULL));
    TEST_ASSERT_EQUAL_UINT8(0u, dtc_snapshot_core_requests_in_run(NULL));
    TEST_ASSERT_EQUAL_UINT8(0u, dtc_snapshot_core_requests_in_epoch(NULL));
    TEST_ASSERT_EQUAL_UINT8(0u, dtc_snapshot_core_dtc_count(NULL));
    TEST_ASSERT_NULL(dtc_snapshot_core_dtc(NULL, 0u));
    TEST_ASSERT_NULL(dtc_snapshot_core_stats(NULL));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_D080_item2_the_budget_constants_follow_gen);
    RUN_TEST(test_D080_item2_sub_functions_are_the_four_gen_values);
    RUN_TEST(test_D080_nothing_is_due_before_the_first_epoch);
    RUN_TEST(test_D080_item2_a_nominal_run_reads_count_list_then_both_records_per_dtc);
    RUN_TEST(test_D080_item2_one_request_is_in_flight_at_a_time);
    RUN_TEST(test_D080_item2_a_count_of_zero_ends_the_run_without_a_list_request);
    RUN_TEST(test_D080_item2_a_count_with_only_the_low_byte_set_continues);
    RUN_TEST(test_D080_item2_a_list_without_a_matching_status_ends_the_run);
    RUN_TEST(test_D080_item2_only_dtcs_with_a_matching_status_are_read);
    RUN_TEST(test_D080_item2_more_dtcs_than_the_cap_are_cut_at_the_cap_and_the_run_stays_in_budget);
    RUN_TEST(test_D080_item2_a_re_send_after_a_session_refusal_counts_against_the_budget);
    RUN_TEST(test_D080_item2_a_count_answer_of_the_wrong_length_ends_the_run);
    RUN_TEST(test_D080_item2_a_list_answer_that_is_not_whole_records_ends_the_run_and_stores_nothing);
    RUN_TEST(test_D080_item2_a_list_header_alone_matches_no_dtc);
    RUN_TEST(test_D080_item2_a_record_answer_shorter_than_its_header_ends_the_run);
    RUN_TEST(test_D080_item2_match_positive_checks_sid_sub_function_and_the_dtc_echo);
    RUN_TEST(test_D080_item2_expected_prefix_is_the_positive_sid_sub_function_and_for_records_the_dtc);
    RUN_TEST(test_D080_item2_nrc_0x11_0x12_0x31_mean_not_served_for_the_epoch);
    RUN_TEST(test_D080_item2_another_nrc_only_ends_the_run_and_the_warm_idle_run_may_come);
    RUN_TEST(test_D080_item4_nrc_0x78_waits_and_keeps_the_request_in_flight);
    RUN_TEST(test_D080_item2_an_nrc_without_a_request_in_flight_does_nothing);
    RUN_TEST(test_D080_item2_nrc_0x7e_and_0x7f_once_resend_the_same_request);
    RUN_TEST(test_D080_item2_the_second_session_refusal_in_the_epoch_halts);
    RUN_TEST(test_D080_item3_a_timeout_halts_the_epoch_with_no_answer);
    RUN_TEST(test_D080_item3_a_failed_segmented_reception_halts_the_epoch);
    RUN_TEST(test_D080_item3_the_first_halt_reason_stays);
    RUN_TEST(test_D080_item3_a_halt_after_the_answer_lets_the_request_in_flight_resolve);
    RUN_TEST(test_D080_item3_a_halt_with_nothing_in_flight_ends_the_run_at_once);
    RUN_TEST(test_D080_item3_a_halt_with_the_nrc_session_refusal_pending_ends_the_run);
    RUN_TEST(test_D080_item1_the_warm_idle_run_follows_a_finished_key_on_run);
    RUN_TEST(test_D080_item1_a_warm_idle_while_the_key_on_run_is_active_is_not_used_up);
    RUN_TEST(test_D080_item1_the_warm_idle_run_after_a_halt_is_refused_and_used_up);
    RUN_TEST(test_D080_item1_the_hold_must_last_the_gen_duration_exactly);
    RUN_TEST(test_D080_item1_any_break_restarts_the_hold);
    RUN_TEST(test_D080_item1_the_hold_timer_is_wrap_safe);
    RUN_TEST(test_D080_item1_nothing_runs_without_an_epoch_and_a_new_epoch_resets_the_timer);
    RUN_TEST(test_D080_item1_a_new_epoch_forgets_halts_and_counts_and_restarts_the_key_on_run);
    RUN_TEST(test_D080_unsend_restores_the_budgets_and_the_request_is_due_again);
    RUN_TEST(test_D080_item1_every_comparison_operator_and_its_boundary);
    RUN_TEST(test_D080_item1_nan_infinity_and_an_unknown_operator_are_never_met);
    RUN_TEST(test_D080_item1_the_gen_warm_idle_table_is_well_formed);
    RUN_TEST(test_D080_every_request_of_a_full_run_passes_the_gen_gate_and_is_never_0x14);
    RUN_TEST(test_D080_null_arguments_are_harmless);
    return UNITY_END();
}
