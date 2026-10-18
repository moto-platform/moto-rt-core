/*
 * L0 tests for DID 0xFD03 LEAN_HEALTH (D-067, E-17 (a)): the pure builder and the
 * unlearned-run tracker (services/lean_health_core), and the comms-task state behind
 * the DID (services/lean_health). Every field position, mask and width comes from gen/.
 */
#include "services/lean_health.h"
#include "services/lean_health_core.h"

#include <string.h>
#include <unity.h>

#define NOW (100000u)
#define MAX_AGE PLATFORM_UDS_LEAN_HEALTH_MAX_AGE_MS
#define FRESH PLATFORM_UDS_LEAN_HEALTH_RECORD_FRESH_MASK
#define LEARNED PLATFORM_UDS_LEAN_HEALTH_OFFSET_LEARNED_MASK
#define REJECTED PLATFORM_UDS_LEAN_HEALTH_OFFSET_REJECTED_MASK
#define SPEED_PROD PLATFORM_UDS_LEAN_HEALTH_SPEED_PRODUCER_FAULT_MASK
#define OUT_CONS PLATFORM_UDS_LEAN_HEALTH_OUTPUT_CONSUMER_FAULT_MASK
#define SPEED_CONS PLATFORM_UDS_LEAN_HEALTH_SPEED_CONSUMER_FAULT_MASK
#define ALIVE_OK PLATFORM_UDS_LEAN_HEALTH_EKF_ALIVE_OK_MASK
#define RESERVED PLATFORM_UDS_LEAN_HEALTH_RESERVED_MASK
#define UNKNOWN PLATFORM_UDS_LEAN_HEALTH_UNLEARNED_S_UNKNOWN

void setUp(void)
{
    lean_health_init();
}

void tearDown(void) {}

static lean_health_rec_t rec_at(uint32_t stamp)
{
    lean_health_rec_t r;
    (void)memset(&r, 0, sizeof r);
    r.have_record = true;
    r.stamp_ms = stamp;
    return r;
}

static lean_health_comms_t no_comms(void)
{
    const lean_health_comms_t c = {false, false, false};
    return c;
}

static uint8_t flags_of(const uint8_t* d)
{
    return d[PLATFORM_UDS_LEAN_HEALTH_RECORD_FRESH_BYTE];
}

static uint16_t rejects_of(const uint8_t* d)
{
    return (uint16_t)(((uint16_t)d[PLATFORM_UDS_LEAN_HEALTH_OFFSET_REJECTS_BYTE] << 8u) |
                      d[PLATFORM_UDS_LEAN_HEALTH_OFFSET_REJECTS_BYTE + 1u]);
}

static uint16_t unlearned_of(const uint8_t* d)
{
    return (uint16_t)(((uint16_t)d[PLATFORM_UDS_LEAN_HEALTH_UNLEARNED_S_BYTE] << 8u) |
                      d[PLATFORM_UDS_LEAN_HEALTH_UNLEARNED_S_BYTE + 1u]);
}

static void build(const lean_health_rec_t* r, const lean_health_comms_t* c, uint32_t now, uint8_t* out)
{
    TEST_ASSERT_TRUE(lean_health_build(r, c, now, out, PLATFORM_UDS_DID_LEAN_HEALTH_LENGTH));
}

/* ------------------------------------------------------------------ builder */

static void test_REQ_FD03_no_record_since_boot_reads_not_fresh_and_unknown(void)
{
    const lean_health_rec_t none = {0};
    const lean_health_comms_t c = no_comms();
    uint8_t d[PLATFORM_UDS_DID_LEAN_HEALTH_LENGTH];
    (void)memset(d, 0xA5, sizeof d);
    build(&none, &c, NOW, d);
    TEST_ASSERT_EQUAL_HEX8(0u, flags_of(d));
    TEST_ASSERT_EQUAL_UINT16(0u, rejects_of(d));
    TEST_ASSERT_EQUAL_UINT16(UNKNOWN, unlearned_of(d));
}

static void test_REQ_FD03_a_fresh_record_reports_its_fields(void)
{
    lean_health_rec_t r = rec_at(NOW - 20u);
    r.ekf.offset_learned = true;
    r.ekf.reject_count = 0x0102u;
    r.ekf.unlearned_s = 0u;
    const lean_health_comms_t c = no_comms();
    uint8_t d[PLATFORM_UDS_DID_LEAN_HEALTH_LENGTH];
    build(&r, &c, NOW, d);
    TEST_ASSERT_EQUAL_HEX8(FRESH | LEARNED, flags_of(d));
    TEST_ASSERT_EQUAL_HEX8(0x01u, d[PLATFORM_UDS_LEAN_HEALTH_OFFSET_REJECTS_BYTE]);
    TEST_ASSERT_EQUAL_HEX8(0x02u, d[PLATFORM_UDS_LEAN_HEALTH_OFFSET_REJECTS_BYTE + 1u]);
    TEST_ASSERT_EQUAL_UINT16(0u, unlearned_of(d));
    r.ekf.offset_learned = false;
    r.ekf.unlearned_s = 0x1234u;
    build(&r, &c, NOW, d);
    TEST_ASSERT_EQUAL_HEX8(FRESH, flags_of(d));
    TEST_ASSERT_EQUAL_UINT16(0x1234u, unlearned_of(d));
}

static void test_REQ_FD03_each_flag_sits_at_its_own_gen_mask_and_reserved_stays_0(void)
{
    lean_health_rec_t r = rec_at(NOW);
    r.ekf.offset_rejected = true;
    r.speed_consumer_fault = true;
    const lean_health_comms_t c = {true, true, true};
    uint8_t d[PLATFORM_UDS_DID_LEAN_HEALTH_LENGTH];
    build(&r, &c, NOW, d);
    TEST_ASSERT_EQUAL_HEX8(FRESH | REJECTED | SPEED_PROD | OUT_CONS | SPEED_CONS | ALIVE_OK, flags_of(d));
    TEST_ASSERT_EQUAL_HEX8(0u, flags_of(d) & RESERVED);
    /* the masks are distinct bits */
    const uint8_t all = FRESH | LEARNED | REJECTED | SPEED_PROD | OUT_CONS | SPEED_CONS | ALIVE_OK | RESERVED;
    TEST_ASSERT_EQUAL_HEX8(0xFFu, all);
    /* one flag at a time */
    lean_health_rec_t q = rec_at(NOW);
    lean_health_comms_t k = no_comms();
    k.speed_producer_fault = true;
    build(&q, &k, NOW, d);
    TEST_ASSERT_EQUAL_HEX8(FRESH | SPEED_PROD, flags_of(d));
    k = no_comms();
    k.output_consumer_fault = true;
    build(&q, &k, NOW, d);
    TEST_ASSERT_EQUAL_HEX8(FRESH | OUT_CONS, flags_of(d));
    k = no_comms();
    k.ekf_alive_ok = true;
    build(&q, &k, NOW, d);
    TEST_ASSERT_EQUAL_HEX8(FRESH | ALIVE_OK, flags_of(d));
    q.speed_consumer_fault = true;
    build(&q, &k, NOW, d); /* k still holds ALIVE_OK */
    TEST_ASSERT_EQUAL_HEX8(FRESH | SPEED_CONS | ALIVE_OK, flags_of(d));
}

static void test_REQ_FD03_the_age_window_is_the_gen_max_age(void)
{
    lean_health_rec_t r = rec_at(NOW - MAX_AGE);
    r.ekf.offset_learned = true;
    r.ekf.unlearned_s = 7u;
    const lean_health_comms_t c = no_comms();
    uint8_t d[PLATFORM_UDS_DID_LEAN_HEALTH_LENGTH];
    build(&r, &c, NOW, d);
    TEST_ASSERT_EQUAL_HEX8(FRESH | LEARNED, flags_of(d)); /* exactly max_age: fresh */
    TEST_ASSERT_EQUAL_UINT16(7u, unlearned_of(d));
    r.stamp_ms = NOW - MAX_AGE - 1u;
    build(&r, &c, NOW, d);
    TEST_ASSERT_EQUAL_HEX8(0u, flags_of(d)); /* one ms older: stale, learned hidden */
    TEST_ASSERT_EQUAL_UINT16(UNKNOWN, unlearned_of(d));
}

static void test_REQ_FD03_a_stale_record_keeps_the_last_ekf_fields_but_never_learned(void)
{
    lean_health_rec_t r = rec_at(NOW - MAX_AGE - 1000u);
    r.ekf.offset_learned = true;
    r.ekf.offset_rejected = true;
    r.ekf.reject_count = 9u;
    r.ekf.unlearned_s = 50u;
    r.speed_consumer_fault = true;
    const lean_health_comms_t c = {false, false, true};
    uint8_t d[PLATFORM_UDS_DID_LEAN_HEALTH_LENGTH];
    build(&r, &c, NOW, d);
    TEST_ASSERT_EQUAL_HEX8(REJECTED | SPEED_CONS | ALIVE_OK, flags_of(d));
    TEST_ASSERT_EQUAL_UINT16(9u, rejects_of(d));
    TEST_ASSERT_EQUAL_UINT16(UNKNOWN, unlearned_of(d));
}

static void test_REQ_FD03_a_stamp_ahead_is_fresh_only_within_one_comms_pass(void)
{
    const lean_health_comms_t c = no_comms();
    uint8_t d[PLATFORM_UDS_DID_LEAN_HEALTH_LENGTH];
    lean_health_rec_t r = rec_at(NOW + LEAN_HEALTH_FUTURE_MAX_MS);
    build(&r, &c, NOW, d);
    TEST_ASSERT_EQUAL_HEX8(FRESH, flags_of(d));
    r.stamp_ms = NOW + LEAN_HEALTH_FUTURE_MAX_MS + 1u;
    build(&r, &c, NOW, d);
    TEST_ASSERT_EQUAL_HEX8(0u, flags_of(d)); /* not a wrapped (huge) age that happens to be fresh */
    r.stamp_ms = NOW + 0x80000000u;
    build(&r, &c, NOW, d);
    TEST_ASSERT_EQUAL_HEX8(0u, flags_of(d));
}

static void test_REQ_FD03_the_timer_wrap_does_not_make_an_old_record_fresh(void)
{
    const lean_health_comms_t c = no_comms();
    uint8_t d[PLATFORM_UDS_DID_LEAN_HEALTH_LENGTH];
    lean_health_rec_t r = rec_at(0xFFFFFF00u);
    build(&r, &c, 0xFFFFFF00u + MAX_AGE, d);
    TEST_ASSERT_EQUAL_HEX8(FRESH, flags_of(d));
    build(&r, &c, 0xFFFFFF00u + MAX_AGE + 1u, d);
    TEST_ASSERT_EQUAL_HEX8(0u, flags_of(d));
    r.stamp_ms = 0xFFFFFFF0u; /* across the wrap: now = 0x10 is 32 ms later */
    build(&r, &c, 0x10u, d);
    TEST_ASSERT_EQUAL_HEX8(FRESH, flags_of(d));
}

static void test_REQ_FD03_counters_at_the_gen_maximum_stay_there(void)
{
    lean_health_rec_t r = rec_at(NOW);
    r.ekf.reject_count = (uint16_t)PLATFORM_UDS_LEAN_HEALTH_OFFSET_REJECTS_MAX;
    r.ekf.unlearned_s = (uint16_t)PLATFORM_UDS_LEAN_HEALTH_UNLEARNED_S_MAX;
    const lean_health_comms_t c = no_comms();
    uint8_t d[PLATFORM_UDS_DID_LEAN_HEALTH_LENGTH];
    build(&r, &c, NOW, d);
    TEST_ASSERT_EQUAL_UINT16(0xFFFFu, rejects_of(d));
    TEST_ASSERT_EQUAL_UINT16(0xFFFFu, unlearned_of(d));
}

static void test_REQ_FD03_a_wrong_length_or_null_is_refused_and_leaves_out_untouched(void)
{
    const lean_health_rec_t r = rec_at(NOW);
    const lean_health_comms_t c = no_comms();
    uint8_t d[PLATFORM_UDS_DID_LEAN_HEALTH_LENGTH + 1u];
    (void)memset(d, 0xA5, sizeof d);
    TEST_ASSERT_FALSE(lean_health_build(&r, &c, NOW, d, PLATFORM_UDS_DID_LEAN_HEALTH_LENGTH - 1u));
    TEST_ASSERT_FALSE(lean_health_build(&r, &c, NOW, d, PLATFORM_UDS_DID_LEAN_HEALTH_LENGTH + 1u));
    TEST_ASSERT_FALSE(lean_health_build(NULL, &c, NOW, d, PLATFORM_UDS_DID_LEAN_HEALTH_LENGTH));
    TEST_ASSERT_FALSE(lean_health_build(&r, NULL, NOW, d, PLATFORM_UDS_DID_LEAN_HEALTH_LENGTH));
    TEST_ASSERT_FALSE(lean_health_build(&r, &c, NOW, NULL, PLATFORM_UDS_DID_LEAN_HEALTH_LENGTH));
    for (uint32_t i = 0u; i < sizeof d; i++) {
        TEST_ASSERT_EQUAL_HEX8(0xA5u, d[i]);
    }
    TEST_ASSERT_FALSE(lean_health_record_fresh(NULL, NOW));
}

/* ------------------------------------------------------------------ unlearned run */

static void test_REQ_FD03_sat16_clamps_at_the_given_maximum(void)
{
    TEST_ASSERT_EQUAL_UINT16(5u, lean_health_sat16(5u, 0xFFFFu));
    TEST_ASSERT_EQUAL_UINT16(0xFFFFu, lean_health_sat16(0xFFFFu, 0xFFFFu));
    TEST_ASSERT_EQUAL_UINT16(0xFFFFu, lean_health_sat16(0x10000u, 0xFFFFu));
    TEST_ASSERT_EQUAL_UINT16(0xFFFFu, lean_health_sat16(0xFFFFFFFFu, 0xFFFFu));
}

static void test_REQ_FD03_the_unlearned_run_counts_whole_seconds_from_the_first_step(void)
{
    lean_health_run_t run = {0};
    TEST_ASSERT_EQUAL_UINT16(0u, lean_health_unlearned_s(&run, false, 5000u)); /* anchored */
    TEST_ASSERT_EQUAL_UINT16(0u, lean_health_unlearned_s(&run, false, 5999u));
    TEST_ASSERT_EQUAL_UINT16(1u, lean_health_unlearned_s(&run, false, 6000u));
    TEST_ASSERT_EQUAL_UINT16(12u, lean_health_unlearned_s(&run, false, 17999u));
}

static void test_REQ_FD03_a_learnt_offset_reads_0_and_a_lost_one_starts_a_new_run(void)
{
    lean_health_run_t run = {0};
    (void)lean_health_unlearned_s(&run, false, 0u);
    TEST_ASSERT_EQUAL_UINT16(3u, lean_health_unlearned_s(&run, false, 3000u));
    TEST_ASSERT_EQUAL_UINT16(0u, lean_health_unlearned_s(&run, true, 3020u));
    TEST_ASSERT_EQUAL_UINT16(0u, lean_health_unlearned_s(&run, true, 90000u));
    /* a rejected window drops the offset: the run starts at that step, not at boot */
    TEST_ASSERT_EQUAL_UINT16(0u, lean_health_unlearned_s(&run, false, 90020u));
    TEST_ASSERT_EQUAL_UINT16(2u, lean_health_unlearned_s(&run, false, 92020u));
}

static void test_REQ_FD03_a_first_step_that_is_already_learnt_starts_no_run(void)
{
    lean_health_run_t run = {0};
    TEST_ASSERT_EQUAL_UINT16(0u, lean_health_unlearned_s(&run, true, 1000u));
    TEST_ASSERT_EQUAL_UINT16(0u, lean_health_unlearned_s(&run, false, 2000u)); /* run starts now */
    TEST_ASSERT_EQUAL_UINT16(1u, lean_health_unlearned_s(&run, false, 3000u));
}

static void test_REQ_FD03_the_unlearned_run_saturates_and_never_wraps(void)
{
    lean_health_run_t run = {0};
    (void)lean_health_unlearned_s(&run, false, 0u);
    TEST_ASSERT_EQUAL_UINT16(0xFFFEu, lean_health_unlearned_s(&run, false, 0xFFFEu * 1000u));
    TEST_ASSERT_EQUAL_UINT16(0xFFFFu, lean_health_unlearned_s(&run, false, 0xFFFFu * 1000u));
    TEST_ASSERT_EQUAL_UINT16(0xFFFFu, lean_health_unlearned_s(&run, false, 0xFFFFFFFFu)); /* 49.7 days */
}

static void test_REQ_FD03_the_unlearned_run_survives_the_timer_wrap(void)
{
    lean_health_run_t run = {0};
    (void)lean_health_unlearned_s(&run, false, 0xFFFFFC18u); /* 1000 ms before the wrap */
    TEST_ASSERT_EQUAL_UINT16(2u, lean_health_unlearned_s(&run, false, 1000u));
}

static void test_REQ_FD03_a_null_tracker_reads_0(void)
{
    TEST_ASSERT_EQUAL_UINT16(0u, lean_health_unlearned_s(NULL, false, 1000u));
}

/* ------------------------------------------------------------------ service */

static void test_REQ_FD03_the_service_starts_and_resets_to_no_record_and_no_flags(void)
{
    uint8_t d[PLATFORM_UDS_DID_LEAN_HEALTH_LENGTH];
    TEST_ASSERT_TRUE(lean_health_read(NOW, d, sizeof d));
    TEST_ASSERT_EQUAL_HEX8(0u, flags_of(d));
    TEST_ASSERT_EQUAL_UINT16(UNKNOWN, unlearned_of(d));
    TEST_ASSERT_EQUAL_UINT16(0u, rejects_of(d));
}

static void test_REQ_FD03_the_service_reads_the_last_copy_and_flags_and_init_clears_them(void)
{
    lean_health_rec_t r = rec_at(NOW - 10u);
    r.ekf.offset_learned = true;
    r.ekf.reject_count = 3u;
    const lean_health_comms_t c = {true, false, true};
    lean_health_set_record(&r);
    lean_health_set_comms(&c);
    uint8_t d[PLATFORM_UDS_DID_LEAN_HEALTH_LENGTH];
    TEST_ASSERT_TRUE(lean_health_read(NOW, d, sizeof d));
    TEST_ASSERT_EQUAL_HEX8(FRESH | LEARNED | SPEED_PROD | ALIVE_OK, flags_of(d));
    TEST_ASSERT_EQUAL_UINT16(3u, rejects_of(d));
    lean_health_set_record(NULL); /* ignored */
    lean_health_set_comms(NULL);
    TEST_ASSERT_TRUE(lean_health_read(NOW, d, sizeof d));
    TEST_ASSERT_EQUAL_HEX8(FRESH | LEARNED | SPEED_PROD | ALIVE_OK, flags_of(d));
    TEST_ASSERT_FALSE(lean_health_read(NOW, d, 4u));
    TEST_ASSERT_FALSE(lean_health_read(NOW, NULL, sizeof d));
    lean_health_init();
    TEST_ASSERT_TRUE(lean_health_read(NOW, d, sizeof d));
    TEST_ASSERT_EQUAL_HEX8(0u, flags_of(d));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_REQ_FD03_no_record_since_boot_reads_not_fresh_and_unknown);
    RUN_TEST(test_REQ_FD03_a_fresh_record_reports_its_fields);
    RUN_TEST(test_REQ_FD03_each_flag_sits_at_its_own_gen_mask_and_reserved_stays_0);
    RUN_TEST(test_REQ_FD03_the_age_window_is_the_gen_max_age);
    RUN_TEST(test_REQ_FD03_a_stale_record_keeps_the_last_ekf_fields_but_never_learned);
    RUN_TEST(test_REQ_FD03_a_stamp_ahead_is_fresh_only_within_one_comms_pass);
    RUN_TEST(test_REQ_FD03_the_timer_wrap_does_not_make_an_old_record_fresh);
    RUN_TEST(test_REQ_FD03_counters_at_the_gen_maximum_stay_there);
    RUN_TEST(test_REQ_FD03_a_wrong_length_or_null_is_refused_and_leaves_out_untouched);
    RUN_TEST(test_REQ_FD03_sat16_clamps_at_the_given_maximum);
    RUN_TEST(test_REQ_FD03_the_unlearned_run_counts_whole_seconds_from_the_first_step);
    RUN_TEST(test_REQ_FD03_a_learnt_offset_reads_0_and_a_lost_one_starts_a_new_run);
    RUN_TEST(test_REQ_FD03_a_first_step_that_is_already_learnt_starts_no_run);
    RUN_TEST(test_REQ_FD03_the_unlearned_run_saturates_and_never_wraps);
    RUN_TEST(test_REQ_FD03_the_unlearned_run_survives_the_timer_wrap);
    RUN_TEST(test_REQ_FD03_a_null_tracker_reads_0);
    RUN_TEST(test_REQ_FD03_the_service_starts_and_resets_to_no_record_and_no_flags);
    RUN_TEST(test_REQ_FD03_the_service_reads_the_last_copy_and_flags_and_init_clears_them);
    return UNITY_END();
}
