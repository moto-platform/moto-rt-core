/*
 * L0 tests for services/vehicle_signals: one sample per gen/ DID, VALID/STALE derived at
 * read time from the generated stale_after_ms (D-025, D-029). No requirement IDs yet
 * (Q-006).
 */
#include "platform_limits.h"
#include "services/vehicle_signals.h"
#include "vehicle_cl250.h"

#include <unity.h>

void setUp(void)
{
    vehicle_signals_init();
}

void tearDown(void) {}

static void test_no_sample_reads_as_none(void)
{
    for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
        vehicle_signal_sample_t s;
        s.state = VEHICLE_SIGNAL_VALID;
        TEST_ASSERT_TRUE(vehicle_signals_get(i, 12345u, &s));
        TEST_ASSERT_EQUAL(VEHICLE_SIGNAL_NONE, s.state);
        TEST_ASSERT_EQUAL_UINT32(0u, s.raw);
        TEST_ASSERT_EQUAL_UINT32(0u, s.timestamp_ms);
        TEST_ASSERT_EQUAL_UINT32(0u, s.age_ms);
    }
}

static void test_sample_is_valid_up_to_the_gen_stale_after_and_stale_after_it(void)
{
    for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
        const uint32_t stale = vehicle_cl250_dids[i].stale_after_ms;
        vehicle_signal_sample_t s;
        TEST_ASSERT_TRUE(vehicle_signals_write(i, 7u + i, 1.5f, 1000u));
        TEST_ASSERT_TRUE(vehicle_signals_get(i, 1000u, &s));
        TEST_ASSERT_EQUAL(VEHICLE_SIGNAL_VALID, s.state);
        TEST_ASSERT_EQUAL_UINT32(7u + i, s.raw);
        TEST_ASSERT_EQUAL_FLOAT(1.5f, s.physical);
        TEST_ASSERT_EQUAL_UINT32(1000u, s.timestamp_ms);
        TEST_ASSERT_EQUAL_UINT32(0u, s.age_ms);
        TEST_ASSERT_TRUE(vehicle_signals_get(i, 1000u + stale, &s));
        TEST_ASSERT_EQUAL(VEHICLE_SIGNAL_VALID, s.state);
        TEST_ASSERT_EQUAL_UINT32(stale, s.age_ms);
        TEST_ASSERT_TRUE(vehicle_signals_get(i, 1000u + stale + 1u, &s));
        TEST_ASSERT_EQUAL(VEHICLE_SIGNAL_STALE, s.state);
        TEST_ASSERT_EQUAL_UINT32(7u + i, s.raw); /* the value stays readable, marked STALE */
    }
}

static void test_a_new_sample_makes_it_valid_again(void)
{
    const uint32_t idx = VEHICLE_CL250_IDX_ENGINE_SPEED;
    vehicle_signal_sample_t s;
    TEST_ASSERT_TRUE(vehicle_signals_write(idx, 1u, 0.25f, 0u));
    TEST_ASSERT_TRUE(vehicle_signals_get(idx, 10000u, &s));
    TEST_ASSERT_EQUAL(VEHICLE_SIGNAL_STALE, s.state);
    TEST_ASSERT_TRUE(vehicle_signals_write(idx, 2u, 0.5f, 10000u));
    TEST_ASSERT_TRUE(vehicle_signals_get(idx, 10000u, &s));
    TEST_ASSERT_EQUAL(VEHICLE_SIGNAL_VALID, s.state);
    TEST_ASSERT_EQUAL_UINT32(2u, s.raw);
}

static void test_age_is_wrap_safe_and_a_future_timestamp_is_stale(void)
{
    const uint32_t idx = VEHICLE_CL250_IDX_VEHICLE_SPEED;
    vehicle_signal_sample_t s;
    TEST_ASSERT_TRUE(vehicle_signals_write(idx, 3u, 3.0f, 0xFFFFFFF0u));
    TEST_ASSERT_TRUE(vehicle_signals_get(idx, 0x10u, &s)); /* 32 ms later, across 2^32 */
    TEST_ASSERT_EQUAL_UINT32(0x20u, s.age_ms);
    TEST_ASSERT_EQUAL(VEHICLE_SIGNAL_VALID, s.state);
    TEST_ASSERT_TRUE(vehicle_signals_get(idx, 0xFFFFFFEFu, &s)); /* before the sample */
    TEST_ASSERT_EQUAL(VEHICLE_SIGNAL_STALE, s.state);
}

static void test_stale_is_sticky_across_the_counter_wrap(void)
{
    const uint32_t idx = VEHICLE_CL250_IDX_VEHICLE_SPEED;
    const uint32_t stale = vehicle_cl250_dids[idx].stale_after_ms;
    vehicle_signal_sample_t s;
    TEST_ASSERT_TRUE(vehicle_signals_write(idx, 5u, 5.0f, 100u));
    vehicle_signals_expire(100u + stale); /* not yet */
    TEST_ASSERT_TRUE(vehicle_signals_get(idx, 100u + stale, &s));
    TEST_ASSERT_EQUAL(VEHICLE_SIGNAL_VALID, s.state);
    vehicle_signals_expire(100u + stale + 1u); /* the writer's pass marks it */
    /* 2^32 ms later the age reads small again; the sample must stay STALE. */
    for (uint32_t k = 0u; k <= stale; k += 50u) {
        TEST_ASSERT_TRUE(vehicle_signals_get(idx, 100u + k, &s));
        TEST_ASSERT_EQUAL(VEHICLE_SIGNAL_STALE, s.state);
    }
    TEST_ASSERT_TRUE(vehicle_signals_write(idx, 6u, 6.0f, 200u)); /* a new sample clears it */
    TEST_ASSERT_TRUE(vehicle_signals_get(idx, 200u, &s));
    TEST_ASSERT_EQUAL(VEHICLE_SIGNAL_VALID, s.state);
}

static void test_a_stale_read_also_sticks(void)
{
    const uint32_t idx = VEHICLE_CL250_IDX_COOLANT_TEMP;
    const uint32_t stale = vehicle_cl250_dids[idx].stale_after_ms;
    vehicle_signal_sample_t s;
    TEST_ASSERT_TRUE(vehicle_signals_write(idx, 1u, 1.0f, 0u));
    TEST_ASSERT_TRUE(vehicle_signals_get(idx, stale + 1u, &s));
    TEST_ASSERT_EQUAL(VEHICLE_SIGNAL_STALE, s.state);
    TEST_ASSERT_TRUE(vehicle_signals_get(idx, 0u, &s)); /* the wrap alias of age 0 */
    TEST_ASSERT_EQUAL(VEHICLE_SIGNAL_STALE, s.state);
}

static void test_out_of_range_index_and_null_are_refused(void)
{
    vehicle_signal_sample_t s;
    s.raw = 99u;
    TEST_ASSERT_FALSE(vehicle_signals_write(VEHICLE_CL250_DID_COUNT, 1u, 1.0f, 0u));
    TEST_ASSERT_FALSE(vehicle_signals_get(VEHICLE_CL250_DID_COUNT, 0u, &s));
    TEST_ASSERT_EQUAL_UINT32(99u, s.raw);
    TEST_ASSERT_FALSE(vehicle_signals_get(0u, 0u, NULL));
}

static void test_init_forgets_samples_and_the_ecu(void)
{
    vehicle_signal_sample_t s;
    TEST_ASSERT_TRUE(vehicle_signals_write(0u, 1u, 1.0f, 5u));
    vehicle_signals_set_ecu_present(true);
    TEST_ASSERT_TRUE(vehicle_signals_ecu_present());
    vehicle_signals_init();
    TEST_ASSERT_FALSE(vehicle_signals_ecu_present());
    TEST_ASSERT_TRUE(vehicle_signals_get(0u, 5u, &s));
    TEST_ASSERT_EQUAL(VEHICLE_SIGNAL_NONE, s.state);
}

/*
 * D-048 item 2: rt-core never uses a speed sample older than
 * PLATFORM_LIMIT_VEHICLE_SPEED_MAX_AGE_MS, which equals 0xF40D's stale_after_ms. A
 * 300 ms sample is still VALID, a 301 ms one is STALE, so the future cornering/ lean
 * estimate (it may only use VALID speed) cannot take it.
 */
static void test_speed_sample_is_valid_at_the_gen_max_age_and_stale_one_ms_later(void)
{
    const uint32_t idx = VEHICLE_CL250_IDX_VEHICLE_SPEED;
    const uint32_t max_age = PLATFORM_LIMIT_VEHICLE_SPEED_MAX_AGE_MS;
    vehicle_signal_sample_t s;
    TEST_ASSERT_EQUAL_UINT32(vehicle_cl250_dids[idx].stale_after_ms, max_age);
    TEST_ASSERT_TRUE(vehicle_signals_write(idx, 60u, 60.0f, 1000u));
    TEST_ASSERT_TRUE(vehicle_signals_get(idx, 1000u + max_age, &s));
    TEST_ASSERT_EQUAL(VEHICLE_SIGNAL_VALID, s.state);
    TEST_ASSERT_EQUAL_UINT32(max_age, s.age_ms);
    TEST_ASSERT_TRUE(vehicle_signals_get(idx, 1000u + max_age + 1u, &s));
    TEST_ASSERT_EQUAL(VEHICLE_SIGNAL_STALE, s.state);
    TEST_ASSERT_EQUAL_UINT32(max_age + 1u, s.age_ms);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_no_sample_reads_as_none);
    RUN_TEST(test_sample_is_valid_up_to_the_gen_stale_after_and_stale_after_it);
    RUN_TEST(test_a_new_sample_makes_it_valid_again);
    RUN_TEST(test_age_is_wrap_safe_and_a_future_timestamp_is_stale);
    RUN_TEST(test_stale_is_sticky_across_the_counter_wrap);
    RUN_TEST(test_a_stale_read_also_sticks);
    RUN_TEST(test_out_of_range_index_and_null_are_refused);
    RUN_TEST(test_init_forgets_samples_and_the_ecu);
    RUN_TEST(test_speed_sample_is_valid_at_the_gen_max_age_and_stale_one_ms_later);
    return UNITY_END();
}
