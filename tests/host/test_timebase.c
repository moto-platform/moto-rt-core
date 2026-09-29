/*
 * L0 tests for services/timebase over the host time source (D-034).
 */
#include "hal/host/hal_time_host.h"
#include "services/timebase.h"

#include <unity.h>

void setUp(void)
{
    hal_time_host_use_manual(0u);
}

void tearDown(void)
{
    hal_time_host_use_monotonic();
}

static void test_manual_clock_moves_only_when_advanced(void)
{
    hal_time_host_use_manual(1234u);
    TEST_ASSERT_EQUAL_UINT32(1234u, timebase_now_ms());
    TEST_ASSERT_EQUAL_UINT32(1234u, timebase_now_ms());
    hal_time_host_advance(66u);
    TEST_ASSERT_EQUAL_UINT32(1300u, timebase_now_ms());
}

static void test_elapsed_and_expired_are_wrap_safe(void)
{
    hal_time_host_use_manual(0xFFFFFFFBu);
    const uint32_t start = timebase_now_ms();
    hal_time_host_advance(10u); /* wraps to 5 */
    const uint32_t now = timebase_now_ms();
    TEST_ASSERT_EQUAL_UINT32(5u, now);
    TEST_ASSERT_EQUAL_UINT32(10u, timebase_elapsed_ms(now, start));
    TEST_ASSERT_TRUE(timebase_expired(now, start, 10u));
    TEST_ASSERT_FALSE(timebase_expired(now, start, 11u));
    TEST_ASSERT_TRUE(timebase_expired(now, start, 0u));
}

static void test_monotonic_clock_starts_near_zero_and_never_goes_back(void)
{
    hal_time_host_use_monotonic();
    uint32_t prev = timebase_now_ms();
    TEST_ASSERT_LESS_THAN_UINT32(100u, prev);
    for (int i = 0; i < 5; i++) {
        hal_time_host_sleep_ms(2u);
        const uint32_t now = timebase_now_ms();
        TEST_ASSERT_TRUE(timebase_elapsed_ms(now, prev) < 1000u);
        TEST_ASSERT_TRUE(now >= prev);
        prev = now;
    }
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(10u, prev);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_manual_clock_moves_only_when_advanced);
    RUN_TEST(test_elapsed_and_expired_are_wrap_safe);
    RUN_TEST(test_monotonic_clock_starts_near_zero_and_never_goes_back);
    return UNITY_END();
}
