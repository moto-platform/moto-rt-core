/*
 * L0 tests for the alive supervision (D-064 item 4, D-065 item 2): the pure monitor
 * (services/alive_core) and the counter (services/alive) on the host atomics.
 */
#include "services/alive.h"
#include "services/alive_core.h"

#include <stdint.h>
#include <unity.h>

#define WINDOW_MS 60u

static alive_monitor_t m;

void setUp(void)
{
    alive_monitor_init(&m, 0u);
}

void tearDown(void) {}

static void test_a_monitor_that_never_saw_a_change_is_stalled_at_once(void)
{
    TEST_ASSERT_TRUE(alive_monitor_stalled(&m, 0u, 0u, WINDOW_MS));
    TEST_ASSERT_TRUE(alive_monitor_stalled(&m, 0u, 10000u, WINDOW_MS));
}

static void test_a_zeroed_monitor_reads_stalled(void)
{
    const alive_monitor_t zero = {0};
    alive_monitor_t z = zero; /* not initialised: the zero state must be the safe one */
    TEST_ASSERT_TRUE(alive_monitor_stalled(&z, 0u, 0u, WINDOW_MS));
    TEST_ASSERT_TRUE(alive_monitor_stalled(&z, 0u, 30u, WINDOW_MS));
    TEST_ASSERT_FALSE(alive_monitor_stalled(&z, 1u, 40u, WINDOW_MS)); /* a real change */
}

static void test_the_baseline_is_the_count_at_init(void)
{
    alive_monitor_init(&m, 7u);
    TEST_ASSERT_TRUE(alive_monitor_stalled(&m, 7u, 100u, WINDOW_MS));
    TEST_ASSERT_FALSE(alive_monitor_stalled(&m, 8u, 101u, WINDOW_MS));
}

static void test_a_change_is_alive_until_more_than_the_window_passed(void)
{
    TEST_ASSERT_FALSE(alive_monitor_stalled(&m, 1u, 1000u, WINDOW_MS));
    TEST_ASSERT_FALSE(alive_monitor_stalled(&m, 1u, 1000u + WINDOW_MS - 1u, WINDOW_MS));
    TEST_ASSERT_FALSE(alive_monitor_stalled(&m, 1u, 1000u + WINDOW_MS, WINDOW_MS)); /* boundary */
    TEST_ASSERT_TRUE(alive_monitor_stalled(&m, 1u, 1000u + WINDOW_MS + 1u, WINDOW_MS));
    /* the next change ends the stall at once and restarts the window from it */
    TEST_ASSERT_FALSE(alive_monitor_stalled(&m, 2u, 2000u, WINDOW_MS));
    TEST_ASSERT_FALSE(alive_monitor_stalled(&m, 2u, 2000u + WINDOW_MS, WINDOW_MS));
}

static void test_a_stall_is_sticky_across_the_timer_wrap(void)
{
    TEST_ASSERT_FALSE(alive_monitor_stalled(&m, 1u, 0xFFFFFF00u, WINDOW_MS));
    TEST_ASSERT_TRUE(alive_monitor_stalled(&m, 1u, 0xFFFFFF00u + WINDOW_MS + 1u, WINDOW_MS));
    /* 2^32 ms later the elapsed time reads small again: still stalled */
    TEST_ASSERT_TRUE(alive_monitor_stalled(&m, 1u, 0xFFFFFF00u + 10u, WINDOW_MS));
}

static void test_the_window_is_wrap_safe(void)
{
    TEST_ASSERT_FALSE(alive_monitor_stalled(&m, 1u, 0xFFFFFFF0u, WINDOW_MS));
    TEST_ASSERT_FALSE(alive_monitor_stalled(&m, 1u, 0xFFFFFFF0u + WINDOW_MS, WINDOW_MS)); /* wrapped */
    TEST_ASSERT_TRUE(alive_monitor_stalled(&m, 1u, 0xFFFFFFF0u + WINDOW_MS + 1u, WINDOW_MS));
}

static void test_any_change_counts_even_a_counter_wrap(void)
{
    alive_monitor_init(&m, UINT32_MAX);
    TEST_ASSERT_FALSE(alive_monitor_stalled(&m, 0u, 5u, WINDOW_MS)); /* UINT32_MAX + 1 */
}

static void test_a_late_supervisor_pass_never_causes_a_stall(void)
{
    TEST_ASSERT_FALSE(alive_monitor_stalled(&m, 1u, 1000u, WINDOW_MS));
    /* the supervisor was starved for 500 ms while the task ran on */
    TEST_ASSERT_FALSE(alive_monitor_stalled(&m, 26u, 1500u, WINDOW_MS));
}

static void test_null_monitor_reads_stalled_and_init_null_is_harmless(void)
{
    alive_monitor_init(NULL, 0u);
    TEST_ASSERT_TRUE(alive_monitor_stalled(NULL, 1u, 0u, WINDOW_MS));
}

static void test_the_counter_counts_bumps_and_wraps(void)
{
    alive_counter_t c;
    alive_init(&c);
    TEST_ASSERT_EQUAL_UINT32(0u, alive_count(&c));
    alive_bump(&c);
    alive_bump(&c);
    TEST_ASSERT_EQUAL_UINT32(2u, alive_count(&c));
    hal_atomic_u32_store(&c.count, UINT32_MAX);
    alive_bump(&c);
    TEST_ASSERT_EQUAL_UINT32(0u, alive_count(&c));
    /* NULL is harmless and reads 0 */
    alive_init(NULL);
    alive_bump(NULL);
    TEST_ASSERT_EQUAL_UINT32(0u, alive_count(NULL));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_a_monitor_that_never_saw_a_change_is_stalled_at_once);
    RUN_TEST(test_a_zeroed_monitor_reads_stalled);
    RUN_TEST(test_the_baseline_is_the_count_at_init);
    RUN_TEST(test_a_change_is_alive_until_more_than_the_window_passed);
    RUN_TEST(test_a_stall_is_sticky_across_the_timer_wrap);
    RUN_TEST(test_the_window_is_wrap_safe);
    RUN_TEST(test_any_change_counts_even_a_counter_wrap);
    RUN_TEST(test_a_late_supervisor_pass_never_causes_a_stall);
    RUN_TEST(test_null_monitor_reads_stalled_and_init_null_is_harmless);
    RUN_TEST(test_the_counter_counts_bumps_and_wraps);
    return UNITY_END();
}
