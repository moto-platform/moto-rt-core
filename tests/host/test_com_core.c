/*
 * L0 tests for services/com_core (D-056 items 3 and 7): the deadline-anchored cycle that
 * every cyclic platform sender shares. Moved unchanged from test_vehicle_republish_core
 * when the heartbeat became the second sender; the periods are gen/ cycle times.
 */
#include "services/com_core.h"

#include "platform.h"

#include <string.h>
#include <unity.h>

void setUp(void) {}
void tearDown(void) {}

static void test_the_cycle_is_due_at_once_then_on_its_deadline_grid(void)
{
    const uint32_t p = PLATFORM_VEHICLE_SPEED_CYCLE_TIME_MS;
    com_cycle_t c;
    memset(&c, 0, sizeof c);
    TEST_ASSERT_TRUE(com_cycle_due(&c, 500u));
    com_cycle_done(&c, 500u, p); /* anchored at 500 */
    TEST_ASSERT_FALSE(com_cycle_due(&c, 500u + p - 1u));
    TEST_ASSERT_TRUE(com_cycle_due(&c, 500u + p));
    com_cycle_done(&c, 500u + p + 10u, p); /* 10 ms late: the grid does not drift */
    TEST_ASSERT_EQUAL_UINT32(500u + 2u * p, c.next_due_ms);
    com_cycle_done(&c, 500u + 4u * p + 30u, p); /* missed two: skipped, no burst */
    TEST_ASSERT_EQUAL_UINT32(500u + 5u * p, c.next_due_ms);
    TEST_ASSERT_FALSE(com_cycle_due(&c, 500u + 4u * p + 31u));
}

static void test_the_cycle_survives_the_ms_counter_wrap(void)
{
    const uint32_t p = PLATFORM_VEHICLE_ENGINE_CYCLE_TIME_MS;
    com_cycle_t c;
    memset(&c, 0, sizeof c);
    const uint32_t start = UINT32_MAX - 20u;
    com_cycle_done(&c, start, p);
    TEST_ASSERT_FALSE(com_cycle_due(&c, start + 10u));
    TEST_ASSERT_FALSE(com_cycle_due(&c, (uint32_t)(start + p - 1u)));
    TEST_ASSERT_TRUE(com_cycle_due(&c, (uint32_t)(start + p)));
    com_cycle_done(&c, (uint32_t)(start + p), p);
    TEST_ASSERT_EQUAL_UINT32((uint32_t)(start + 2u * p), c.next_due_ms);
    /* a zero period re-anchors at now (defensive) */
    com_cycle_done(&c, 7u, 0u);
    TEST_ASSERT_EQUAL_UINT32(7u, c.next_due_ms);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_the_cycle_is_due_at_once_then_on_its_deadline_grid);
    RUN_TEST(test_the_cycle_survives_the_ms_counter_wrap);
    return UNITY_END();
}
