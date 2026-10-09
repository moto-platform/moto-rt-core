/*
 * L0 tests for features/heartbeat/heartbeat_core (D-064): NODE_MODE, the DEGRADED
 * condition, the never-decreasing uptime, the saturating ERROR_COUNT / UPTIME and the
 * E2E-protected 0x081 frame. Every expected value comes from gen/ (choices, decode,
 * the receiver's E2E check), never a hand-written ID, DataID or scale.
 */
#include "app/host/sim_listener.h"
#include "features/heartbeat/heartbeat_core.h"

#include <string.h>
#include <unity.h>

void setUp(void) {}
void tearDown(void) {}

static struct platform_heartbeat_rt_core_t unpack(const uint8_t data[PLATFORM_HEARTBEAT_RT_CORE_LENGTH])
{
    struct platform_heartbeat_rt_core_t msg;
    sim_listener_heartbeat_decode(data, &msg); /* gen/ pack as the layout oracle */
    return msg;
}

/* ------------------------------------------------------------------ mode */

static void test_node_mode_is_init_first_then_degraded_on_a_fault_else_normal(void)
{
    TEST_ASSERT_EQUAL_UINT8(PLATFORM_HEARTBEAT_RT_CORE_NODE_MODE_INIT_CHOICE, heartbeat_node_mode(true, false));
    TEST_ASSERT_EQUAL_UINT8(PLATFORM_HEARTBEAT_RT_CORE_NODE_MODE_INIT_CHOICE, heartbeat_node_mode(true, true));
    TEST_ASSERT_EQUAL_UINT8(PLATFORM_HEARTBEAT_RT_CORE_NODE_MODE_DEGRADED_CHOICE, heartbeat_node_mode(false, true));
    TEST_ASSERT_EQUAL_UINT8(PLATFORM_HEARTBEAT_RT_CORE_NODE_MODE_NORMAL_CHOICE, heartbeat_node_mode(false, false));
}

static void test_any_single_fault_condition_degrades(void)
{
    TEST_ASSERT_FALSE(heartbeat_fault_active(false, false, false, false));
    TEST_ASSERT_TRUE(heartbeat_fault_active(true, false, false, false));
    TEST_ASSERT_TRUE(heartbeat_fault_active(false, true, false, false));
    TEST_ASSERT_TRUE(heartbeat_fault_active(false, false, true, false));
    TEST_ASSERT_TRUE(heartbeat_fault_active(false, false, false, true));
}

/* ------------------------------------------------------------------ uptime */

static void test_uptime_counts_from_the_anchor_and_survives_the_timebase_wrap(void)
{
    heartbeat_uptime_t up;
    const uint32_t start = UINT32_MAX - 1500u;
    heartbeat_uptime_init(&up, start);
    TEST_ASSERT_EQUAL_UINT32(start, up.elapsed_ms); /* the timebase counts from reset */
    heartbeat_uptime_update(&up, (uint32_t)(start + 1000u));
    heartbeat_uptime_update(&up, (uint32_t)(start + 3000u)); /* wrapped past 0 */
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, up.elapsed_ms);     /* saturated, never small */
    heartbeat_uptime_update(&up, (uint32_t)(start + 9000u));
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, up.elapsed_ms);
}

static void test_uptime_accumulates_across_a_wrap_without_saturating(void)
{
    heartbeat_uptime_t up;
    heartbeat_uptime_init(&up, 0u);
    up.last_ms = UINT32_MAX - 10u; /* a reading near the wrap after a long run */
    up.elapsed_ms = 5000u;
    heartbeat_uptime_update(&up, 20u); /* 31 ms later, across the wrap */
    TEST_ASSERT_EQUAL_UINT32(5031u, up.elapsed_ms);
    TEST_ASSERT_EQUAL_UINT32(20u, up.last_ms);
}

static void test_uptime_ignores_a_backward_reading(void)
{
    heartbeat_uptime_t up;
    heartbeat_uptime_init(&up, 10000u);
    heartbeat_uptime_update(&up, 9000u); /* earlier than the last reading: adds nothing */
    TEST_ASSERT_EQUAL_UINT32(10000u, up.elapsed_ms);
    TEST_ASSERT_EQUAL_UINT32(10000u, up.last_ms);
    heartbeat_uptime_update(&up, 10500u);
    TEST_ASSERT_EQUAL_UINT32(10500u, up.elapsed_ms);
}

/* ------------------------------------------------------------------ message */

static void test_the_message_carries_mode_error_count_and_whole_seconds(void)
{
    heartbeat_uptime_t up = {12999u, 0u};
    struct platform_heartbeat_rt_core_t msg;
    heartbeat_msg(false, true, 3u, &up, &msg);
    TEST_ASSERT_EQUAL_UINT8(PLATFORM_HEARTBEAT_RT_CORE_NODE_MODE_DEGRADED_CHOICE, msg.node_mode);
    TEST_ASSERT_EQUAL_UINT8(3u, msg.error_count);
    TEST_ASSERT_EQUAL_UINT16(12u, msg.uptime); /* whole seconds, never rounded up */
    TEST_ASSERT_EQUAL_UINT8(0u, msg.e2_e_crc); /* E2E is added by heartbeat_frame() */
    TEST_ASSERT_EQUAL_UINT8(0u, msg.e2_e_counter);
}

static void test_error_count_and_uptime_saturate_at_their_signal_maximum(void)
{
    struct platform_heartbeat_rt_core_t msg;
    struct platform_heartbeat_rt_core_t max;
    heartbeat_uptime_t up = {UINT32_MAX, 0u};
    heartbeat_msg(false, false, UINT32_MAX, &up, &msg);
    /* the maxima of the signals: the largest raw values the gen/ range check accepts */
    TEST_ASSERT_TRUE(platform_heartbeat_rt_core_error_count_is_in_range(msg.error_count));
    TEST_ASSERT_TRUE(platform_heartbeat_rt_core_uptime_is_in_range(msg.uptime));
    memset(&max, 0xFF, sizeof max);
    TEST_ASSERT_EQUAL_UINT8(max.error_count, msg.error_count);
    TEST_ASSERT_EQUAL_UINT16(max.uptime, msg.uptime);
    /* one below and at the limit */
    heartbeat_msg(false, false, (uint32_t)max.error_count - 1u, &up, &msg);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)(max.error_count - 1u), msg.error_count);
    heartbeat_msg(false, false, (uint32_t)max.error_count + 1u, &up, &msg);
    TEST_ASSERT_EQUAL_UINT8(max.error_count, msg.error_count);
    up.elapsed_ms = ((uint32_t)max.uptime + 1u) * 1000u; /* 65536 s */
    heartbeat_msg(false, false, 0u, &up, &msg);
    TEST_ASSERT_EQUAL_UINT16(max.uptime, msg.uptime);
    up.elapsed_ms = (uint32_t)max.uptime * 1000u - 1u; /* 65534.999 s */
    heartbeat_msg(false, false, 0u, &up, &msg);
    TEST_ASSERT_EQUAL_UINT16((uint16_t)(max.uptime - 1u), msg.uptime);
}

static void test_frames_pass_the_receiver_e2e_check_and_the_counter_moves_only_when_committed(void)
{
    heartbeat_uptime_t up = {42000u, 0u};
    struct platform_heartbeat_rt_core_t msg;
    heartbeat_msg(false, false, 7u, &up, &msg);
    moto_e2e_tx_state_t committed;
    moto_e2e_tx_init(&committed);
    moto_e2e_rx_state_t rx;
    moto_e2e_rx_init(&rx);
    uint8_t a[PLATFORM_HEARTBEAT_RT_CORE_LENGTH];
    uint8_t b[PLATFORM_HEARTBEAT_RT_CORE_LENGTH];
    moto_e2e_tx_state_t next;

    TEST_ASSERT_TRUE(heartbeat_frame(&msg, &committed, &next, a));
    TEST_ASSERT_EQUAL(MOTO_E2E_INITIAL,
                      moto_e2e_check(PLATFORM_HEARTBEAT_RT_CORE_E2E_DATA_ID, PLATFORM_HEARTBEAT_RT_CORE_E2E_MAX_DELTA_COUNTER,
                                     PLATFORM_HEARTBEAT_RT_CORE_E2E_TIMEOUT_MS, a, sizeof a, &rx, 0u));
    /* not committed (refused write): the retry carries the same counter */
    TEST_ASSERT_TRUE(heartbeat_frame(&msg, &committed, &next, b));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(a, b, sizeof a);
    committed = next;
    TEST_ASSERT_TRUE(heartbeat_frame(&msg, &committed, &next, b));
    TEST_ASSERT_EQUAL(MOTO_E2E_OK,
                      moto_e2e_check(PLATFORM_HEARTBEAT_RT_CORE_E2E_DATA_ID, PLATFORM_HEARTBEAT_RT_CORE_E2E_MAX_DELTA_COUNTER,
                                     PLATFORM_HEARTBEAT_RT_CORE_E2E_TIMEOUT_MS, b, sizeof b, &rx,
                                     PLATFORM_HEARTBEAT_RT_CORE_CYCLE_TIME_MS));
    const struct platform_heartbeat_rt_core_t got = unpack(b);
    TEST_ASSERT_EQUAL_UINT8(PLATFORM_HEARTBEAT_RT_CORE_NODE_MODE_NORMAL_CHOICE, got.node_mode);
    TEST_ASSERT_EQUAL_UINT8(7u, got.error_count);
    TEST_ASSERT_EQUAL_UINT16(42u, got.uptime);
    /* another DataID (a masquerading sender) fails the CRC */
    TEST_ASSERT_TRUE(heartbeat_frame(&msg, &next, &committed, b));
    TEST_ASSERT_EQUAL(MOTO_E2E_WRONG_CRC,
                      moto_e2e_check(PLATFORM_VEHICLE_SPEED_E2E_DATA_ID, PLATFORM_HEARTBEAT_RT_CORE_E2E_MAX_DELTA_COUNTER,
                                     PLATFORM_HEARTBEAT_RT_CORE_E2E_TIMEOUT_MS, b, sizeof b, &rx,
                                     2u * PLATFORM_HEARTBEAT_RT_CORE_CYCLE_TIME_MS));
}

static void test_the_frame_builder_refuses_null_arguments(void)
{
    struct platform_heartbeat_rt_core_t msg;
    memset(&msg, 0, sizeof msg);
    moto_e2e_tx_state_t c;
    moto_e2e_tx_state_t n;
    moto_e2e_tx_init(&c);
    uint8_t data[PLATFORM_HEARTBEAT_RT_CORE_LENGTH];
    TEST_ASSERT_FALSE(heartbeat_frame(NULL, &c, &n, data));
    TEST_ASSERT_FALSE(heartbeat_frame(&msg, NULL, &n, data));
    TEST_ASSERT_FALSE(heartbeat_frame(&msg, &c, NULL, data));
    TEST_ASSERT_FALSE(heartbeat_frame(&msg, &c, &n, NULL));
    TEST_ASSERT_TRUE(heartbeat_frame(&msg, &c, &n, data));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_node_mode_is_init_first_then_degraded_on_a_fault_else_normal);
    RUN_TEST(test_any_single_fault_condition_degrades);
    RUN_TEST(test_uptime_counts_from_the_anchor_and_survives_the_timebase_wrap);
    RUN_TEST(test_uptime_accumulates_across_a_wrap_without_saturating);
    RUN_TEST(test_uptime_ignores_a_backward_reading);
    RUN_TEST(test_the_message_carries_mode_error_count_and_whole_seconds);
    RUN_TEST(test_error_count_and_uptime_saturate_at_their_signal_maximum);
    RUN_TEST(test_frames_pass_the_receiver_e2e_check_and_the_counter_moves_only_when_committed);
    RUN_TEST(test_the_frame_builder_refuses_null_arguments);
    return UNITY_END();
}
