/*
 * L0 tests for services/com (D-056 items 3, 5 and 7): one platform frame through can_if
 * and the classification of the write result, on the host CAN port bound to the
 * in-process bus. The senders' own tests (republisher, heartbeat) cover the E2E commit.
 */
#include "services/com.h"

#include "hal/host/can_port_host.h"
#include "hal/host/hal_time_host.h"
#include "platform.h"
#include "services/can_if.h"
#include "services/can_sm.h"

#include <string.h>
#include <unity.h>

static vbus_t bus_p;
static vbus_t bus_v;
static uint8_t node_dut;
static uint8_t node_peer;
static uint8_t node_vehicle;
static com_msg_stats_t stats;

/* Any gen/ state-range message: the content does not matter to com. */
#define ID PLATFORM_VEHICLE_ENGINE_FRAME_ID
#define EXT (PLATFORM_VEHICLE_ENGINE_IS_EXTENDED != 0)
#define LEN ((uint8_t)PLATFORM_VEHICLE_ENGINE_LENGTH)

static const uint8_t payload[CAN_PORT_MAX_DLC] = {1u, 2u, 3u, 4u, 5u, 6u, 7u, 8u};

void setUp(void)
{
    hal_time_host_use_manual(1000u);
    vbus_init(&bus_p);
    vbus_init(&bus_v);
    TEST_ASSERT_TRUE(vbus_attach(&bus_p, &node_dut));
    TEST_ASSERT_TRUE(vbus_attach(&bus_p, &node_peer));
    TEST_ASSERT_TRUE(vbus_attach(&bus_v, &node_vehicle));
    TEST_ASSERT_TRUE(can_port_host_bind_vbus(CAN_PORT_PLATFORM, &bus_p, node_dut));
    TEST_ASSERT_TRUE(can_port_host_bind_vbus(CAN_PORT_VEHICLE, &bus_v, node_vehicle));
    can_if_init();
    TEST_ASSERT_EQUAL(CAN_PORT_OK, can_if_apply_filters(CAN_PORT_PLATFORM));
    can_sm_step(CAN_PORT_PLATFORM);
    memset(&stats, 0, sizeof stats);
}

void tearDown(void)
{
    can_port_host_unbind_all();
    hal_time_host_use_monotonic();
}

static void test_an_accepted_frame_finishes_the_cycle_and_reaches_the_platform_bus(void)
{
    can_port_status_t st = CAN_PORT_ERR_ARG;
    TEST_ASSERT_TRUE(com_send(ID, EXT, payload, LEN, &stats, &st));
    TEST_ASSERT_EQUAL(CAN_PORT_OK, st);
    TEST_ASSERT_EQUAL_UINT32(1u, stats.sent);
    TEST_ASSERT_EQUAL_UINT32(0u, stats.retried + stats.dropped);
    can_frame_t f;
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_recv(&bus_p, node_peer, &f));
    TEST_ASSERT_EQUAL_UINT32(ID, f.id);
    TEST_ASSERT_EQUAL(EXT, f.extended);
    TEST_ASSERT_EQUAL_UINT8(LEN, f.dlc);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(payload, f.data, LEN);
    TEST_ASSERT_EQUAL_UINT32(0u, vbus_frame_count(&bus_v)); /* never the vehicle bus */
}

static void test_a_full_tx_fifo_is_retried_and_does_not_finish_the_cycle(void)
{
    can_port_status_t st = CAN_PORT_ERR_ARG;
    vbus_set_tx_stalled(&bus_p, node_dut, true);
    TEST_ASSERT_TRUE(com_send(ID, EXT, payload, LEN, &stats, &st)); /* the one FIFO element */
    TEST_ASSERT_FALSE(com_send(ID, EXT, payload, LEN, &stats, &st));
    TEST_ASSERT_EQUAL(CAN_PORT_TX_FULL, st);
    TEST_ASSERT_EQUAL_UINT32(1u, stats.sent);
    TEST_ASSERT_EQUAL_UINT32(1u, stats.retried);
    TEST_ASSERT_EQUAL_UINT32(0u, stats.dropped);
    vbus_set_tx_stalled(&bus_p, node_dut, false);
}

static void test_a_frame_not_built_or_too_long_is_dropped_and_nothing_is_sent(void)
{
    can_port_status_t st = CAN_PORT_OK;
    TEST_ASSERT_TRUE(com_send(ID, EXT, NULL, LEN, &stats, &st));
    TEST_ASSERT_EQUAL(CAN_PORT_ERR_ARG, st);
    st = CAN_PORT_OK;
    TEST_ASSERT_TRUE(com_send(ID, EXT, payload, (uint8_t)(CAN_PORT_MAX_DLC + 1u), &stats, &st));
    TEST_ASSERT_EQUAL(CAN_PORT_ERR_ARG, st);
    TEST_ASSERT_EQUAL_UINT32(2u, stats.dropped);
    TEST_ASSERT_EQUAL_UINT32(0u, stats.sent + stats.retried);
    TEST_ASSERT_EQUAL_UINT32(0u, vbus_frame_count(&bus_p));
}

static void test_a_bus_off_port_drops_the_cycle(void)
{
    can_port_status_t st = CAN_PORT_OK;
    vbus_set_bus_off(&bus_p, node_dut, true);
    can_sm_step(CAN_PORT_PLATFORM);
    TEST_ASSERT_TRUE(com_send(ID, EXT, payload, LEN, &stats, &st));
    TEST_ASSERT_NOT_EQUAL(CAN_PORT_OK, st);
    TEST_ASSERT_NOT_EQUAL(CAN_PORT_TX_FULL, st);
    TEST_ASSERT_EQUAL_UINT32(1u, stats.dropped);
    TEST_ASSERT_EQUAL_UINT32(0u, stats.sent + stats.retried);
}

static void test_the_counters_saturate(void)
{
    can_port_status_t st = CAN_PORT_ERR_ARG;
    stats.sent = UINT32_MAX;
    stats.retried = UINT32_MAX;
    stats.dropped = UINT32_MAX;
    TEST_ASSERT_TRUE(com_send(ID, EXT, payload, LEN, &stats, &st));
    vbus_set_tx_stalled(&bus_p, node_dut, true);
    (void)com_send(ID, EXT, payload, LEN, &stats, &st); /* pending element */
    TEST_ASSERT_FALSE(com_send(ID, EXT, payload, LEN, &stats, &st));
    TEST_ASSERT_TRUE(com_send(ID, EXT, NULL, LEN, &stats, &st));
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, stats.sent);
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, stats.retried);
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, stats.dropped);
    vbus_set_tx_stalled(&bus_p, node_dut, false);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_an_accepted_frame_finishes_the_cycle_and_reaches_the_platform_bus);
    RUN_TEST(test_a_full_tx_fifo_is_retried_and_does_not_finish_the_cycle);
    RUN_TEST(test_a_frame_not_built_or_too_long_is_dropped_and_nothing_is_sent);
    RUN_TEST(test_a_bus_off_port_drops_the_cycle);
    RUN_TEST(test_the_counters_saturate);
    return UNITY_END();
}
