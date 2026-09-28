/*
 * L0 tests for the in-process virtual CAN bus of the host platform layer (D-034).
 */
#include "hal/host/vbus.h"

#include <string.h>
#include <unity.h>

static vbus_t bus;

void setUp(void)
{
    vbus_init(&bus);
}

void tearDown(void) {}

static can_frame_t frame(uint32_t id, bool ext, uint8_t dlc, uint8_t seed)
{
    can_frame_t f;
    memset(&f, 0, sizeof f);
    f.id = id;
    f.extended = ext;
    f.dlc = dlc;
    for (uint8_t i = 0u; i < dlc; i++) {
        f.data[i] = (uint8_t)(seed + i);
    }
    return f;
}

static void test_attach_is_limited_to_max_nodes(void)
{
    uint8_t n = 0xFFu;
    for (uint8_t i = 0u; i < VBUS_MAX_NODES; i++) {
        TEST_ASSERT_TRUE(vbus_attach(&bus, &n));
        TEST_ASSERT_EQUAL_UINT8(i, n);
    }
    TEST_ASSERT_FALSE(vbus_attach(&bus, &n));
    TEST_ASSERT_FALSE(vbus_attach(NULL, &n));
    TEST_ASSERT_FALSE(vbus_attach(&bus, NULL));
}

static void test_frame_reaches_every_other_node_but_not_the_sender(void)
{
    uint8_t a, b, c;
    TEST_ASSERT_TRUE(vbus_attach(&bus, &a));
    TEST_ASSERT_TRUE(vbus_attach(&bus, &b));
    TEST_ASSERT_TRUE(vbus_attach(&bus, &c));
    const can_frame_t f = frame(0x18DA10F1u, true, 8u, 0x10u);
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_send(&bus, a, &f));

    can_frame_t got;
    TEST_ASSERT_EQUAL(CAN_PORT_EMPTY, vbus_recv(&bus, a, &got));
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_recv(&bus, b, &got));
    TEST_ASSERT_EQUAL_MEMORY(&f, &got, sizeof f);
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_recv(&bus, c, &got));
    TEST_ASSERT_EQUAL_MEMORY(&f, &got, sizeof f);
    TEST_ASSERT_EQUAL(CAN_PORT_EMPTY, vbus_recv(&bus, b, &got));
    TEST_ASSERT_EQUAL_UINT32(1u, vbus_frame_count(&bus));
}

static void test_frames_keep_their_order(void)
{
    uint8_t a, b;
    TEST_ASSERT_TRUE(vbus_attach(&bus, &a));
    TEST_ASSERT_TRUE(vbus_attach(&bus, &b));
    for (uint8_t i = 0u; i < 10u; i++) {
        const can_frame_t f = frame(0x100u + i, false, 1u, i);
        TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_send(&bus, a, &f));
    }
    for (uint8_t i = 0u; i < 10u; i++) {
        can_frame_t got;
        TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_recv(&bus, b, &got));
        TEST_ASSERT_EQUAL_UINT32(0x100u + i, got.id);
    }
}

static void test_full_rx_queue_drops_and_counts_overruns(void)
{
    uint8_t a, b;
    TEST_ASSERT_TRUE(vbus_attach(&bus, &a));
    TEST_ASSERT_TRUE(vbus_attach(&bus, &b));
    for (uint32_t i = 0u; i < VBUS_RX_DEPTH + 3u; i++) {
        const can_frame_t f = frame(i & 0x7FFu, false, 0u, 0u);
        TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_send(&bus, a, &f));
    }
    TEST_ASSERT_EQUAL_UINT32(3u, vbus_overruns(&bus, b));
    can_frame_t got;
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_recv(&bus, b, &got));
    TEST_ASSERT_EQUAL_UINT32(0u, got.id); /* the oldest frames are kept */
}

static void test_blocked_tx_reports_full_and_sends_nothing(void)
{
    uint8_t a, b;
    TEST_ASSERT_TRUE(vbus_attach(&bus, &a));
    TEST_ASSERT_TRUE(vbus_attach(&bus, &b));
    vbus_set_tx_blocked(&bus, a, true);
    TEST_ASSERT_FALSE(vbus_tx_free(&bus, a));
    const can_frame_t f = frame(0x123u, false, 2u, 0u);
    TEST_ASSERT_EQUAL(CAN_PORT_TX_FULL, vbus_send(&bus, a, &f));
    can_frame_t got;
    TEST_ASSERT_EQUAL(CAN_PORT_EMPTY, vbus_recv(&bus, b, &got));
    vbus_set_tx_blocked(&bus, a, false);
    TEST_ASSERT_TRUE(vbus_tx_free(&bus, a));
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_send(&bus, a, &f));
}

static void test_invalid_frames_and_nodes_are_rejected(void)
{
    uint8_t a;
    TEST_ASSERT_TRUE(vbus_attach(&bus, &a));
    can_frame_t f = frame(0x800u, false, 1u, 0u); /* 12-bit ID in standard format */
    TEST_ASSERT_EQUAL(CAN_PORT_ERR_ARG, vbus_send(&bus, a, &f));
    f = frame(0x20000000u, true, 1u, 0u);          /* 30-bit ID */
    TEST_ASSERT_EQUAL(CAN_PORT_ERR_ARG, vbus_send(&bus, a, &f));
    f = frame(0x100u, false, 1u, 0u);
    f.dlc = 9u;
    TEST_ASSERT_EQUAL(CAN_PORT_ERR_ARG, vbus_send(&bus, a, &f));
    f.dlc = 1u;
    TEST_ASSERT_EQUAL(CAN_PORT_ERR_ARG, vbus_send(&bus, 3u, &f)); /* not attached */
    TEST_ASSERT_EQUAL(CAN_PORT_ERR_ARG, vbus_send(&bus, VBUS_MAX_NODES, &f));
    TEST_ASSERT_EQUAL(CAN_PORT_ERR_ARG, vbus_recv(&bus, a, NULL));
    TEST_ASSERT_EQUAL_UINT32(0u, vbus_frame_count(&bus));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_attach_is_limited_to_max_nodes);
    RUN_TEST(test_frame_reaches_every_other_node_but_not_the_sender);
    RUN_TEST(test_frames_keep_their_order);
    RUN_TEST(test_full_rx_queue_drops_and_counts_overruns);
    RUN_TEST(test_blocked_tx_reports_full_and_sends_nothing);
    RUN_TEST(test_invalid_frames_and_nodes_are_rejected);
    return UNITY_END();
}
