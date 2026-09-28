/*
 * SocketCAN backend of the host CAN port (Linux). Needs a vcan0 interface:
 *   sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0
 * Exits 77 (ctest SKIP) when vcan0 is not available.
 */
#include "hal/host/can_port_host.h"
#include "hal/host/hal_time_host.h"

#include <stdio.h>
#include <string.h>
#include <unity.h>

#define SKIP_EXIT 77
#define WAIT_MS 200u

void setUp(void) {}
void tearDown(void) {}

static can_port_status_t read_with_wait(can_port_id_t port, can_frame_t* out)
{
    for (uint32_t i = 0u; i < WAIT_MS; i++) {
        can_port_status_t st = can_port_read(port, out);
        if (st != CAN_PORT_EMPTY) {
            return st;
        }
        hal_time_host_sleep_ms(1u);
    }
    return CAN_PORT_EMPTY;
}

static void round_trip(uint32_t id, bool ext, uint8_t dlc)
{
    can_frame_t f;
    memset(&f, 0, sizeof f);
    f.id = id;
    f.extended = ext;
    f.dlc = dlc;
    for (uint8_t i = 0u; i < dlc; i++) {
        f.data[i] = (uint8_t)(0xA0u + i);
    }
    TEST_ASSERT_TRUE(can_port_tx_free(CAN_PORT_VEHICLE));
    TEST_ASSERT_EQUAL(CAN_PORT_OK, can_port_write(CAN_PORT_VEHICLE, &f));
    can_frame_t got;
    TEST_ASSERT_EQUAL(CAN_PORT_OK, read_with_wait(CAN_PORT_PLATFORM, &got));
    TEST_ASSERT_EQUAL_UINT32(id, got.id);
    TEST_ASSERT_EQUAL(ext, got.extended);
    TEST_ASSERT_EQUAL_UINT8(dlc, got.dlc);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(f.data, got.data, CAN_PORT_MAX_DLC);
    /* the sending socket does not see its own frame */
    TEST_ASSERT_EQUAL(CAN_PORT_EMPTY, can_port_read(CAN_PORT_VEHICLE, &got));
}

static void test_extended_frame_round_trip(void)
{
    round_trip(0x18DA10F1u, true, 8u);
}

static void test_standard_frame_round_trip(void)
{
    round_trip(0x123u, false, 3u);
}

static void test_invalid_frame_is_refused(void)
{
    can_frame_t f;
    memset(&f, 0, sizeof f);
    f.id = 0x800u;
    f.dlc = 1u;
    TEST_ASSERT_EQUAL(CAN_PORT_ERR_ARG, can_port_write(CAN_PORT_VEHICLE, &f));
}

int main(void)
{
    if (!can_port_host_bind_socketcan(CAN_PORT_VEHICLE, "vcan0") ||
        !can_port_host_bind_socketcan(CAN_PORT_PLATFORM, "vcan0")) {
        printf("vcan0 not available: skipped\n");
        can_port_host_unbind_all();
        return SKIP_EXIT;
    }
    UNITY_BEGIN();
    RUN_TEST(test_extended_frame_round_trip);
    RUN_TEST(test_standard_frame_round_trip);
    RUN_TEST(test_invalid_frame_is_refused);
    const int rc = UNITY_END();
    can_port_host_unbind_all();
    return rc;
}
