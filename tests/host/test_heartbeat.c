/*
 * L0 tests for the heartbeat glue (features/heartbeat, D-064) on the host CAN port bound
 * to the in-process bus, with a manual clock: what a platform receiver (SAFETY, IO, CONN,
 * LINUX) gets on 0x081, when, with which NODE_MODE / ERROR_COUNT / UPTIME and which E2E
 * status. The UDS client and server are not stepped: the tests write their diag results
 * by hand, and the server's monitors never run (architecture-guard MAJOR-1).
 */
#include "app/host/sim_listener.h"
#include "features/heartbeat/heartbeat.h"
#include "hal/host/can_port_host.h"
#include "hal/host/hal_time_host.h"
#include "platform_uds.h"
#include "services/can_if.h"
#include "services/can_sm.h"
#include "services/diag.h"
#include "services/timebase.h"
#include "vehicle_cl250.h"

#include <string.h>
#include <unity.h>

#define START_MS 5000u

static vbus_t bus_p;
static vbus_t bus_v;
static uint8_t node_dut;
static uint8_t node_peer;
static uint8_t node_vehicle;
static heartbeat_t hb;
static moto_e2e_rx_state_t rx;
static bool client_running; /* the test writes a healthy tester status every pass */
static bool client_latched;

void setUp(void)
{
    hal_time_host_use_manual(START_MS);
    vbus_init(&bus_p);
    vbus_init(&bus_v);
    TEST_ASSERT_TRUE(vbus_attach(&bus_p, &node_dut));
    TEST_ASSERT_TRUE(vbus_attach(&bus_p, &node_peer));
    TEST_ASSERT_TRUE(vbus_attach(&bus_v, &node_vehicle));
    TEST_ASSERT_TRUE(can_port_host_bind_vbus(CAN_PORT_PLATFORM, &bus_p, node_dut));
    TEST_ASSERT_TRUE(can_port_host_bind_vbus(CAN_PORT_VEHICLE, &bus_v, node_vehicle));
    can_if_init();
    diag_init(timebase_now_ms());
    TEST_ASSERT_TRUE(heartbeat_open(&hb));
    TEST_ASSERT_EQUAL(CAN_PORT_OK, can_if_apply_filters(CAN_PORT_PLATFORM));
    can_sm_step(CAN_PORT_VEHICLE);
    can_sm_step(CAN_PORT_PLATFORM);
    moto_e2e_rx_init(&rx);
    client_running = true;
    client_latched = false;
}

void tearDown(void)
{
    can_port_host_unbind_all();
    hal_time_host_use_monotonic();
}

/* One comms pass (app/comms order, client and server replaced by the test), then one ms. */
static void pass(void)
{
    const uint32_t now = timebase_now_ms();
    can_sm_step(CAN_PORT_VEHICLE);
    if (client_running) {
        const diag_vehicle_tester_t st = {true, true, client_latched,
                                          (uint8_t)PLATFORM_UDS_VEHICLE_TESTER_STATUS_FAULT_NONE};
        diag_set_vehicle_tester(&st, now);
    }
    can_sm_step(CAN_PORT_PLATFORM);
    heartbeat_step(&hb);
    hal_time_host_advance(1u);
}

typedef struct {
    uint32_t frames;
    uint32_t e2e_ok;
    uint32_t e2e_initial;
    uint32_t e2e_other;
    uint32_t init;
    uint32_t normal;
    uint32_t degraded;
    struct platform_heartbeat_rt_core_t last;
} seen_t;

/* Drains the receiver and runs its E2E check, as SAFETY does (D-042 item 3). */
static void drain(seen_t* s)
{
    can_frame_t f;
    while (vbus_recv(&bus_p, node_peer, &f) == CAN_PORT_OK) {
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(PLATFORM_HEARTBEAT_RT_CORE_FRAME_ID, f.id, "only 0x081 here");
        TEST_ASSERT_EQUAL(PLATFORM_HEARTBEAT_RT_CORE_IS_EXTENDED != 0, f.extended);
        TEST_ASSERT_EQUAL_UINT8(PLATFORM_HEARTBEAT_RT_CORE_LENGTH, f.dlc);
        s->frames++;
        const moto_e2e_status_t st = moto_e2e_check(
            PLATFORM_HEARTBEAT_RT_CORE_E2E_DATA_ID, PLATFORM_HEARTBEAT_RT_CORE_E2E_MAX_DELTA_COUNTER,
            PLATFORM_HEARTBEAT_RT_CORE_E2E_TIMEOUT_MS, f.data, f.dlc, &rx, timebase_now_ms());
        s->e2e_ok += (st == MOTO_E2E_OK) ? 1u : 0u;
        s->e2e_initial += (st == MOTO_E2E_INITIAL) ? 1u : 0u;
        s->e2e_other += ((st != MOTO_E2E_OK) && (st != MOTO_E2E_INITIAL)) ? 1u : 0u;
        sim_listener_heartbeat_decode(f.data, &s->last); /* gen/ pack as the layout oracle */
        s->init += (s->last.node_mode == PLATFORM_HEARTBEAT_RT_CORE_NODE_MODE_INIT_CHOICE) ? 1u : 0u;
        s->normal += (s->last.node_mode == PLATFORM_HEARTBEAT_RT_CORE_NODE_MODE_NORMAL_CHOICE) ? 1u : 0u;
        s->degraded += (s->last.node_mode == PLATFORM_HEARTBEAT_RT_CORE_NODE_MODE_DEGRADED_CHOICE) ? 1u : 0u;
    }
}

static void run(seen_t* s, uint32_t ms)
{
    for (uint32_t i = 0u; i < ms; i++) {
        pass();
        drain(s);
    }
}

/* Latches the vehicle port: CAN_SM_VEHICLE_BUS_OFF_LATCH bus-offs, each recovered after
 * its backoff except the last (D-030, D-054). No heartbeat pass runs meanwhile. */
static void latch_vehicle_port(void)
{
    for (uint32_t k = 0u; k + 1u < CAN_SM_VEHICLE_BUS_OFF_LATCH; k++) {
        can_sm_step(CAN_PORT_VEHICLE);
        const uint32_t before = vbus_recover_count(&bus_v, node_vehicle);
        vbus_set_bus_off(&bus_v, node_vehicle, true);
        can_sm_step(CAN_PORT_VEHICLE);
        for (uint32_t i = 0u; (i < (2u * VEHICLE_CL250_BUS_OFF_BACKOFF_MAX_MS)) &&
                              (vbus_recover_count(&bus_v, node_vehicle) == before);
             i++) {
            hal_time_host_advance(1u);
            can_sm_step(CAN_PORT_VEHICLE);
        }
    }
    can_sm_step(CAN_PORT_VEHICLE);
    vbus_set_bus_off(&bus_v, node_vehicle, true);
    can_sm_step(CAN_PORT_VEHICLE);
    TEST_ASSERT_EQUAL(CAN_SM_LATCHED, can_sm_state(CAN_PORT_VEHICLE));
}

/* ------------------------------------------------------------------ nominal */

static void test_0x081_goes_out_every_cycle_from_the_first_pass_init_then_normal(void)
{
    seen_t s;
    memset(&s, 0, sizeof s);
    pass();
    drain(&s);
    TEST_ASSERT_EQUAL_UINT32(1u, s.frames); /* at once */
    TEST_ASSERT_EQUAL_UINT32(1u, s.init);   /* built before any pass completed */
    TEST_ASSERT_EQUAL_UINT32(1u, s.e2e_initial);
    run(&s, 1000u - 1u);
    TEST_ASSERT_EQUAL_UINT32(1000u / PLATFORM_HEARTBEAT_RT_CORE_CYCLE_TIME_MS, s.frames);
    TEST_ASSERT_EQUAL_UINT32(1u, s.init); /* only the first */
    TEST_ASSERT_EQUAL_UINT32(s.frames - 1u, s.normal);
    TEST_ASSERT_EQUAL_UINT32(s.frames - 1u, s.e2e_ok);
    TEST_ASSERT_EQUAL_UINT32(0u, s.e2e_other);
    TEST_ASSERT_EQUAL_UINT8(0u, s.last.error_count);
    TEST_ASSERT_EQUAL_UINT32(s.frames, hb.stats.sent);
    TEST_ASSERT_EQUAL_UINT32(0u, hb.stats.retried + hb.stats.dropped);
    TEST_ASSERT_EQUAL_UINT32(0u, vbus_frame_count(&bus_v)); /* nothing on the vehicle bus */
}

static void test_uptime_counts_whole_seconds_since_boot(void)
{
    seen_t s;
    memset(&s, 0, sizeof s);
    run(&s, 1u);
    TEST_ASSERT_EQUAL_UINT16(START_MS / 1000u, s.last.uptime); /* the timebase counts from reset */
    run(&s, 2000u);
    TEST_ASSERT_EQUAL_UINT16((START_MS + 2000u) / 1000u, s.last.uptime); /* last frame at +2000 ms */
}

/* ------------------------------------------------------------------ faults */

static void test_a_failed_monitor_degrades_until_it_passes_and_each_onset_counts_once(void)
{
    seen_t s;
    memset(&s, 0, sizeof s);
    run(&s, PLATFORM_HEARTBEAT_RT_CORE_CYCLE_TIME_MS + 1u); /* INIT, then the first NORMAL */
    TEST_ASSERT_EQUAL_UINT8(PLATFORM_HEARTBEAT_RT_CORE_NODE_MODE_NORMAL_CHOICE, s.last.node_mode);
    for (uint32_t i = 0u; i < 3u * PLATFORM_HEARTBEAT_RT_CORE_CYCLE_TIME_MS; i++) {
        diag_dtc_report(PLATFORM_UDS_DTC_IDX_VEHICLE_ECU_COMM_LOST, true); /* every pass */
        run(&s, 1u);
    }
    TEST_ASSERT_EQUAL_UINT8(PLATFORM_HEARTBEAT_RT_CORE_NODE_MODE_DEGRADED_CHOICE, s.last.node_mode);
    TEST_ASSERT_EQUAL_UINT8(1u, s.last.error_count);
    diag_dtc_report(PLATFORM_UDS_DTC_IDX_VEHICLE_ECU_COMM_LOST, false);
    run(&s, PLATFORM_HEARTBEAT_RT_CORE_CYCLE_TIME_MS);
    TEST_ASSERT_EQUAL_UINT8(PLATFORM_HEARTBEAT_RT_CORE_NODE_MODE_NORMAL_CHOICE, s.last.node_mode);
    TEST_ASSERT_EQUAL_UINT8(1u, s.last.error_count); /* faults since boot: kept */
    diag_dtc_report(PLATFORM_UDS_DTC_IDX_VEHICLE_ECU_COMM_LOST, true);
    diag_dtc_report(PLATFORM_UDS_DTC_IDX_VEHICLE_TESTER_LATCHED, true);
    run(&s, PLATFORM_HEARTBEAT_RT_CORE_CYCLE_TIME_MS);
    TEST_ASSERT_EQUAL_UINT8(PLATFORM_HEARTBEAT_RT_CORE_NODE_MODE_DEGRADED_CHOICE, s.last.node_mode);
    TEST_ASSERT_EQUAL_UINT8(3u, s.last.error_count);
    TEST_ASSERT_EQUAL_UINT32(0u, s.e2e_other);
}

static void test_a_dtc_clear_hides_no_fault_and_keeps_the_error_count(void)
{
    seen_t s;
    memset(&s, 0, sizeof s);
    diag_dtc_report(PLATFORM_UDS_DTC_IDX_VEHICLE_BUS_OFF_LATCHED, true);
    run(&s, PLATFORM_HEARTBEAT_RT_CORE_CYCLE_TIME_MS + 1u);
    TEST_ASSERT_EQUAL_UINT8(PLATFORM_HEARTBEAT_RT_CORE_NODE_MODE_DEGRADED_CHOICE, s.last.node_mode);
    diag_dtc_clear_all(); /* UDS 0x14 from a platform tester */
    TEST_ASSERT_EQUAL_UINT8(0u, diag_dtc_status(PLATFORM_UDS_DTC_IDX_VEHICLE_BUS_OFF_LATCHED));
    run(&s, PLATFORM_HEARTBEAT_RT_CORE_CYCLE_TIME_MS);
    TEST_ASSERT_EQUAL_UINT8(PLATFORM_HEARTBEAT_RT_CORE_NODE_MODE_DEGRADED_CHOICE, s.last.node_mode);
    TEST_ASSERT_EQUAL_UINT8(1u, s.last.error_count);
    diag_dtc_report(PLATFORM_UDS_DTC_IDX_VEHICLE_BUS_OFF_LATCHED, true); /* the monitor again */
    run(&s, PLATFORM_HEARTBEAT_RT_CORE_CYCLE_TIME_MS);
    TEST_ASSERT_EQUAL_UINT8(1u, s.last.error_count); /* not a new onset */
}

static void test_a_stopped_client_degrades_although_no_monitor_reports_it(void)
{
    seen_t s;
    memset(&s, 0, sizeof s);
    run(&s, PLATFORM_HEARTBEAT_RT_CORE_CYCLE_TIME_MS);
    client_running = false; /* no status; the server's diag_supervise() never runs either */
    run(&s, PLATFORM_UDS_VEHICLE_TESTER_STATUS_MAX_AGE_MS + PLATFORM_HEARTBEAT_RT_CORE_CYCLE_TIME_MS);
    TEST_ASSERT_FALSE(diag_fault_active()); /* no DTC monitor saw it */
    TEST_ASSERT_EQUAL_UINT8(PLATFORM_HEARTBEAT_RT_CORE_NODE_MODE_DEGRADED_CHOICE, s.last.node_mode);
    client_running = true;
    run(&s, PLATFORM_HEARTBEAT_RT_CORE_CYCLE_TIME_MS);
    TEST_ASSERT_EQUAL_UINT8(PLATFORM_HEARTBEAT_RT_CORE_NODE_MODE_NORMAL_CHOICE, s.last.node_mode);
}

static void test_a_latched_client_degrades_although_no_monitor_reports_it(void)
{
    seen_t s;
    memset(&s, 0, sizeof s);
    client_latched = true;
    run(&s, 2u * PLATFORM_HEARTBEAT_RT_CORE_CYCLE_TIME_MS);
    TEST_ASSERT_EQUAL_UINT32(0u, s.normal);
    TEST_ASSERT_EQUAL_UINT8(PLATFORM_HEARTBEAT_RT_CORE_NODE_MODE_DEGRADED_CHOICE, s.last.node_mode);
}

static void test_a_latched_vehicle_port_degrades_without_the_server_monitor(void)
{
    seen_t s;
    memset(&s, 0, sizeof s);
    run(&s, PLATFORM_HEARTBEAT_RT_CORE_CYCLE_TIME_MS + 1u); /* INIT, then the first NORMAL */
    TEST_ASSERT_EQUAL_UINT8(PLATFORM_HEARTBEAT_RT_CORE_NODE_MODE_NORMAL_CHOICE, s.last.node_mode);
    latch_vehicle_port();
    memset(&s, 0, sizeof s);
    moto_e2e_rx_init(&rx);
    run(&s, 2u * PLATFORM_HEARTBEAT_RT_CORE_CYCLE_TIME_MS);
    TEST_ASSERT_FALSE(diag_fault_active()); /* U0001-88 is reported by the server only */
    TEST_ASSERT_EQUAL_UINT32(0u, s.normal);
    TEST_ASSERT_EQUAL_UINT8(PLATFORM_HEARTBEAT_RT_CORE_NODE_MODE_DEGRADED_CHOICE, s.last.node_mode);
}

static void test_an_unknown_vehicle_port_state_is_not_read_as_a_latch(void)
{
    can_if_init(); /* the vehicle port state is unknown until its first can_sm_step() */
    TEST_ASSERT_TRUE(heartbeat_open(&hb));
    TEST_ASSERT_EQUAL(CAN_PORT_OK, can_if_apply_filters(CAN_PORT_PLATFORM));
    TEST_ASSERT_FALSE(can_sm_state_known(CAN_PORT_VEHICLE));
    seen_t s;
    memset(&s, 0, sizeof s);
    for (uint32_t i = 0u; i <= PLATFORM_HEARTBEAT_RT_CORE_CYCLE_TIME_MS; i++) {
        const diag_vehicle_tester_t st = {true, true, false,
                                          (uint8_t)PLATFORM_UDS_VEHICLE_TESTER_STATUS_FAULT_NONE};
        diag_set_vehicle_tester(&st, timebase_now_ms());
        can_sm_step(CAN_PORT_PLATFORM);
        heartbeat_step(&hb);
        hal_time_host_advance(1u);
        drain(&s);
    }
    TEST_ASSERT_FALSE(can_sm_state_known(CAN_PORT_VEHICLE));
    TEST_ASSERT_EQUAL_UINT32(2u, s.frames);
    /* no latch is invented (the UDS client reports an unreadable port its own way) */
    TEST_ASSERT_EQUAL_UINT8(PLATFORM_HEARTBEAT_RT_CORE_NODE_MODE_NORMAL_CHOICE, s.last.node_mode);
}

/* ------------------------------------------------------------------ transport */

static void test_after_a_stall_only_the_newest_frame_goes_out_and_never_reads_ok(void)
{
    seen_t s;
    memset(&s, 0, sizeof s);
    run(&s, PLATFORM_HEARTBEAT_RT_CORE_CYCLE_TIME_MS); /* frame 0 */
    vbus_set_tx_stalled(&bus_p, node_dut, true);
    run(&s, 10u * PLATFORM_HEARTBEAT_RT_CORE_CYCLE_TIME_MS + 5u);
    TEST_ASSERT_EQUAL_UINT32(1u, s.frames);
    TEST_ASSERT_GREATER_THAN_UINT32(0u, vbus_tx_replaced(&bus_p, node_dut));
    TEST_ASSERT_GREATER_THAN_UINT32(0u, hb.stats.retried); /* TX_FULL while a replace cancels */
    TEST_ASSERT_EQUAL_UINT32(0u, hb.stats.dropped);
    vbus_set_tx_stalled(&bus_p, node_dut, false);
    run(&s, 1u);
    TEST_ASSERT_EQUAL_UINT32(2u, s.frames); /* one frame, at most one cycle old */
    TEST_ASSERT_EQUAL_UINT32(0u, s.e2e_ok); /* the gap passed the 300 ms timeout: INITIAL */
    TEST_ASSERT_EQUAL_UINT32(2u, s.e2e_initial);
    TEST_ASSERT_EQUAL_UINT32(0u, s.e2e_other);
}

static void test_a_bus_off_platform_port_drops_cycles_and_the_receiver_resyncs(void)
{
    seen_t s;
    memset(&s, 0, sizeof s);
    run(&s, 2u * PLATFORM_HEARTBEAT_RT_CORE_CYCLE_TIME_MS);
    TEST_ASSERT_EQUAL_UINT32(1u, s.e2e_initial);
    vbus_set_bus_off(&bus_p, node_dut, true);
    run(&s, 4u * PLATFORM_HEARTBEAT_RT_CORE_CYCLE_TIME_MS);
    TEST_ASSERT_GREATER_THAN_UINT32(0u, hb.stats.dropped);
    vbus_set_bus_off(&bus_p, node_dut, false);
    const uint32_t ok_before = s.e2e_ok;
    run(&s, 30000u); /* backoff recovery, then the cycle again */
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(2u, s.e2e_initial, "a gap past the timeout resyncs, never OK");
    TEST_ASSERT_GREATER_THAN_UINT32(ok_before, s.e2e_ok);
    TEST_ASSERT_EQUAL_UINT32(0u, s.e2e_other);
}

static void test_a_refused_open_sends_nothing(void)
{
    heartbeat_t second;
    TEST_ASSERT_FALSE(heartbeat_open(&second)); /* the port is sealed */
    TEST_ASSERT_FALSE(second.opened);
    TEST_ASSERT_FALSE(heartbeat_open(NULL));
    heartbeat_step(&second);
    heartbeat_step(NULL);
    TEST_ASSERT_EQUAL_UINT32(0u, vbus_frame_count(&bus_p));
}

static void test_an_id_already_listed_is_refused(void)
{
    can_if_init(); /* unsealed again */
    heartbeat_t other;
    TEST_ASSERT_TRUE(heartbeat_open(&other));
    TEST_ASSERT_FALSE(heartbeat_open(&hb)); /* CAN_IF_ERR_DUP */
    heartbeat_step(&hb);
    TEST_ASSERT_EQUAL_UINT32(0u, vbus_frame_count(&bus_p));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_0x081_goes_out_every_cycle_from_the_first_pass_init_then_normal);
    RUN_TEST(test_uptime_counts_whole_seconds_since_boot);
    RUN_TEST(test_a_failed_monitor_degrades_until_it_passes_and_each_onset_counts_once);
    RUN_TEST(test_a_dtc_clear_hides_no_fault_and_keeps_the_error_count);
    RUN_TEST(test_a_stopped_client_degrades_although_no_monitor_reports_it);
    RUN_TEST(test_a_latched_client_degrades_although_no_monitor_reports_it);
    RUN_TEST(test_a_latched_vehicle_port_degrades_without_the_server_monitor);
    RUN_TEST(test_an_unknown_vehicle_port_state_is_not_read_as_a_latch);
    RUN_TEST(test_after_a_stall_only_the_newest_frame_goes_out_and_never_reads_ok);
    RUN_TEST(test_a_bus_off_platform_port_drops_cycles_and_the_receiver_resyncs);
    RUN_TEST(test_a_refused_open_sends_nothing);
    RUN_TEST(test_an_id_already_listed_is_refused);
    return UNITY_END();
}
