/*
 * L0 tests for the EKF alive supervision of the comms pass (D-064 item 4, D-065 item 2,
 * E-17 MAJOR-5): the real comms_pass() with the UDS client, the simulated CL250 ECU,
 * the republisher, the heartbeat and the server, on the in-process buses with a manual
 * clock. The EKF task is simulated by bumping its alive counter every gen/ EKF cycle.
 * What a platform receiver sees on 0x081 is checked: NODE_MODE NORMAL only while the
 * registered EKF completes its cycles, DEGRADED when it stalls or never ran, or when its
 * lean link is not open (D-086, ISSUES E-22 (1)). The EKF counter is the lean link's,
 * as at boot.
 */
#include "app/comms.h"
#include "app/host/sim_ecu.h"
#include "app/host/sim_listener.h"
#include "features/cornering/lean.h"
#include "hal/host/can_port_host.h"
#include "hal/host/hal_time_host.h"
#include "platform.h"
#include "services/alive.h"
#include "services/can_if.h"
#include "services/can_sm.h"
#include "services/diag.h"
#include "services/timebase.h"
#include "services/vehicle_signals.h"

#include <string.h>
#include <unity.h>

#define START_MS 1000u
#define EKF_CYCLE_MS PLATFORM_EKF_LEAN_CYCLE_TIME_MS
#define HB_CYCLE_MS PLATFORM_HEARTBEAT_RT_CORE_CYCLE_TIME_MS

static vbus_t bus_v;
static vbus_t bus_p;
static uint8_t node_vehicle;
static uint8_t node_platform;
static uint8_t node_ppeer;
static uds_client_t client;
static uds_server_t server;
static vehicle_republish_t republisher;
static heartbeat_t heartbeat;
static comms_ekf_t ekf;
static lean_link_t lnk;
static lean_tx_t tx;
static lean_tx_t* lean_arg; /* what the pass gets as its lean sender */
static sim_ecu_t ecu;

typedef struct {
    uint32_t frames;
    uint32_t normal;
    uint32_t degraded;
    uint32_t first_degraded_ms; /* time of the first DEGRADED frame, 0 if none */
    uint32_t first_normal_ms;   /* time of the first NORMAL frame, 0 if none */
    struct platform_heartbeat_rt_core_t last;
} hb_seen_t;

void setUp(void)
{
    hal_time_host_use_manual(START_MS);
    vbus_init(&bus_v);
    vbus_init(&bus_p);
    TEST_ASSERT_TRUE(vbus_attach(&bus_v, &node_vehicle));
    TEST_ASSERT_TRUE(vbus_attach(&bus_p, &node_platform));
    TEST_ASSERT_TRUE(vbus_attach(&bus_p, &node_ppeer));
    TEST_ASSERT_TRUE(can_port_host_bind_vbus(CAN_PORT_VEHICLE, &bus_v, node_vehicle));
    TEST_ASSERT_TRUE(can_port_host_bind_vbus(CAN_PORT_PLATFORM, &bus_p, node_platform));
    TEST_ASSERT_TRUE(sim_ecu_init(&ecu, &bus_v));
    ecu.require_session = true;
    can_if_init();
    diag_init(timebase_now_ms());
    vehicle_signals_init();
    memset(&client, 0, sizeof client);
    memset(&server, 0, sizeof server);
    TEST_ASSERT_EQUAL(ISOTP_OK, uds_client_open(&client));
    TEST_ASSERT_EQUAL(ISOTP_OK, uds_server_open(&server));
    TEST_ASSERT_TRUE(vehicle_republish_open(&republisher));
    TEST_ASSERT_TRUE(heartbeat_open(&heartbeat));
    comms_ekf_init(&ekf);
    lean_link_init(&lnk);
    memset(&tx, 0, sizeof tx);
    lean_arg = &tx;
}

void tearDown(void)
{
    can_port_host_unbind_all();
    hal_time_host_use_monotonic();
}

/* The boot order of the cornering README: register the lean link's counter, open 0x020 (or
 * not), then the filters. */
static void start(bool open_lean)
{
    TEST_ASSERT_TRUE(comms_ekf_register(&ekf, lean_link_alive(&lnk)));
    if (open_lean) {
        TEST_ASSERT_TRUE(comms_open_lean(&ekf, &tx, &lnk));
        TEST_ASSERT_TRUE(tx.opened);
    }
    TEST_ASSERT_TRUE(comms_apply_filters());
}

/* ms comms passes of 1 ms; the simulated EKF bumps its counter every EKF cycle while
 * ekf_runs. Every 0x081 the platform peer receives is counted. */
static hb_seen_t run(uint32_t ms, bool ekf_runs, comms_ekf_t* e)
{
    hb_seen_t seen;
    memset(&seen, 0, sizeof seen);
    for (uint32_t i = 0u; i < ms; i++) {
        const uint32_t now = timebase_now_ms();
        if (ekf_runs && ((now % EKF_CYCLE_MS) == 0u)) {
            alive_bump(&lnk.alive); /* the EKF task completed a cycle */
        }
        sim_ecu_step(&ecu, now);
        comms_pass(&client, &server, &republisher, &heartbeat, e, lean_arg);
        can_frame_t got;
        while (vbus_recv(&bus_p, node_ppeer, &got) == CAN_PORT_OK) {
            if (got.id != PLATFORM_HEARTBEAT_RT_CORE_FRAME_ID) {
                continue;
            }
            sim_listener_heartbeat_decode(got.data, &seen.last);
            seen.frames++;
            if (seen.last.node_mode == PLATFORM_HEARTBEAT_RT_CORE_NODE_MODE_NORMAL_CHOICE) {
                seen.normal++;
                seen.first_normal_ms = (seen.first_normal_ms == 0u) ? now : seen.first_normal_ms;
            } else if (seen.last.node_mode == PLATFORM_HEARTBEAT_RT_CORE_NODE_MODE_DEGRADED_CHOICE) {
                seen.degraded++;
                seen.first_degraded_ms = (seen.first_degraded_ms == 0u) ? now : seen.first_degraded_ms;
            } else {
                /* INIT: the first frame only */
            }
        }
        hal_time_host_advance(1u);
    }
    return seen;
}

static void test_no_registered_ekf_leaves_node_mode_as_before(void)
{
    TEST_ASSERT_FALSE(comms_ekf_registered(&ekf));
    TEST_ASSERT_TRUE(comms_apply_filters());
    const hb_seen_t s = run(1000u, false, &ekf);
    TEST_ASSERT_EQUAL_UINT32(0u, s.degraded);
    TEST_ASSERT_EQUAL_UINT32(s.frames - 1u, s.normal); /* all but the INIT frame */
}

static void test_a_registered_ekf_that_never_ran_reads_degraded_from_the_first_cycle(void)
{
    start(true);
    TEST_ASSERT_TRUE(comms_ekf_registered(&ekf));
    const hb_seen_t s = run(1000u, false, &ekf);
    TEST_ASSERT_EQUAL_UINT32(0u, s.normal);
    TEST_ASSERT_EQUAL_UINT32(s.frames - 1u, s.degraded);
    TEST_ASSERT_EQUAL_UINT8(0u, s.last.error_count); /* not a DTC onset */
}

static void test_the_baseline_is_the_count_at_registration_not_zero(void)
{
    for (uint32_t k = 0u; k < 5u; k++) {
        alive_bump(&lnk.alive); /* the EKF ran before it was registered */
    }
    start(true);
    const hb_seen_t s = run(1000u, false, &ekf);
    TEST_ASSERT_EQUAL_UINT32(0u, s.normal); /* a nonzero count is no proof of life */
}

static void test_a_running_ekf_reads_normal_and_a_stall_degrades_within_the_window(void)
{
    start(true);
    hb_seen_t s = run(1000u, true, &ekf);
    TEST_ASSERT_EQUAL_UINT32(0u, s.degraded);
    TEST_ASSERT_EQUAL_UINT32(s.frames - 1u, s.normal);

    /* run() bumps at multiples of the EKF cycle: find the last bump before the stall */
    const uint32_t stall_ms = timebase_now_ms();
    const uint32_t last_bump_ms = stall_ms - 1u - ((stall_ms - 1u) % EKF_CYCLE_MS);
    s = run(1000u, false, &ekf);
    TEST_ASSERT_NOT_EQUAL_UINT32(0u, s.first_degraded_ms);
    /* stalled once more than the window passed, sent at the next heartbeat cycle */
    TEST_ASSERT_GREATER_THAN_UINT32(last_bump_ms + COMMS_EKF_ALIVE_WINDOW_MS, s.first_degraded_ms);
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(last_bump_ms + COMMS_EKF_ALIVE_WINDOW_MS + 1u + HB_CYCLE_MS,
                                     s.first_degraded_ms);
    TEST_ASSERT_EQUAL_UINT8(PLATFORM_HEARTBEAT_RT_CORE_NODE_MODE_DEGRADED_CHOICE, s.last.node_mode);

    /* the EKF resumes: NORMAL from the next heartbeat cycle on */
    const uint32_t resume_ms = timebase_now_ms();
    s = run(1000u, true, &ekf);
    TEST_ASSERT_NOT_EQUAL_UINT32(0u, s.first_normal_ms);
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(resume_ms + EKF_CYCLE_MS + HB_CYCLE_MS, s.first_normal_ms);
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(1u, s.degraded); /* at most the frame before the resume */
    TEST_ASSERT_EQUAL_UINT8(PLATFORM_HEARTBEAT_RT_CORE_NODE_MODE_NORMAL_CHOICE, s.last.node_mode);
}

static void test_one_late_ekf_cycle_is_not_a_stall(void)
{
    start(true);
    (void)run(500u, true, &ekf); /* last bump 20 ms before the end */
    /* two cycles missed: the next bump comes 3 cycles = the window after the last, not more */
    hb_seen_t s = run(2u * EKF_CYCLE_MS, false, &ekf);
    TEST_ASSERT_EQUAL_UINT32(0u, s.degraded);
    s = run(1000u, true, &ekf);
    TEST_ASSERT_EQUAL_UINT32(0u, s.degraded);
}

static void test_a_missing_supervision_object_reads_degraded(void)
{
    TEST_ASSERT_TRUE(comms_apply_filters());
    const hb_seen_t s = run(1000u, true, NULL);
    TEST_ASSERT_EQUAL_UINT32(0u, s.normal);
    TEST_ASSERT_EQUAL_UINT32(s.frames - 1u, s.degraded);
}

static void test_register_refuses_null_and_a_second_counter(void)
{
    alive_counter_t other;
    alive_init(&other);
    TEST_ASSERT_FALSE(comms_ekf_register(NULL, &lnk.alive));
    TEST_ASSERT_FALSE(comms_ekf_register(&ekf, NULL));
    TEST_ASSERT_FALSE(comms_ekf_registered(&ekf));
    TEST_ASSERT_FALSE(comms_ekf_registered(NULL));
    TEST_ASSERT_TRUE(comms_ekf_register(&ekf, &lnk.alive));
    TEST_ASSERT_FALSE(comms_ekf_register(&ekf, &other));
    TEST_ASSERT_EQUAL_PTR(&lnk.alive, ekf.counter); /* the first stays */
    comms_ekf_init(NULL);                          /* harmless */
}

/* D-086, ISSUES E-22 (1): a registered EKF whose lean link is not open reads as a
 * stalled EKF, even while its counter keeps changing. */
static void test_a_registered_ekf_without_a_lean_sender_reads_degraded_E22(void)
{
    start(false);
    lean_arg = NULL;
    const hb_seen_t s = run(1000u, true, &ekf);
    TEST_ASSERT_EQUAL_UINT32(0u, s.normal);
    TEST_ASSERT_EQUAL_UINT32(s.frames - 1u, s.degraded);
}

static void test_a_registered_ekf_with_a_never_opened_lean_tx_reads_degraded_E22(void)
{
    start(false);
    TEST_ASSERT_FALSE(tx.opened);
    const hb_seen_t s = run(1000u, true, &ekf);
    TEST_ASSERT_EQUAL_UINT32(0u, s.normal);
    TEST_ASSERT_EQUAL_UINT32(s.frames - 1u, s.degraded);
}

static void test_no_ekf_and_no_lean_sender_stays_normal_E22(void)
{
    TEST_ASSERT_TRUE(comms_apply_filters());
    lean_arg = NULL;
    const hb_seen_t s = run(1000u, true, &ekf);
    TEST_ASSERT_EQUAL_UINT32(0u, s.degraded);
    TEST_ASSERT_EQUAL_UINT32(s.frames - 1u, s.normal);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_no_registered_ekf_leaves_node_mode_as_before);
    RUN_TEST(test_a_registered_ekf_that_never_ran_reads_degraded_from_the_first_cycle);
    RUN_TEST(test_the_baseline_is_the_count_at_registration_not_zero);
    RUN_TEST(test_a_running_ekf_reads_normal_and_a_stall_degrades_within_the_window);
    RUN_TEST(test_one_late_ekf_cycle_is_not_a_stall);
    RUN_TEST(test_a_missing_supervision_object_reads_degraded);
    RUN_TEST(test_register_refuses_null_and_a_second_counter);
    RUN_TEST(test_a_registered_ekf_without_a_lean_sender_reads_degraded_E22);
    RUN_TEST(test_a_registered_ekf_with_a_never_opened_lean_tx_reads_degraded_E22);
    RUN_TEST(test_no_ekf_and_no_lean_sender_stays_normal_E22);
    return UNITY_END();
}
