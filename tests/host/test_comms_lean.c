/*
 * SIL tests for the lean path end to end (D-065 PR 3b, E-17): the real comms_pass() with
 * the UDS client and the simulated CL250 ECU (speed DID 0xF40D fixed per phase), the
 * heartbeat, the 0x020 sender and the EKF task's lean_ekf_step() on the host IMU fake, on
 * the in-process buses with a manual clock. What a platform receiver sees is checked:
 * 0x020 frames (E2E, INVALID or not) and the 0x081 NODE_MODE.
 *
 * A 0x020 frame is recognised as INVALID by re-packing the gen/ INVALID message with the
 * frame's own E2E CRC and counter (gen/ offsets) and comparing all bytes: rt_core's gen/
 * has no 0x020 unpack, and no bit position is written by hand.
 */
#include "app/comms.h"
#include "app/host/sim_ecu.h"
#include "app/host/sim_listener.h"
#include "features/cornering/lean.h"
#include "features/cornering/lean_tx.h"
#include "hal/hal_atomic.h"
#include "hal/host/can_port_host.h"
#include "hal/host/hal_time_host.h"
#include "hal/host/imu_host.h"
#include "moto_e2e.h"
#include "platform.h"
#include "platform_e2e.h"
#include "services/can_if.h"
#include "services/can_sm.h"
#include "services/diag.h"
#include "services/timebase.h"
#include "services/vehicle_signals.h"

#include <math.h>
#include <string.h>
#include <unity.h>

#define START_MS 1000u
#define EKF_CYCLE_MS PLATFORM_EKF_LEAN_CYCLE_TIME_MS
#define Z_OFFSET (8)

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
static lean_link_t link;
static lean_ekf_t lean;
static lean_tx_t tx;
static sim_ecu_t ecu;
static uint32_t imu_pushed;
static int16_t imu_z = Z_OFFSET; /* raw z the IMU fake streams (a straight line by default) */
static moto_e2e_rx_state_t rx;

typedef struct {
    uint32_t hb_frames;
    uint32_t hb_normal;
    uint32_t hb_degraded;
    uint32_t lean_frames;
    uint32_t lean_invalid;
    uint32_t lean_e2e_bad;     /* frames the receiver's E2E check did not accept */
    uint32_t lean_e2e_ok;      /* counter exactly +1 */
    uint32_t lean_e2e_initial; /* resync after a gap beyond the timeout */
    uint32_t lean_gap_max_ms;  /* longest gap between 0x020 frames */
    uint32_t lean_last_ms;
    bool last_lean_invalid;
    uint8_t last_lean[PLATFORM_EKF_LEAN_LENGTH]; /* the last 0x020 frame's bytes */
} seen_t;

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
    ecu.fixed_enabled = true;
    ecu.fixed_did = VEHICLE_CL250_DID_VEHICLE_SPEED;
    ecu.fixed_raw = 0u;
    can_if_init();
    diag_init(timebase_now_ms());
    vehicle_signals_init();
    imu_host_reset();
    imu_pushed = 0u;
    imu_z = Z_OFFSET;
    memset(&client, 0, sizeof client);
    memset(&server, 0, sizeof server);
    TEST_ASSERT_EQUAL(ISOTP_OK, uds_client_open(&client));
    TEST_ASSERT_EQUAL(ISOTP_OK, uds_server_open(&server));
    TEST_ASSERT_TRUE(vehicle_republish_open(&republisher));
    TEST_ASSERT_TRUE(heartbeat_open(&heartbeat));
    comms_ekf_init(&ekf);
    lean_link_init(&link);
    lean_ekf_init(&lean, &link);
    moto_e2e_rx_init(&rx);
}

void tearDown(void)
{
    can_port_host_unbind_all();
    hal_time_host_use_monotonic();
}

/* The boot order of the cornering README: register, open 0x020, then the filters. */
static bool boot(const alive_counter_t* registered)
{
    if (registered != NULL) {
        TEST_ASSERT_TRUE(comms_ekf_register(&ekf, registered));
    }
    const bool opened = comms_open_lean(&ekf, &tx, &link);
    TEST_ASSERT_TRUE(comms_apply_filters());
    return opened;
}

static bool frame_is_invalid(const uint8_t* data)
{
    struct platform_ekf_lean_t m;
    (void)platform_ekf_lean_init(&m);
    m.lean_angle_state = PLATFORM_EKF_LEAN_LEAN_ANGLE_STATE_INVALID_CHOICE;
    m.lean_angle = 0;
    m.lean_angle_quality = 0u;
    m.e2_e_crc = data[MOTO_E2E_CRC_OFFSET];
    m.e2_e_counter = (uint8_t)(data[MOTO_E2E_COUNTER_OFFSET] & MOTO_E2E_COUNTER_MASK);
    uint8_t ref[PLATFORM_EKF_LEAN_LENGTH];
    TEST_ASSERT_EQUAL_INT((int)PLATFORM_EKF_LEAN_LENGTH, platform_ekf_lean_pack(ref, &m, sizeof ref));
    return memcmp(ref, data, sizeof ref) == 0;
}

/* True if data is the gen/ 0x020 frame with this state and LEAN for some QUALITY 1..100
 * (packed with the frame's own E2E CRC and counter, as in frame_is_invalid()). */
static bool frame_matches(const uint8_t* data, uint8_t state, int16_t lean_raw)
{
    for (uint8_t q = 1u; q <= 100u; q++) {
        struct platform_ekf_lean_t m;
        (void)platform_ekf_lean_init(&m);
        m.lean_angle_state = state;
        m.lean_angle = lean_raw;
        m.lean_angle_quality = q;
        m.e2_e_crc = data[MOTO_E2E_CRC_OFFSET];
        m.e2_e_counter = (uint8_t)(data[MOTO_E2E_COUNTER_OFFSET] & MOTO_E2E_COUNTER_MASK);
        uint8_t ref[PLATFORM_EKF_LEAN_LENGTH];
        TEST_ASSERT_EQUAL_INT((int)PLATFORM_EKF_LEAN_LENGTH, platform_ekf_lean_pack(ref, &m, sizeof ref));
        if (memcmp(ref, data, sizeof ref) == 0) {
            return true;
        }
    }
    return false;
}

/* Raw z gyro of a steady coordinated turn at roll_deg and v_kmh, plus the offset. */
static int16_t turn_z(float roll_deg, float v_kmh)
{
    const float v = v_kmh / LEAN_KMH_PER_MPS;
    const float r = LEAN_GRAVITY_MPS2 * sinf(roll_deg * 0.017453292f) / v;
    return (int16_t)(lroundf(r / IMU_HOST_DEFAULT_RPS_PER_LSB) + Z_OFFSET);
}

/* The IMU fake gets an 800 Hz stream up to now: at rest, straight, or a steady turn. */
static void feed_imu(uint32_t now)
{
    const uint32_t due = ((now - START_MS) * IMU_HOST_DEFAULT_ODR_HZ) / 1000u;
    while (imu_pushed < due) {
        const int16_t d = ((imu_pushed % 2u) == 0u) ? -1 : 1;
        const hal_imu_sample_t s = {{d, d, (int16_t)(imu_z + d)}};
        (void)imu_host_push(&s);
        imu_pushed++;
    }
}

/* ms passes of 1 ms at speed_kmh. The EKF task steps every divider EKF cycles (0: never),
 * before the comms pass (higher priority). Every frame the platform peer receives is
 * counted. */
static seen_t run(uint32_t ms, uint32_t speed_kmh, uint32_t divider)
{
    seen_t s;
    memset(&s, 0, sizeof s);
    ecu.fixed_raw = speed_kmh; /* 0xF40D formula A: raw = km/h */
    for (uint32_t i = 0u; i < ms; i++) {
        const uint32_t now = timebase_now_ms();
        feed_imu(now);
        if ((divider != 0u) && ((now % (EKF_CYCLE_MS * divider)) == 0u)) {
            lean_ekf_step(&lean, now);
        }
        sim_ecu_step(&ecu, now);
        comms_pass(&client, &server, &republisher, &heartbeat, &ekf, &tx);
        can_frame_t got;
        while (vbus_recv(&bus_p, node_ppeer, &got) == CAN_PORT_OK) {
            if (got.id == PLATFORM_HEARTBEAT_RT_CORE_FRAME_ID) {
                struct platform_heartbeat_rt_core_t hb;
                sim_listener_heartbeat_decode(got.data, &hb);
                s.hb_frames++;
                s.hb_normal += (hb.node_mode == PLATFORM_HEARTBEAT_RT_CORE_NODE_MODE_NORMAL_CHOICE) ? 1u : 0u;
                s.hb_degraded += (hb.node_mode == PLATFORM_HEARTBEAT_RT_CORE_NODE_MODE_DEGRADED_CHOICE) ? 1u : 0u;
            } else if (got.id == PLATFORM_EKF_LEAN_FRAME_ID) {
                const moto_e2e_status_t st = moto_e2e_check(
                    PLATFORM_EKF_LEAN_E2E_DATA_ID, PLATFORM_EKF_LEAN_E2E_MAX_DELTA_COUNTER,
                    PLATFORM_EKF_LEAN_E2E_TIMEOUT_MS, got.data, got.dlc, &rx, now);
                s.lean_e2e_bad += ((st == MOTO_E2E_OK) || (st == MOTO_E2E_INITIAL)) ? 0u : 1u;
                s.lean_e2e_ok += (st == MOTO_E2E_OK) ? 1u : 0u;
                s.lean_e2e_initial += (st == MOTO_E2E_INITIAL) ? 1u : 0u;
                if ((s.lean_frames > 0u) && ((now - s.lean_last_ms) > s.lean_gap_max_ms)) {
                    s.lean_gap_max_ms = now - s.lean_last_ms;
                }
                s.lean_last_ms = now;
                s.lean_frames++;
                s.last_lean_invalid = frame_is_invalid(got.data);
                (void)memcpy(s.last_lean, got.data, sizeof s.last_lean);
                s.lean_invalid += s.last_lean_invalid ? 1u : 0u;
            } else {
                /* 0x021, 0x110: not checked here */
            }
        }
        hal_time_host_advance(1u);
    }
    return s;
}

/* Boots with the EKF registered, stands still 3 s (offset learnt) and rides straight 2 s
 * at 54 km/h: the lean is ESTIMATED at the end. */
static void ride_to_estimated(void)
{
    TEST_ASSERT_TRUE(boot(lean_link_alive(&link)));
    (void)run(3000u, 0u, 1u);
    TEST_ASSERT_TRUE(lean.imu.core.offset_learned);
    const seen_t s = run(2000u, 54u, 1u);
    TEST_ASSERT_EQUAL(LEAN_OUT_ESTIMATED, lean.last.state);
    TEST_ASSERT_FALSE(s.last_lean_invalid);
}

static void test_a_running_ekf_sends_0x020_every_cycle_with_e2e(void)
{
    ride_to_estimated();
    const seen_t s = run(1000u, 54u, 1u);
    TEST_ASSERT_EQUAL_UINT32(1000u / EKF_CYCLE_MS, s.lean_frames);
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(EKF_CYCLE_MS + 1u, s.lean_gap_max_ms);
    TEST_ASSERT_EQUAL_UINT32(0u, s.lean_e2e_bad);
    TEST_ASSERT_EQUAL_UINT32(0u, s.lean_invalid);
    TEST_ASSERT_EQUAL_UINT32(s.hb_frames, s.hb_normal);
}

static void test_an_ekf_that_never_runs_reads_degraded_and_0x020_invalid(void)
{
    TEST_ASSERT_TRUE(boot(lean_link_alive(&link)));
    const seen_t s = run(1000u, 54u, 0u);
    TEST_ASSERT_EQUAL_UINT32(0u, s.hb_normal);
    TEST_ASSERT_EQUAL_UINT32(s.hb_frames - 1u, s.hb_degraded); /* all but INIT */
    TEST_ASSERT_GREATER_THAN_UINT32(0u, s.lean_frames);
    TEST_ASSERT_EQUAL_UINT32(s.lean_frames, s.lean_invalid);
}

static void test_an_ekf_at_a_third_of_its_rate_sends_invalid_while_node_mode_is_normal(void)
{
    ride_to_estimated();
    /* the outputs of the last full-rate cycles may still go out within their age budget */
    const seen_t t = run(LEAN_TX_AGE_MAX_MS + EKF_CYCLE_MS, 54u, 3u);
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(2u, t.lean_frames - t.lean_invalid);
    const seen_t s = run(2000u, 54u, 3u);
    /* max-gap alive supervision (MINOR-5, accepted): the heartbeat still reads NORMAL */
    TEST_ASSERT_EQUAL_UINT32(s.hb_frames, s.hb_normal);
    /* the 0x020 output age budget (and the core's 40 ms dt limit) catch it instead */
    TEST_ASSERT_GREATER_THAN_UINT32(0u, s.lean_frames);
    TEST_ASSERT_EQUAL_UINT32(s.lean_frames, s.lean_invalid);
}

static void test_a_stalled_comms_task_leaves_the_ekf_output_invalid_within_300_ms(void)
{
    ride_to_estimated();
    /* comms stalls: the EKF task alone runs on with the last handed-over speed */
    uint32_t invalid_after = 0u;
    for (uint32_t i = 0u; (i < 1000u) && (invalid_after == 0u); i++) {
        const uint32_t now = timebase_now_ms();
        feed_imu(now);
        if ((now % EKF_CYCLE_MS) == 0u) {
            lean_ekf_step(&lean, now);
            if (lean.last.state == LEAN_OUT_INVALID) {
                invalid_after = now - lean.speed.stamp_ms;
            }
        }
        hal_time_host_advance(1u);
    }
    TEST_ASSERT_NOT_EQUAL_UINT32(0u, invalid_after);
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(PLATFORM_LIMIT_VEHICLE_SPEED_MAX_AGE_MS + EKF_CYCLE_MS, invalid_after);
    TEST_ASSERT_EQUAL(LEAN_REASON_SPEED, lean.last.reason);
}

static void test_without_a_registered_ekf_0x020_is_never_sent(void)
{
    TEST_ASSERT_FALSE(boot(NULL));
    TEST_ASSERT_FALSE(tx.opened);
    TEST_ASSERT_FALSE(comms_open_lean(&ekf, NULL, &link)); /* NULL tx: nothing to reset */
    TEST_ASSERT_FALSE(comms_open_lean(NULL, &tx, &link));
    const seen_t s = run(3000u, 0u, 1u); /* the EKF runs, unsupervised */
    TEST_ASSERT_EQUAL_UINT32(0u, s.lean_frames);
    TEST_ASSERT_GREATER_THAN_UINT32(0u, s.hb_frames);
}

static void test_a_foreign_registered_counter_does_not_open_0x020_MAJOR_3(void)
{
    alive_counter_t other;
    alive_init(&other);
    TEST_ASSERT_FALSE(boot(&other));
    const seen_t s = run(1000u, 0u, 1u);
    TEST_ASSERT_EQUAL_UINT32(0u, s.lean_frames);
}

static void test_a_refused_frame_is_retried_and_the_counter_advances_only_when_accepted(void)
{
    ride_to_estimated();
    const moto_e2e_tx_state_t before = tx.e2e;
    vbus_set_tx_blocked(&bus_p, node_platform, true);
    const seen_t b = run(EKF_CYCLE_MS + 5u, 54u, 1u); /* due once, refused in every pass */
    TEST_ASSERT_EQUAL_UINT32(0u, b.lean_frames);
    TEST_ASSERT_GREATER_THAN_UINT32(0u, tx.stats.retried);
    TEST_ASSERT_EQUAL_UINT8(before.counter, tx.e2e.counter); /* D-056 item 5 */
    vbus_set_tx_blocked(&bus_p, node_platform, false);
    const seen_t s = run(2u * EKF_CYCLE_MS, 54u, 1u);
    TEST_ASSERT_GREATER_THAN_UINT32(0u, s.lean_frames);
    TEST_ASSERT_EQUAL_UINT32(s.lean_frames, s.lean_e2e_ok); /* no counter gap */
    TEST_ASSERT_EQUAL_UINT32(0u, s.lean_invalid);           /* the retry carries a fresh output */
    TEST_ASSERT_EQUAL_UINT32(0u, tx.stats.dropped);
}

static void test_a_bus_off_drops_cycles_without_advancing_the_counter_and_the_receiver_resyncs(void)
{
    ride_to_estimated();
    const moto_e2e_tx_state_t before = tx.e2e;
    vbus_set_bus_off(&bus_p, node_platform, true);
    const seen_t b = run(5u * EKF_CYCLE_MS, 54u, 1u);
    TEST_ASSERT_EQUAL_UINT32(0u, b.lean_frames);
    TEST_ASSERT_GREATER_THAN_UINT32(0u, tx.stats.dropped); /* one per cycle, no retry storm */
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(5u, tx.stats.dropped);
    TEST_ASSERT_EQUAL_UINT8(before.counter, tx.e2e.counter);
    vbus_set_bus_off(&bus_p, node_platform, false);
    const seen_t s = run(CAN_SM_PLATFORM_BACKOFF_INITIAL_MS + (10u * EKF_CYCLE_MS), 54u, 1u);
    TEST_ASSERT_GREATER_THAN_UINT32(0u, s.lean_frames);
    TEST_ASSERT_EQUAL_UINT32(1u, s.lean_e2e_initial); /* the gap exceeded the timeout */
    TEST_ASSERT_EQUAL_UINT32(0u, s.lean_e2e_bad);
}

static void test_a_corrupted_output_word_ages_out_to_invalid_MINOR_1(void)
{
    ride_to_estimated();
    hal_atomic_u32_store(&link.out_snap.middle, 3u); /* a RAM fault: no slot index */
    /* the last record may still go out within its age budget, then never again */
    const seen_t t = run(LEAN_TX_AGE_MAX_MS + EKF_CYCLE_MS, 54u, 1u);
    TEST_ASSERT_TRUE(snapshot_producer_fault(&link.out_snap));
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(2u, t.lean_frames - t.lean_invalid);
    const seen_t s = run(1000u, 54u, 1u);
    TEST_ASSERT_GREATER_THAN_UINT32(0u, s.lean_frames);
    TEST_ASSERT_EQUAL_UINT32(s.lean_frames, s.lean_invalid);
}

static void test_a_corrupted_speed_word_sends_invalid_at_once_MINOR_1(void)
{
    ride_to_estimated();
    hal_atomic_u32_store(&link.speed_snap.middle, 3u);
    const seen_t s = run(1000u, 54u, 1u);
    TEST_ASSERT_TRUE(snapshot_producer_fault(&link.speed_snap));
    TEST_ASSERT_GREATER_THAN_UINT32(0u, s.lean_frames);
    TEST_ASSERT_EQUAL_UINT32(s.lean_frames, s.lean_invalid);
}

static void test_another_ekf_supervision_stops_0x020_and_reads_degraded_MINOR_2(void)
{
    ride_to_estimated();
    comms_ekf_init(&ekf); /* the comms task now gets a zeroed comms_ekf_t: "no EKF" */
    (void)run(200u, 54u, 1u);
    const seen_t s = run(1000u, 54u, 1u);
    TEST_ASSERT_EQUAL_UINT32(0u, s.lean_frames); /* receivers see the timeout (D-042) */
    TEST_ASSERT_GREATER_THAN_UINT32(0u, s.hb_frames);
    TEST_ASSERT_EQUAL_UINT32(s.hb_frames, s.hb_degraded); /* never blind (D-065 amendment) */
}

static void test_a_null_supervision_with_an_open_lean_sends_no_0x020_MINOR_2(void)
{
    ride_to_estimated();
    uint32_t lean_frames = 0u;
    for (uint32_t i = 0u; i < 500u; i++) {
        const uint32_t now = timebase_now_ms();
        feed_imu(now);
        if ((now % EKF_CYCLE_MS) == 0u) {
            lean_ekf_step(&lean, now);
        }
        sim_ecu_step(&ecu, now);
        comms_pass(&client, &server, &republisher, &heartbeat, NULL, &tx);
        can_frame_t got;
        while (vbus_recv(&bus_p, node_ppeer, &got) == CAN_PORT_OK) {
            lean_frames += (got.id == PLATFORM_EKF_LEAN_FRAME_ID) ? 1u : 0u;
        }
        hal_time_host_advance(1u);
    }
    TEST_ASSERT_EQUAL_UINT32(0u, lean_frames);
}

static void test_a_real_lean_beyond_the_limit_goes_out_clamped_both_ways(void)
{
    ride_to_estimated();
    imu_z = turn_z(62.0f, 54.0f);
    seen_t s = run(4000u, 54u, 1u);
    TEST_ASSERT_EQUAL(LEAN_OUT_ESTIMATED, lean.last.state);
    TEST_ASSERT_GREATER_THAN_FLOAT(PLATFORM_LIMIT_LEAN_ANGLE_CLAMP_MAX_DEG, lean.last.roll_deg); /* EKF unclamped */
    TEST_ASSERT_TRUE(frame_matches(s.last_lean, PLATFORM_EKF_LEAN_LEAN_ANGLE_STATE_CLAMPED_CHOICE,
                                   platform_ekf_lean_lean_angle_encode(PLATFORM_LIMIT_LEAN_ANGLE_CLAMP_MAX_DEG)));
    TEST_ASSERT_EQUAL_UINT32(0u, s.lean_e2e_bad);
    imu_z = turn_z(-62.0f, 54.0f);
    s = run(4000u, 54u, 1u);
    TEST_ASSERT_EQUAL(LEAN_OUT_ESTIMATED, lean.last.state);
    TEST_ASSERT_LESS_THAN_FLOAT(-PLATFORM_LIMIT_LEAN_ANGLE_CLAMP_MAX_DEG, lean.last.roll_deg);
    TEST_ASSERT_TRUE(frame_matches(s.last_lean, PLATFORM_EKF_LEAN_LEAN_ANGLE_STATE_CLAMPED_CHOICE,
                                   platform_ekf_lean_lean_angle_encode(-PLATFORM_LIMIT_LEAN_ANGLE_CLAMP_MAX_DEG)));
    TEST_ASSERT_EQUAL_UINT32(0u, s.lean_e2e_bad);
}

static void test_a_comms_side_snapshot_fault_sends_invalid_MAJOR_4(void)
{
    ride_to_estimated();
    link.out_snap.consumer_fault = true; /* comms reads the output */
    seen_t s = run(2u * EKF_CYCLE_MS, 54u, 1u);
    TEST_ASSERT_GREATER_THAN_UINT32(0u, s.lean_frames);
    TEST_ASSERT_EQUAL_UINT32(s.lean_frames, s.lean_invalid);
    link.out_snap.consumer_fault = false;
    link.speed_snap.producer_fault = true; /* comms writes the speed */
    s = run(2u * EKF_CYCLE_MS, 54u, 1u);
    TEST_ASSERT_GREATER_THAN_UINT32(0u, s.lean_frames);
    TEST_ASSERT_EQUAL_UINT32(s.lean_frames, s.lean_invalid);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_a_running_ekf_sends_0x020_every_cycle_with_e2e);
    RUN_TEST(test_an_ekf_that_never_runs_reads_degraded_and_0x020_invalid);
    RUN_TEST(test_an_ekf_at_a_third_of_its_rate_sends_invalid_while_node_mode_is_normal);
    RUN_TEST(test_a_stalled_comms_task_leaves_the_ekf_output_invalid_within_300_ms);
    RUN_TEST(test_without_a_registered_ekf_0x020_is_never_sent);
    RUN_TEST(test_a_foreign_registered_counter_does_not_open_0x020_MAJOR_3);
    RUN_TEST(test_a_refused_frame_is_retried_and_the_counter_advances_only_when_accepted);
    RUN_TEST(test_a_bus_off_drops_cycles_without_advancing_the_counter_and_the_receiver_resyncs);
    RUN_TEST(test_a_comms_side_snapshot_fault_sends_invalid_MAJOR_4);
    RUN_TEST(test_a_corrupted_output_word_ages_out_to_invalid_MINOR_1);
    RUN_TEST(test_a_corrupted_speed_word_sends_invalid_at_once_MINOR_1);
    RUN_TEST(test_another_ekf_supervision_stops_0x020_and_reads_degraded_MINOR_2);
    RUN_TEST(test_a_null_supervision_with_an_open_lean_sends_no_0x020_MINOR_2);
    RUN_TEST(test_a_real_lean_beyond_the_limit_goes_out_clamped_both_ways);
    return UNITY_END();
}
