/*
 * L0 tests for the lean glue (features/cornering/lean + lean_tx, D-065 item 2, PR 3b):
 * the EKF-task step on the host IMU fake and the comms-task speed hand-over through
 * services/vehicle_signals and the two snapshots. Single-threaded: the "tasks" are called
 * in turn with an explicit time.
 */
#include "features/cornering/lean.h"
#include "features/cornering/lean_tx.h"
#include "hal/host/imu_host.h"
#include "services/can_if.h"
#include "services/vehicle_signals.h"
#include "vehicle_cl250.h"

#include <math.h>
#include <string.h>
#include <unity.h>

#define CYCLE_MS PLATFORM_EKF_LEAN_CYCLE_TIME_MS
#define N_CYCLE (16u) /* IMU samples per cycle at the fake's 800 Hz */
#define Z_OFFSET (8)  /* raw z gyro offset, about 0.49 deg/s */
#define DEG2RAD (0.017453292f)

static lean_link_t link;
static lean_ekf_t ekf;
static lean_tx_t tx;
static uint32_t now;

void setUp(void)
{
    imu_host_reset();
    vehicle_signals_init();
    can_if_init();
    lean_link_init(&link);
    lean_ekf_init(&ekf, &link);
    TEST_ASSERT_TRUE(lean_tx_open(&tx, &link, lean_link_alive(&link)));
    now = 1000u;
}

void tearDown(void) {}

/* One cycle's IMU samples around raw (x, z) with -1/+1 LSB of noise. */
static void push_imu(int16_t x, int16_t z)
{
    for (uint32_t i = 0u; i < N_CYCLE; i++) {
        const int16_t d = ((i % 2u) == 0u) ? -1 : 1;
        const hal_imu_sample_t s = {{(int16_t)(x + d), d, (int16_t)(z + d)}};
        TEST_ASSERT_TRUE(imu_host_push(&s));
    }
}

/* The client wrote a speed sample sent at stamp_ms; the comms pass hands it over. */
static void comms_speed(float kmh, uint32_t stamp_ms)
{
    TEST_ASSERT_TRUE(vehicle_signals_write(VEHICLE_CL250_IDX_VEHICLE_SPEED, (uint32_t)kmh, kmh, stamp_ms));
    lean_tx_publish_speed(&tx, now);
}

/* One EKF cycle at now (speed fresh, IMU fed), then now += CYCLE_MS. */
static lean_out_t cycle(float kmh, int16_t x, int16_t z)
{
    comms_speed(kmh, now);
    push_imu(x, z);
    lean_ekf_step(&ekf, now);
    now += CYCLE_MS;
    return ekf.last;
}

/* Raw z gyro of a steady coordinated turn at roll_deg and v_kmh, plus the offset. */
static int16_t turn_z(float roll_deg, float v_kmh)
{
    const float v = v_kmh / LEAN_KMH_PER_MPS;
    const float r = LEAN_GRAVITY_MPS2 * sinf(roll_deg * DEG2RAD) / v;
    return (int16_t)(lroundf(r / IMU_HOST_DEFAULT_RPS_PER_LSB) + Z_OFFSET);
}

/* 2.2 s at standstill: the z offset is learnt. */
static void stand(void)
{
    for (uint32_t i = 0u; i < 110u; i++) {
        (void)cycle(0.0f, 0, Z_OFFSET);
    }
    TEST_ASSERT_TRUE(ekf.imu.core.offset_learned);
}

static lean_record_t newest_record(void)
{
    uint8_t i = 0u;
    (void)snapshot_read_index(&link.out_snap, &i);
    return link.out_slots[i];
}

static void test_boot_slots_are_invalid(void)
{
    const lean_record_t r = newest_record();
    TEST_ASSERT_EQUAL(LEAN_OUT_INVALID, r.out.state);
    TEST_ASSERT_EQUAL_UINT32(0u, r.stamp_ms);
    struct platform_ekf_lean_t m;
    lean_tx_msg(&r, false, now, &m);
    TEST_ASSERT_EQUAL_UINT8(PLATFORM_EKF_LEAN_LEAN_ANGLE_STATE_INVALID_CHOICE, m.lean_angle_state);
    TEST_ASSERT_EQUAL(LEAN_SPEED_UNUSABLE, link.speed_slots[0].state);
}

static void test_without_a_learnt_offset_the_lean_is_invalid(void)
{
    for (uint32_t i = 0u; i < 100u; i++) {
        const lean_out_t out = cycle(54.0f, 0, Z_OFFSET); /* riding from boot */
        TEST_ASSERT_EQUAL(LEAN_OUT_INVALID, out.state);
        TEST_ASSERT_EQUAL(LEAN_REASON_INPUT, out.reason);
    }
    TEST_ASSERT_EQUAL(IMU_REASON_OFFSET_UNLEARNED, ekf.last_imu.reason);
}

static void test_after_a_stop_turns_both_ways_read_true_with_the_offset_removed(void)
{
    stand();
    lean_out_t out = {0};
    for (uint32_t i = 0u; i < 100u; i++) {
        out = cycle(54.0f, 0, turn_z(30.0f, 54.0f));
    }
    TEST_ASSERT_EQUAL(LEAN_OUT_ESTIMATED, out.state);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 30.0f, out.roll_deg);
    for (uint32_t i = 0u; i < 150u; i++) {
        out = cycle(54.0f, 0, turn_z(-30.0f, 54.0f));
    }
    TEST_ASSERT_EQUAL(LEAN_OUT_ESTIMATED, out.state);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, -30.0f, out.roll_deg);
    /* the published record is this output, stamped with the IMU read */
    const lean_record_t r = newest_record();
    TEST_ASSERT_EQUAL_MEMORY(&out, &r.out, sizeof out);
    TEST_ASSERT_EQUAL_UINT32(now - CYCLE_MS, r.stamp_ms);
    TEST_ASSERT_FALSE(r.fault);
}

static void test_alive_bumps_once_per_complete_cycle_and_never_on_no_new(void)
{
    const uint32_t c0 = alive_count(lean_link_alive(&link));
    (void)cycle(0.0f, 0, Z_OFFSET); /* INVALID (START), still a complete cycle */
    TEST_ASSERT_EQUAL_UINT32(c0 + 1u, alive_count(lean_link_alive(&link)));
    /* a second call in the same ms: nothing published, no bump, FIFO untouched */
    now -= CYCLE_MS;
    const lean_record_t before = newest_record();
    push_imu(0, Z_OFFSET);
    lean_ekf_step(&ekf, now);
    TEST_ASSERT_EQUAL(IMU_REASON_NO_NEW, ekf.last_imu.reason);
    TEST_ASSERT_EQUAL_UINT32(c0 + 1u, alive_count(lean_link_alive(&link)));
    TEST_ASSERT_EQUAL_UINT32(N_CYCLE, imu_host_level());
    const lean_record_t after = newest_record();
    TEST_ASSERT_EQUAL_MEMORY(&before, &after, sizeof before);
}

static void test_missed_imu_periods_reach_the_core_as_a_gap(void)
{
    stand();
    for (uint32_t i = 0u; i < 100u; i++) {
        (void)cycle(54.0f, 0, Z_OFFSET);
    }
    TEST_ASSERT_EQUAL(LEAN_OUT_ESTIMATED, ekf.last.state);
    for (uint32_t missed = 1u; missed <= 3u; missed++) {
        for (uint32_t m = 0u; m < missed; m++) { /* no samples: STALE, INVALID without a core step */
            comms_speed(54.0f, now);
            lean_ekf_step(&ekf, now);
            now += CYCLE_MS;
            TEST_ASSERT_EQUAL(LEAN_OUT_INVALID, ekf.last.state);
            TEST_ASSERT_EQUAL(LEAN_REASON_INPUT, ekf.last.reason);
            TEST_ASSERT_EQUAL(IMU_REASON_STALE, ekf.last_imu.reason);
        }
        const lean_out_t out = cycle(54.0f, 0, Z_OFFSET);
        if (missed == 1u) {
            TEST_ASSERT_EQUAL(LEAN_OUT_ESTIMATED, out.state); /* 40 ms gap: predicted across */
        } else {
            TEST_ASSERT_EQUAL(LEAN_REASON_DT, out.reason); /* beyond 40 ms: reinit */
            for (uint32_t i = 0u; i < 60u; i++) {
                (void)cycle(54.0f, 0, Z_OFFSET);
            }
            TEST_ASSERT_EQUAL(LEAN_OUT_ESTIMATED, ekf.last.state);
        }
    }
}

static void test_a_stalled_comms_task_makes_the_lean_invalid_within_300_ms(void)
{
    stand();
    for (uint32_t i = 0u; i < 100u; i++) {
        (void)cycle(54.0f, 0, Z_OFFSET);
    }
    TEST_ASSERT_EQUAL(LEAN_OUT_ESTIMATED, ekf.last.state);
    const uint32_t last_speed = now - CYCLE_MS; /* the last stamp comms handed over */
    uint32_t invalid_at = 0u;
    for (uint32_t i = 0u; (i < 50u) && (invalid_at == 0u); i++) { /* comms stalled: EKF only */
        push_imu(0, Z_OFFSET);
        lean_ekf_step(&ekf, now);
        if (ekf.last.state == LEAN_OUT_INVALID) {
            invalid_at = now;
            TEST_ASSERT_EQUAL(LEAN_REASON_SPEED, ekf.last.reason);
        }
        now += CYCLE_MS;
    }
    TEST_ASSERT_NOT_EQUAL_UINT32(0u, invalid_at);
    TEST_ASSERT_TRUE((invalid_at - last_speed) <= (PLATFORM_LIMIT_VEHICLE_SPEED_MAX_AGE_MS + CYCLE_MS));
}

static void test_only_a_valid_speed_inside_the_gen_range_is_handed_over(void)
{
    comms_speed(54.0f, now);
    lean_ekf_step(&ekf, now);
    TEST_ASSERT_EQUAL(LEAN_SPEED_VALID, ekf.speed.state);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-5f, 15.0f, ekf.speed.speed_mps);
    TEST_ASSERT_EQUAL_UINT32(now, ekf.speed.stamp_ms);
    const float bad[] = {256.0f, -1.0f, NAN};
    for (uint32_t i = 0u; i < (sizeof bad / sizeof bad[0]); i++) {
        now += CYCLE_MS;
        if (vehicle_signals_write(VEHICLE_CL250_IDX_VEHICLE_SPEED, 0u, bad[i], now)) {
            lean_tx_publish_speed(&tx, now);
            lean_ekf_step(&ekf, now);
            TEST_ASSERT_EQUAL(LEAN_SPEED_UNUSABLE, ekf.speed.state);
        }
    }
    /* a STALE sample (older than its gen/ stale_after_ms) is not VALID either */
    comms_speed(54.0f, now - 1000u);
    lean_ekf_step(&ekf, now);
    TEST_ASSERT_EQUAL(LEAN_SPEED_UNUSABLE, ekf.speed.state);
}

static void test_standstill_needs_two_recent_zero_samples(void)
{
    /* 0 and 10 km/h alternating per sample: never two zeros in a row, never learnt */
    for (uint32_t i = 0u; i < 300u; i++) {
        (void)cycle(((i % 2u) == 0u) ? 0.0f : 10.0f, 0, Z_OFFSET);
    }
    TEST_ASSERT_FALSE(ekf.imu.core.offset_learned);
    /* 1 km/h is not standstill */
    for (uint32_t i = 0u; i < 150u; i++) {
        (void)cycle(1.0f, 0, Z_OFFSET);
    }
    TEST_ASSERT_FALSE(ekf.imu.core.offset_learned);
    /* a zero that stops being refreshed is standstill only up to its 200 ms age */
    (void)cycle(0.0f, 0, Z_OFFSET);
    (void)cycle(0.0f, 0, Z_OFFSET); /* two zero samples seen, then no more updates */
    for (uint32_t i = 0u; i < 150u; i++) {
        push_imu(0, Z_OFFSET);
        lean_ekf_step(&ekf, now);
        now += CYCLE_MS;
    }
    TEST_ASSERT_FALSE(ekf.imu.core.offset_learned);
    /* fresh zeros: learnt after 2 s */
    stand();
}

static void test_snapshot_faults_reach_the_record_and_read_invalid(void)
{
    stand();
    for (uint32_t i = 0u; i < 100u; i++) {
        (void)cycle(54.0f, 0, Z_OFFSET);
    }
    link.speed_snap.consumer_fault = true; /* the EKF side's latch */
    (void)cycle(54.0f, 0, Z_OFFSET);
    lean_record_t r = newest_record();
    TEST_ASSERT_TRUE(r.fault);
    struct platform_ekf_lean_t m;
    lean_tx_msg(&r, false, r.stamp_ms, &m);
    TEST_ASSERT_EQUAL_UINT8(PLATFORM_EKF_LEAN_LEAN_ANGLE_STATE_INVALID_CHOICE, m.lean_angle_state);
}

static void test_0x020_opens_only_for_this_links_counter_MAJOR_3(void)
{
    lean_link_t other;
    lean_link_init(&other);
    lean_tx_t t;
    can_if_init();
    TEST_ASSERT_FALSE(lean_tx_open(&t, &link, lean_link_alive(&other)));
    TEST_ASSERT_FALSE(t.opened);
    TEST_ASSERT_FALSE(lean_tx_open(&t, &link, NULL));
    TEST_ASSERT_FALSE(lean_tx_open(&t, NULL, lean_link_alive(&link)));
    TEST_ASSERT_FALSE(lean_tx_open(NULL, &link, lean_link_alive(&link)));
    TEST_ASSERT_NULL(lean_link_alive(NULL));
    /* a closed sender hands over no speed */
    TEST_ASSERT_TRUE(vehicle_signals_write(VEHICLE_CL250_IDX_VEHICLE_SPEED, 54u, 54.0f, now));
    lean_tx_publish_speed(&t, now);
    lean_tx_publish_speed(NULL, now);
    lean_tx_step(&t, now);
    lean_tx_step(NULL, now);
    lean_ekf_step(&ekf, now);
    TEST_ASSERT_EQUAL(LEAN_SPEED_UNUSABLE, ekf.speed.state);
    TEST_ASSERT_TRUE(lean_tx_open(&t, &link, lean_link_alive(&link)));
}

/* One EKF cycle with no new speed sample (the last one stays in the snapshot). */
static void imu_only_cycle(void)
{
    push_imu(0, Z_OFFSET);
    lean_ekf_step(&ekf, now);
    now += CYCLE_MS;
}

static void test_an_unusable_speed_breaks_the_standstill_window_at_once(void)
{
    /* 1.9 s of zeros: not yet learnt */
    for (uint32_t i = 0u; i < 95u; i++) {
        (void)cycle(0.0f, 0, Z_OFFSET);
    }
    TEST_ASSERT_FALSE(ekf.imu.core.offset_learned);
    /* an out-of-range sample (UNUSABLE, handed over as 0 m/s) right after the zeros, then
     * no new sample for 180 ms: the window must not reach 2 s through it */
    TEST_ASSERT_TRUE(vehicle_signals_write(VEHICLE_CL250_IDX_VEHICLE_SPEED, 0u, 256.0f, now));
    lean_tx_publish_speed(&tx, now);
    imu_only_cycle();
    TEST_ASSERT_EQUAL(LEAN_SPEED_UNUSABLE, ekf.speed.state);
    for (uint32_t i = 0u; i < 9u; i++) {
        imu_only_cycle();
    }
    TEST_ASSERT_FALSE(ekf.imu.core.offset_learned);
    /* 1.2 s more of zeros: 2.4 s since the first zero, but only 1.2 s since the break */
    for (uint32_t i = 0u; i < 60u; i++) {
        (void)cycle(0.0f, 0, Z_OFFSET);
    }
    TEST_ASSERT_FALSE(ekf.imu.core.offset_learned);
    stand();
}

static void test_the_standstill_window_starts_at_the_second_zero_sample(void)
{
    for (uint32_t i = 0u; i < 10u; i++) {
        (void)cycle(10.0f, 0, Z_OFFSET);
    }
    /* from here a 0 km/h sample every 100 ms, as a slower poll would deliver */
    uint32_t second_zero_ms = 0u;
    bool learned = false;
    for (uint32_t k = 0u; (k < 150u) && !learned; k++) {
        if ((k % 5u) == 0u) {
            comms_speed(0.0f, now);
            second_zero_ms = (k == 5u) ? now : second_zero_ms;
        }
        push_imu(0, Z_OFFSET);
        lean_ekf_step(&ekf, now);
        learned = ekf.imu.core.offset_learned;
        if (!learned) {
            now += CYCLE_MS;
        }
    }
    TEST_ASSERT_TRUE(learned);
    /* the first zero alone never starts the window (one glitched 0 is not enough): it
     * opens with the IMU period that ends at the second zero, one cycle before it */
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(IMU_OFFSET_WINDOW_MS - CYCLE_MS, now - second_zero_ms);
}

static void test_a_refused_dedicated_buffer_leaves_0x020_closed(void)
{
    /* setUp already registered 0x020: a second registration is refused (CAN_IF_ERR_DUP) */
    lean_tx_t t;
    TEST_ASSERT_FALSE(lean_tx_open(&t, &link, lean_link_alive(&link)));
    TEST_ASSERT_FALSE(t.opened);
    TEST_ASSERT_NULL(t.link);
    lean_tx_step(&t, now);
    TEST_ASSERT_EQUAL_UINT32(0u, t.stats.sent + t.stats.retried + t.stats.dropped);
}

static void test_null_and_unbound_steps_do_nothing(void)
{
    lean_ekf_t unbound;
    lean_ekf_init(&unbound, NULL);
    lean_ekf_step(&unbound, now);
    lean_ekf_step(NULL, now);
    lean_ekf_init(NULL, &link);
    lean_link_init(NULL);
    TEST_ASSERT_EQUAL_UINT32(0u, alive_count(lean_link_alive(&link)));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_boot_slots_are_invalid);
    RUN_TEST(test_without_a_learnt_offset_the_lean_is_invalid);
    RUN_TEST(test_after_a_stop_turns_both_ways_read_true_with_the_offset_removed);
    RUN_TEST(test_alive_bumps_once_per_complete_cycle_and_never_on_no_new);
    RUN_TEST(test_missed_imu_periods_reach_the_core_as_a_gap);
    RUN_TEST(test_a_stalled_comms_task_makes_the_lean_invalid_within_300_ms);
    RUN_TEST(test_only_a_valid_speed_inside_the_gen_range_is_handed_over);
    RUN_TEST(test_standstill_needs_two_recent_zero_samples);
    RUN_TEST(test_snapshot_faults_reach_the_record_and_read_invalid);
    RUN_TEST(test_0x020_opens_only_for_this_links_counter_MAJOR_3);
    RUN_TEST(test_an_unusable_speed_breaks_the_standstill_window_at_once);
    RUN_TEST(test_the_standstill_window_starts_at_the_second_zero_sample);
    RUN_TEST(test_a_refused_dedicated_buffer_leaves_0x020_closed);
    RUN_TEST(test_null_and_unbound_steps_do_nothing);
    return UNITY_END();
}
