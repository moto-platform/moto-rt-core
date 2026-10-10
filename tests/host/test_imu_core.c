/*
 * L0 tests for the pure IMU core (services/imu_core, D-065 item 2, PR 3a): interval mean,
 * the INVALID checks and their order, and the standstill z offset learning (user,
 * 2026-10-10: 2 s, three-axis stillness 1 deg/s, offset at most 2 deg/s, fail-closed
 * until learnt).
 */
#include "services/imu_core.h"

#include <math.h>
#include <stdint.h>
#include <unity.h>

#define SCALE (0.001f) /* rad/s per LSB: raw 26 = 0.026 rad/s */
#define ODR (800u)
#define PERIOD_MS (20u)
#define N_PERIOD (16u) /* samples per 20 ms at 800 Hz */
#define DEG (0.017453292f)

static imu_core_t core;
static imu_config_t cfg;
static uint32_t now;

void setUp(void)
{
    imu_core_init(&core);
    cfg.gyro_rps_per_lsb = SCALE;
    cfg.odr_hz = ODR;
    now = 1000u;
}

void tearDown(void) {}

/* Sensor noise of -1/+1 LSB on alternate samples (none at full scale): the mean of an
 * even count is unchanged, and the period never reads as frozen. */
static int16_t noisy(int16_t v, uint16_t i)
{
    if ((v == INT16_MAX) || (v == INT16_MIN)) {
        return v;
    }
    return (int16_t)(v + (((i % 2u) == 0u) ? -1 : 1));
}

/* One period of n raw samples around [x, y, z], ending PERIOD_MS after the previous one;
 * noise on (noisy) or bit-identical samples (exact). */
static imu_sample_t period_n(int16_t x, int16_t y, int16_t z, uint16_t n, bool standstill, bool noise)
{
    imu_acc_t a;
    imu_acc_reset(&a);
    for (uint16_t i = 0u; i < n; i++) {
        const int16_t g[3] = {noise ? noisy(x, i) : x, noise ? noisy(y, i) : y, noise ? noisy(z, i) : z};
        imu_acc_add(&a, g);
    }
    now += PERIOD_MS;
    return imu_core_finish(&core, &a, &cfg, now, standstill);
}

static imu_sample_t period(int16_t x, int16_t y, int16_t z, uint16_t n, bool standstill)
{
    return period_n(x, y, z, n, standstill, true);
}

static imu_sample_t period_exact(int16_t x, int16_t y, int16_t z, uint16_t n)
{
    return period_n(x, y, z, n, false, false);
}

/* Starts the core (the first period reads START) and stands still with raw z offset
 * zr until the offset is learnt (one 2 s window). */
static void learn(int16_t zr)
{
    TEST_ASSERT_EQUAL(IMU_REASON_START, period(0, 0, zr, N_PERIOD, true).reason);
    for (uint32_t i = 0u; i < (IMU_OFFSET_WINDOW_MS / PERIOD_MS); i++) {
        (void)period(0, 0, zr, N_PERIOD, true);
    }
    TEST_ASSERT_TRUE(core.offset_learned);
}

static void test_the_zero_state_and_the_first_period_are_invalid(void)
{
    const imu_core_t zero = {0};
    core = zero;
    const imu_sample_t s = period(10, 10, 10, N_PERIOD, false);
    TEST_ASSERT_EQUAL(IMU_SAMPLE_INVALID, s.state);
    TEST_ASSERT_EQUAL(IMU_REASON_START, s.reason);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, s.gyro_x_rps);
    TEST_ASSERT_EQUAL_UINT32(now, s.stamp_ms);
    TEST_ASSERT_EQUAL_UINT16(N_PERIOD, s.count);
}

static void test_null_arguments_read_arg_and_change_nothing(void)
{
    imu_acc_t a;
    imu_acc_reset(&a);
    learn(0);
    const imu_core_t before = core;
    TEST_ASSERT_EQUAL(IMU_REASON_ARG, imu_core_finish(NULL, &a, &cfg, 5000u, true).reason);
    TEST_ASSERT_EQUAL(IMU_REASON_ARG, imu_core_finish(&core, NULL, &cfg, 5000u, true).reason);
    TEST_ASSERT_EQUAL(IMU_REASON_ARG, imu_core_finish(&core, &a, NULL, 5000u, true).reason);
    TEST_ASSERT_EQUAL_MEMORY(&before, &core, sizeof core);
    imu_core_init(NULL);
    imu_acc_reset(NULL);
    imu_acc_add(NULL, (const int16_t[3]){0, 0, 0});
    imu_acc_add(&a, NULL);
    TEST_ASSERT_EQUAL_UINT16(0u, a.count);
    TEST_ASSERT_FALSE(imu_core_new_period(NULL, 0u));
}

static void test_the_rates_are_the_mean_of_all_samples_in_the_period(void)
{
    learn(0);
    imu_acc_t a;
    imu_acc_reset(&a);
    for (int16_t i = 1; i <= (int16_t)N_PERIOD; i++) {
        const int16_t g[3] = {i, (int16_t)(-2 * i), (int16_t)(100 + i)};
        imu_acc_add(&a, g);
    }
    now += PERIOD_MS;
    const imu_sample_t s = imu_core_finish(&core, &a, &cfg, now, false);
    TEST_ASSERT_EQUAL(IMU_SAMPLE_VALID, s.state);
    TEST_ASSERT_EQUAL(IMU_REASON_NONE, s.reason);
    TEST_ASSERT_EQUAL_FLOAT(8.5f * SCALE, s.gyro_x_rps);   /* mean of 1..16, not 16 */
    TEST_ASSERT_EQUAL_FLOAT(-17.0f * SCALE, s.gyro_y_rps);
    TEST_ASSERT_EQUAL_FLOAT(108.5f * SCALE, s.gyro_z_rps);
    TEST_ASSERT_EQUAL_UINT16(N_PERIOD, s.count);
    TEST_ASSERT_EQUAL_UINT32(now, s.stamp_ms);
}

/* Feeds a tone of amplitude amp (rad/s) at f_hz on x and z for 10 periods, starting at
 * a phase that is not a multiple of the period; returns the largest |mean| and, through
 * last, the largest |last sample| (what a point sample would have read). */
static float tone(float f_hz, float amp, float* last)
{
    float worst = 0.0f;
    *last = 0.0f;
    uint32_t k = 3u;
    for (uint32_t p = 0u; p < 10u; p++) {
        imu_acc_t a;
        imu_acc_reset(&a);
        float v = 0.0f;
        for (uint32_t i = 0u; i < N_PERIOD; i++) {
            v = amp * sinf(6.2831853f * f_hz * (float)k / (float)ODR);
            const int16_t r = (int16_t)lroundf(v / SCALE);
            const int16_t g[3] = {r, 0, r};
            imu_acc_add(&a, g);
            k++;
        }
        now += PERIOD_MS;
        const imu_sample_t s = imu_core_finish(&core, &a, &cfg, now, false);
        TEST_ASSERT_EQUAL(IMU_SAMPLE_VALID, s.state);
        worst = fmaxf(worst, fmaxf(fabsf(s.gyro_x_rps), fabsf(s.gyro_z_rps)));
        *last = fmaxf(*last, fabsf(v));
    }
    return worst;
}

static void test_vibration_does_not_alias_into_the_sample_MAJOR_4(void)
{
    learn(0);
    const float amp = 20.0f * DEG; /* 20 deg/s of firing vibration */
    float last = 0.0f;
    /* 50 Hz (the EKF rate) and 100 Hz: the boxcar's zeros. A point sample reads up to
     * the full amplitude, constant from period to period: an alias at 0 Hz. */
    TEST_ASSERT_TRUE(tone(50.0f, amp, &last) < 1.0e-3f);
    TEST_ASSERT_TRUE(last > (0.5f * amp));
    TEST_ASSERT_TRUE(tone(100.0f, amp, &last) < 1.0e-3f);
    /* 75 Hz and 25 Hz: |sinc| 0.21 and 0.64 (plus the IMU's own low-pass on target) */
    TEST_ASSERT_TRUE(tone(75.0f, amp, &last) <= (0.25f * amp));
    TEST_ASSERT_TRUE(tone(25.0f, amp, &last) <= (0.67f * amp));
}

static void test_a_bad_configuration_reads_config(void)
{
    learn(0);
    const float bad_scale[] = {0.0f, -SCALE, NAN, INFINITY};
    for (uint32_t i = 0u; i < (sizeof bad_scale / sizeof bad_scale[0]); i++) {
        cfg.gyro_rps_per_lsb = bad_scale[i];
        TEST_ASSERT_EQUAL(IMU_REASON_CONFIG, period(1, 1, 1, N_PERIOD, false).reason);
    }
    cfg.gyro_rps_per_lsb = SCALE;
    cfg.odr_hz = IMU_ODR_MIN_HZ - 1u;
    TEST_ASSERT_EQUAL(IMU_REASON_CONFIG, period(1, 1, 1, N_PERIOD, false).reason);
    cfg.odr_hz = IMU_ODR_MAX_HZ + 1u;
    TEST_ASSERT_EQUAL(IMU_REASON_CONFIG, period(1, 1, 1, 64u, false).reason);
    cfg.odr_hz = IMU_ODR_MAX_HZ;
    TEST_ASSERT_EQUAL(IMU_REASON_NONE, period(1, 1, 1, 32u, false).reason);
    cfg.odr_hz = IMU_ODR_MIN_HZ;
    TEST_ASSERT_EQUAL(IMU_REASON_NONE, period(1, 1, 1, N_PERIOD, false).reason);
    /* the scale must match a 450..2600 deg/s full scale (MINOR-2 on PR 3a, MINOR-A) */
    const float lsb_per_fs = 32768.0f;
    cfg.gyro_rps_per_lsb = (449.0f * DEG) / lsb_per_fs;
    TEST_ASSERT_EQUAL(IMU_REASON_CONFIG, period(1, 1, 1, N_PERIOD, false).reason);
    cfg.gyro_rps_per_lsb = (2601.0f * DEG) / lsb_per_fs;
    TEST_ASSERT_EQUAL(IMU_REASON_CONFIG, period(1, 1, 1, N_PERIOD, false).reason);
    cfg.gyro_rps_per_lsb = IMU_SCALE_MIN_RPS_PER_LSB;
    TEST_ASSERT_EQUAL(IMU_REASON_NONE, period(1, 1, 1, N_PERIOD, false).reason);
    cfg.gyro_rps_per_lsb = IMU_SCALE_MAX_RPS_PER_LSB;
    TEST_ASSERT_EQUAL(IMU_REASON_NONE, period(1, 1, 1, N_PERIOD, false).reason);
    cfg.gyro_rps_per_lsb = 3.0e38f; /* finite but absurd: the means stay finite by the bound */
    const imu_sample_t s = period(1000, 0, 0, N_PERIOD, false);
    TEST_ASSERT_EQUAL(IMU_REASON_CONFIG, s.reason);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, s.gyro_x_rps);
}

static void test_the_candidate_parts_datasheet_scales_are_accepted_MINOR_A(void)
{
    learn(0);
    /* LSM6DSO 500/1000/2000 deg/s: 17.5/35/70 mdps per LSB; BMI270 500/1000/2000 deg/s:
     * 65.6/32.8/16.4 LSB per deg/s (499.5 deg/s full scale at the lowest) */
    const float scales[] = {17.5e-3f * DEG, 35.0e-3f * DEG, 70.0e-3f * DEG,
                            DEG / 65.6f,    DEG / 32.8f,    DEG / 16.4f};
    for (uint32_t i = 0u; i < (sizeof scales / sizeof scales[0]); i++) {
        cfg.gyro_rps_per_lsb = scales[i];
        TEST_ASSERT_EQUAL(IMU_REASON_NONE, period(1, 1, 1, N_PERIOD, false).reason);
    }
}

static void test_port_faults_read_invalid_in_check_order(void)
{
    learn(0);
    imu_acc_t a;
    imu_acc_reset(&a);
    const int16_t g[3] = {1, 1, 1};
    for (uint32_t i = 0u; i < N_PERIOD; i++) {
        imu_acc_add(&a, g);
    }
    a.backlog = true;
    now += PERIOD_MS;
    TEST_ASSERT_EQUAL(IMU_REASON_BACKLOG, imu_core_finish(&core, &a, &cfg, now, false).reason);
    a.overrun = true;
    now += PERIOD_MS;
    TEST_ASSERT_EQUAL(IMU_REASON_OVERRUN, imu_core_finish(&core, &a, &cfg, now, false).reason);
    a.hal_error = true;
    now += PERIOD_MS;
    TEST_ASSERT_EQUAL(IMU_REASON_HAL, imu_core_finish(&core, &a, &cfg, now, false).reason);
    cfg.odr_hz = 0u; /* a port fault leaves the config unread: HAL wins */
    now += PERIOD_MS;
    TEST_ASSERT_EQUAL(IMU_REASON_HAL, imu_core_finish(&core, &a, &cfg, now, false).reason);
    a.hal_error = false;
    now += PERIOD_MS;
    TEST_ASSERT_EQUAL(IMU_REASON_CONFIG, imu_core_finish(&core, &a, &cfg, now, false).reason);
    /* every one ended its period: a clean period after them is VALID, not STALE */
    cfg.odr_hz = ODR;
    TEST_ASSERT_EQUAL(IMU_REASON_NONE, period(1, 1, 1, N_PERIOD, false).reason);
}

static void test_too_few_samples_for_the_elapsed_time_are_stale(void)
{
    learn(0);
    TEST_ASSERT_EQUAL(IMU_REASON_STALE, period(1, 1, 1, 0u, false).reason);
    /* 20 ms at 800 Hz: at least 8 of the 16 */
    TEST_ASSERT_EQUAL(IMU_REASON_STALE, period(1, 1, 1, 7u, false).reason);
    TEST_ASSERT_EQUAL(IMU_REASON_NONE, period(1, 1, 1, 8u, false).reason);
    /* a long gap: never enough samples, and elapsed * ODR does not overflow */
    imu_acc_t a;
    imu_acc_reset(&a);
    for (uint32_t i = 0u; i < 70u; i++) {
        imu_acc_add(&a, (const int16_t[3]){1, 1, 1});
    }
    TEST_ASSERT_EQUAL_UINT16(IMU_DRAIN_MAX, a.count); /* bounded */
    now += 6000000u;
    TEST_ASSERT_EQUAL(IMU_REASON_STALE, imu_core_finish(&core, &a, &cfg, now, false).reason);
    /* a backward stamp reads as a huge gap */
    now -= 2u * PERIOD_MS;
    TEST_ASSERT_EQUAL(IMU_REASON_STALE, period(1, 1, 1, N_PERIOD, false).reason);
}

static void test_more_samples_than_the_elapsed_time_allows_are_excess_MAJOR_1(void)
{
    learn(0);
    /* 20 ms at 800 Hz: 16 expected, at most 24 + 2 ms of latency (2) + 2 */
    TEST_ASSERT_EQUAL(IMU_REASON_NONE, period(1, 1, 1, 28u, false).reason);
    TEST_ASSERT_EQUAL(IMU_REASON_EXCESS, period(1, 1, 1, 29u, false).reason);
    TEST_ASSERT_EQUAL(IMU_REASON_NONE, period(1, 1, 1, N_PERIOD, false).reason);
}

static void test_a_short_catch_up_period_within_the_latency_is_not_excess_MINOR_B(void)
{
    learn(0);
    cfg.odr_hz = IMU_ODR_MAX_HZ;
    TEST_ASSERT_EQUAL(IMU_REASON_NONE, period(1, 1, 1, 32u, false).reason);
    /* catch-up periods of 1..5 ms holding their own samples plus 2 ms carried over by
     * the port's latency, plus 1 ms of stamp resolution */
    for (uint32_t e = 1u; e <= 5u; e++) {
        const uint16_t n = (uint16_t)(((e + 3u) * IMU_ODR_MAX_HZ) / 1000u);
        imu_acc_t a;
        imu_acc_reset(&a);
        for (uint16_t i = 0u; i < n; i++) {
            imu_acc_add(&a, (const int16_t[3]){noisy(1, i), 1, 1});
        }
        now += e;
        TEST_ASSERT_EQUAL(IMU_REASON_NONE, imu_core_finish(&core, &a, &cfg, now, false).reason);
    }
}

static void test_watermark_batching_never_reads_valid_MAJOR_1(void)
{
    learn(0);
    /* a port that delivers 32 samples every 40 ms: a period without and one with twice
     * the samples, never a VALID mean shifted against its stamp */
    for (uint32_t i = 0u; i < 50u; i++) {
        TEST_ASSERT_EQUAL(IMU_REASON_STALE, period(1, 1, 1, 0u, false).reason);
        TEST_ASSERT_EQUAL(IMU_REASON_EXCESS, period(1, 1, 1, 32u, false).reason);
    }
}

static void test_a_frozen_gyro_reads_invalid_MINOR_1(void)
{
    learn(0);
    /* riding: the 3rd identical period (MINOR-D on the fixes) */
    for (uint32_t i = 1u; i < IMU_FROZEN_PERIODS_RIDING; i++) {
        TEST_ASSERT_EQUAL(IMU_REASON_NONE, period_exact(3, 3, 0, N_PERIOD).reason);
    }
    /* a one-sample period (1 ms) is no evidence: the run neither grows nor ends */
    imu_acc_t one;
    imu_acc_reset(&one);
    imu_acc_add(&one, (const int16_t[3]){3, 3, 0});
    now += 1u;
    TEST_ASSERT_EQUAL(IMU_REASON_NONE, imu_core_finish(&core, &one, &cfg, now, false).reason);
    for (uint32_t i = 0u; i < 100u; i++) { /* from the 3rd period on, and it stays */
        TEST_ASSERT_EQUAL(IMU_REASON_FROZEN, period_exact(3, 3, 0, N_PERIOD).reason);
    }
    /* one axis changing is enough to count as alive again: y, then z */
    imu_acc_t a;
    imu_acc_reset(&a);
    for (uint32_t i = 0u; i < N_PERIOD; i++) {
        imu_acc_add(&a, (const int16_t[3]){3, (i == 0u) ? 4 : 3, 0});
    }
    now += PERIOD_MS;
    TEST_ASSERT_EQUAL(IMU_REASON_NONE, imu_core_finish(&core, &a, &cfg, now, false).reason);
    for (uint32_t i = 1u; i < IMU_FROZEN_PERIODS_RIDING; i++) {
        (void)period_exact(3, 3, 0, N_PERIOD);
    }
    TEST_ASSERT_EQUAL(IMU_REASON_FROZEN, period_exact(3, 3, 0, N_PERIOD).reason);
    imu_acc_reset(&a);
    for (uint32_t i = 0u; i < N_PERIOD; i++) {
        imu_acc_add(&a, (const int16_t[3]){3, 3, (i == 0u) ? 1 : 0});
    }
    now += PERIOD_MS;
    TEST_ASSERT_EQUAL(IMU_REASON_NONE, imu_core_finish(&core, &a, &cfg, now, false).reason);
    /* at standstill a quiet part may repeat: 25 periods; each closes nothing until then */
    imu_acc_reset(&a);
    for (uint32_t i = 0u; i < N_PERIOD; i++) {
        imu_acc_add(&a, (const int16_t[3]){0, 0, (i == 0u) ? 1 : 0});
    }
    now += PERIOD_MS;
    TEST_ASSERT_EQUAL(IMU_REASON_NONE, imu_core_finish(&core, &a, &cfg, now, true).reason);
    for (uint32_t i = 1u; i < IMU_FROZEN_PERIODS_STILL; i++) {
        TEST_ASSERT_EQUAL(IMU_REASON_NONE, period_n(0, 0, 0, N_PERIOD, true, false).reason);
    }
    TEST_ASSERT_EQUAL(IMU_REASON_FROZEN, period_n(0, 0, 0, N_PERIOD, true, false).reason);
    /* a frozen gyro at standstill is never learnt as offset: the window closes */
    setUp();
    TEST_ASSERT_EQUAL(IMU_REASON_START, period_n(0, 0, 20, N_PERIOD, true, false).reason);
    for (uint32_t i = 0u; i < 200u; i++) {
        (void)period_n(0, 0, 20, N_PERIOD, true, false);
    }
    TEST_ASSERT_FALSE(core.offset_learned);
}

static void test_a_sample_at_full_scale_is_saturated(void)
{
    learn(0);
    TEST_ASSERT_EQUAL(IMU_REASON_SATURATED, period(0, INT16_MAX, 0, N_PERIOD, false).reason);
    TEST_ASSERT_EQUAL(IMU_REASON_SATURATED, period(0, 0, INT16_MIN, N_PERIOD, false).reason);
    TEST_ASSERT_EQUAL(IMU_REASON_SATURATED, period(INT16_MAX, 0, 0, N_PERIOD, false).reason);
    const imu_sample_t s = period_exact(INT16_MAX, 0, 0, 4u);
    TEST_ASSERT_EQUAL(IMU_REASON_STALE, s.reason); /* stale before saturation */
    TEST_ASSERT_EQUAL(IMU_REASON_NONE,
                      period_exact(INT16_MAX - 1, (int16_t)(INT16_MIN + 1), 0, N_PERIOD).reason);
    /* one saturated sample among many still taints the period */
    imu_acc_t a;
    imu_acc_reset(&a);
    for (uint32_t i = 0u; i < N_PERIOD; i++) {
        imu_acc_add(&a, (const int16_t[3]){0, 0, (i == 5u) ? INT16_MAX : 0});
    }
    now += PERIOD_MS;
    TEST_ASSERT_EQUAL(IMU_REASON_SATURATED, imu_core_finish(&core, &a, &cfg, now, false).reason);
}

static void test_a_call_in_the_same_ms_is_no_new_and_keeps_the_period(void)
{
    learn(0);
    TEST_ASSERT_TRUE(imu_core_new_period(&core, now + 1u));
    TEST_ASSERT_FALSE(imu_core_new_period(&core, now));
    imu_acc_t a;
    imu_acc_reset(&a);
    const imu_sample_t s = imu_core_finish(&core, &a, &cfg, now, true);
    TEST_ASSERT_EQUAL(IMU_SAMPLE_INVALID, s.state);
    TEST_ASSERT_EQUAL(IMU_REASON_NO_NEW, s.reason);
    TEST_ASSERT_EQUAL(IMU_REASON_NONE, period(1, 1, 1, N_PERIOD, false).reason);
}

static void test_no_sample_is_valid_before_the_z_offset_is_learnt(void)
{
    TEST_ASSERT_EQUAL(IMU_REASON_START, period(0, 0, 26, N_PERIOD, false).reason);
    for (uint32_t i = 0u; i < 500u; i++) { /* 10 s of riding: never learnt */
        const imu_sample_t s = period(5, 5, 26, N_PERIOD, false);
        TEST_ASSERT_EQUAL(IMU_SAMPLE_INVALID, s.state);
        TEST_ASSERT_EQUAL(IMU_REASON_OFFSET_UNLEARNED, s.reason);
        TEST_ASSERT_EQUAL_FLOAT(0.0f, s.gyro_x_rps);
        TEST_ASSERT_EQUAL_FLOAT(0.0f, s.gyro_z_rps);
    }
}

static void test_the_offset_is_learnt_after_2_s_of_standstill(void)
{
    TEST_ASSERT_EQUAL(IMU_REASON_START, period(0, 0, 26, N_PERIOD, true).reason);
    /* the window starts with the first standstill period (at the START stamp) */
    for (uint32_t i = 1u; i < (IMU_OFFSET_WINDOW_MS / PERIOD_MS); i++) {
        TEST_ASSERT_EQUAL(IMU_REASON_OFFSET_UNLEARNED, period(0, 0, 26, N_PERIOD, true).reason);
    }
    const imu_sample_t s = period(0, 0, 26, N_PERIOD, true); /* 2000 ms */
    TEST_ASSERT_EQUAL(IMU_SAMPLE_VALID, s.state);
    TEST_ASSERT_EQUAL_FLOAT(0.026f, core.offset_z_rps);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-6f, 0.0f, s.gyro_z_rps);
    TEST_ASSERT_EQUAL_UINT32(1u, core.learn_count);
}

static void test_the_learnt_offset_is_removed_in_a_turn(void)
{
    /* the user's test: a 1.5 deg/s z offset (raw 26) at standstill, then a right turn at
     * 10 deg/s of body z rate and a left one: both read the true rate, not one low */
    learn(26);
    const int16_t turn = 175; /* 0.175 rad/s, about 10 deg/s */
    const imu_sample_t r = period(0, 0, (int16_t)(turn + 26), N_PERIOD, false);
    const imu_sample_t l = period(0, 0, (int16_t)(-turn + 26), N_PERIOD, false);
    TEST_ASSERT_EQUAL(IMU_SAMPLE_VALID, r.state);
    TEST_ASSERT_EQUAL(IMU_SAMPLE_VALID, l.state);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-6f, 0.175f, r.gyro_z_rps);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-6f, -0.175f, l.gyro_z_rps);
    /* x and y keep their raw means: the EKF estimates the roll-rate bias itself */
    TEST_ASSERT_EQUAL_FLOAT(0.009f, period(9, 7, 26, N_PERIOD, false).gyro_x_rps);
}

static void test_motion_restarts_the_window_on_any_axis(void)
{
    static const int16_t moved[3][3] = {{18, 0, 0}, {0, 18, 0}, {0, 0, 18}};
    for (uint32_t axis = 0u; axis < 3u; axis++) {
        setUp();
        TEST_ASSERT_EQUAL(IMU_REASON_START, period(0, 0, 0, N_PERIOD, true).reason);
        for (uint32_t i = 1u; i < 50u; i++) { /* 1 s still */
            (void)period(0, 0, 0, N_PERIOD, true);
        }
        /* 1.03 deg/s on one axis: beyond the 1 deg/s spread, the window restarts here */
        (void)period(moved[axis][0], moved[axis][1], moved[axis][2], N_PERIOD, true);
        for (uint32_t i = 0u; i < 98u; i++) {
            (void)period(moved[axis][0], moved[axis][1], moved[axis][2], N_PERIOD, true);
        }
        TEST_ASSERT_FALSE(core.offset_learned); /* 1.98 s after the restart */
        (void)period(moved[axis][0], moved[axis][1], moved[axis][2], N_PERIOD, true);
        TEST_ASSERT_TRUE(core.offset_learned);
        TEST_ASSERT_EQUAL_FLOAT((axis == 2u) ? 0.018f : 0.0f, core.offset_z_rps);
    }
}

static void test_a_spread_within_1_deg_s_keeps_the_window(void)
{
    TEST_ASSERT_EQUAL(IMU_REASON_START, period(0, 0, 0, N_PERIOD, true).reason);
    for (uint32_t i = 1u; i <= 100u; i++) {
        const int16_t v = ((i % 2u) == 0u) ? 0 : 17; /* 0.97 deg/s on every axis, down first */
        (void)period(v, v, v, N_PERIOD, true);
    }
    TEST_ASSERT_TRUE(core.offset_learned);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-6f, 0.0085f, core.offset_z_rps);
}

static void test_leaving_standstill_or_an_invalid_period_restarts_the_window(void)
{
    TEST_ASSERT_EQUAL(IMU_REASON_START, period(0, 0, 10, N_PERIOD, true).reason);
    for (uint32_t i = 1u; i < 99u; i++) {
        (void)period(0, 0, 10, N_PERIOD, true);
    }
    (void)period(0, 0, 10, N_PERIOD, false); /* moved off: window closed */
    for (uint32_t i = 0u; i < 99u; i++) {
        (void)period(0, 0, 10, N_PERIOD, true);
    }
    TEST_ASSERT_FALSE(core.offset_learned);
    (void)period(0, 0, 10, 2u, true); /* a stale period: window closed */
    for (uint32_t i = 0u; i < 99u; i++) {
        (void)period(0, 0, 10, N_PERIOD, true);
    }
    TEST_ASSERT_FALSE(core.offset_learned);
    (void)period(0, 0, 10, N_PERIOD, true);
    TEST_ASSERT_TRUE(core.offset_learned);
}

static void test_an_offset_beyond_2_deg_s_is_rejected_fail_closed(void)
{
    /* +-0.034 rad/s (1.95 deg/s) is accepted, +-0.035 (2.005 deg/s) is not */
    learn(34);
    TEST_ASSERT_EQUAL_FLOAT(0.034f, core.offset_z_rps);
    setUp();
    learn(-34);
    /* a later window beyond the limit drops the old offset: INVALID, reason RANGE */
    for (uint32_t i = 0u; i < 100u; i++) {
        (void)period(0, 0, -35, N_PERIOD, true);
    }
    TEST_ASSERT_FALSE(core.offset_learned);
    TEST_ASSERT_EQUAL_UINT32(1u, core.reject_count);
    const imu_sample_t s = period(0, 0, -35, N_PERIOD, false);
    TEST_ASSERT_EQUAL(IMU_SAMPLE_INVALID, s.state);
    TEST_ASSERT_EQUAL(IMU_REASON_OFFSET_RANGE, s.reason);
    /* a good window afterwards makes the samples VALID again */
    for (uint32_t i = 0u; i < 100u; i++) {
        (void)period(0, 0, 20, N_PERIOD, true);
    }
    TEST_ASSERT_TRUE(core.offset_learned);
    TEST_ASSERT_EQUAL(IMU_REASON_NONE, period(0, 0, 20, N_PERIOD, false).reason);
    setUp();
    TEST_ASSERT_EQUAL(IMU_REASON_START, period(0, 0, 35, N_PERIOD, true).reason);
    for (uint32_t i = 0u; i < 100u; i++) {
        (void)period(0, 0, 35, N_PERIOD, true);
    }
    TEST_ASSERT_FALSE(core.offset_learned);
    TEST_ASSERT_EQUAL(IMU_REASON_OFFSET_RANGE, period(0, 0, 35, N_PERIOD, false).reason);
}

static void test_every_stop_relearns_the_offset(void)
{
    learn(10);
    for (uint32_t i = 0u; i < 500u; i++) { /* ride; the offset drifts with temperature */
        (void)period(0, 0, 14, N_PERIOD, false);
    }
    TEST_ASSERT_EQUAL_FLOAT(0.010f, core.offset_z_rps);
    for (uint32_t i = 0u; i < 100u; i++) {
        (void)period(0, 0, 14, N_PERIOD, true);
    }
    TEST_ASSERT_EQUAL_FLOAT(0.014f, core.offset_z_rps);
    TEST_ASSERT_EQUAL_UINT32(2u, core.learn_count);
    /* a long stop keeps relearning, one window after another */
    for (uint32_t i = 0u; i < 100u; i++) {
        (void)period(0, 0, 15, N_PERIOD, true);
    }
    TEST_ASSERT_EQUAL_FLOAT(0.015f, core.offset_z_rps);
}

static void test_the_timebase_wrap_is_harmless(void)
{
    now = 0xFFFFFF00u;
    learn(5);
    TEST_ASSERT_TRUE(now < 0x1000u); /* the learning window spanned the wrap */
    TEST_ASSERT_EQUAL(IMU_REASON_NONE, period(1, 1, 5, N_PERIOD, false).reason);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_the_zero_state_and_the_first_period_are_invalid);
    RUN_TEST(test_null_arguments_read_arg_and_change_nothing);
    RUN_TEST(test_the_rates_are_the_mean_of_all_samples_in_the_period);
    RUN_TEST(test_vibration_does_not_alias_into_the_sample_MAJOR_4);
    RUN_TEST(test_a_bad_configuration_reads_config);
    RUN_TEST(test_port_faults_read_invalid_in_check_order);
    RUN_TEST(test_too_few_samples_for_the_elapsed_time_are_stale);
    RUN_TEST(test_more_samples_than_the_elapsed_time_allows_are_excess_MAJOR_1);
    RUN_TEST(test_a_short_catch_up_period_within_the_latency_is_not_excess_MINOR_B);
    RUN_TEST(test_watermark_batching_never_reads_valid_MAJOR_1);
    RUN_TEST(test_the_candidate_parts_datasheet_scales_are_accepted_MINOR_A);
    RUN_TEST(test_a_frozen_gyro_reads_invalid_MINOR_1);
    RUN_TEST(test_a_sample_at_full_scale_is_saturated);
    RUN_TEST(test_a_call_in_the_same_ms_is_no_new_and_keeps_the_period);
    RUN_TEST(test_no_sample_is_valid_before_the_z_offset_is_learnt);
    RUN_TEST(test_the_offset_is_learnt_after_2_s_of_standstill);
    RUN_TEST(test_the_learnt_offset_is_removed_in_a_turn);
    RUN_TEST(test_motion_restarts_the_window_on_any_axis);
    RUN_TEST(test_a_spread_within_1_deg_s_keeps_the_window);
    RUN_TEST(test_leaving_standstill_or_an_invalid_period_restarts_the_window);
    RUN_TEST(test_an_offset_beyond_2_deg_s_is_rejected_fail_closed);
    RUN_TEST(test_every_stop_relearns_the_offset);
    RUN_TEST(test_the_timebase_wrap_is_harmless);
    return UNITY_END();
}
