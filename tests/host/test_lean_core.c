/*
 * L0 tests for the pure lean core (D-065 item 1, PR 2): the 2-state roll EKF, its speed
 * rules (age, future stamp, minimum), the innovation gate, the INVALID rules and the
 * QUALITY map. A small simulator feeds a steady coordinated turn: gyro_z = g sin(roll)/v
 * (body z down, pitch 0), gyro_x = d(roll)/dt + bias.
 */
#include "features/cornering/lean_core.h"

#include "platform_limits.h"

#include <math.h>
#include <stdint.h>
#include <string.h>
#include <unity.h>

#define DT_MS 20u /* nominal EKF period (D-064 item 4) */
#define V_RIDE 15.0f
#define RAD_PER_DEG (0.017453292f)
#define DEG(x) ((x) * RAD_PER_DEG)

static lean_core_t c;
static uint32_t now;

void setUp(void)
{
    lean_core_init(&c);
    now = 1000u;
}

void tearDown(void) {}

static lean_imu_t imu_at(float gx, float gz, uint32_t t)
{
    const lean_imu_t s = {gx, gz, t};
    return s;
}

static lean_speed_t speed_at(float v, uint32_t stamp, lean_speed_state_t st)
{
    const lean_speed_t s = {v, stamp, st};
    return s;
}

static float yaw_for(float roll_rad, float v)
{
    return LEAN_GRAVITY_MPS2 * sinf(roll_rad) / v;
}

/* One step at `now` with a fresh VALID speed, then now += DT_MS. */
static lean_out_t step_turn(float roll_rad, float roll_rate, float bias, float v)
{
    const lean_imu_t s = imu_at(roll_rate + bias, yaw_for(roll_rad, v), now);
    const lean_speed_t sp = speed_at(v, now, LEAN_SPEED_VALID);
    const lean_out_t out = lean_core_step(&c, &s, &sp);
    now += DT_MS;
    return out;
}

/* n steps of a steady turn; returns the last output. */
static lean_out_t ride(float roll_rad, float bias, float v, uint32_t n)
{
    lean_out_t out = {0};
    for (uint32_t i = 0u; i < n; i++) {
        out = step_turn(roll_rad, 0.0f, bias, v);
    }
    return out;
}

static void assert_invalid(lean_out_t out, lean_reason_t reason)
{
    TEST_ASSERT_EQUAL_INT(LEAN_OUT_INVALID, out.state);
    TEST_ASSERT_EQUAL_INT(reason, out.reason);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out.roll_deg);
    TEST_ASSERT_EQUAL_UINT8(0u, out.quality);
}

static void assert_estimated(lean_out_t out)
{
    TEST_ASSERT_EQUAL_INT(LEAN_OUT_ESTIMATED, out.state);
    TEST_ASSERT_EQUAL_INT(LEAN_REASON_NONE, out.reason);
    TEST_ASSERT_TRUE(out.quality >= LEAN_QUALITY_MIN);
    TEST_ASSERT_TRUE(out.quality <= LEAN_QUALITY_MAX);
}

static void assert_reinit(void)
{
    TEST_ASSERT_EQUAL_FLOAT(0.0f, c.roll);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, c.bias);
    TEST_ASSERT_EQUAL_FLOAT(LEAN_P0_ROLL, c.p00);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, c.p01);
    TEST_ASSERT_EQUAL_FLOAT(LEAN_P0_BIAS, c.p11);
    TEST_ASSERT_FALSE(c.rejecting);
    TEST_ASSERT_TRUE(c.ready);
}

/* Converged straight riding at V_RIDE, quality 100. */
static void converge(void)
{
    const lean_out_t out = ride(0.0f, 0.0f, V_RIDE, 500u);
    assert_estimated(out);
    TEST_ASSERT_EQUAL_UINT8(100u, out.quality);
}

/* ---- start-up ---- */

static void test_first_step_is_invalid_dt_and_a_fresh_filter_reads_variance(void)
{
    assert_invalid(step_turn(0.0f, 0.0f, 0.0f, V_RIDE), LEAN_REASON_DT);
    /* P0 = (30 deg)^2: not converged after one update */
    assert_invalid(step_turn(0.0f, 0.0f, 0.0f, V_RIDE), LEAN_REASON_VARIANCE);
}

static void test_a_zeroed_object_never_reads_p_zero_as_certainty(void)
{
    memset(&c, 0, sizeof c);
    assert_invalid(step_turn(0.0f, 0.0f, 0.0f, V_RIDE), LEAN_REASON_DT);
    assert_reinit();
    assert_invalid(step_turn(0.0f, 0.0f, 0.0f, V_RIDE), LEAN_REASON_VARIANCE);
}

static void test_converges_to_estimated_within_one_second(void)
{
    uint32_t first = 0u;
    for (uint32_t i = 1u; i <= 50u; i++) {
        if ((step_turn(0.0f, 0.0f, 0.0f, V_RIDE).state == LEAN_OUT_ESTIMATED) && (first == 0u)) {
            first = i;
        }
    }
    TEST_ASSERT_TRUE(first > 2u);
    TEST_ASSERT_TRUE(first <= 50u);
}

/* ---- nominal ---- */

static void test_nominal_straight_reads_zero_with_full_quality(void)
{
    converge();
    const lean_out_t out = step_turn(0.0f, 0.0f, 0.0f, V_RIDE);
    assert_estimated(out);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, out.roll_deg);
}

static void test_nominal_steady_turn_right_and_left(void)
{
    lean_out_t out = ride(DEG(30.0f), 0.0f, V_RIDE, 500u);
    assert_estimated(out);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 30.0f, out.roll_deg);
    TEST_ASSERT_EQUAL_UINT8(100u, out.quality);

    lean_core_init(&c);
    out = ride(DEG(-45.0f), 0.0f, 25.0f, 500u);
    assert_estimated(out);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, -45.0f, out.roll_deg);
}

static void test_tracks_a_roll_ramp_without_leaving_estimated(void)
{
    converge();
    /* 0 -> 40 deg in 1 s, quasi-steady yaw rate, then hold */
    const float rate = DEG(40.0f);
    float roll = 0.0f;
    for (uint32_t i = 0u; i < 50u; i++) {
        roll += rate * ((float)DT_MS * 0.001f);
        const lean_out_t out = step_turn(roll, rate, 0.0f, V_RIDE);
        assert_estimated(out);
        TEST_ASSERT_FLOAT_WITHIN(1.0f, roll * LEAN_DEG_PER_RAD, out.roll_deg);
    }
    const lean_out_t out = ride(roll, 0.0f, V_RIDE, 100u);
    assert_estimated(out);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 40.0f, out.roll_deg);
}

static void test_gyro_bias_converges_on_straight_riding(void)
{
    const float bias = DEG(1.0f); /* 1 deg/s */
    (void)ride(0.0f, bias, V_RIDE, 6000u); /* 120 s */
    TEST_ASSERT_FLOAT_WITHIN(DEG(0.1f), bias, c.bias);
    const lean_out_t out = step_turn(0.0f, 0.0f, bias, V_RIDE);
    assert_estimated(out);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, out.roll_deg);
}

static void test_gyro_bias_converges_in_a_turn(void)
{
    const float bias = DEG(-1.5f);
    const lean_out_t out = ride(DEG(25.0f), bias, 20.0f, 6000u);
    TEST_ASSERT_FLOAT_WITHIN(DEG(0.15f), bias, c.bias);
    assert_estimated(out);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 25.0f, out.roll_deg);
}

/* ---- speed ---- */

static lean_out_t step_with_speed(lean_speed_t sp)
{
    const lean_imu_t s = imu_at(0.0f, 0.0f, now);
    const lean_out_t out = lean_core_step(&c, &s, &sp);
    now += DT_MS;
    return out;
}

static void test_zero_and_low_speed_are_invalid(void)
{
    converge();
    assert_invalid(step_with_speed(speed_at(0.0f, now, LEAN_SPEED_VALID)), LEAN_REASON_SPEED);
    assert_invalid(step_with_speed(speed_at(-3.0f, now, LEAN_SPEED_VALID)), LEAN_REASON_SPEED);
    assert_invalid(step_with_speed(speed_at(4.99f, now, LEAN_SPEED_VALID)), LEAN_REASON_SPEED);
    assert_estimated(step_with_speed(speed_at(LEAN_SPEED_MIN_MPS, now, LEAN_SPEED_VALID)));
}

static void test_no_speed_is_invalid_but_prediction_continues(void)
{
    converge();
    const float p00 = c.p00;
    assert_invalid(step_with_speed(speed_at(V_RIDE, now, LEAN_SPEED_UNUSABLE)), LEAN_REASON_SPEED);
    TEST_ASSERT_TRUE(c.p00 > p00); /* predicted, no update */
    /* allow-list: an unknown state value is unusable */
    assert_invalid(step_with_speed(speed_at(V_RIDE, now, (lean_speed_state_t)7)), LEAN_REASON_SPEED);
    assert_estimated(step_with_speed(speed_at(V_RIDE, now, LEAN_SPEED_VALID)));
}

static void test_non_finite_speed_is_invalid(void)
{
    converge();
    assert_invalid(step_with_speed(speed_at(NAN, now, LEAN_SPEED_VALID)), LEAN_REASON_SPEED);
    assert_invalid(step_with_speed(speed_at(INFINITY, now, LEAN_SPEED_VALID)), LEAN_REASON_SPEED);
    assert_invalid(step_with_speed(speed_at(-INFINITY, now, LEAN_SPEED_VALID)), LEAN_REASON_SPEED);
    assert_estimated(step_with_speed(speed_at(V_RIDE, now, LEAN_SPEED_VALID)));
}

static void test_stale_speed_boundary(void)
{
    converge();
    const uint32_t max_age = PLATFORM_LIMIT_VEHICLE_SPEED_MAX_AGE_MS;
    assert_estimated(step_with_speed(speed_at(V_RIDE, now - max_age, LEAN_SPEED_VALID)));
    assert_invalid(step_with_speed(speed_at(V_RIDE, now - max_age - 1u, LEAN_SPEED_VALID)),
                   LEAN_REASON_SPEED);
}

static void test_age_grown_margin_counts_against_the_minimum(void)
{
    converge();
    /* 200 ms old: margin 0.2 s * 5 m/s^2 = 1 m/s; 5.5 - 1 < 5 */
    assert_invalid(step_with_speed(speed_at(5.5f, now - 200u, LEAN_SPEED_VALID)), LEAN_REASON_SPEED);
    assert_estimated(step_with_speed(speed_at(5.5f, now, LEAN_SPEED_VALID)));
    assert_estimated(step_with_speed(speed_at(6.5f, now - 200u, LEAN_SPEED_VALID)));
}

static void test_a_stalled_speed_producer_reads_invalid_within_the_max_age(void)
{
    converge();
    const uint32_t frozen = now;
    lean_out_t out;
    do {
        out = step_with_speed(speed_at(V_RIDE, frozen, LEAN_SPEED_VALID));
    } while ((out.state == LEAN_OUT_ESTIMATED) && ((now - frozen) < 1000u));
    assert_invalid(out, LEAN_REASON_SPEED);
    TEST_ASSERT_TRUE((now - DT_MS - frozen) <= (PLATFORM_LIMIT_VEHICLE_SPEED_MAX_AGE_MS + DT_MS));
}

static void test_a_slightly_future_stamp_reads_age_zero_never_a_wrapped_age(void)
{
    converge();
    assert_estimated(step_with_speed(speed_at(V_RIDE, now + 1u, LEAN_SPEED_VALID)));
    assert_estimated(step_with_speed(speed_at(V_RIDE, now + LEAN_SPEED_FUTURE_MAX_MS, LEAN_SPEED_VALID)));
    /* with age 0 the margin is 0: the minimum speed itself passes */
    assert_estimated(step_with_speed(speed_at(LEAN_SPEED_MIN_MPS, now + 10u, LEAN_SPEED_VALID)));
    assert_invalid(step_with_speed(speed_at(V_RIDE, now + LEAN_SPEED_FUTURE_MAX_MS + 1u, LEAN_SPEED_VALID)),
                   LEAN_REASON_SPEED);
    assert_invalid(step_with_speed(speed_at(V_RIDE, now + 0x80000000u, LEAN_SPEED_VALID)),
                   LEAN_REASON_SPEED);
}

static void test_timebase_wrap_for_dt_and_speed_age(void)
{
    now = UINT32_MAX - 2000u;
    lean_core_init(&c);
    lean_out_t out = {0};
    for (uint32_t i = 0u; i < 300u; i++) { /* crosses 2^32 after 100 steps */
        out = step_turn(DEG(20.0f), 0.0f, 0.0f, V_RIDE);
    }
    TEST_ASSERT_TRUE(now < 10000u);
    assert_estimated(out);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 20.0f, out.roll_deg);
    /* a stamp from before the wrap, 100 ms old */
    const lean_imu_t s = imu_at(0.0f, yaw_for(DEG(20.0f), V_RIDE), 50u);
    const lean_speed_t sp = speed_at(V_RIDE, UINT32_MAX - 49u, LEAN_SPEED_VALID);
    c.last_ms = 30u;
    assert_estimated(lean_core_step(&c, &s, &sp));
}

/* ---- gyro input ---- */

static void test_non_finite_gyro_is_invalid_and_leaves_the_state_unchanged(void)
{
    converge();
    const float bad[] = {NAN, INFINITY, -INFINITY};
    for (uint32_t i = 0u; i < 3u; i++) {
        const lean_core_t before = c;
        const lean_speed_t sp = speed_at(V_RIDE, now, LEAN_SPEED_VALID);
        lean_imu_t s = imu_at(bad[i], 0.0f, now);
        assert_invalid(lean_core_step(&c, &s, &sp), LEAN_REASON_INPUT);
        s = imu_at(0.0f, bad[i], now);
        assert_invalid(lean_core_step(&c, &s, &sp), LEAN_REASON_INPUT);
        TEST_ASSERT_EQUAL_MEMORY(&before, &c, sizeof c);
    }
    assert_estimated(step_turn(0.0f, 0.0f, 0.0f, V_RIDE));
}

static void test_gyro_rate_limit_boundary(void)
{
    converge();
    const lean_speed_t sp = speed_at(V_RIDE, now, LEAN_SPEED_VALID);
    lean_imu_t s = imu_at(LEAN_RATE_MAX_RPS, 0.0f, now);
    assert_invalid(lean_core_step(&c, &s, &sp), LEAN_REASON_INPUT);
    s = imu_at(0.0f, -LEAN_RATE_MAX_RPS, now);
    assert_invalid(lean_core_step(&c, &s, &sp), LEAN_REASON_INPUT);
    /* just below the limit is a usable sample (the result itself is not asserted) */
    s = imu_at(nextafterf(LEAN_RATE_MAX_RPS, 0.0f), 0.0f, now);
    TEST_ASSERT_NOT_EQUAL_INT(LEAN_REASON_INPUT, lean_core_step(&c, &s, &sp).reason);
}

static void test_bad_samples_spanning_more_than_the_dt_limit_reinitialise(void)
{
    converge();
    for (uint32_t i = 0u; i < 3u; i++) {
        const lean_speed_t sp = speed_at(V_RIDE, now, LEAN_SPEED_VALID);
        const lean_imu_t s = imu_at(NAN, 0.0f, now);
        assert_invalid(lean_core_step(&c, &s, &sp), LEAN_REASON_INPUT);
        now += DT_MS;
    }
    /* 80 ms since the last good sample */
    assert_invalid(step_turn(0.0f, 0.0f, 0.0f, V_RIDE), LEAN_REASON_DT);
    assert_reinit();
}

/* ---- dt ---- */

static void test_dt_limits(void)
{
    converge();
    const lean_speed_t sp = speed_at(V_RIDE, now, LEAN_SPEED_VALID);
    const uint32_t last = c.last_ms;
    lean_imu_t s = imu_at(0.0f, 0.0f, last + LEAN_DT_MAX_MS);
    assert_estimated(lean_core_step(&c, &s, &sp));

    s = imu_at(0.0f, 0.0f, c.last_ms); /* dt 0 */
    assert_invalid(lean_core_step(&c, &s, &sp), LEAN_REASON_DT);
    assert_reinit();

    converge();
    s = imu_at(0.0f, 0.0f, c.last_ms + LEAN_DT_MAX_MS + 1u);
    assert_invalid(lean_core_step(&c, &s, &sp), LEAN_REASON_DT);
    assert_reinit();
    TEST_ASSERT_EQUAL_UINT32(s.stamp_ms, c.last_ms);

    converge();
    s = imu_at(0.0f, 0.0f, c.last_ms - 1u); /* backwards */
    assert_invalid(lean_core_step(&c, &s, &sp), LEAN_REASON_DT);
}

/* ---- innovation gate ---- */

static void test_a_diverged_filter_reads_invalid_after_the_reject_window(void)
{
    converge();
    c.roll = DEG(40.0f); /* diverged: the bike rides straight */
    const uint32_t start = now; /* stamp of the first reject */
    lean_out_t out;
    /* up to and including start + window: gyro propagation, ESTIMATED, QUALITY capped */
    while ((now - start) <= LEAN_GATE_REJECT_MAX_MS) {
        out = step_turn(0.0f, 0.0f, 0.0f, V_RIDE);
        assert_estimated(out);
        TEST_ASSERT_TRUE(out.quality <= LEAN_QUALITY_REJECT_MAX);
        TEST_ASSERT_TRUE(c.rejecting);
    }
    /* the first reject longer than the window after the first one */
    out = step_turn(0.0f, 0.0f, 0.0f, V_RIDE);
    assert_invalid(out, LEAN_REASON_GATE);
    assert_reinit();
    /* reconverges from the reinit */
    out = ride(0.0f, 0.0f, V_RIDE, 100u);
    assert_estimated(out);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, out.roll_deg);
}

static void test_a_single_outlier_caps_quality_and_an_accepted_update_ends_the_run(void)
{
    converge();
    const lean_speed_t sp = speed_at(V_RIDE, now, LEAN_SPEED_VALID);
    const lean_imu_t s = imu_at(0.0f, 1.0f, now); /* 1 rad/s yaw while straight */
    now += DT_MS;
    const float roll = c.roll;
    lean_out_t out = lean_core_step(&c, &s, &sp);
    assert_estimated(out);
    TEST_ASSERT_EQUAL_UINT8(LEAN_QUALITY_REJECT_MAX, out.quality);
    TEST_ASSERT_TRUE(c.rejecting);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, roll, c.roll); /* not updated */
    out = step_turn(0.0f, 0.0f, 0.0f, V_RIDE);
    assert_estimated(out);
    TEST_ASSERT_FALSE(c.rejecting);
    TEST_ASSERT_EQUAL_UINT8(100u, out.quality);
}

static void test_a_reject_leaves_a_quality_below_the_cap_unchanged(void)
{
    converge();
    c.p00 = DEG(4.0f) * DEG(4.0f); /* quality about 25 */
    const lean_speed_t sp = speed_at(V_RIDE, now, LEAN_SPEED_VALID);
    const lean_imu_t s = imu_at(0.0f, 2.0f, now);
    const lean_out_t out = lean_core_step(&c, &s, &sp);
    TEST_ASSERT_TRUE(c.rejecting);
    assert_estimated(out);
    TEST_ASSERT_TRUE(out.quality < LEAN_QUALITY_REJECT_MAX);
    TEST_ASSERT_EQUAL_UINT8(lean_core_quality(sqrtf(c.p00)), out.quality);
}

static void test_steps_without_speed_hold_the_reject_run(void)
{
    converge();
    c.roll = DEG(40.0f);
    (void)step_turn(0.0f, 0.0f, 0.0f, V_RIDE);
    TEST_ASSERT_TRUE(c.rejecting);
    for (uint32_t i = 0u; i < 10u; i++) { /* 200 ms without speed */
        assert_invalid(step_with_speed(speed_at(0.0f, now, LEAN_SPEED_VALID)), LEAN_REASON_SPEED);
        TEST_ASSERT_TRUE(c.rejecting);
    }
    /* the next reject finds the run older than the window */
    assert_invalid(step_turn(0.0f, 0.0f, 0.0f, V_RIDE), LEAN_REASON_GATE);
}

/* ---- filter health ---- */

static void check_filter_reason(void)
{
    assert_invalid(step_turn(0.0f, 0.0f, 0.0f, V_RIDE), LEAN_REASON_FILTER);
    assert_reinit();
    converge();
}

static void test_non_finite_or_implausible_state_reinitialises(void)
{
    converge();
    c.roll = NAN;
    check_filter_reason();
    c.roll = INFINITY;
    check_filter_reason();
    c.bias = NAN;
    check_filter_reason();
    c.p00 = NAN;
    check_filter_reason();
    c.p01 = INFINITY;
    check_filter_reason();
    c.p11 = NAN;
    check_filter_reason();
    c.p00 = -1.0f;
    check_filter_reason();
    c.p11 = -1e-6f; /* small: p00 stays positive after predict */
    check_filter_reason();
    c.p00 = 1e-3f; /* not positive semi-definite after predict */
    c.p11 = 1e-3f;
    c.p01 = 1e-2f;
    check_filter_reason();
    c.bias = 1.5f * LEAN_BIAS_MAX_RPS;
    check_filter_reason();
    c.bias = -1.5f * LEAN_BIAS_MAX_RPS;
    check_filter_reason();
}

static void test_an_update_that_pushes_the_bias_out_of_range_reinitialises(void)
{
    converge();
    /* a large roll/bias covariance turns an innovation within the gate into a bias jump */
    c.roll = 0.0f;
    c.bias = 0.0f;
    c.p00 = 0.01f;
    c.p01 = 0.09f;
    c.p11 = 1.0f;
    const float v = 10.0f;
    const lean_speed_t sp = speed_at(v, now, LEAN_SPEED_VALID);
    const lean_imu_t s = imu_at(0.0f, 0.3f, now);
    now += DT_MS;
    assert_invalid(lean_core_step(&c, &s, &sp), LEAN_REASON_FILTER);
    assert_reinit();
}

/* ---- variance and roll range ---- */

static void test_variance_grows_without_speed_until_it_reads_variance(void)
{
    converge();
    lean_out_t out = step_with_speed(speed_at(0.0f, now, LEAN_SPEED_VALID));
    assert_invalid(out, LEAN_REASON_SPEED);
    uint32_t n = 0u;
    while ((out.reason == LEAN_REASON_SPEED) && (n < 10000u)) {
        out = step_with_speed(speed_at(0.0f, now, LEAN_SPEED_VALID));
        n++;
    }
    assert_invalid(out, LEAN_REASON_VARIANCE); /* checked before the speed */
    TEST_ASSERT_TRUE(sqrtf(c.p00) > LEAN_SIGMA_MAX_RAD);
}

static void test_roll_at_or_beyond_90_deg_is_invalid(void)
{
    converge();
    c.roll = LEAN_ROLL_MAX_RAD;
    assert_invalid(step_turn(LEAN_ROLL_MAX_RAD, 0.0f, 0.0f, V_RIDE), LEAN_REASON_ROLL_RANGE);
    converge();
    c.roll = -DEG(120.0f);
    (void)step_turn(-DEG(120.0f), 0.0f, 0.0f, V_RIDE);
    TEST_ASSERT_TRUE(fabsf(c.roll) >= LEAN_ROLL_MAX_RAD);
    converge();
    c.roll = DEG(80.0f); /* below 90: still the model's range */
    assert_estimated(step_turn(DEG(80.0f), 0.0f, 0.0f, V_RIDE));
}

/* Large leans are estimated, not clamped: CLAMPED is the glue's (PR 3), on the output. */
static void test_large_leans_are_estimated_and_the_output_copy_is_independent(void)
{
    lean_out_t out = ride(DEG(60.0f), 0.0f, 20.0f, 500u);
    assert_estimated(out);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 60.0f, out.roll_deg);
    const lean_core_t before = c;
    out.roll_deg = 0.5f * out.roll_deg; /* the glue rewrites its copy (PR 3: the gen/ clamp) */
    out.state = LEAN_OUT_INVALID;
    TEST_ASSERT_EQUAL_MEMORY(&before, &c, sizeof c);
}

/* ---- quality ---- */

static void test_quality_map(void)
{
    TEST_ASSERT_EQUAL_UINT8(100u, lean_core_quality(0.0f));
    TEST_ASSERT_EQUAL_UINT8(100u, lean_core_quality(LEAN_SIGMA_GOOD_RAD));
    TEST_ASSERT_EQUAL_UINT8(99u, lean_core_quality(nextafterf(LEAN_SIGMA_GOOD_RAD, 1.0f)));
    TEST_ASSERT_EQUAL_UINT8(50u, lean_core_quality(DEG(3.0f)));
    TEST_ASSERT_EQUAL_UINT8(LEAN_QUALITY_MIN, lean_core_quality(LEAN_SIGMA_MAX_RAD));
    TEST_ASSERT_EQUAL_UINT8(0u, lean_core_quality(nextafterf(LEAN_SIGMA_MAX_RAD, 1.0f)));
    TEST_ASSERT_EQUAL_UINT8(0u, lean_core_quality(-1e-6f));
    TEST_ASSERT_EQUAL_UINT8(0u, lean_core_quality(NAN));
    TEST_ASSERT_EQUAL_UINT8(0u, lean_core_quality(INFINITY));
    /* monotone non-increasing */
    uint8_t prev = 100u;
    for (uint32_t i = 0u; i <= 1000u; i++) {
        const uint8_t q = lean_core_quality(LEAN_SIGMA_MAX_RAD * ((float)i / 1000.0f));
        TEST_ASSERT_TRUE(q <= prev);
        prev = q;
    }
}

/* ---- arguments ---- */

static void test_null_arguments(void)
{
    const lean_imu_t s = imu_at(0.0f, 0.0f, now);
    const lean_speed_t sp = speed_at(V_RIDE, now, LEAN_SPEED_VALID);
    lean_core_init(NULL);
    assert_invalid(lean_core_step(NULL, &s, &sp), LEAN_REASON_ARG);
    assert_invalid(lean_core_step(&c, NULL, &sp), LEAN_REASON_ARG);
    assert_invalid(lean_core_step(&c, &s, NULL), LEAN_REASON_ARG);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_first_step_is_invalid_dt_and_a_fresh_filter_reads_variance);
    RUN_TEST(test_a_zeroed_object_never_reads_p_zero_as_certainty);
    RUN_TEST(test_converges_to_estimated_within_one_second);
    RUN_TEST(test_nominal_straight_reads_zero_with_full_quality);
    RUN_TEST(test_nominal_steady_turn_right_and_left);
    RUN_TEST(test_tracks_a_roll_ramp_without_leaving_estimated);
    RUN_TEST(test_gyro_bias_converges_on_straight_riding);
    RUN_TEST(test_gyro_bias_converges_in_a_turn);
    RUN_TEST(test_zero_and_low_speed_are_invalid);
    RUN_TEST(test_no_speed_is_invalid_but_prediction_continues);
    RUN_TEST(test_non_finite_speed_is_invalid);
    RUN_TEST(test_stale_speed_boundary);
    RUN_TEST(test_age_grown_margin_counts_against_the_minimum);
    RUN_TEST(test_a_stalled_speed_producer_reads_invalid_within_the_max_age);
    RUN_TEST(test_a_slightly_future_stamp_reads_age_zero_never_a_wrapped_age);
    RUN_TEST(test_timebase_wrap_for_dt_and_speed_age);
    RUN_TEST(test_non_finite_gyro_is_invalid_and_leaves_the_state_unchanged);
    RUN_TEST(test_gyro_rate_limit_boundary);
    RUN_TEST(test_bad_samples_spanning_more_than_the_dt_limit_reinitialise);
    RUN_TEST(test_dt_limits);
    RUN_TEST(test_a_diverged_filter_reads_invalid_after_the_reject_window);
    RUN_TEST(test_a_single_outlier_caps_quality_and_an_accepted_update_ends_the_run);
    RUN_TEST(test_a_reject_leaves_a_quality_below_the_cap_unchanged);
    RUN_TEST(test_steps_without_speed_hold_the_reject_run);
    RUN_TEST(test_non_finite_or_implausible_state_reinitialises);
    RUN_TEST(test_an_update_that_pushes_the_bias_out_of_range_reinitialises);
    RUN_TEST(test_variance_grows_without_speed_until_it_reads_variance);
    RUN_TEST(test_roll_at_or_beyond_90_deg_is_invalid);
    RUN_TEST(test_large_leans_are_estimated_and_the_output_copy_is_independent);
    RUN_TEST(test_quality_map);
    RUN_TEST(test_null_arguments);
    return UNITY_END();
}
