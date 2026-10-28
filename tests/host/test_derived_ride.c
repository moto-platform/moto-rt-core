/*
 * Class-B ride summaries (D-070 parts 2-3, D-087 item 2) in the pure core:
 *
 * 1. Golden replay of the class-B vectors (the _b.h files in tests/host/vectors, written by the
 *    vehicle-server reference). The rides table is the epoch-marker table: at start_ms of ride N
 *    the test opens a stop epoch (counter N + 1), at end_ms it closes it (ECU absent) and
 *    compares the closed values of that ride: valid flag and flag code exact, values within
 *    the per-signal tolerance (resolution / 2, never widened). The single-ride scenarios
 *    replay the class-A inputs of the same name (the start opens the ride and the end closes
 *    it); two_rides carries its own inputs, an ECU-absent gap and a second ride that must
 *    start from a fresh state.
 * 2. Hand-computed unit tests of the epoch handling (close + reopen in one record, a jump by
 *    more than 1, the uint16 wrap, open -> false -> true, samples of the old epoch never
 *    reused) and of the accumulators.
 */
#include "features/derived/derived_core.h"
#include "vectors/derived_accel_gears.h"
#include "vectors/derived_accel_gears_b.h"
#include "vectors/derived_edge_values.h"
#include "vectors/derived_edge_values_b.h"
#include "vectors/derived_idle_warmup.h"
#include "vectors/derived_idle_warmup_b.h"
#include "vectors/derived_stale_gap.h"
#include "vectors/derived_stale_gap_b.h"
#include "vectors/derived_two_rides_b.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unity.h>

void setUp(void) {}
void tearDown(void) {}

typedef struct {
    const char* name;
    const derived_vector_input_t* in;
    uint32_t n_in;
    const derived_vector_ride_t* rides;
    uint32_t n_rides;
    const derived_vector_b_output_t* out;
    uint32_t n_out;
    const derived_vector_tol_t* tol;
    uint32_t n_tol;
} bscenario_t;

#define BSCENARIO(n, N, IN, NIN) \
    { #n, IN, NIN, n##_rides, N##_RIDE_COUNT, n##_b_outputs, N##_B_OUTPUT_COUNT, n##_b_tol, N##_B_TOL_COUNT }

static const bscenario_t scenarios[] = {
    BSCENARIO(derived_idle_warmup, DERIVED_IDLE_WARMUP, derived_idle_warmup_inputs, DERIVED_IDLE_WARMUP_INPUT_COUNT),
    BSCENARIO(derived_accel_gears, DERIVED_ACCEL_GEARS, derived_accel_gears_inputs, DERIVED_ACCEL_GEARS_INPUT_COUNT),
    BSCENARIO(derived_stale_gap, DERIVED_STALE_GAP, derived_stale_gap_inputs, DERIVED_STALE_GAP_INPUT_COUNT),
    BSCENARIO(derived_edge_values, DERIVED_EDGE_VALUES, derived_edge_values_inputs, DERIVED_EDGE_VALUES_INPUT_COUNT),
    BSCENARIO(derived_two_rides, DERIVED_TWO_RIDES, derived_two_rides_b_inputs, DERIVED_TWO_RIDES_B_INPUT_COUNT),
};
#define N_SCENARIOS (sizeof scenarios / sizeof scenarios[0])

/* The class-A rows and tolerances of the A headers are not used here (their inputs are). */
static void touch_unused_class_a_vectors(void)
{
    (void)sizeof(derived_idle_warmup_outputs);
    (void)sizeof(derived_idle_warmup_tol);
    (void)sizeof(derived_accel_gears_outputs);
    (void)sizeof(derived_accel_gears_tol);
    (void)sizeof(derived_stale_gap_outputs);
    (void)sizeof(derived_stale_gap_tol);
    (void)sizeof(derived_edge_values_outputs);
    (void)sizeof(derived_edge_values_tol);
}

static derived_core_t core;
static derived_in_t in[VEHICLE_CL250_DID_COUNT];
static derived_epoch_t ep;

static void rig(void)
{
    derived_core_init(&core);
    (void)memset(in, 0, sizeof in);
    ep.known = true;
    ep.open = false;
    ep.counter = 0u;
}

static void step(uint32_t now)
{
    derived_core_step_epoch(&core, now, in, &ep);
}

static void sample(uint32_t did, float v, uint32_t stamp)
{
    in[did].value = v;
    in[did].stamp_ms = stamp;
    in[did].present = true;
}

static void epoch(bool open, uint16_t counter)
{
    ep.open = open;
    ep.counter = counter;
}

static derived_out_t cur(uint32_t sig)
{
    derived_out_t o;
    TEST_ASSERT_TRUE(derived_core_ride_out(&core, DERIVED_RIDE_CURRENT, sig, &o));
    return o;
}

static derived_out_t prev(uint32_t sig)
{
    derived_out_t o;
    TEST_ASSERT_TRUE(derived_core_ride_out(&core, DERIVED_RIDE_PREVIOUS, sig, &o));
    return o;
}

static void assert_val(derived_out_t o, float expected, float tol)
{
    TEST_ASSERT_EQUAL_INT_MESSAGE(DERIVED_VALID, o.status, "not valid");
    TEST_ASSERT_FLOAT_WITHIN(tol, expected, o.value);
}

static void assert_inv(derived_out_t o, derived_flag_t flag)
{
    TEST_ASSERT_EQUAL_INT(DERIVED_INVALID, o.status);
    TEST_ASSERT_EQUAL_INT(flag, o.flag);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, o.value);
}

/* ---- golden replay ----------------------------------------------------------------------- */

static float tol_of(const bscenario_t* sc, uint32_t sig)
{
    for (uint32_t i = 0u; i < sc->n_tol; i++) {
        if (sc->tol[i].signal == sig) {
            return sc->tol[i].abs_tol;
        }
    }
    return 0.0f;
}

static uint32_t n_mismatch;
static uint32_t n_rows;

static void compare_ride(const bscenario_t* sc, uint16_t ride)
{
    for (uint32_t r = 0u; r < sc->n_out; r++) {
        const derived_vector_b_output_t* const e = &sc->out[r];
        if (e->ride != ride) {
            continue;
        }
        derived_out_t o;
        TEST_ASSERT_TRUE(derived_core_ride_out(&core, DERIVED_RIDE_PREVIOUS, e->signal, &o));
        const float tol = tol_of(sc, e->signal);
        const bool ok = e->valid ? ((o.status == DERIVED_VALID) && (fabsf(o.value - e->value) <= tol))
                                 : ((o.status == DERIVED_INVALID) && ((uint8_t)o.flag == e->flag));
        n_rows++;
        if (!ok) {
            fprintf(stderr, "%s ride %u sig %u: expected %s flag %u value %.6f, core status %d flag %d value %.6f tol %.6f\n",
                    sc->name, (unsigned)ride, (unsigned)e->signal, e->valid ? "valid" : "invalid",
                    (unsigned)e->flag, (double)e->value, (int)o.status, (int)o.flag, (double)o.value, (double)tol);
            n_mismatch++;
        }
    }
}

static void replay(const bscenario_t* sc)
{
    rig();
    uint32_t ci = 0u;
    const uint32_t end = sc->rides[sc->n_rides - 1u].end_ms;
    for (uint32_t t = 0u; t <= end; t++) {
        while ((ci < sc->n_in) && (sc->in[ci].time_ms <= t)) {
            sample(sc->in[ci].did_idx, sc->in[ci].value, sc->in[ci].time_ms);
            ci++;
        }
        for (uint32_t k = 0u; k < sc->n_rides; k++) {
            if (t == sc->rides[k].start_ms) {
                epoch(true, (uint16_t)(sc->rides[k].ride + 1u));
            }
            if (t == sc->rides[k].end_ms) {
                epoch(false, (uint16_t)(sc->rides[k].ride + 1u));
            }
        }
        step(t);
        for (uint32_t k = 0u; k < sc->n_rides; k++) {
            if (t == sc->rides[k].end_ms) {
                compare_ride(sc, sc->rides[k].ride);
            }
        }
    }
    TEST_ASSERT_EQUAL_UINT32(sc->n_rides, core.ride.closed_count);
    TEST_ASSERT_FALSE(core.ride.open);
}

static void test_golden_rides_match_the_reference(void)
{
    touch_unused_class_a_vectors();
    n_mismatch = 0u;
    n_rows = 0u;
    for (uint32_t i = 0u; i < N_SCENARIOS; i++) {
        replay(&scenarios[i]);
    }
    printf("class-B golden replay: %u closed values compared, %u mismatches\n", (unsigned)n_rows,
           (unsigned)n_mismatch);
    TEST_ASSERT_EQUAL_UINT32(0u, n_mismatch);
    TEST_ASSERT_TRUE(n_rows > 40u);
}

static void test_vectors_carry_no_gps_and_the_computed_set_is_the_golden_set(void)
{
    bool golden[VEHICLE_CL250_DERIVED_COUNT];
    (void)memset(golden, 0, sizeof golden);
    uint32_t n_golden = 0u;
    for (uint32_t i = 0u; i < N_SCENARIOS; i++) {
        for (uint32_t k = 0u; k < scenarios[i].n_tol; k++) {
            const uint32_t sig = scenarios[i].tol[k].signal;
            const vehicle_cl250_derived_t* const spec = &vehicle_cl250_derived[sig];
            for (uint32_t j = 0u; j < spec->input_count; j++) {
                TEST_ASSERT_NOT_EQUAL(VEHICLE_CL250_DERIVED_INPUT_SENSOR, spec->inputs[j].kind); /* no GPS, no sensor */
            }
            if (!golden[sig]) {
                golden[sig] = true;
                n_golden++;
            }
        }
        for (uint32_t k = 0u; k < scenarios[i].n_in; k++) {
            TEST_ASSERT_TRUE(scenarios[i].in[k].did_idx < VEHICLE_CL250_DID_COUNT);
        }
    }
    derived_core_init(&core);
    uint32_t differences = 0u;
    uint32_t class_b_computed = 0u;
    for (uint32_t s = 0u; s < VEHICLE_CL250_DERIVED_COUNT; s++) {
        if (vehicle_cl250_derived[s].derived_class != VEHICLE_CL250_DERIVED_CLASS_RIDE) {
            continue;
        }
        class_b_computed += derived_core_is_computed(&core, s) ? 1u : 0u;
        if (golden[s] != derived_core_is_computed(&core, s)) {
            fprintf(stderr, "class B signal %u: golden %d, computed %d (reason %d)\n", (unsigned)s, (int)golden[s],
                    (int)derived_core_is_computed(&core, s), (int)derived_core_reason(&core, s));
            differences++;
        }
    }
    printf("computed set (class B): %u computed, %u in the golden vectors, %u differences\n",
           (unsigned)class_b_computed, (unsigned)n_golden, (unsigned)differences);
    TEST_ASSERT_EQUAL_UINT32(0u, differences);
    TEST_ASSERT_EQUAL_UINT32(12u, n_golden);
    TEST_ASSERT_EQUAL_UINT32(n_golden, class_b_computed);
    TEST_ASSERT_EQUAL_UINT32(DERIVED_RIDE_SLOTS, class_b_computed);
}

static void test_other_class_b_signals_are_not_computed_with_a_reason(void)
{
    derived_core_init(&core);
    TEST_ASSERT_EQUAL_INT(DERIVED_NC_ELEMENTS, derived_core_reason(&core, VEHICLE_CL250_DERIVED_OPERATING_POINT_TIME));
    TEST_ASSERT_EQUAL_INT(DERIVED_NC_ELEMENTS, derived_core_reason(&core, VEHICLE_CL250_DERIVED_RIDE_COOLANT_TIME_AT_TEMP));
    TEST_ASSERT_EQUAL_INT(DERIVED_NC_ELEMENTS, derived_core_reason(&core, VEHICLE_CL250_DERIVED_RIDE_FINAL_DRIVE_RATIO));
    const uint32_t others[] = {VEHICLE_CL250_DERIVED_RIDE_DISTANCE_GPS, VEHICLE_CL250_DERIVED_RIDE_BUS_OFF_COUNT,
                               VEHICLE_CL250_DERIVED_RIDE_HEAT_SOAK_MAX, VEHICLE_CL250_DERIVED_RIDE_SHIFT_COUNT,
                               VEHICLE_CL250_DERIVED_WINDOW_KEY_ON_REST_VOLTAGE,
                               VEHICLE_CL250_DERIVED_WINDOW_POST_STOP_COOLANT_TAU,
                               VEHICLE_CL250_DERIVED_SPEEDOMETER_ERROR};
    for (uint32_t i = 0u; i < (sizeof others / sizeof others[0]); i++) {
        TEST_ASSERT_FALSE(derived_core_is_computed(&core, others[i]));
        TEST_ASSERT_NOT_EQUAL(DERIVED_NC_NONE, derived_core_reason(&core, others[i]));
        derived_out_t o;
        TEST_ASSERT_TRUE(derived_core_ride_out(&core, DERIVED_RIDE_CURRENT, others[i], &o));
        TEST_ASSERT_EQUAL_INT(DERIVED_NOT_COMPUTED, o.status);
    }
    /* a class-A signal is not a ride row either */
    derived_out_t o;
    TEST_ASSERT_TRUE(derived_core_ride_out(&core, DERIVED_RIDE_PREVIOUS, VEHICLE_CL250_DERIVED_AIR_DENSITY, &o));
    TEST_ASSERT_EQUAL_INT(DERIVED_NOT_COMPUTED, o.status);
    TEST_ASSERT_FALSE(derived_core_ride_out(&core, DERIVED_RIDE_PREVIOUS, VEHICLE_CL250_DERIVED_COUNT, &o));
    TEST_ASSERT_FALSE(derived_core_ride_out(&core, DERIVED_RIDE_PREVIOUS, 0u, NULL));
    TEST_ASSERT_FALSE(derived_core_ride_out(NULL, DERIVED_RIDE_PREVIOUS, 0u, &o));
}

/* ---- epoch handling ------------------------------------------------------------------------- */

#define COOLANT VEHICLE_CL250_IDX_COOLANT_TEMP
#define START_COOLANT VEHICLE_CL250_DERIVED_RIDE_START_COOLANT_TEMP
#define END_COOLANT VEHICLE_CL250_DERIVED_RIDE_END_COOLANT_TEMP

/* A coolant sample at `t`, one step. */
static void coolant(float v, uint32_t t)
{
    sample(COOLANT, v, t);
    step(t);
}

static void test_a_ride_follows_its_epoch(void)
{
    rig();
    step(0u);
    TEST_ASSERT_EQUAL_INT(DERIVED_NO_DATA, cur(START_COOLANT).status); /* no epoch, no ride */
    epoch(true, 1u);
    coolant(60.0f, 10u);
    coolant(62.0f, 810u);
    assert_val(cur(START_COOLANT), 60.0f, 0.0f);
    assert_val(cur(END_COOLANT), 62.0f, 0.0f);
    TEST_ASSERT_EQUAL_INT(DERIVED_NO_DATA, prev(START_COOLANT).status);
    assert_inv(cur(VEHICLE_CL250_DERIVED_RIDE_DISTANCE), DERIVED_FLAG_INVALID_INPUT); /* no speed at all */
    epoch(false, 1u);
    step(900u);
    TEST_ASSERT_EQUAL_INT(DERIVED_NO_DATA, cur(START_COOLANT).status);
    assert_val(prev(START_COOLANT), 60.0f, 0.0f);
    assert_val(prev(END_COOLANT), 62.0f, 0.0f);
    TEST_ASSERT_EQUAL_UINT32(900u, prev(START_COOLANT).stamp_ms);
}

/* Old ride 60 .. 62, then a record that changes the epoch and carries a 90 sample. */
static void old_ride_then(bool open, uint16_t counter)
{
    rig();
    epoch(true, 65535u - 3u);
    coolant(60.0f, 10u);
    coolant(62.0f, 810u);
    epoch(open, counter);
    coolant(90.0f, 1000u);
}

static void check_close_and_reopen(void)
{
    assert_val(prev(END_COOLANT), 62.0f, 0.0f);   /* the new sample is not in the old ride */
    assert_val(prev(START_COOLANT), 60.0f, 0.0f);
    assert_val(cur(START_COOLANT), 90.0f, 0.0f);  /* it starts the new one */
    assert_val(cur(END_COOLANT), 90.0f, 0.0f);
    TEST_ASSERT_EQUAL_UINT32(1u, core.ride.closed_count);
}

static void test_close_and_reopen_seen_as_one_record(void)
{
    old_ride_then(true, 65535u - 2u); /* counter + 1, open still true */
    check_close_and_reopen();
}

static void test_a_jump_by_more_than_one_is_a_new_ride(void)
{
    old_ride_then(true, 17u);
    check_close_and_reopen();
}

static void test_the_counter_wraps(void)
{
    rig();
    epoch(true, 65535u);
    coolant(60.0f, 10u);
    coolant(62.0f, 810u);
    epoch(true, 0u); /* 65535 -> 0 */
    coolant(90.0f, 1000u);
    check_close_and_reopen();
}

static void test_open_false_then_true_and_the_old_samples_are_not_reused(void)
{
    rig();
    epoch(true, 1u);
    coolant(60.0f, 10u);
    coolant(62.0f, 810u);
    epoch(false, 1u);
    coolant(70.0f, 1000u); /* during the ECU-absent stretch: in no ride */
    TEST_ASSERT_EQUAL_INT(DERIVED_NO_DATA, cur(START_COOLANT).status);
    assert_val(prev(END_COOLANT), 62.0f, 0.0f);
    epoch(true, 2u);
    step(1100u); /* the new epoch begins, the producer still holds the 70 sample */
    TEST_ASSERT_EQUAL_INT(DERIVED_INVALID, cur(START_COOLANT).status);
    TEST_ASSERT_EQUAL_INT(DERIVED_FLAG_INVALID_INPUT, cur(START_COOLANT).flag);
    coolant(75.0f, 1200u);
    assert_val(cur(START_COOLANT), 75.0f, 0.0f); /* the first sample of the NEW epoch */
    assert_val(prev(END_COOLANT), 62.0f, 0.0f);  /* the previous ride is untouched */
}

static void test_two_rides_in_a_row_keep_only_the_last_closed_one(void)
{
    rig();
    epoch(true, 1u);
    coolant(60.0f, 10u);
    epoch(false, 1u);
    step(100u);
    epoch(true, 2u);
    coolant(80.0f, 200u);
    coolant(85.0f, 300u);
    epoch(false, 2u);
    step(400u);
    assert_val(prev(START_COOLANT), 80.0f, 0.0f);
    assert_val(prev(END_COOLANT), 85.0f, 0.0f);
    TEST_ASSERT_EQUAL_UINT32(2u, core.ride.closed_count);
    TEST_ASSERT_EQUAL_UINT32(2u, prev(START_COOLANT).seq);
}

static void test_the_part_1_step_has_no_ride(void)
{
    rig();
    in[COOLANT].value = 60.0f;
    in[COOLANT].stamp_ms = 0u;
    in[COOLANT].present = true;
    derived_core_step(&core, 0u, in);
    TEST_ASSERT_FALSE(core.ride.open);
    ep.known = false;
    ep.open = true;
    step(10u); /* an epoch that is not known opens nothing */
    TEST_ASSERT_FALSE(core.ride.open);
}

static void test_a_new_ride_starts_the_class_a_state_afresh(void)
{
    rig();
    epoch(true, 1u);
    for (uint32_t t = 0u; t <= 2000u; t += 110u) {
        sample(VEHICLE_CL250_IDX_VEHICLE_SPEED, 36.0f, t);
        step(t);
    }
    const float first = core.out[VEHICLE_CL250_DERIVED_TRIP_DISTANCE].value;
    TEST_ASSERT_TRUE(first > 0.01f);
    epoch(true, 2u);
    sample(VEHICLE_CL250_IDX_VEHICLE_SPEED, 36.0f, 5000u);
    step(5000u);
    /* the trip distance restarts at 0 (the new ride's first tick), it does not carry on */
    TEST_ASSERT_EQUAL_FLOAT(0.0f, core.out[VEHICLE_CL250_DERIVED_TRIP_DISTANCE].value);
    TEST_ASSERT_EQUAL_UINT32(5000u, core.out[VEHICLE_CL250_DERIVED_TRIP_DISTANCE].stamp_ms);
    assert_val(prev(VEHICLE_CL250_DERIVED_RIDE_DISTANCE), first, 0.02f);
}

/* ---- accumulators ------------------------------------------------------------------------------ */

static void test_fuel_used_holds_a_gap_no_longer_than_the_dids_own_limit(void)
{
    const uint32_t limit = vehicle_cl250_dids[VEHICLE_CL250_IDX_ENGINE_FUEL_RATE].stale_after_ms;
    TEST_ASSERT_TRUE(limit < 10000u);
    rig();
    epoch(true, 1u);
    sample(VEHICLE_CL250_IDX_ENGINE_FUEL_RATE, 3.6f, 0u);
    step(0u);
    sample(VEHICLE_CL250_IDX_ENGINE_FUEL_RATE, 3.6f, 2000u);
    step(2000u);
    sample(VEHICLE_CL250_IDX_ENGINE_FUEL_RATE, 3.6f, 12000u); /* a 10 s gap */
    step(12000u);
    /* 3.6 L/h = 0.001 L/s: 2 s, then the gap held for the DID's own limit; the segment
     * still open adds nothing */
    const float expected = 0.001f * ((2000.0f + (float)limit) / 1000.0f);
    assert_val(cur(VEHICLE_CL250_DERIVED_RIDE_FUEL_USED), expected, 1.0e-6f);
    epoch(false, 1u);
    step(12100u);
    assert_val(prev(VEHICLE_CL250_DERIVED_RIDE_FUEL_USED), expected, 1.0e-6f);
    /* no distance: the consumption is invalid input, not a number */
    assert_inv(prev(VEHICLE_CL250_DERIVED_RIDE_FUEL_CONSUMPTION), DERIVED_FLAG_INVALID_INPUT);
}

static void test_trim_mean_and_population_std(void)
{
    rig();
    epoch(true, 1u);
    step(0u);
    assert_inv(cur(VEHICLE_CL250_DERIVED_RIDE_STFT_STD), DERIVED_FLAG_INVALID_INPUT); /* no sample */
    assert_inv(cur(VEHICLE_CL250_DERIVED_RIDE_LTFT_MEAN), DERIVED_FLAG_INVALID_INPUT);
    const float trims[4] = {1.0f, 2.0f, 3.0f, 4.0f};
    for (uint32_t k = 0u; k < 4u; k++) {
        sample(VEHICLE_CL250_IDX_SHORT_TERM_FUEL_TRIM_B1, trims[k], 100u * k);
        sample(VEHICLE_CL250_IDX_LONG_TERM_FUEL_TRIM_B1, trims[k] - 1.0f, 100u * k);
        step(100u * k);
        if (k == 0u) {
            assert_inv(cur(VEHICLE_CL250_DERIVED_RIDE_STFT_STD), DERIVED_FLAG_INSUFFICIENT_SAMPLES);
        }
    }
    assert_val(cur(VEHICLE_CL250_DERIVED_RIDE_STFT_STD), 1.118034f, 1.0e-4f); /* sqrt(1.25) */
    assert_val(cur(VEHICLE_CL250_DERIVED_RIDE_LTFT_MEAN), 1.5f, 1.0e-5f);     /* mean of 0, 1, 2, 3 */
}

static void test_a_summary_outside_its_range_is_invalid_not_clamped(void)
{
    rig();
    epoch(true, 1u);
    coolant(200.0f, 10u); /* start / end coolant range is -40 .. 120 */
    assert_inv(cur(START_COOLANT), DERIVED_FLAG_OUT_OF_RANGE);
    TEST_ASSERT_EQUAL_FLOAT(200.0f, cur(START_COOLANT).unranged);
}

static void test_a_non_finite_sample_is_a_math_error(void)
{
    rig();
    epoch(true, 1u);
    coolant(NAN, 10u);
    assert_inv(cur(START_COOLANT), DERIVED_FLAG_MATH_ERROR);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_golden_rides_match_the_reference);
    RUN_TEST(test_vectors_carry_no_gps_and_the_computed_set_is_the_golden_set);
    RUN_TEST(test_other_class_b_signals_are_not_computed_with_a_reason);
    RUN_TEST(test_a_ride_follows_its_epoch);
    RUN_TEST(test_close_and_reopen_seen_as_one_record);
    RUN_TEST(test_a_jump_by_more_than_one_is_a_new_ride);
    RUN_TEST(test_the_counter_wraps);
    RUN_TEST(test_open_false_then_true_and_the_old_samples_are_not_reused);
    RUN_TEST(test_two_rides_in_a_row_keep_only_the_last_closed_one);
    RUN_TEST(test_the_part_1_step_has_no_ride);
    RUN_TEST(test_a_new_ride_starts_the_class_a_state_afresh);
    RUN_TEST(test_fuel_used_holds_a_gap_no_longer_than_the_dids_own_limit);
    RUN_TEST(test_trim_mean_and_population_std);
    RUN_TEST(test_a_summary_outside_its_range_is_invalid_not_clamped);
    RUN_TEST(test_a_non_finite_sample_is_a_math_error);
    return UNITY_END();
}
