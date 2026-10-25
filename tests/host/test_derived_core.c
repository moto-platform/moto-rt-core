/*
 * Tests of the pure derived-signal core (D-070). Two kinds of evidence, because golden
 * vectors are necessary but not sufficient (architecture-guard MAJOR on the C3 plan):
 *
 * 1. Golden replay (tests/host/vectors, written by the moto-server Python reference): each
 *    scenario's DID inputs go through the core in time order, stepped every millisecond, and
 *    every expected output row is compared at its time: the core's evaluation tick must be
 *    the reference grid time of the row, the valid flag and the flag code must match
 *    exactly, and a valid value must be within the per-signal tolerance (defs resolution
 *    / 2; never widened). Equal counts: no evaluation of the core lacks a row.
 *    The only allowance is the documented boundary rule below.
 * 2. Independent unit tests with values computed by hand.
 *
 * Boundary allowance (float32 core vs the Python double reference): a row that does not
 * match is skipped (counted, listed, asserted against EXPECTED_SKIPPED) only when
 *   (range)   one side is OUT_OF_RANGE and the unranged value of the core lies within one
 *             tolerance of the gen/ min or max of the signal, or
 *   (gear)    gear_estimate differs and the measured ratio is within 1e-3 (relative) of the
 *             +-5 % match threshold, or
 *   (cascade) the mismatch is INVALID_INPUT / STALE_INPUT / a value that depends on a
 *             signal skipped by one of the two rules above at the same time.
 * Integer comparisons (staleness, <= 0 thresholds on exactly quantised inputs) can not
 * flip between float32 and double, so they have no allowance.
 */
#define _POSIX_C_SOURCE 200809L
#include "features/derived/derived_core.h"
#include "features/derived/derived_gear.h"
#include "vectors/derived_accel_gears.h"
#include "vectors/derived_edge_values.h"
#include "vectors/derived_idle_warmup.h"
#include "vectors/derived_stale_gap.h"
#include "vehicle_cl250_constants.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unity.h>

void setUp(void) {}
void tearDown(void) {}

/* The flag numbering of the core is the one of the vectors. */
_Static_assert((int)DERIVED_FLAG_NONE == (int)DERIVED_VECTOR_FLAG_NONE, "flag NONE");
_Static_assert((int)DERIVED_FLAG_STALE_INPUT == (int)DERIVED_VECTOR_FLAG_STALE_INPUT, "flag");
_Static_assert((int)DERIVED_FLAG_INVALID_INPUT == (int)DERIVED_VECTOR_FLAG_INVALID_INPUT, "flag");
_Static_assert((int)DERIVED_FLAG_INSUFFICIENT_SAMPLES == (int)DERIVED_VECTOR_FLAG_INSUFFICIENT_SAMPLES, "flag");
_Static_assert((int)DERIVED_FLAG_OUT_OF_RANGE == (int)DERIVED_VECTOR_FLAG_OUT_OF_RANGE, "flag");
_Static_assert((int)DERIVED_FLAG_MATH_ERROR == (int)DERIVED_VECTOR_FLAG_MATH_ERROR, "flag");
_Static_assert((int)DERIVED_FLAG_STANDSTILL == (int)DERIVED_VECTOR_FLAG_STANDSTILL, "flag");
_Static_assert((int)DERIVED_FLAG_STRAIGHT_LINE == (int)DERIVED_VECTOR_FLAG_STRAIGHT_LINE, "flag");
_Static_assert((int)DERIVED_FLAG_NO_CONSUMPTION == (int)DERIVED_VECTOR_FLAG_NO_CONSUMPTION, "flag");
_Static_assert((int)DERIVED_FLAG_NO_POWER == (int)DERIVED_VECTOR_FLAG_NO_POWER, "flag");
_Static_assert((int)DERIVED_FLAG_ENGINE_OFF == (int)DERIVED_VECTOR_FLAG_ENGINE_OFF, "flag");

typedef struct {
    const char* name;
    const derived_vector_input_t* in;
    uint32_t n_in;
    const derived_vector_output_t* out;
    uint32_t n_out;
    const derived_vector_tol_t* tol;
    uint32_t n_tol;
} scenario_t;

#define SCENARIO(n, N)                                                                          \
    {                                                                                           \
        #n, n##_inputs, N##_INPUT_COUNT, n##_outputs, N##_OUTPUT_COUNT, n##_tol, N##_TOL_COUNT  \
    }

static const scenario_t scenarios[] = {
    SCENARIO(derived_idle_warmup, DERIVED_IDLE_WARMUP),
    SCENARIO(derived_accel_gears, DERIVED_ACCEL_GEARS),
    SCENARIO(derived_stale_gap, DERIVED_STALE_GAP),
    SCENARIO(derived_edge_values, DERIVED_EDGE_VALUES),
};
#define N_SCENARIOS (sizeof scenarios / sizeof scenarios[0])

static derived_core_t core; /* ~13 kB: static, not stack */
static derived_in_t in[VEHICLE_CL250_DID_COUNT];

/* ---- golden replay ---------------------------------------------------------------------- */

typedef struct {
    uint32_t scenario;
    uint32_t time_ms;
    uint32_t sig;
} row_id_t;

/* The rows the boundary rule skipped, filled by the replay; compared to EXPECTED_SKIPPED. */
#define MAX_SKIPPED (64u)
static row_id_t skipped[MAX_SKIPPED];
static uint32_t n_skipped;
static uint32_t n_mismatch;
static uint32_t n_rows_total;
static double step_ns_total;
static uint32_t steps_total;
static double step_ns_max;

static float tol_of(const scenario_t* sc, uint32_t sig)
{
    for (uint32_t i = 0u; i < sc->n_tol; i++) {
        if (sc->tol[i].signal == sig) {
            return sc->tol[i].abs_tol;
        }
    }
    return 0.0f;
}

static bool was_skipped(uint32_t scenario, uint32_t time_ms, uint32_t sig)
{
    for (uint32_t i = 0u; i < n_skipped; i++) {
        if ((skipped[i].scenario == scenario) && (skipped[i].time_ms == time_ms) && (skipped[i].sig == sig)) {
            return true;
        }
    }
    return false;
}

/* (gear) the measured ratio is within 1e-3 of the match threshold of its nearest gear. */
static bool gear_near_threshold(void)
{
    static const float ratios[6] = {VEHICLE_CL250_GEAR_RATIO_1, VEHICLE_CL250_GEAR_RATIO_2,
                                    VEHICLE_CL250_GEAR_RATIO_3, VEHICLE_CL250_GEAR_RATIO_4,
                                    VEHICLE_CL250_GEAR_RATIO_5, VEHICLE_CL250_GEAR_RATIO_6};
    const derived_out_t* const r = derived_core_out(&core, VEHICLE_CL250_DERIVED_GEARBOX_RATIO_MEASURED);
    for (uint32_t i = 0u; i < 6u; i++) {
        const float dev = fabsf((r->unranged / ratios[i]) - 1.0f);
        if (fabsf(dev - DERIVED_GEAR_MATCH_TOLERANCE) < 1.0e-3f) {
            return true;
        }
    }
    return false;
}

static bool near_range(const derived_out_t* o, uint32_t sig, float tol)
{
    const vehicle_cl250_derived_t* const s = &vehicle_cl250_derived[sig];
    return (fabsf(o->unranged - s->min) <= tol) || (fabsf(o->unranged - s->max) <= tol);
}

/* True when the core's row equals the expected one. */
static bool row_matches(const derived_out_t* o, const derived_vector_output_t* e, float tol)
{
    if (e->valid) {
        return (o->status == DERIVED_VALID) && (fabsf(o->value - e->value) <= tol);
    }
    return (o->status == DERIVED_INVALID) && ((uint8_t)o->flag == e->flag);
}

/* The boundary rule; scenario/time identify the row for the cascade. */
static bool boundary_allowed(uint32_t scenario, const derived_vector_output_t* e, const derived_out_t* o,
                             float tol)
{
    const vehicle_cl250_derived_t* const spec = &vehicle_cl250_derived[e->signal];
    const bool range_side = (!e->valid && (e->flag == (uint8_t)DERIVED_FLAG_OUT_OF_RANGE)) ||
                            (o->status == DERIVED_INVALID && o->flag == DERIVED_FLAG_OUT_OF_RANGE);
    if (range_side && near_range(o, e->signal, tol)) {
        return true;
    }
    if ((e->signal == VEHICLE_CL250_DERIVED_GEAR_ESTIMATE) && gear_near_threshold()) {
        return true;
    }
    for (uint32_t k = 0u; k < spec->input_count; k++) {
        if ((spec->inputs[k].kind == VEHICLE_CL250_DERIVED_INPUT_DERIVED) &&
            was_skipped(scenario, e->time_ms, spec->inputs[k].index)) {
            return true; /* cascade */
        }
    }
    return false;
}

static double now_ns(void)
{
    struct timespec ts;
    (void)clock_gettime(CLOCK_MONOTONIC, &ts);
    return ((double)ts.tv_sec * 1.0e9) + (double)ts.tv_nsec;
}

/* Replays one scenario stepping every `stride_ms` ms (1: the reference comparison). */
static void replay(uint32_t scenario)
{
    const scenario_t* const sc = &scenarios[scenario];
    static uint32_t prev_seq[VEHICLE_CL250_DERIVED_COUNT];
    static uint32_t rows_of[VEHICLE_CL250_DERIVED_COUNT];
    static uint32_t evals_of[VEHICLE_CL250_DERIVED_COUNT];
    static uint32_t last_row_ms[VEHICLE_CL250_DERIVED_COUNT];
    derived_core_init(&core);
    (void)memset(in, 0, sizeof in);
    (void)memset(prev_seq, 0, sizeof prev_seq);
    (void)memset(rows_of, 0, sizeof rows_of);
    (void)memset(evals_of, 0, sizeof evals_of);
    (void)memset(last_row_ms, 0, sizeof last_row_ms);
    for (uint32_t r = 0u; r < sc->n_out; r++) {
        last_row_ms[sc->out[r].signal] = sc->out[r].time_ms;
    }
    const uint32_t end_ms = sc->out[sc->n_out - 1u].time_ms;
    uint32_t ci = 0u;
    uint32_t co = 0u;
    for (uint32_t t = 0u; t <= end_ms; t++) {
        while ((ci < sc->n_in) && (sc->in[ci].time_ms <= t)) {
            in[sc->in[ci].did_idx].value = sc->in[ci].value;
            in[sc->in[ci].did_idx].stamp_ms = sc->in[ci].time_ms;
            in[sc->in[ci].did_idx].present = true;
            ci++;
        }
        const double t0 = now_ns();
        derived_core_step(&core, t, in);
        const double dt = now_ns() - t0;
        step_ns_total += dt;
        step_ns_max = (dt > step_ns_max) ? dt : step_ns_max;
        steps_total++;
        /* count this step's evaluations (only those a reference row can answer) */
        for (uint32_t s = 0u; s < VEHICLE_CL250_DERIVED_COUNT; s++) {
            const derived_out_t* const o = derived_core_out(&core, s);
            if ((o->seq != prev_seq[s]) && (t <= last_row_ms[s])) {
                evals_of[s]++;
            }
        }
        while ((co < sc->n_out) && (sc->out[co].time_ms == t)) {
            const derived_vector_output_t* const e = &sc->out[co];
            const derived_out_t* const o = derived_core_out(&core, e->signal);
            const float tol = tol_of(sc, e->signal);
            rows_of[e->signal]++;
            n_rows_total++;
            const bool evaluated = (o->seq != prev_seq[e->signal]) && (o->stamp_ms == t);
            if (!evaluated) {
                fprintf(stderr, "%s: t=%u sig=%u: no evaluation on the reference grid (stamp %u)\n", sc->name,
                        (unsigned)t, (unsigned)e->signal, (unsigned)o->stamp_ms);
                n_mismatch++;
            } else if (!row_matches(o, e, tol)) {
                if ((n_skipped < MAX_SKIPPED) && boundary_allowed(scenario, e, o, tol)) {
                    skipped[n_skipped].scenario = scenario;
                    skipped[n_skipped].time_ms = t;
                    skipped[n_skipped].sig = e->signal;
                    n_skipped++;
                } else {
                    fprintf(stderr,
                            "%s: t=%u sig=%u MISMATCH expected %s flag %u value %.6f, core status %d flag %d "
                            "value %.6f (unranged %.6f) tol %.6f\n",
                            sc->name, (unsigned)t, (unsigned)e->signal, e->valid ? "valid" : "invalid",
                            (unsigned)e->flag, (double)e->value, (int)o->status, (int)o->flag, (double)o->value,
                            (double)o->unranged, (double)tol);
                    n_mismatch++;
                }
            } else {
                /* equal */
            }
            co++;
        }
        for (uint32_t s = 0u; s < VEHICLE_CL250_DERIVED_COUNT; s++) {
            prev_seq[s] = derived_core_out(&core, s)->seq;
        }
    }
    TEST_ASSERT_EQUAL_UINT32(sc->n_out, co);
    for (uint32_t s = 0u; s < VEHICLE_CL250_DERIVED_COUNT; s++) {
        if (rows_of[s] != evals_of[s]) {
            fprintf(stderr, "%s: sig=%u: %u reference rows but %u core evaluations\n", sc->name, (unsigned)s,
                    (unsigned)rows_of[s], (unsigned)evals_of[s]);
            n_mismatch++;
        }
    }
}

/* The rows the boundary rule is expected to skip over all four scenarios (none so far). */
static const row_id_t EXPECTED_SKIPPED[] = {{0u, 0u, 0u}};
#define N_EXPECTED_SKIPPED (0u)

static void test_golden_replay_matches_the_reference(void)
{
    n_skipped = 0u;
    n_mismatch = 0u;
    n_rows_total = 0u;
    for (uint32_t i = 0u; i < N_SCENARIOS; i++) {
        replay(i);
    }
    for (uint32_t i = 0u; i < n_skipped; i++) {
        fprintf(stderr, "boundary-skipped: %s t=%u sig=%u\n", scenarios[skipped[i].scenario].name,
                (unsigned)skipped[i].time_ms, (unsigned)skipped[i].sig);
    }
    printf("golden replay: %u rows compared, %u skipped by the boundary rule, %u mismatches\n",
           (unsigned)n_rows_total, (unsigned)n_skipped, (unsigned)n_mismatch);
    printf("cost: %u steps, mean %.0f ns/step, max %.0f ns/step (host, -O0 sanitizers), at most %u "
           "evaluations per step, %u signals computed\n",
           (unsigned)steps_total, step_ns_total / (double)steps_total, step_ns_max,
           (unsigned)core.evals_max, (unsigned)core.computed_count);
    TEST_ASSERT_EQUAL_UINT32(0u, n_mismatch);
    TEST_ASSERT_EQUAL_UINT32(N_EXPECTED_SKIPPED, n_skipped);
    /* the list may be empty: count from the array so gcc's -Wtype-limits sees no `i < 0` */
    const uint32_t listed = (uint32_t)(sizeof EXPECTED_SKIPPED / sizeof EXPECTED_SKIPPED[0]);
    for (uint32_t i = 0u; (i < listed) && (i < n_skipped); i++) {
        TEST_ASSERT_TRUE(was_skipped(EXPECTED_SKIPPED[i].scenario, EXPECTED_SKIPPED[i].time_ms,
                                     EXPECTED_SKIPPED[i].sig));
    }
    /* bounded work per step: one evaluation per computed signal plus the three integral
     * breakpoint updates (the event update replaces a grid evaluation) */
    TEST_ASSERT_TRUE(core.evals_max <= (core.computed_count + 3u));
}


/* ---- test rig ----------------------------------------------------------------------------- */

static void rig(void)
{
    derived_core_init(&core);
    (void)memset(in, 0, sizeof in);
}

static void put(uint32_t did, float v, uint32_t stamp)
{
    in[did].value = v;
    in[did].stamp_ms = stamp;
    in[did].present = true;
}

/* Every DID present with a benign value, stamped `stamp`. */
static void all_dids(float v, uint32_t stamp)
{
    for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
        put(i, v, stamp);
    }
}

/* Re-stamps every present DID (a new sample with the same value). */
static void touch_all(uint32_t stamp)
{
    for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
        in[i].stamp_ms = stamp;
        in[i].present = true;
    }
}

static const derived_out_t* out(uint32_t sig)
{
    return derived_core_out(&core, sig);
}

static void assert_valid(uint32_t sig, float expected, float tol)
{
    const derived_out_t* const o = out(sig);
    TEST_ASSERT_EQUAL_INT_MESSAGE(DERIVED_VALID, o->status, "signal not valid");
    TEST_ASSERT_FLOAT_WITHIN(tol, expected, o->value);
}

static void assert_invalid(uint32_t sig, derived_flag_t flag)
{
    const derived_out_t* const o = out(sig);
    TEST_ASSERT_EQUAL_INT(DERIVED_INVALID, o->status);
    TEST_ASSERT_EQUAL_INT(flag, o->flag);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, o->value);
}

/* ---- the computed set ------------------------------------------------------------------------ */

static void test_computed_set_equals_the_golden_set(void)
{
    bool golden[VEHICLE_CL250_DERIVED_COUNT];
    (void)memset(golden, 0, sizeof golden);
    uint32_t n_golden = 0u;
    for (uint32_t i = 0u; i < N_SCENARIOS; i++) {
        for (uint32_t k = 0u; k < scenarios[i].n_tol; k++) {
            if (!golden[scenarios[i].tol[k].signal]) {
                golden[scenarios[i].tol[k].signal] = true;
                n_golden++;
            }
        }
    }
    derived_core_init(&core);
    uint32_t differences = 0u;
    for (uint32_t s = 0u; s < VEHICLE_CL250_DERIVED_COUNT; s++) {
        if (golden[s] != derived_core_is_computed(&core, s)) {
            fprintf(stderr, "signal %u: golden %d, computed %d (reason %d)\n", (unsigned)s, (int)golden[s],
                    (int)derived_core_is_computed(&core, s), (int)derived_core_reason(&core, s));
            differences++;
        }
    }
    printf("computed set: %u computed, %u in the golden vectors, %u differences\n",
           (unsigned)core.computed_count, (unsigned)n_golden, (unsigned)differences);
    TEST_ASSERT_EQUAL_UINT32(0u, differences);
    TEST_ASSERT_EQUAL_UINT32(28u, n_golden);
    TEST_ASSERT_EQUAL_UINT32(n_golden, core.computed_count);
}

static void test_not_computed_signals_report_a_distinct_reason(void)
{
    derived_core_init(&core);
    uint32_t seen[8] = {0u};
    for (uint32_t s = 0u; s < VEHICLE_CL250_DERIVED_COUNT; s++) {
        const derived_nc_reason_t r = derived_core_reason(&core, s);
        const derived_out_t* const o = derived_core_out(&core, s);
        if (derived_core_is_computed(&core, s)) {
            TEST_ASSERT_EQUAL_INT(DERIVED_NC_NONE, r);
            TEST_ASSERT_EQUAL_INT(DERIVED_NO_DATA, o->status);
        } else {
            TEST_ASSERT_NOT_EQUAL(DERIVED_NC_NONE, r);
            TEST_ASSERT_EQUAL_INT(DERIVED_NOT_COMPUTED, o->status);
            seen[(uint32_t)r]++;
        }
        TEST_ASSERT_EQUAL_INT(DERIVED_FLAG_NONE, o->flag);
    }
    /* a ride summary is class B; an IMU-input signal is not ready; wheel slip has no formula */
    TEST_ASSERT_EQUAL_INT(DERIVED_NC_CLASS, derived_core_reason(&core, VEHICLE_CL250_DERIVED_RIDE_DISTANCE));
    TEST_ASSERT_EQUAL_INT(DERIVED_NC_NOT_READY, derived_core_reason(&core, VEHICLE_CL250_DERIVED_ACCEL_IMU_RESIDUAL));
    TEST_ASSERT_EQUAL_INT(DERIVED_NC_NOT_READY, derived_core_reason(&core, VEHICLE_CL250_DERIVED_ROAD_GRADE));
    TEST_ASSERT_TRUE(seen[(uint32_t)DERIVED_NC_CLASS] > 0u);
    TEST_ASSERT_TRUE(seen[(uint32_t)DERIVED_NC_NOT_READY] > 0u);
    TEST_ASSERT_TRUE(seen[(uint32_t)DERIVED_NC_NO_FORMULA] > 0u);
    TEST_ASSERT_EQUAL_UINT32(0u, seen[(uint32_t)DERIVED_NC_SIZING]); /* every window fits */
    TEST_ASSERT_EQUAL_UINT32(0u, seen[(uint32_t)DERIVED_NC_GEN]);
    printf("not computed: %u class B, %u not ready, %u no formula, %u input\n",
           (unsigned)seen[(uint32_t)DERIVED_NC_CLASS], (unsigned)seen[(uint32_t)DERIVED_NC_NOT_READY],
           (unsigned)seen[(uint32_t)DERIVED_NC_NO_FORMULA], (unsigned)seen[(uint32_t)DERIVED_NC_INPUT]);
}

static void test_dependency_order_in_the_gen_table(void)
{
    for (uint32_t s = 0u; s < VEHICLE_CL250_DERIVED_COUNT; s++) {
        const vehicle_cl250_derived_t* const spec = &vehicle_cl250_derived[s];
        for (uint32_t k = 0u; k < spec->input_count; k++) {
            if (spec->inputs[k].kind == VEHICLE_CL250_DERIVED_INPUT_DERIVED) {
                TEST_ASSERT_TRUE_MESSAGE(spec->inputs[k].index < s, "a derived input comes later in the table");
            }
        }
    }
}

static void test_sizing_covers_the_gen_windows(void)
{
    /* the sample buffers hold window_ms / poll period + 2 samples of every windowed signal */
    TEST_ASSERT_TRUE(vehicle_cl250_derived[VEHICLE_CL250_DERIVED_FUEL_CONSUMPTION_AVG].window_ms /
                             vehicle_cl250_dids[VEHICLE_CL250_IDX_VEHICLE_SPEED].poll_period_ms + 2u <=
                     DERIVED_WIN_SPEED_CAP);
    TEST_ASSERT_TRUE(vehicle_cl250_derived[VEHICLE_CL250_DERIVED_FUEL_CONSUMPTION_AVG].window_ms /
                             vehicle_cl250_dids[VEHICLE_CL250_IDX_ENGINE_FUEL_RATE].poll_period_ms + 2u <=
                     DERIVED_WIN_FUEL_RATE_CAP);
    TEST_ASSERT_TRUE(vehicle_cl250_derived[VEHICLE_CL250_DERIVED_COOLANT_RATE].window_ms /
                             vehicle_cl250_dids[VEHICLE_CL250_IDX_COOLANT_TEMP].poll_period_ms + 2u <=
                     DERIVED_WIN_COOLANT_CAP);
    TEST_ASSERT_TRUE(vehicle_cl250_derived[VEHICLE_CL250_DERIVED_OIL_TEMP_RATE].window_ms /
                             vehicle_cl250_dids[VEHICLE_CL250_IDX_ENGINE_OIL_TEMP].poll_period_ms + 2u <=
                     DERIVED_WIN_OIL_CAP);
    TEST_ASSERT_TRUE(vehicle_cl250_derived[VEHICLE_CL250_DERIVED_VEHICLE_JERK].window_ms /
                             vehicle_cl250_derived[VEHICLE_CL250_DERIVED_VEHICLE_ACCEL].period_ms + 2u <=
                     DERIVED_WIN_ACCEL_CAP);
    printf("static RAM: derived_core_t %u bytes (sample buffers %u, signal tables %u)\n",
           (unsigned)sizeof(derived_core_t),
           (unsigned)(sizeof core.speed_win + sizeof core.fuel_rate_win + sizeof core.coolant_win +
                      sizeof core.oil_win + sizeof core.accel_win),
           (unsigned)(sizeof core.out + sizeof core.sched));
}

/* ---- vectors vs the pinned gen table ----------------------------------------------------------- */

static uint32_t fnv(uint32_t h, const void* p, size_t n)
{
    const uint8_t* b = (const uint8_t*)p;
    for (size_t i = 0u; i < n; i++) {
        h ^= b[i];
        h *= 16777619u;
    }
    return h;
}

/* FNV-1a over everything in the pinned gen/ that the vectors' expected outputs depend on:
 * for every signal in the vectors its period, stale limit, window, range, resolution and
 * inputs; for every DID those read the poll period; and the vehicle constants the
 * generator uses. A defs bump that changes any of them changes this value, and the test
 * then says "regenerate the vectors" instead of failing with unrelated value differences. */
static uint32_t gen_fingerprint(void)
{
    bool used_did[VEHICLE_CL250_DID_COUNT] = {false};
    uint32_t h = 2166136261u;
    for (uint32_t s = 0u; s < VEHICLE_CL250_DERIVED_COUNT; s++) {
        bool in_vectors = false;
        for (uint32_t i = 0u; i < N_SCENARIOS; i++) {
            for (uint32_t k = 0u; k < scenarios[i].n_tol; k++) {
                in_vectors = in_vectors || (scenarios[i].tol[k].signal == s);
            }
        }
        if (in_vectors) {
            const vehicle_cl250_derived_t* const d = &vehicle_cl250_derived[s];
            h = fnv(h, &s, sizeof s);
            h = fnv(h, &d->period_ms, sizeof d->period_ms);
            h = fnv(h, &d->stale_after_ms, sizeof d->stale_after_ms);
            h = fnv(h, &d->window_ms, sizeof d->window_ms);
            h = fnv(h, &d->min, sizeof d->min);
            h = fnv(h, &d->max, sizeof d->max);
            h = fnv(h, &d->resolution, sizeof d->resolution);
            h = fnv(h, &d->input_count, sizeof d->input_count);
            for (uint32_t k = 0u; k < d->input_count; k++) {
                h = fnv(h, &d->inputs[k].kind, sizeof d->inputs[k].kind);
                h = fnv(h, &d->inputs[k].index, sizeof d->inputs[k].index);
                if (d->inputs[k].kind == VEHICLE_CL250_DERIVED_INPUT_DID) {
                    used_did[d->inputs[k].index] = true;
                }
            }
        }
    }
    for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
        if (used_did[i] || (i == (uint32_t)VEHICLE_CL250_IDX_BATTERY_VOLTAGE)) { /* the extra DID */
            h = fnv(h, &i, sizeof i);
            h = fnv(h, &vehicle_cl250_dids[i].poll_period_ms, sizeof vehicle_cl250_dids[i].poll_period_ms);
            h = fnv(h, &vehicle_cl250_dids[i].stale_after_ms, sizeof vehicle_cl250_dids[i].stale_after_ms); /* D-070 */
        }
    }
    const float constants[] = {
        VEHICLE_CL250_PRIMARY_REDUCTION_RATIO, VEHICLE_CL250_SECONDARY_REDUCTION_RATIO,
        VEHICLE_CL250_GEAR_RATIO_1, VEHICLE_CL250_GEAR_RATIO_2, VEHICLE_CL250_GEAR_RATIO_3,
        VEHICLE_CL250_GEAR_RATIO_4, VEHICLE_CL250_GEAR_RATIO_5, VEHICLE_CL250_GEAR_RATIO_6,
        VEHICLE_CL250_REAR_WHEEL_RADIUS_DYNAMIC_M, VEHICLE_CL250_FUEL_DENSITY_KG_PER_L,
        VEHICLE_CL250_STOICHIOMETRIC_AFR, VEHICLE_CL250_FUEL_TANK_CAPACITY_L};
    return fnv(h, constants, sizeof constants);
}

#define GEN_FINGERPRINT (0xC4A62B59u)

static void test_vectors_match_the_pinned_gen_table(void)
{
    const uint32_t h = gen_fingerprint();
    if (h != GEN_FINGERPRINT) {
        fprintf(stderr, "gen fingerprint is 0x%08X: regenerate the vectors (see vectors/README.md) and update\n",
                (unsigned)h);
    }
    TEST_ASSERT_EQUAL_HEX32(GEN_FINGERPRINT, h);
    /* the tolerance of every vector signal is resolution / 2 of the gen entry */
    for (uint32_t i = 0u; i < N_SCENARIOS; i++) {
        for (uint32_t k = 0u; k < scenarios[i].n_tol; k++) {
            TEST_ASSERT_FLOAT_WITHIN(1.0e-6f, vehicle_cl250_derived[scenarios[i].tol[k].signal].resolution / 2.0f,
                                     scenarios[i].tol[k].abs_tol);
        }
        /* inputs follow each DID's poll period (gaps are whole periods) */
        uint32_t last[VEHICLE_CL250_DID_COUNT];
        bool seen[VEHICLE_CL250_DID_COUNT] = {false};
        for (uint32_t r = 0u; r < scenarios[i].n_in; r++) {
            const uint32_t d = scenarios[i].in[r].did_idx;
            if (seen[d]) {
                TEST_ASSERT_EQUAL_UINT32(0u, (scenarios[i].in[r].time_ms - last[d]) %
                                                 vehicle_cl250_dids[d].poll_period_ms);
            }
            seen[d] = true;
            last[d] = scenarios[i].in[r].time_ms;
        }
    }
}

/* ---- hand-computed values ---------------------------------------------------------------------- */

static void test_torque_and_power(void)
{
    rig();
    all_dids(10.0f, 0u);
    put(VEHICLE_CL250_IDX_ACTUAL_TORQUE_PCT, 20.0f, 0u);
    put(VEHICLE_CL250_IDX_DEMAND_TORQUE_PCT, 30.0f, 0u);
    put(VEHICLE_CL250_IDX_REFERENCE_TORQUE, 23.0f, 0u);
    put(VEHICLE_CL250_IDX_ENGINE_SPEED, 3000.0f, 0u);
    derived_core_step(&core, 0u, in);
    assert_valid(VEHICLE_CL250_DERIVED_ENGINE_TORQUE, 4.6f, 1.0e-4f);   /* 20 % of 23 Nm */
    assert_valid(VEHICLE_CL250_DERIVED_DEMAND_TORQUE, 6.9f, 1.0e-4f);   /* 30 % of 23 Nm */
    assert_valid(VEHICLE_CL250_DERIVED_TORQUE_GAP, 2.3f, 1.0e-4f);      /* 6.9 - 4.6 */
    assert_valid(VEHICLE_CL250_DERIVED_ENGINE_POWER, 1.44513f, 1.0e-4f); /* 4.6 Nm * 314.159 rad/s */
}

static void test_air_and_altitude(void)
{
    rig();
    all_dids(10.0f, 0u);
    put(VEHICLE_CL250_IDX_BAROMETRIC_PRESSURE, 99.0f, 0u);
    put(VEHICLE_CL250_IDX_INTAKE_AIR_TEMP, 20.0f, 0u);
    derived_core_step(&core, 0u, in);
    assert_valid(VEHICLE_CL250_DERIVED_ALTITUDE_CHANGE, 0.0f, 1.0e-3f); /* the reference is itself */
    put(VEHICLE_CL250_IDX_INTAKE_AIR_TEMP, 25.0f, 0u);
    touch_all(5000u);
    put(VEHICLE_CL250_IDX_BAROMETRIC_PRESSURE, 99.0f, 5000u);
    derived_core_step(&core, 5000u, in);
    /* density of dry air at 99 kPa and 25 degC: 99000 / (287.058 * 298.15) */
    assert_valid(VEHICLE_CL250_DERIVED_AIR_DENSITY, 1.156727f, 1.0e-4f);
    assert_valid(VEHICLE_CL250_DERIVED_POWER_CORRECTION_FACTOR, 1.0f, 1.0e-4f); /* the J1349 reference */
    touch_all(10000u);
    put(VEHICLE_CL250_IDX_BAROMETRIC_PRESSURE, 98.0f, 10000u);
    put(VEHICLE_CL250_IDX_INTAKE_AIR_TEMP, 20.0f, 10000u);
    derived_core_step(&core, 10000u, in);
    /* (287.058 * 293.15 / 9.80665) * ln(99 / 98) */
    assert_valid(VEHICLE_CL250_DERIVED_ALTITUDE_CHANGE, 87.1177f, 0.01f);
}

static void test_fuel_and_residuals(void)
{
    rig();
    all_dids(10.0f, 0u);
    put(VEHICLE_CL250_IDX_COMMANDED_EQUIV_RATIO, 1.0f, 0u);
    put(VEHICLE_CL250_IDX_MAF_RATE, 4.0f, 0u);
    put(VEHICLE_CL250_IDX_ENGINE_FUEL_RATE, 3.0f, 0u);
    put(VEHICLE_CL250_IDX_VEHICLE_SPEED, 60.0f, 0u);
    put(VEHICLE_CL250_IDX_ENGINE_SPEED, 3000.0f, 0u);
    put(VEHICLE_CL250_IDX_SHORT_TERM_FUEL_TRIM_B1, 2.5f, 0u);
    put(VEHICLE_CL250_IDX_LONG_TERM_FUEL_TRIM_B1, 2.0f, 0u);
    put(VEHICLE_CL250_IDX_THROTTLE_POS, 10.0f, 0u);
    put(VEHICLE_CL250_IDX_ABSOLUTE_THROTTLE_POS_B, 12.0f, 0u);
    put(VEHICLE_CL250_IDX_RELATIVE_THROTTLE_POS, 11.0f, 0u);
    put(VEHICLE_CL250_IDX_COMMANDED_THROTTLE, 13.0f, 0u);
    put(VEHICLE_CL250_IDX_ACCEL_PEDAL_POS_D, 40.0f, 0u);
    put(VEHICLE_CL250_IDX_RELATIVE_ACCEL_PEDAL_POS, 38.0f, 0u);
    derived_core_step(&core, 0u, in);
    assert_valid(VEHICLE_CL250_DERIVED_AIR_FUEL_RATIO, VEHICLE_CL250_STOICHIOMETRIC_AFR, 1.0e-4f);            /* lambda 1 */
    assert_valid(VEHICLE_CL250_DERIVED_FUEL_RATE_MAF, 1.314888f, 1.0e-4f);         /* 4 * 3.6 / (14.7 * 0.745) */
    assert_valid(VEHICLE_CL250_DERIVED_FUEL_CONSUMPTION_INSTANT, 5.0f, 1.0e-4f);   /* 3 L/h at 60 km/h */
    assert_valid(VEHICLE_CL250_DERIVED_FUEL_PER_STROKE, 24.83333f, 1.0e-3f);       /* 3 * 0.745e6 / 90000 */
    assert_valid(VEHICLE_CL250_DERIVED_TOTAL_FUEL_TRIM, 4.5f, 1.0e-4f);
    assert_valid(VEHICLE_CL250_DERIVED_THROTTLE_CHANNEL_RESIDUAL, 2.0f, 1.0e-4f);  /* max(2, 1, 1) */
    assert_valid(VEHICLE_CL250_DERIVED_THROTTLE_COMMAND_RESIDUAL, 3.0f, 1.0e-4f);
    assert_valid(VEHICLE_CL250_DERIVED_GRIP_CHANNEL_RESIDUAL, 2.0f, 1.0e-4f);
}

static void test_gearbox_ratio_and_gear(void)
{
    rig();
    all_dids(10.0f, 0u);
    put(VEHICLE_CL250_IDX_VEHICLE_SPEED, 60.0f, 0u);
    put(VEHICLE_CL250_IDX_ENGINE_SPEED, 8000.0f, 0u);
    derived_core_step(&core, 0u, in);
    /* 8000 / (wheel rpm at 60 km/h on the dynamic radius * primary * secondary reduction ratio) */
    assert_valid(VEHICLE_CL250_DERIVED_GEARBOX_RATIO_MEASURED, 2.107924f, 1.0e-3f);
    assert_valid(VEHICLE_CL250_DERIVED_GEAR_ESTIMATE, 0.0f, 1.0e-6f); /* between 1st (3.416) and 2nd (2.25)... */
    touch_all(110u);
    /* the rpm of exactly 3rd at 60 km/h: ratio * wheel rpm * primary * secondary reduction */
    const float wheel_rpm = ((60.0f / 3.6f) / (2.0f * 3.14159265f * VEHICLE_CL250_REAR_WHEEL_RADIUS_DYNAMIC_M)) * 60.0f;
    put(VEHICLE_CL250_IDX_ENGINE_SPEED,
        VEHICLE_CL250_GEAR_RATIO_3 * wheel_rpm * VEHICLE_CL250_PRIMARY_REDUCTION_RATIO * VEHICLE_CL250_SECONDARY_REDUCTION_RATIO,
        110u);
    derived_core_step(&core, 110u, in);
    assert_valid(VEHICLE_CL250_DERIVED_GEARBOX_RATIO_MEASURED, VEHICLE_CL250_GEAR_RATIO_3, 1.0e-3f);
    assert_valid(VEHICLE_CL250_DERIVED_GEAR_ESTIMATE, 3.0f, 1.0e-6f);
    touch_all(220u);
    put(VEHICLE_CL250_IDX_VEHICLE_SPEED, 0.0f, 220u);
    derived_core_step(&core, 220u, in);
    assert_invalid(VEHICLE_CL250_DERIVED_GEARBOX_RATIO_MEASURED, DERIVED_FLAG_STANDSTILL);
    assert_invalid(VEHICLE_CL250_DERIVED_GEAR_ESTIMATE, DERIVED_FLAG_INVALID_INPUT); /* an invalid derived input */
    assert_invalid(VEHICLE_CL250_DERIVED_FUEL_CONSUMPTION_INSTANT, DERIVED_FLAG_STANDSTILL);
}

static void test_zero_guards_give_their_flags(void)
{
    rig();
    all_dids(10.0f, 0u);
    put(VEHICLE_CL250_IDX_ACTUAL_TORQUE_PCT, 0.0f, 0u);
    put(VEHICLE_CL250_IDX_ENGINE_SPEED, 0.0f, 0u);
    put(VEHICLE_CL250_IDX_ENGINE_FUEL_RATE, 0.0f, 0u);
    derived_core_step(&core, 0u, in);
    assert_invalid(VEHICLE_CL250_DERIVED_FUEL_PER_STROKE, DERIVED_FLAG_ENGINE_OFF);
    assert_invalid(VEHICLE_CL250_DERIVED_BSFC, DERIVED_FLAG_NO_POWER); /* torque 0, power 0 */
    touch_all(110u);
    derived_core_step(&core, 110u, in);
    /* the 60 s means of a 0 L/h rate: no consumption, so no range */
    assert_invalid(VEHICLE_CL250_DERIVED_FUEL_RANGE, DERIVED_FLAG_NO_CONSUMPTION);
}

static void test_windows_slope_and_mean(void)
{
    rig();
    all_dids(10.0f, 0u);
    put(VEHICLE_CL250_IDX_VEHICLE_SPEED, 0.0f, 0u);
    derived_core_step(&core, 0u, in);
    assert_invalid(VEHICLE_CL250_DERIVED_VEHICLE_ACCEL, DERIVED_FLAG_INSUFFICIENT_SAMPLES);
    for (uint32_t k = 1u; k <= 3u; k++) {
        touch_all(110u * k);
        put(VEHICLE_CL250_IDX_VEHICLE_SPEED, 3.6f * (float)k, 110u * k); /* +1 m/s per 110 ms */
        derived_core_step(&core, 110u * k, in);
    }
    assert_valid(VEHICLE_CL250_DERIVED_VEHICLE_ACCEL, 9.090909f, 1.0e-3f); /* 1 m/s / 0.11 s */
    assert_valid(VEHICLE_CL250_DERIVED_VEHICLE_JERK, 0.0f, 1.0e-3f);        /* constant acceleration */
    /* coolant: +0.1 degC per 800 ms is 7.5 degC/min */
    rig();
    all_dids(10.0f, 0u);
    for (uint32_t k = 0u; k < 3u; k++) {
        touch_all(800u * k);
        put(VEHICLE_CL250_IDX_COOLANT_TEMP, 60.0f + (0.1f * (float)k), 800u * k);
        derived_core_step(&core, 800u * k, in);
    }
    assert_valid(VEHICLE_CL250_DERIVED_COOLANT_RATE, 7.5f, 1.0e-2f);
    /* fuel_consumption_avg: 100 * mean(3 L/h) / mean(60 km/h) = 5 L/100km, 144 km on 7.2 L */
    rig();
    all_dids(10.0f, 0u);
    put(VEHICLE_CL250_IDX_ENGINE_FUEL_RATE, 3.0f, 0u);
    put(VEHICLE_CL250_IDX_VEHICLE_SPEED, 60.0f, 0u);
    put(VEHICLE_CL250_IDX_FUEL_LEVEL, 60.0f, 0u);
    derived_core_step(&core, 0u, in);
    assert_invalid(VEHICLE_CL250_DERIVED_FUEL_CONSUMPTION_AVG, DERIVED_FLAG_INSUFFICIENT_SAMPLES);
    touch_all(110u);
    derived_core_step(&core, 110u, in);
    assert_valid(VEHICLE_CL250_DERIVED_FUEL_CONSUMPTION_AVG, 5.0f, 1.0e-3f);
    assert_valid(VEHICLE_CL250_DERIVED_FUEL_RANGE, 144.0f, 1.0e-2f);
}

static void test_integrals_hold_across_gaps_only_within_the_stale_limit(void)
{
    rig();
    for (uint32_t t = 0u; t <= 4950u; t += 110u) {
        put(VEHICLE_CL250_IDX_VEHICLE_SPEED, 36.0f, t); /* 10 m/s = 0.01 km/s */
        derived_core_step(&core, t, in);
    }
    derived_core_step(&core, 5000u, in);
    assert_valid(VEHICLE_CL250_DERIVED_TRIP_DISTANCE, 0.05f, 1.0e-5f);
    /* a gap: after the last sample at 990 ms the speed is held for the speed DID's own 300 ms stale limit and
     * not a millisecond more, then samples resume at 20 s */
    rig();
    for (uint32_t t = 0u; t <= 990u; t += 110u) {
        put(VEHICLE_CL250_IDX_VEHICLE_SPEED, 36.0f, t);
        derived_core_step(&core, t, in);
    }
    put(VEHICLE_CL250_IDX_VEHICLE_SPEED, 36.0f, 20000u);
    derived_core_step(&core, 20000u, in);
    assert_valid(VEHICLE_CL250_DERIVED_TRIP_DISTANCE, 0.0129f, 1.0e-5f); /* 0.0099 + 0.3 s * 0.01 */
}

/* ---- the stop event -------------------------------------------------------------------------------- */

static void sample_stop(uint32_t t, float speed, float rpm)
{
    put(VEHICLE_CL250_IDX_VEHICLE_SPEED, speed, t);
    put(VEHICLE_CL250_IDX_ENGINE_SPEED, rpm, t);
    derived_core_step(&core, t, in);
}

static void test_stop_event(void)
{
    rig();
    sample_stop(0u, 10.0f, 1500.0f);
    sample_stop(110u, 0.0f, 1500.0f); /* the stop starts */
    sample_stop(220u, 0.0f, 1500.0f);
    TEST_ASSERT_EQUAL_UINT32(0u, out(VEHICLE_CL250_DERIVED_STOP_EVENT)->seq);
    sample_stop(330u, 5.0f, 1500.0f); /* it ends: 220 ms */
    TEST_ASSERT_EQUAL_UINT32(1u, out(VEHICLE_CL250_DERIVED_STOP_EVENT)->seq);
    TEST_ASSERT_EQUAL_UINT32(330u, out(VEHICLE_CL250_DERIVED_STOP_EVENT)->stamp_ms);
    assert_valid(VEHICLE_CL250_DERIVED_STOP_EVENT, 0.22f, 1.0e-4f);

    /* standing at the start of the data is no stop */
    rig();
    sample_stop(0u, 0.0f, 1500.0f);
    sample_stop(110u, 0.0f, 1500.0f);
    sample_stop(220u, 5.0f, 1500.0f);
    TEST_ASSERT_EQUAL_UINT32(0u, out(VEHICLE_CL250_DERIVED_STOP_EVENT)->seq);

    /* engine off: not a stop with the engine running */
    rig();
    sample_stop(0u, 10.0f, 1500.0f);
    sample_stop(110u, 0.0f, 0.0f);
    sample_stop(220u, 5.0f, 1500.0f);
    TEST_ASSERT_EQUAL_UINT32(0u, out(VEHICLE_CL250_DERIVED_STOP_EVENT)->seq);

    /* a gap longer than three speed polls ends the stop and the moving state */
    rig();
    sample_stop(0u, 10.0f, 1500.0f);
    sample_stop(110u, 0.0f, 1500.0f);
    sample_stop(441u, 5.0f, 1500.0f); /* 331 ms after the last sample */
    TEST_ASSERT_EQUAL_UINT32(0u, out(VEHICLE_CL250_DERIVED_STOP_EVENT)->seq);
    /* ... and exactly three polls (330 ms) is still followed */
    rig();
    sample_stop(0u, 10.0f, 1500.0f);
    sample_stop(110u, 0.0f, 1500.0f);
    sample_stop(440u, 5.0f, 1500.0f);
    TEST_ASSERT_EQUAL_UINT32(1u, out(VEHICLE_CL250_DERIVED_STOP_EVENT)->seq);
}

static void test_stop_event_longer_than_the_range_is_invalid(void)
{
    rig();
    sample_stop(0u, 10.0f, 1500.0f);
    uint32_t t = 300u;
    for (; t <= 3601000u; t += 300u) {
        sample_stop(t, 0.0f, 1500.0f);
    }
    sample_stop(t, 5.0f, 1500.0f);
    assert_invalid(VEHICLE_CL250_DERIVED_STOP_EVENT, DERIVED_FLAG_OUT_OF_RANGE);
    TEST_ASSERT_TRUE(out(VEHICLE_CL250_DERIVED_STOP_EVENT)->unranged > 3600.0f); /* not clamped */
}

/* ---- staleness, range, NaN ---------------------------------------------------------------------------- */

static void test_stale_exactly_at_the_limit_and_one_past(void)
{
    const uint32_t limit = vehicle_cl250_derived[VEHICLE_CL250_DERIVED_AIR_FUEL_RATIO].stale_after_ms;
    /* the reference calls an input stale when age > stale_after_ms, so age == limit is fresh */
    rig();
    all_dids(1.0f, 0u);
    derived_core_step(&core, limit, in);
    assert_valid(VEHICLE_CL250_DERIVED_AIR_FUEL_RATIO, VEHICLE_CL250_STOICHIOMETRIC_AFR, 1.0e-4f);
    rig();
    all_dids(1.0f, 0u);
    derived_core_step(&core, limit + 1u, in);
    assert_invalid(VEHICLE_CL250_DERIVED_AIR_FUEL_RATIO, DERIVED_FLAG_STALE_INPUT);
}

static void test_a_did_uses_its_own_limit_not_the_signals(void)
{
    /* the speed DID goes stale at 300 ms, the gearbox ratio's own limit is 330 ms */
    TEST_ASSERT_TRUE(vehicle_cl250_dids[VEHICLE_CL250_IDX_VEHICLE_SPEED].stale_after_ms <
                     vehicle_cl250_derived[VEHICLE_CL250_DERIVED_GEARBOX_RATIO_MEASURED].stale_after_ms);
    rig();
    all_dids(10.0f, 0u);
    put(VEHICLE_CL250_IDX_VEHICLE_SPEED, 60.0f, 0u);
    put(VEHICLE_CL250_IDX_ENGINE_SPEED, 3000.0f, 0u);
    derived_core_step(&core, 0u, in);
    assert_valid(VEHICLE_CL250_DERIVED_GEARBOX_RATIO_MEASURED, 1.6f, 1.0f); /* some valid ratio */
    in[VEHICLE_CL250_IDX_ENGINE_SPEED].stamp_ms = 330u; /* the rpm is fresh, the speed is not */
    derived_core_step(&core, 330u, in);
    assert_invalid(VEHICLE_CL250_DERIVED_GEARBOX_RATIO_MEASURED, DERIVED_FLAG_STALE_INPUT);
}

static void test_range_is_checked_and_never_clamped(void)
{
    rig();
    all_dids(10.0f, 0u);
    put(VEHICLE_CL250_IDX_COMMANDED_EQUIV_RATIO, 3.0f, 0u); /* AFR 44.1 > 30 */
    derived_core_step(&core, 0u, in);
    assert_invalid(VEHICLE_CL250_DERIVED_AIR_FUEL_RATIO, DERIVED_FLAG_OUT_OF_RANGE);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-3f, 44.1f, out(VEHICLE_CL250_DERIVED_AIR_FUEL_RATIO)->unranged);
    rig();
    all_dids(10.0f, 0u);
    put(VEHICLE_CL250_IDX_COMMANDED_EQUIV_RATIO, 0.1f, 0u); /* AFR 1.47 < 5 */
    derived_core_step(&core, 0u, in);
    assert_invalid(VEHICLE_CL250_DERIVED_AIR_FUEL_RATIO, DERIVED_FLAG_OUT_OF_RANGE);
    /* exactly on the limit is in range: 5 / 14.7 */
    rig();
    all_dids(10.0f, 0u);
    put(VEHICLE_CL250_IDX_COMMANDED_EQUIV_RATIO, 5.0f / VEHICLE_CL250_STOICHIOMETRIC_AFR, 0u);
    derived_core_step(&core, 0u, in);
    TEST_ASSERT_EQUAL_INT(DERIVED_VALID, out(VEHICLE_CL250_DERIVED_AIR_FUEL_RATIO)->status);
    /* an invalid torque makes its consumers INVALID_INPUT, not a stale or range flag */
    rig();
    all_dids(10.0f, 0u);
    put(VEHICLE_CL250_IDX_ACTUAL_TORQUE_PCT, 200.0f, 0u);
    put(VEHICLE_CL250_IDX_REFERENCE_TORQUE, 23.0f, 0u); /* 46 Nm > 30 */
    derived_core_step(&core, 0u, in);
    assert_invalid(VEHICLE_CL250_DERIVED_ENGINE_TORQUE, DERIVED_FLAG_OUT_OF_RANGE);
    assert_invalid(VEHICLE_CL250_DERIVED_ENGINE_POWER, DERIVED_FLAG_INVALID_INPUT);
    assert_invalid(VEHICLE_CL250_DERIVED_TORQUE_GAP, DERIVED_FLAG_INVALID_INPUT);
}

static void test_non_finite_input_is_a_math_error(void)
{
    rig();
    all_dids(10.0f, 0u);
    put(VEHICLE_CL250_IDX_COMMANDED_EQUIV_RATIO, NAN, 0u);
    derived_core_step(&core, 0u, in);
    assert_invalid(VEHICLE_CL250_DERIVED_AIR_FUEL_RATIO, DERIVED_FLAG_MATH_ERROR);
    rig();
    all_dids(10.0f, 0u);
    put(VEHICLE_CL250_IDX_COMMANDED_EQUIV_RATIO, INFINITY, 0u);
    derived_core_step(&core, 0u, in);
    assert_invalid(VEHICLE_CL250_DERIVED_AIR_FUEL_RATIO, DERIVED_FLAG_MATH_ERROR);
    /* a finite input whose result overflows */
    rig();
    all_dids(10.0f, 0u);
    put(VEHICLE_CL250_IDX_COMMANDED_EQUIV_RATIO, 3.0e38f, 0u);
    derived_core_step(&core, 0u, in);
    assert_invalid(VEHICLE_CL250_DERIVED_AIR_FUEL_RATIO, DERIVED_FLAG_MATH_ERROR);
}

static void test_lapsed_input_stays_unusable_until_a_new_sample(void)
{
    rig();
    all_dids(1.0f, 1000u);
    derived_core_step(&core, 1000u, in);
    assert_valid(VEHICLE_CL250_DERIVED_AIR_FUEL_RATIO, VEHICLE_CL250_STOICHIOMETRIC_AFR, 1.0e-4f);
    derived_core_step(&core, 41000u, in); /* far beyond every limit: latched as lapsed */
    TEST_ASSERT_TRUE(core.did[VEHICLE_CL250_IDX_COMMANDED_EQUIV_RATIO].lapsed);
    assert_invalid(VEHICLE_CL250_DERIVED_AIR_FUEL_RATIO, DERIVED_FLAG_STALE_INPUT);
    touch_all(43000u);
    derived_core_step(&core, 43000u, in);
    TEST_ASSERT_FALSE(core.did[VEHICLE_CL250_IDX_COMMANDED_EQUIV_RATIO].lapsed);
    assert_valid(VEHICLE_CL250_DERIVED_AIR_FUEL_RATIO, VEHICLE_CL250_STOICHIOMETRIC_AFR, 1.0e-4f);
    /* a sample that disappears (present false) is stale, not held */
    in[VEHICLE_CL250_IDX_COMMANDED_EQUIV_RATIO].present = false;
    derived_core_step(&core, 45000u, in);
    assert_invalid(VEHICLE_CL250_DERIVED_AIR_FUEL_RATIO, DERIVED_FLAG_STALE_INPUT);
}

static void test_a_late_step_evaluates_the_latest_tick_only(void)
{
    rig();
    all_dids(1.0f, 0u);
    derived_core_step(&core, 0u, in);
    const uint32_t seq = out(VEHICLE_CL250_DERIVED_AIR_FUEL_RATIO)->seq;
    touch_all(7300u);
    derived_core_step(&core, 7300u, in); /* ticks 2000, 4000, 6000 are due: only 6000 is evaluated */
    TEST_ASSERT_EQUAL_UINT32(seq + 1u, out(VEHICLE_CL250_DERIVED_AIR_FUEL_RATIO)->seq);
    TEST_ASSERT_EQUAL_UINT32(6000u, out(VEHICLE_CL250_DERIVED_AIR_FUEL_RATIO)->stamp_ms);
    touch_all(7400u);
    derived_core_step(&core, 7400u, in); /* the next tick is 8000 */
    TEST_ASSERT_EQUAL_UINT32(seq + 1u, out(VEHICLE_CL250_DERIVED_AIR_FUEL_RATIO)->seq);
}

static void test_coarse_stepping_stays_on_the_reference_grid(void)
{
    const scenario_t* const sc = &scenarios[1]; /* accel_gears */
    derived_core_init(&core);
    (void)memset(in, 0, sizeof in);
    uint32_t ci = 0u;
    uint32_t prev[VEHICLE_CL250_DERIVED_COUNT] = {0u};
    uint32_t evaluated = 0u;
    for (uint32_t t = 0u; t <= 30000u; t += 7u) {
        while ((ci < sc->n_in) && (sc->in[ci].time_ms <= t)) {
            put(sc->in[ci].did_idx, sc->in[ci].value, sc->in[ci].time_ms);
            ci++;
        }
        derived_core_step(&core, t, in);
        for (uint32_t s = 0u; s < VEHICLE_CL250_DERIVED_COUNT; s++) {
            const derived_out_t* const o = out(s);
            if (o->seq != prev[s]) {
                evaluated++;
                if (s != VEHICLE_CL250_DERIVED_STOP_EVENT) {
                    /* every input starts at 0, so every grid starts at 0 */
                    TEST_ASSERT_EQUAL_UINT32(0u, o->stamp_ms % vehicle_cl250_derived[s].period_ms);
                    TEST_ASSERT_TRUE((t - o->stamp_ms) < 7u);
                }
            }
            prev[s] = o->seq;
        }
    }
    TEST_ASSERT_TRUE(evaluated > 500u);
}

static void test_arguments_and_unknown_signals(void)
{
    derived_core_init(NULL);
    derived_core_step(NULL, 0u, in);
    derived_core_step(&core, 0u, NULL);
    TEST_ASSERT_NULL(derived_core_out(NULL, 0u));
    derived_core_init(&core);
    TEST_ASSERT_NULL(derived_core_out(&core, VEHICLE_CL250_DERIVED_COUNT));
    TEST_ASSERT_FALSE(derived_core_is_computed(NULL, 0u));
    TEST_ASSERT_FALSE(derived_core_is_computed(&core, VEHICLE_CL250_DERIVED_COUNT));
    TEST_ASSERT_EQUAL_INT(DERIVED_NC_GEN, derived_core_reason(&core, VEHICLE_CL250_DERIVED_COUNT));
    TEST_ASSERT_EQUAL_INT(DERIVED_NC_GEN, derived_core_reason(NULL, 0u));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_golden_replay_matches_the_reference);
    RUN_TEST(test_computed_set_equals_the_golden_set);
    RUN_TEST(test_not_computed_signals_report_a_distinct_reason);
    RUN_TEST(test_dependency_order_in_the_gen_table);
    RUN_TEST(test_sizing_covers_the_gen_windows);
    RUN_TEST(test_vectors_match_the_pinned_gen_table);
    RUN_TEST(test_torque_and_power);
    RUN_TEST(test_air_and_altitude);
    RUN_TEST(test_fuel_and_residuals);
    RUN_TEST(test_gearbox_ratio_and_gear);
    RUN_TEST(test_zero_guards_give_their_flags);
    RUN_TEST(test_windows_slope_and_mean);
    RUN_TEST(test_integrals_hold_across_gaps_only_within_the_stale_limit);
    RUN_TEST(test_stop_event);
    RUN_TEST(test_stop_event_longer_than_the_range_is_invalid);
    RUN_TEST(test_stale_exactly_at_the_limit_and_one_past);
    RUN_TEST(test_a_did_uses_its_own_limit_not_the_signals);
    RUN_TEST(test_range_is_checked_and_never_clamped);
    RUN_TEST(test_non_finite_input_is_a_math_error);
    RUN_TEST(test_lapsed_input_stays_unusable_until_a_new_sample);
    RUN_TEST(test_a_late_step_evaluates_the_latest_tick_only);
    RUN_TEST(test_coarse_stepping_stays_on_the_reference_grid);
    RUN_TEST(test_arguments_and_unknown_signals);
    return UNITY_END();
}
