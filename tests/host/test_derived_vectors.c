/*
 * Shape checks of the golden vectors in tests/host/vectors (D-070 item 2): the data the
 * future features/derived host test compares against. There is no derived module yet, so
 * this only checks that the generated headers compile against the pinned gen/ headers and
 * are well formed: inputs and outputs in time order, every index inside its gen/ table,
 * an invalid row carries a flag and 0, a valid row none, every signal has a tolerance.
 * The expected values need the defs derived table they were generated from (v0.14.0, see
 * vectors/README.md); this test does not depend on it.
 */
#include "vectors/derived_accel_gears.h"
#include "vectors/derived_edge_values.h"
#include "vectors/derived_idle_warmup.h"
#include "vectors/derived_stale_gap.h"

#include <math.h>
#include <unity.h>

void setUp(void) {}
void tearDown(void) {}

static void check_inputs(const derived_vector_input_t* in, uint32_t n)
{
    TEST_ASSERT_TRUE(n > 0u);
    for (uint32_t i = 0u; i < n; i++) {
        TEST_ASSERT_TRUE(in[i].did_idx < VEHICLE_CL250_DID_COUNT);
        TEST_ASSERT_TRUE(isfinite(in[i].value));
        if (i > 0u) {
            TEST_ASSERT_TRUE(in[i].time_ms >= in[i - 1u].time_ms);
        }
    }
}

static bool has_tol(const derived_vector_tol_t* tol, uint32_t n, uint16_t signal)
{
    for (uint32_t i = 0u; i < n; i++) {
        if (tol[i].signal == signal) {
            return tol[i].abs_tol > 0.0f;
        }
    }
    return false;
}

static void check_outputs(const derived_vector_output_t* out, uint32_t n,
                          const derived_vector_tol_t* tol, uint32_t n_tol)
{
    uint32_t valid = 0u;
    TEST_ASSERT_TRUE(n > 0u);
    for (uint32_t i = 0u; i < n; i++) {
        TEST_ASSERT_TRUE(out[i].signal < VEHICLE_CL250_DERIVED_COUNT);
        TEST_ASSERT_TRUE(out[i].flag <= (uint8_t)DERIVED_VECTOR_FLAG_ENGINE_OFF);
        TEST_ASSERT_TRUE(isfinite(out[i].value));
        TEST_ASSERT_TRUE(has_tol(tol, n_tol, out[i].signal));
        if (out[i].valid) {
            TEST_ASSERT_EQUAL_UINT8(DERIVED_VECTOR_FLAG_NONE, out[i].flag);
            valid++;
        } else {
            TEST_ASSERT_NOT_EQUAL(DERIVED_VECTOR_FLAG_NONE, out[i].flag);
            TEST_ASSERT_EQUAL_FLOAT(0.0f, out[i].value);
        }
        if (i > 0u) {
            TEST_ASSERT_TRUE(out[i].time_ms >= out[i - 1u].time_ms);
        }
    }
    TEST_ASSERT_TRUE(valid > 0u);
}

#define CHECK_SCENARIO(name, NAME)                                                         \
    do {                                                                                   \
        TEST_ASSERT_EQUAL_UINT32(NAME##_INPUT_COUNT,                                       \
                                 sizeof(name##_inputs) / sizeof(name##_inputs[0]));        \
        TEST_ASSERT_EQUAL_UINT32(NAME##_OUTPUT_COUNT,                                      \
                                 sizeof(name##_outputs) / sizeof(name##_outputs[0]));      \
        TEST_ASSERT_EQUAL_UINT32(NAME##_TOL_COUNT, sizeof(name##_tol) / sizeof(name##_tol[0])); \
        check_inputs(name##_inputs, NAME##_INPUT_COUNT);                                   \
        check_outputs(name##_outputs, NAME##_OUTPUT_COUNT, name##_tol, NAME##_TOL_COUNT);  \
    } while (0)

static void test_idle_warmup_is_well_formed(void)
{
    CHECK_SCENARIO(derived_idle_warmup, DERIVED_IDLE_WARMUP);
}

static void test_accel_gears_is_well_formed(void)
{
    CHECK_SCENARIO(derived_accel_gears, DERIVED_ACCEL_GEARS);
}

static void test_stale_gap_is_well_formed(void)
{
    CHECK_SCENARIO(derived_stale_gap, DERIVED_STALE_GAP);
}

static void test_edge_values_is_well_formed(void)
{
    CHECK_SCENARIO(derived_edge_values, DERIVED_EDGE_VALUES);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_idle_warmup_is_well_formed);
    RUN_TEST(test_accel_gears_is_well_formed);
    RUN_TEST(test_stale_gap_is_well_formed);
    RUN_TEST(test_edge_values_is_well_formed);
    return UNITY_END();
}
