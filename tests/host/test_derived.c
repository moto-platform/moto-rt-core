/* The derived glue (D-070): vehicle_signals -> derived_inputs -> core, as the tasks run. */
#include "features/derived/derived.h"
#include "services/derived_inputs.h"
#include "services/vehicle_signals.h"

#include <unity.h>

void setUp(void)
{
    vehicle_signals_init();
    derived_inputs_init();
    derived_init();
}
void tearDown(void) {}

static void test_nothing_published_means_no_data(void)
{
    derived_step(0u);
    derived_out_t o;
    TEST_ASSERT_TRUE(derived_get(VEHICLE_CL250_DERIVED_AIR_FUEL_RATIO, &o));
    TEST_ASSERT_EQUAL_INT(DERIVED_NO_DATA, o.status);
    TEST_ASSERT_TRUE(derived_get(VEHICLE_CL250_DERIVED_RIDE_DISTANCE, &o));
    TEST_ASSERT_EQUAL_INT(DERIVED_NOT_COMPUTED, o.status);
    TEST_ASSERT_EQUAL_UINT32(28u, derived_core()->computed_count);
}

static void test_a_published_sample_reaches_the_outputs(void)
{
    (void)vehicle_signals_write(VEHICLE_CL250_IDX_COMMANDED_EQUIV_RATIO, 32768u, 1.0f, 100u);
    derived_inputs_publish(100u);
    derived_step(100u);
    derived_out_t o;
    TEST_ASSERT_TRUE(derived_get(VEHICLE_CL250_DERIVED_AIR_FUEL_RATIO, &o));
    TEST_ASSERT_EQUAL_INT(DERIVED_VALID, o.status);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-4f, 14.7f, o.value);
    TEST_ASSERT_EQUAL_UINT32(100u, o.stamp_ms);
    /* the comms task stops publishing: the sample ages out by its own stamp */
    derived_step(100u + 6000u + 2000u);
    TEST_ASSERT_TRUE(derived_get(VEHICLE_CL250_DERIVED_AIR_FUEL_RATIO, &o));
    TEST_ASSERT_EQUAL_INT(DERIVED_INVALID, o.status);
    TEST_ASSERT_EQUAL_INT(DERIVED_FLAG_STALE_INPUT, o.flag);
}

static void test_derived_get_refuses_bad_arguments(void)
{
    derived_out_t o;
    TEST_ASSERT_FALSE(derived_get(VEHICLE_CL250_DERIVED_COUNT, &o));
    TEST_ASSERT_FALSE(derived_get(0u, NULL));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_nothing_published_means_no_data);
    RUN_TEST(test_a_published_sample_reaches_the_outputs);
    RUN_TEST(test_derived_get_refuses_bad_arguments);
    return UNITY_END();
}
