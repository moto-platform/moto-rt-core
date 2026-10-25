/* The derived glue (D-070): vehicle_signals -> derived_inputs -> core, as the tasks run. */
#include "features/derived/derived.h"
#include "services/derived_inputs.h"
#include "services/tester_epoch.h"
#include "services/vehicle_signals.h"

#include <unity.h>

void setUp(void)
{
    vehicle_signals_init();
    tester_epoch_set(false, 0u);
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
    TEST_ASSERT_TRUE(derived_get(VEHICLE_CL250_DERIVED_WHEEL_SLIP, &o));
    TEST_ASSERT_EQUAL_INT(DERIVED_NOT_COMPUTED, o.status);
    TEST_ASSERT_EQUAL_UINT32(28u + 12u, derived_core()->computed_count); /* class A + the class-B scalars */
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

static void test_a_ride_follows_the_published_epoch(void)
{
    derived_out_t o;
    tester_epoch_set(true, 1u);
    (void)vehicle_signals_write(VEHICLE_CL250_IDX_COOLANT_TEMP, 100u, 60.0f, 100u);
    derived_inputs_publish(100u);
    derived_step(100u);
    TEST_ASSERT_TRUE(derived_ride_get(DERIVED_RIDE_CURRENT, VEHICLE_CL250_DERIVED_RIDE_START_COOLANT_TEMP, &o));
    TEST_ASSERT_EQUAL_INT(DERIVED_VALID, o.status);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-4f, 60.0f, o.value);
    TEST_ASSERT_TRUE(derived_ride_get(DERIVED_RIDE_PREVIOUS, VEHICLE_CL250_DERIVED_RIDE_START_COOLANT_TEMP, &o));
    TEST_ASSERT_EQUAL_INT(DERIVED_NO_DATA, o.status);
    /* the ECU goes absent: the epoch ends and the ride closes */
    tester_epoch_set(false, 1u);
    derived_inputs_publish(200u);
    derived_step(200u);
    TEST_ASSERT_TRUE(derived_ride_get(DERIVED_RIDE_CURRENT, VEHICLE_CL250_DERIVED_RIDE_START_COOLANT_TEMP, &o));
    TEST_ASSERT_EQUAL_INT(DERIVED_NO_DATA, o.status);
    TEST_ASSERT_TRUE(derived_ride_get(DERIVED_RIDE_PREVIOUS, VEHICLE_CL250_DERIVED_RIDE_END_COOLANT_TEMP, &o));
    TEST_ASSERT_EQUAL_INT(DERIVED_VALID, o.status);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-4f, 60.0f, o.value);
    TEST_ASSERT_FALSE(derived_ride_get(DERIVED_RIDE_PREVIOUS, VEHICLE_CL250_DERIVED_COUNT, &o));
    TEST_ASSERT_FALSE(derived_ride_get(DERIVED_RIDE_PREVIOUS, 0u, NULL));
    tester_epoch_set(false, 0u);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_nothing_published_means_no_data);
    RUN_TEST(test_a_published_sample_reaches_the_outputs);
    RUN_TEST(test_derived_get_refuses_bad_arguments);
    RUN_TEST(test_a_ride_follows_the_published_epoch);
    return UNITY_END();
}
