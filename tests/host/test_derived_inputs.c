/* services/derived_inputs: the DID copy from vehicle_signals into the snapshot (D-070). */
#include "services/derived_inputs.h"
#include "services/tester_epoch.h"
#include "services/vehicle_signals.h"

#include <string.h>
#include <unity.h>

void setUp(void) {}
void tearDown(void) {}

/* Runs first: before derived_inputs_init() nothing is published and the read is the boot
 * record (the module is static, so this test must precede the others). */
static void test_before_init_nothing_is_published(void)
{
    derived_inputs_record_t rec;
    memset(&rec, 0xAA, sizeof rec);
    vehicle_signals_init();
    (void)vehicle_signals_write(1u, 100u, 50.0f, 10u);
    derived_inputs_publish(20u);
    TEST_ASSERT_FALSE(derived_inputs_read(&rec));
    TEST_ASSERT_FALSE(derived_inputs_read(NULL));
}

static void test_boot_record_is_invalid(void)
{
    derived_inputs_record_t rec;
    derived_inputs_init();
    TEST_ASSERT_FALSE(derived_inputs_read(&rec));
    TEST_ASSERT_EQUAL_UINT32(0u, rec.pass);
    for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
        TEST_ASSERT_EQUAL_UINT8((uint8_t)VEHICLE_SIGNAL_NONE, rec.did[i].state);
    }
    TEST_ASSERT_FALSE(derived_inputs_producer_fault());
    TEST_ASSERT_FALSE(derived_inputs_consumer_fault());
}

static void test_publish_copies_value_stamp_and_state(void)
{
    derived_inputs_record_t rec;
    derived_inputs_init();
    vehicle_signals_init();
    (void)vehicle_signals_write(VEHICLE_CL250_IDX_VEHICLE_SPEED, 60u, 60.0f, 1000u);
    derived_inputs_publish(1050u);
    TEST_ASSERT_TRUE(derived_inputs_read(&rec));
    TEST_ASSERT_EQUAL_UINT32(1u, rec.pass);
    TEST_ASSERT_EQUAL_UINT32(1050u, rec.published_ms);
    TEST_ASSERT_EQUAL_FLOAT(60.0f, rec.did[VEHICLE_CL250_IDX_VEHICLE_SPEED].value);
    TEST_ASSERT_EQUAL_UINT32(1000u, rec.did[VEHICLE_CL250_IDX_VEHICLE_SPEED].stamp_ms);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)VEHICLE_SIGNAL_VALID, rec.did[VEHICLE_CL250_IDX_VEHICLE_SPEED].state);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)VEHICLE_SIGNAL_NONE, rec.did[VEHICLE_CL250_IDX_ENGINE_SPEED].state);
    /* nothing new: false, and the caller's previous record is what the slot still holds */
    TEST_ASSERT_FALSE(derived_inputs_read(&rec));
    TEST_ASSERT_EQUAL_UINT32(1u, rec.pass);
}

static void test_a_sample_that_ages_out_is_published_as_stale(void)
{
    derived_inputs_record_t rec;
    derived_inputs_init();
    vehicle_signals_init();
    (void)vehicle_signals_write(VEHICLE_CL250_IDX_VEHICLE_SPEED, 60u, 60.0f, 1000u);
    derived_inputs_publish(1000u + 10000u); /* far beyond the DID's stale_after_ms */
    TEST_ASSERT_TRUE(derived_inputs_read(&rec));
    TEST_ASSERT_EQUAL_UINT8((uint8_t)VEHICLE_SIGNAL_STALE, rec.did[VEHICLE_CL250_IDX_VEHICLE_SPEED].state);
    TEST_ASSERT_EQUAL_UINT32(1000u, rec.did[VEHICLE_CL250_IDX_VEHICLE_SPEED].stamp_ms); /* its own stamp */
}

static void test_the_epoch_travels_in_the_same_record(void)
{
    derived_inputs_record_t rec;
    derived_inputs_init();
    vehicle_signals_init();
    TEST_ASSERT_FALSE(tester_epoch_get().open);
    TEST_ASSERT_EQUAL_UINT16(0u, tester_epoch_get().counter);
    tester_epoch_set(true, 41u);
    (void)vehicle_signals_write(VEHICLE_CL250_IDX_COOLANT_TEMP, 100u, 60.0f, 500u);
    derived_inputs_publish(520u);
    TEST_ASSERT_TRUE(derived_inputs_read(&rec));
    TEST_ASSERT_EQUAL_UINT8(1u, rec.epoch_open);
    TEST_ASSERT_EQUAL_UINT16(41u, rec.epoch_counter);
    tester_epoch_set(false, 41u);
    derived_inputs_publish(540u);
    TEST_ASSERT_TRUE(derived_inputs_read(&rec));
    TEST_ASSERT_EQUAL_UINT8(0u, rec.epoch_open);
    TEST_ASSERT_EQUAL_UINT16(41u, rec.epoch_counter);
    tester_epoch_set(false, 0u);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_before_init_nothing_is_published);
    RUN_TEST(test_boot_record_is_invalid);
    RUN_TEST(test_publish_copies_value_stamp_and_state);
    RUN_TEST(test_a_sample_that_ages_out_is_published_as_stale);
    RUN_TEST(test_the_epoch_travels_in_the_same_record);
    return UNITY_END();
}
