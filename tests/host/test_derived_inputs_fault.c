/*
 * Fault paths of services/derived_inputs (D-070, safety-reviewer MINOR-2): the real service,
 * snapshot and vehicle_signals linked against a test double of hal/hal_atomic.h that can
 * hand back a corrupted middle word (index 3) once, like the hooked double of
 * test_snapshot_interleave. A producer fault stops the publishing (the write slot may be the
 * consumer's front); a consumer fault keeps the consumer on its old record.
 */
#include "hal/hal_atomic.h"
#include "services/derived_inputs.h"
#include "services/vehicle_signals.h"

#include <stdbool.h>
#include <unity.h>

/* ------------------------------------------------------- hal_atomic test double */

static const hal_atomic_u32_t* derived_word; /* the service's middle word, captured at init */
static bool capture;
static bool corrupt_exchange;
static bool corrupt_load;

#define BAD_WORD (3u | 0x4u) /* index 3 with the "new" flag */

uint32_t hal_atomic_u32_load(const hal_atomic_u32_t* a)
{
    if (corrupt_load && (a == derived_word)) {
        corrupt_load = false;
        return BAD_WORD;
    }
    return a->value;
}

void hal_atomic_u32_store(hal_atomic_u32_t* a, uint32_t v)
{
    if (capture) {
        derived_word = a;
    }
    a->value = v;
}

uint32_t hal_atomic_u32_exchange(hal_atomic_u32_t* a, uint32_t v)
{
    const uint32_t old = a->value;
    a->value = v;
    if (corrupt_exchange && (a == derived_word)) {
        corrupt_exchange = false;
        return BAD_WORD;
    }
    return old;
}

static void boot(void)
{
    vehicle_signals_init();
    capture = true; /* the first atomic store of the init is the snapshot's middle word */
    derived_inputs_init();
    capture = false;
    corrupt_exchange = false;
    corrupt_load = false;
}

void setUp(void) {}
void tearDown(void) {}

/* Must run first: the module is static and stays initialised once derived_inputs_init() ran. */
static void test_publish_before_init_is_counted(void)
{
    derived_inputs_record_t rec;
    TEST_ASSERT_EQUAL_UINT32(0u, derived_inputs_skipped_count());
    derived_inputs_publish(1u);
    derived_inputs_publish(2u);
    derived_inputs_publish(3u);
    TEST_ASSERT_EQUAL_UINT32(3u, derived_inputs_skipped_count());
    TEST_ASSERT_FALSE(derived_inputs_read(&rec));
    boot();
    TEST_ASSERT_EQUAL_UINT32(0u, derived_inputs_skipped_count());
}

static void test_a_producer_fault_stops_the_publishing(void)
{
    derived_inputs_record_t rec;
    boot();
    (void)vehicle_signals_write(VEHICLE_CL250_IDX_VEHICLE_SPEED, 60u, 60.0f, 100u);
    derived_inputs_publish(100u);
    TEST_ASSERT_TRUE(derived_inputs_read(&rec));
    TEST_ASSERT_EQUAL_UINT32(1u, rec.pass);

    corrupt_exchange = true;
    derived_inputs_publish(110u); /* the exchange hands back a bad word: the fault latches */
    TEST_ASSERT_TRUE(derived_inputs_producer_fault());
    TEST_ASSERT_FALSE(derived_inputs_consumer_fault());
    TEST_ASSERT_TRUE(derived_inputs_read(&rec)); /* the record of the latching publish was stored */
    TEST_ASSERT_EQUAL_UINT32(2u, rec.pass);

    derived_inputs_publish(120u); /* latched: no further write or exchange */
    derived_inputs_publish(130u);
    TEST_ASSERT_FALSE(derived_inputs_read(&rec));
    TEST_ASSERT_EQUAL_UINT32(2u, rec.pass);
    TEST_ASSERT_EQUAL_UINT32(0u, derived_inputs_skipped_count());
}

static void test_a_consumer_fault_keeps_the_old_record(void)
{
    derived_inputs_record_t rec;
    boot();
    derived_inputs_publish(100u);
    corrupt_load = true;
    TEST_ASSERT_FALSE(derived_inputs_read(&rec)); /* a bad middle word is never taken */
    TEST_ASSERT_TRUE(derived_inputs_consumer_fault());
    TEST_ASSERT_FALSE(derived_inputs_producer_fault());
    TEST_ASSERT_EQUAL_UINT32(0u, rec.pass); /* still the boot record */
    derived_inputs_publish(110u);
    TEST_ASSERT_FALSE(derived_inputs_read(&rec)); /* latched: it ages out by its stamps */
    TEST_ASSERT_EQUAL_UINT32(0u, rec.pass);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_publish_before_init_is_counted);
    RUN_TEST(test_a_producer_fault_stops_the_publishing);
    RUN_TEST(test_a_consumer_fault_keeps_the_old_record);
    return UNITY_END();
}
