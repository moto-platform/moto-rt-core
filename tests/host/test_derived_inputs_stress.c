/*
 * Two-thread stress test of the DID hand-over (D-070): the comms-task side (write
 * vehicle_signals, derived_inputs_publish) in one thread and the derived-task side
 * (derived_inputs_read) in another, sharing only the services/derived_inputs snapshot, as on
 * the H7. vehicle_signals is touched by the producer thread only (it is single-task).
 * Run under ASan + UBSan (host-tests) and ThreadSanitizer (host-tsan preset).
 *
 * A record is never torn: pass p writes every DID with value p and stamp p and publishes at
 * now = p, so every entry of a record must carry the record's pass counter. Also checked:
 * passes never go backwards and the last pass arrives. Unity asserts only in the main
 * thread; the workers count what they saw.
 */
#define _POSIX_C_SOURCE 200809L
#include "services/derived_inputs.h"
#include "services/tester_epoch.h"
#include "services/vehicle_signals.h"

#include <pthread.h>
#include <sched.h>
#include <stdatomic.h>
#include <string.h>
#include <unity.h>

#define PASSES (200000u)

static atomic_bool producer_done;

typedef struct {
    uint32_t reads;
    uint32_t fresh;
    uint32_t torn;
    uint32_t backwards;
    uint32_t last_pass;
    uint32_t publish_faults;
} consumer_result_t;

static consumer_result_t result;

void setUp(void) {}
void tearDown(void) {}

static void check(const derived_inputs_record_t* rec, consumer_result_t* r)
{
    if (rec->pass < r->last_pass) {
        r->backwards++;
    }
    r->last_pass = rec->pass;
    /* the epoch fields travel in the same record: counter = pass, open = pass is even */
    if ((rec->pass != 0u) && ((rec->epoch_counter != (uint16_t)rec->pass) ||
                              (rec->epoch_open != (uint8_t)((rec->pass % 2u) == 0u)))) {
        r->torn++;
    }
    for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
        const bool ok = (rec->pass == 0u)
                            ? (rec->did[i].state == (uint8_t)VEHICLE_SIGNAL_NONE)
                            : ((rec->did[i].state == (uint8_t)VEHICLE_SIGNAL_VALID) &&
                               (rec->did[i].value == (float)rec->pass) && (rec->did[i].stamp_ms == rec->pass) &&
                               (rec->published_ms == rec->pass));
        if (!ok) {
            r->torn++;
        }
    }
}

static void* producer(void* arg)
{
    (void)arg;
    for (uint32_t p = 1u; p <= PASSES; p++) {
        for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
            (void)vehicle_signals_write(i, p, (float)p, p);
        }
        tester_epoch_set((p % 2u) == 0u, (uint16_t)p);
        derived_inputs_publish(p);
        if ((p % 64u) == 0u) {
            (void)sched_yield();
        }
    }
    atomic_store(&producer_done, true);
    return NULL;
}

static void* consumer(void* arg)
{
    (void)arg;
    derived_inputs_record_t rec;
    bool finished = false;
    while (!finished) {
        finished = atomic_load(&producer_done);
        bool fresh = derived_inputs_read(&rec);
        while (fresh || finished) {
            result.reads++;
            if (fresh) {
                result.fresh++;
            }
            check(&rec, &result);
            fresh = derived_inputs_read(&rec);
            if (!fresh) {
                break;
            }
        }
        (void)sched_yield();
    }
    return NULL;
}

static void test_records_are_never_torn(void)
{
    pthread_t tp;
    pthread_t tc;
    vehicle_signals_init();
    derived_inputs_init();
    memset(&result, 0, sizeof result);
    atomic_store(&producer_done, false);
    TEST_ASSERT_EQUAL_INT(0, pthread_create(&tc, NULL, consumer, NULL));
    TEST_ASSERT_EQUAL_INT(0, pthread_create(&tp, NULL, producer, NULL));
    TEST_ASSERT_EQUAL_INT(0, pthread_join(tp, NULL));
    TEST_ASSERT_EQUAL_INT(0, pthread_join(tc, NULL));
    printf("derived_inputs stress: %u records read, %u fresh, last pass %u\n", (unsigned)result.reads,
           (unsigned)result.fresh, (unsigned)result.last_pass);
    TEST_ASSERT_EQUAL_UINT32(0u, result.torn);
    TEST_ASSERT_EQUAL_UINT32(0u, result.backwards);
    TEST_ASSERT_TRUE(result.fresh > 0u);
    TEST_ASSERT_EQUAL_UINT32(PASSES, result.last_pass);
    TEST_ASSERT_FALSE(derived_inputs_producer_fault());
    TEST_ASSERT_FALSE(derived_inputs_consumer_fault());
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_records_are_never_torn);
    return UNITY_END();
}
