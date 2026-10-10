/*
 * Two-thread stress test of the lock-free exchange (D-065 item 2, E-17): a producer and a
 * consumer thread on services/snapshot, and a bumping and a sampling thread on
 * services/alive, all on the host atomics (hal/host/hal_atomic_host.c). Run under ASan +
 * UBSan (host-tests) and ThreadSanitizer (host-tsan preset): TSan reports any slot the
 * two sides touch at the same time, the checks below report a torn or out-of-order record.
 *
 * Unity asserts only in the main thread; the workers count what they saw, and the main
 * thread checks it after pthread_join(). No clock is used.
 */
#include "services/alive.h"
#include "services/snapshot.h"

#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <unity.h>

#define PUBLISHES 300000u
#define PAYLOAD_WORDS 15u
#define BUMPS 300000u

typedef struct {
    uint32_t seq;
    uint32_t payload[PAYLOAD_WORDS]; /* every word derived from seq: a torn record shows */
} rec_t;

static snapshot_t snap;
static rec_t slots[SNAPSHOT_SLOTS];

typedef struct {
    uint32_t reads;
    uint32_t fresh_reads;
    uint32_t torn;        /* a record whose words do not all belong to its seq */
    uint32_t backwards;   /* a seq lower than one read before */
    uint32_t stale_fresh; /* "new" but not newer than the previous record */
    uint32_t last_seq;
} consumer_result_t;

static consumer_result_t cres;

static uint32_t word(uint32_t seq, uint32_t k)
{
    return (seq * 2654435761u) ^ (k * 40503u);
}

static void* producer(void* arg)
{
    (void)arg;
    for (uint32_t seq = 1u; seq <= PUBLISHES; seq++) {
        rec_t* r = &slots[snapshot_write_index(&snap)];
        for (uint32_t k = 0u; k < PAYLOAD_WORDS; k++) {
            r->payload[k] = word(seq, k);
        }
        r->seq = seq;
        snapshot_publish(&snap);
    }
    return NULL;
}

static void* consumer(void* arg)
{
    (void)arg;
    uint32_t prev = 0u;
    while (prev < PUBLISHES) {
        uint8_t i = 0u;
        const bool fresh = snapshot_read_index(&snap, &i);
        const rec_t* r = &slots[i];
        const uint32_t seq = r->seq;
        cres.reads++;
        cres.fresh_reads += fresh ? 1u : 0u;
        if (seq != 0u) {
            for (uint32_t k = 0u; k < PAYLOAD_WORDS; k++) {
                cres.torn += (r->payload[k] != word(seq, k)) ? 1u : 0u;
            }
        }
        cres.backwards += (seq < prev) ? 1u : 0u;
        cres.stale_fresh += (fresh && (seq <= prev)) ? 1u : 0u;
        prev = (seq > prev) ? seq : prev;
    }
    cres.last_seq = prev;
    return NULL;
}

static alive_counter_t alive;

typedef struct {
    uint32_t samples;
    uint32_t backwards; /* a count lower than one sampled before (single writer: never) */
    uint32_t last;
} sampler_result_t;

static sampler_result_t sres;

static void* bumper(void* arg)
{
    (void)arg;
    for (uint32_t k = 0u; k < BUMPS; k++) {
        alive_bump(&alive);
    }
    return NULL;
}

static void* sampler(void* arg)
{
    (void)arg;
    uint32_t prev = 0u;
    while (prev < BUMPS) {
        const uint32_t c = alive_count(&alive);
        sres.samples++;
        sres.backwards += (c < prev) ? 1u : 0u;
        prev = (c > prev) ? c : prev;
    }
    sres.last = prev;
    return NULL;
}

void setUp(void) {}

void tearDown(void) {}

static void test_the_consumer_never_sees_a_torn_or_older_record(void)
{
    snapshot_init(&snap);
    for (uint32_t i = 0u; i < SNAPSHOT_SLOTS; i++) {
        slots[i].seq = 0u; /* the initial, safe record */
    }
    pthread_t p;
    pthread_t c;
    TEST_ASSERT_EQUAL_INT(0, pthread_create(&c, NULL, consumer, NULL));
    TEST_ASSERT_EQUAL_INT(0, pthread_create(&p, NULL, producer, NULL));
    TEST_ASSERT_EQUAL_INT(0, pthread_join(p, NULL));
    TEST_ASSERT_EQUAL_INT(0, pthread_join(c, NULL));

    TEST_ASSERT_EQUAL_UINT32(0u, cres.torn);
    TEST_ASSERT_EQUAL_UINT32(0u, cres.backwards);
    TEST_ASSERT_EQUAL_UINT32(0u, cres.stale_fresh);
    TEST_ASSERT_EQUAL_UINT32(PUBLISHES, cres.last_seq); /* the last record always arrives */
    TEST_ASSERT_GREATER_THAN_UINT32(0u, cres.fresh_reads);
    printf("snapshot stress: %u reads, %u with a new record, of %u publishes\n", (unsigned)cres.reads,
           (unsigned)cres.fresh_reads, (unsigned)PUBLISHES);
}

static void test_a_sampler_sees_the_alive_count_only_grow(void)
{
    alive_init(&alive);
    pthread_t b;
    pthread_t s;
    TEST_ASSERT_EQUAL_INT(0, pthread_create(&s, NULL, sampler, NULL));
    TEST_ASSERT_EQUAL_INT(0, pthread_create(&b, NULL, bumper, NULL));
    TEST_ASSERT_EQUAL_INT(0, pthread_join(b, NULL));
    TEST_ASSERT_EQUAL_INT(0, pthread_join(s, NULL));

    TEST_ASSERT_EQUAL_UINT32(0u, sres.backwards);
    TEST_ASSERT_EQUAL_UINT32(BUMPS, sres.last);
    TEST_ASSERT_EQUAL_UINT32(BUMPS, alive_count(&alive));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_the_consumer_never_sees_a_torn_or_older_record);
    RUN_TEST(test_a_sampler_sees_the_alive_count_only_grow);
    return UNITY_END();
}
