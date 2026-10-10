/*
 * Deterministic interleaving tests of services/snapshot (D-065 item 2, safety-reviewer
 * MINOR-4, ISO 26262-6 §9): the real snapshot.c, linked against a single-threaded
 * test double of hal/hal_atomic.h that calls a hook after every atomic operation. The
 * hook runs the other side at exactly the points where a preemption on the single-core
 * H7 can change the outcome:
 *   - the consumer between its load and its exchange (a publish lands in between),
 *   - the producer between its exchange and taking the old middle as its back slot.
 * Every other step of either side is local to it. All sequences of up to SEQ_LEN
 * operations over the five operation shapes below are enumerated, and after each
 * operation the protocol invariants are checked against an oracle.
 */
#include "services/snapshot.h"

#include <stdbool.h>
#include <stdint.h>
#include <unity.h>

/* ------------------------------------------------------- hal_atomic test double */

typedef enum { OP_LOAD, OP_STORE, OP_EXCHANGE } atomic_op_t;

static void (*hook)(atomic_op_t op);

static void after(atomic_op_t op)
{
    void (*h)(atomic_op_t) = hook;
    if (h != NULL) {
        hook = NULL; /* the injected side runs without hooks */
        h(op);
    }
}

uint32_t hal_atomic_u32_load(const hal_atomic_u32_t* a)
{
    const uint32_t v = a->value;
    after(OP_LOAD);
    return v;
}

void hal_atomic_u32_store(hal_atomic_u32_t* a, uint32_t v)
{
    a->value = v;
    after(OP_STORE);
}

uint32_t hal_atomic_u32_exchange(hal_atomic_u32_t* a, uint32_t v)
{
    const uint32_t old = a->value;
    a->value = v;
    after(OP_EXCHANGE);
    return old;
}

/* ------------------------------------------------------------------ the model */

#define SEQ_LEN 6u
#define SHAPES 5u

typedef struct {
    uint32_t seq; /* 0: the initial record */
} rec_t;

static snapshot_t s;
static rec_t slots[SNAPSHOT_SLOTS];
static uint32_t next_seq;    /* the seq of the next publish */
static uint32_t latest;      /* the seq of the last completed publish exchange */
static uint32_t held;        /* the seq the consumer holds (its front) */
static uint32_t violations;  /* oracle failures, counted, asserted in the test */

static void check(bool ok)
{
    violations += ok ? 0u : 1u;
}

static void publish(void)
{
    const uint8_t back = snapshot_write_index(&s);
    check(back != s.front); /* never the consumer's slot */
    slots[back].seq = next_seq;
    const uint32_t seq = next_seq;
    next_seq++;
    snapshot_publish(&s);
    latest = seq;
}

static void consume(void)
{
    check(slots[s.front].seq == held); /* the held record was never overwritten */
    uint8_t i = 0xFFu;
    const bool fresh = snapshot_read_index(&s, &i);
    check(i < SNAPSHOT_SLOTS);
    if (fresh) {
        check(slots[i].seq == latest); /* the newest record, never an older one */
        check(slots[i].seq > held);    /* "new" means newer */
    } else {
        check(slots[i].seq == held);
    }
    held = slots[i].seq;
}

/* hooks: the other side, injected at its preemption point */
static uint32_t inject_publishes;

static void publishes_after_consumer_load(atomic_op_t op)
{
    if (op == OP_LOAD) {
        for (uint32_t k = 0u; k < inject_publishes; k++) {
            publish();
        }
    } else {
        hook = publishes_after_consumer_load; /* not the load yet: stay armed */
    }
}

static void read_after_producer_exchange(atomic_op_t op)
{
    if (op == OP_EXCHANGE) {
        /* the producer's exchange is done; its back is not updated yet */
        uint8_t i = 0xFFu;
        const bool fresh = snapshot_read_index(&s, &i);
        check(i < SNAPSHOT_SLOTS);
        check(fresh);
        check(slots[i].seq == (next_seq - 1u)); /* the record being published */
        held = slots[i].seq;
    } else {
        hook = read_after_producer_exchange;
    }
}

static void distinct(void)
{
    const uint32_t mid = s.middle.value & 0x3u;
    check((s.front != s.back) && (s.front != mid) && (s.back != mid));
    check(!s.producer_fault && !s.consumer_fault);
}

/* The five operation shapes: plain publish, plain read, a read preempted after its load
 * by one or two publishes, and a publish preempted after its exchange by a read. */
static void run_shape(uint32_t shape)
{
    switch (shape) {
    case 0u:
        publish();
        break;
    case 1u:
        consume();
        break;
    case 2u:
    case 3u:
        inject_publishes = shape - 1u;
        hook = publishes_after_consumer_load;
        consume();
        hook = NULL;
        break;
    default: {
        const uint8_t back = snapshot_write_index(&s);
        slots[back].seq = next_seq;
        const uint32_t seq = next_seq;
        next_seq++;
        hook = read_after_producer_exchange;
        snapshot_publish(&s);
        hook = NULL;
        latest = seq;
        break;
    }
    }
    distinct();
}

static void reset(void)
{
    hook = NULL;
    snapshot_init(&s);
    for (uint32_t i = 0u; i < SNAPSHOT_SLOTS; i++) {
        slots[i].seq = 0u;
    }
    next_seq = 1u;
    latest = 0u;
    held = 0u;
}

void setUp(void)
{
    reset();
    violations = 0u;
}

void tearDown(void) {}

static void test_every_interleaving_up_to_six_operations_keeps_the_protocol(void)
{
    uint32_t total = 1u;
    for (uint32_t k = 0u; k < SEQ_LEN; k++) {
        total *= SHAPES;
    }
    for (uint32_t code = 0u; code < total; code++) {
        reset();
        uint32_t c = code;
        for (uint32_t k = 0u; k < SEQ_LEN; k++) {
            run_shape(c % SHAPES);
            c /= SHAPES;
        }
        consume(); /* the last published record always arrives */
        check(held == latest);
    }
    TEST_ASSERT_EQUAL_UINT32(0u, violations);
}

static void test_a_publish_between_load_and_exchange_hands_over_the_newer_record(void)
{
    publish();             /* seq 1, new */
    inject_publishes = 1u; /* seq 2 lands between the consumer's load and exchange */
    hook = publishes_after_consumer_load;
    uint8_t i = 0xFFu;
    TEST_ASSERT_TRUE(snapshot_read_index(&s, &i));
    TEST_ASSERT_EQUAL_UINT32(2u, slots[i].seq);
    TEST_ASSERT_EQUAL_UINT32(0u, violations);
}

/* safety-reviewer MINOR-3: a word corrupted after the consumer's load is caught by the
 * second check, on the exchange's result. */
static void corrupt_after_load(atomic_op_t op)
{
    if (op == OP_LOAD) {
        s.middle.value = 3u | 0x4u;
    } else {
        hook = corrupt_after_load;
    }
}

static void test_a_middle_corrupted_between_load_and_exchange_is_never_used(void)
{
    publish();
    const uint8_t front = s.front;
    hook = corrupt_after_load;
    uint8_t i = 0xFFu;
    TEST_ASSERT_FALSE(snapshot_read_index(&s, &i));
    TEST_ASSERT_EQUAL_UINT8(front, i);
    TEST_ASSERT_TRUE(snapshot_consumer_fault(&s));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_every_interleaving_up_to_six_operations_keeps_the_protocol);
    RUN_TEST(test_a_publish_between_load_and_exchange_hands_over_the_newer_record);
    RUN_TEST(test_a_middle_corrupted_between_load_and_exchange_is_never_used);
    return UNITY_END();
}
