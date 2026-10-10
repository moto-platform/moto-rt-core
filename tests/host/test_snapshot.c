/*
 * L0 tests for the SPSC triple buffer (services/snapshot, D-065 item 2), single-threaded:
 * the index protocol. The two-thread stress test is test_snapshot_stress.c.
 */
#include "services/snapshot.h"

#include <stdint.h>
#include <unity.h>

typedef struct {
    uint32_t seq; /* 0: the initial (safe) record */
} rec_t;

static snapshot_t s;
static rec_t slots[SNAPSHOT_SLOTS];

void setUp(void)
{
    snapshot_init(&s);
    for (uint32_t i = 0u; i < SNAPSHOT_SLOTS; i++) {
        slots[i].seq = 0u;
    }
}

void tearDown(void) {}

static void publish(uint32_t seq)
{
    slots[snapshot_write_index(&s)].seq = seq;
    snapshot_publish(&s);
}

static uint32_t read_seq(bool* fresh)
{
    uint8_t i = 0xFFu;
    *fresh = snapshot_read_index(&s, &i);
    TEST_ASSERT_LESS_THAN_UINT8(SNAPSHOT_SLOTS, i); /* always a valid slot */
    return slots[i].seq;
}

static void test_before_any_publish_the_reader_gets_the_initial_slot_and_no_news(void)
{
    bool fresh = true;
    TEST_ASSERT_EQUAL_UINT32(0u, read_seq(&fresh));
    TEST_ASSERT_FALSE(fresh);
}

static void test_the_reader_gets_the_newest_record_once_then_keeps_it(void)
{
    bool fresh = false;
    publish(1u);
    TEST_ASSERT_EQUAL_UINT32(1u, read_seq(&fresh));
    TEST_ASSERT_TRUE(fresh);
    TEST_ASSERT_EQUAL_UINT32(1u, read_seq(&fresh)); /* nothing new: the same record */
    TEST_ASSERT_FALSE(fresh);
}

static void test_unread_records_are_replaced_never_queued(void)
{
    bool fresh = false;
    publish(1u);
    publish(2u);
    publish(3u);
    TEST_ASSERT_EQUAL_UINT32(3u, read_seq(&fresh));
    TEST_ASSERT_TRUE(fresh);
    TEST_ASSERT_EQUAL_UINT32(3u, read_seq(&fresh));
    TEST_ASSERT_FALSE(fresh);
}

static void test_the_writer_never_gets_the_slot_the_reader_holds(void)
{
    for (uint32_t seq = 1u; seq < 50u; seq++) {
        publish(seq);
        bool fresh = false;
        uint8_t front = 0u;
        fresh = snapshot_read_index(&s, &front);
        TEST_ASSERT_TRUE(fresh);
        TEST_ASSERT_EQUAL_UINT32(seq, slots[front].seq);
        /* the producer's next slot is never the consumer's front */
        TEST_ASSERT_NOT_EQUAL_UINT8(front, snapshot_write_index(&s));
        if ((seq % 3u) == 0u) {
            publish(seq + 1000u); /* an unread record in the middle */
            TEST_ASSERT_NOT_EQUAL_UINT8(front, snapshot_write_index(&s));
            TEST_ASSERT_EQUAL_UINT32(seq, slots[front].seq); /* the held record is untouched */
            (void)snapshot_read_index(&s, &front);
            TEST_ASSERT_EQUAL_UINT32(seq + 1000u, slots[front].seq);
        }
    }
}

static void test_the_three_indices_stay_distinct(void)
{
    for (uint32_t k = 0u; k < 20u; k++) {
        publish(k + 1u);
        if ((k % 2u) == 0u) {
            uint8_t i = 0u;
            (void)snapshot_read_index(&s, &i);
        }
        const uint32_t mid = hal_atomic_u32_load(&s.middle) & 0x3u;
        TEST_ASSERT_NOT_EQUAL_UINT32(s.front, s.back);
        TEST_ASSERT_NOT_EQUAL_UINT32(s.front, mid);
        TEST_ASSERT_NOT_EQUAL_UINT32(s.back, mid);
    }
}

/* safety-reviewer MINOR-3: a corrupted middle word never yields an index outside the
 * slots; each side latches its fault and stops exchanging. */
static void test_a_corrupted_middle_index_is_never_used(void)
{
    publish(1u);
    bool fresh = false;
    TEST_ASSERT_EQUAL_UINT32(1u, read_seq(&fresh));
    const uint8_t front = s.front;

    hal_atomic_u32_store(&s.middle, 3u | 0x4u); /* index 3 with the "new" flag */
    uint8_t i = 0xFFu;
    TEST_ASSERT_FALSE(snapshot_read_index(&s, &i));
    TEST_ASSERT_EQUAL_UINT8(front, i);
    TEST_ASSERT_TRUE(snapshot_consumer_fault(&s));
    TEST_ASSERT_EQUAL_UINT32(3u | 0x4u, hal_atomic_u32_load(&s.middle)); /* not exchanged */

    /* the producer swaps the bad word out but keeps its own back slot */
    const uint8_t back = snapshot_write_index(&s);
    snapshot_publish(&s);
    TEST_ASSERT_TRUE(snapshot_producer_fault(&s));
    TEST_ASSERT_EQUAL_UINT8(back, snapshot_write_index(&s));
    snapshot_publish(&s); /* latched: no further exchange */
    TEST_ASSERT_EQUAL_UINT32((uint32_t)back | 0x4u, hal_atomic_u32_load(&s.middle));

    /* the latched consumer keeps its front even with a valid new record */
    TEST_ASSERT_FALSE(snapshot_read_index(&s, &i));
    TEST_ASSERT_EQUAL_UINT8(front, i);
}

static void test_null_arguments_are_harmless(void)
{
    uint8_t i = 0u;
    snapshot_init(NULL);
    snapshot_publish(NULL);
    TEST_ASSERT_EQUAL_UINT8(0u, snapshot_write_index(NULL));
    TEST_ASSERT_FALSE(snapshot_read_index(NULL, &i));
    TEST_ASSERT_TRUE(snapshot_producer_fault(NULL));
    TEST_ASSERT_TRUE(snapshot_consumer_fault(NULL));
    TEST_ASSERT_FALSE(snapshot_producer_fault(&s));
    TEST_ASSERT_FALSE(snapshot_consumer_fault(&s));
    publish(5u);
    TEST_ASSERT_FALSE(snapshot_read_index(&s, NULL)); /* does not consume the news */
    TEST_ASSERT_TRUE(snapshot_read_index(&s, &i));
    TEST_ASSERT_EQUAL_UINT32(5u, slots[i].seq);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_before_any_publish_the_reader_gets_the_initial_slot_and_no_news);
    RUN_TEST(test_the_reader_gets_the_newest_record_once_then_keeps_it);
    RUN_TEST(test_unread_records_are_replaced_never_queued);
    RUN_TEST(test_the_writer_never_gets_the_slot_the_reader_holds);
    RUN_TEST(test_the_three_indices_stay_distinct);
    RUN_TEST(test_a_corrupted_middle_index_is_never_used);
    RUN_TEST(test_null_arguments_are_harmless);
    return UNITY_END();
}
