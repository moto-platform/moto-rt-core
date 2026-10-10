#include "services/snapshot.h"

#include <stddef.h>

/* middle word: bits 0-1 the slot index, bit 2 set while the consumer has not taken it */
#define INDEX_MASK 0x3u
#define NEW_FLAG 0x4u

_Static_assert(SNAPSHOT_SLOTS == 3u, "the index swap needs exactly three slots");

static bool valid_index(uint32_t word)
{
    return (word & INDEX_MASK) < SNAPSHOT_SLOTS;
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: called by the EKF/comms setup (D-065 PR 3) and the tests
void snapshot_init(snapshot_t* s)
{
    if (s == NULL) {
        return;
    }
    s->front = 0u;
    s->back = 2u;
    s->producer_fault = false;
    s->consumer_fault = false;
    hal_atomic_u32_store(&s->middle, 1u);
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: called by the producer task (D-065 PR 3) and the tests
uint8_t snapshot_write_index(const snapshot_t* s)
{
    return (s == NULL) ? 0u : s->back;
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: called by the producer task (D-065 PR 3) and the tests
void snapshot_publish(snapshot_t* s)
{
    if ((s == NULL) || s->producer_fault) {
        return;
    }
    /* release: the slot's contents are visible before its index is */
    const uint32_t old = hal_atomic_u32_exchange(&s->middle, (uint32_t)s->back | NEW_FLAG);
    if (valid_index(old)) {
        s->back = (uint8_t)(old & INDEX_MASK);
    } else {
        s->producer_fault = true; /* keep back: never an index outside the slots */
    }
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: called by the consumer task (D-065 PR 3) and the tests
bool snapshot_read_index(snapshot_t* s, uint8_t* index)
{
    if ((s == NULL) || (index == NULL)) {
        return false;
    }
    bool fresh = false;
    if (!s->consumer_fault) {
        /* Only swap when there is something new: the middle slot without the flag is the
         * one this consumer released before, older than its front. A publish between this
         * load and the exchange only makes the exchange return a newer slot. */
        const uint32_t seen = hal_atomic_u32_load(&s->middle);
        if (!valid_index(seen)) {
            s->consumer_fault = true; /* not exchanged: the front stays ours */
        } else if ((seen & NEW_FLAG) != 0u) {
            const uint32_t old = hal_atomic_u32_exchange(&s->middle, (uint32_t)s->front);
            if (valid_index(old)) {
                s->front = (uint8_t)(old & INDEX_MASK);
                fresh = true;
            } else {
                s->consumer_fault = true; /* keep front: never an index outside the slots */
            }
        } else {
            /* nothing new: keep the front */
        }
    }
    *index = s->front;
    return fresh;
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: read by the producer task (D-065 PR 3) and the tests
bool snapshot_producer_fault(const snapshot_t* s)
{
    return (s == NULL) || s->producer_fault;
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: read by the consumer task (D-065 PR 3) and the tests
bool snapshot_consumer_fault(const snapshot_t* s)
{
    return (s == NULL) || s->consumer_fault;
}
