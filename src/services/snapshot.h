#ifndef SERVICES_SNAPSHOT_H
#define SERVICES_SNAPSHOT_H

/*
 * Single-producer / single-consumer triple buffer (D-065 item 2): hands the newest value
 * of a record from one task to another of a different priority, without a lock and
 * without a retry on either side (never a seqlock: a higher-priority reader that retries
 * while it preempts the writer would livelock, E-17 BLOCKER-2). Each call is bounded: no
 * loop here, and the atomic exchange is bounded by the HAL port (hal/README.md).
 *
 * Generic and index-only: the caller owns three slots of its own record type
 * (`T slots[SNAPSHOT_SLOTS]`) and this service only hands out slot indices. At any time
 * the producer owns one slot (back), the consumer owns one (front), and the third
 * (middle) is the last published one, exchanged atomically through hal/hal_atomic.h.
 *
 *   producer: i = snapshot_write_index(s); fill slots[i]; snapshot_publish(s);
 *   consumer: (void)snapshot_read_index(s, &i); read slots[i];
 *
 * The consumer always gets the newest complete record, or keeps its previous one when
 * nothing new was published; it never sees a half-written record. The producer makes one
 * exchange per publish; the consumer one load, plus one exchange when there is something
 * new.
 *
 * Rules: exactly one producer task and one consumer task per snapshot; the producer
 * writes only slots[snapshot_write_index()], the consumer reads only the index it got;
 * initialise the snapshot and all three slots to a safe value (e.g. INVALID) before
 * either task starts. Records carry their own stamp/state: the snapshot says "newest",
 * not "fresh".
 */

#include "hal/hal_atomic.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SNAPSHOT_SLOTS 3u

typedef struct {
    hal_atomic_u32_t middle; /* index of the last published slot, plus a "new" flag */
    uint8_t back;            /* producer only */
    uint8_t front;           /* consumer only */
    bool producer_fault;     /* producer only: latched, see snapshot_publish() */
    bool consumer_fault;     /* consumer only: latched, see snapshot_read_index() */
} snapshot_t;

/* Boot only, before the producer and the consumer run: front 0, middle 1, back 2. */
void snapshot_init(snapshot_t* s);

/* Producer: the slot to fill next. Stable until snapshot_publish(). */
uint8_t snapshot_write_index(const snapshot_t* s);

/* Producer: publishes the filled slot (release) and takes the old middle as its next
 * back slot. A record the consumer has not read yet is replaced, never queued.
 * Defensive (safety-reviewer MINOR-3, MISRA Dir 4.1): an old middle index outside the
 * three slots (a corrupted word) is never used: the producer keeps its back slot and
 * latches producer_fault, after which it publishes nothing more (the consumer's records
 * then age out by their own stamps). */
void snapshot_publish(snapshot_t* s);

/* Consumer: *index receives the slot to read (acquire). True if it is a record that was
 * published since the last call, false if *index is the previous front (nothing new, or
 * nothing published yet: then it is the caller's initial slot 0). *index is always one of
 * the three slots. Defensive (MINOR-3): a middle index outside them is never taken; the
 * consumer keeps its front, latches consumer_fault and from then on returns false (its
 * record ages out by its stamp). A corrupted word means a RAM fault beyond this design
 * (H7 RAM ECC); the latch keeps every index in bounds, it cannot repair the exchange. */
bool snapshot_read_index(snapshot_t* s, uint8_t* index);

/* Producer task only: true once snapshot_publish() latched a fault. */
bool snapshot_producer_fault(const snapshot_t* s);

/* Consumer task only: true once snapshot_read_index() latched a fault. */
bool snapshot_consumer_fault(const snapshot_t* s);

#ifdef __cplusplus
}
#endif

#endif /* SERVICES_SNAPSHOT_H */
