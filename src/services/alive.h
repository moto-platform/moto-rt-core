#ifndef SERVICES_ALIVE_H
#define SERVICES_ALIVE_H

/*
 * Alive counter of a supervised task (D-064 item 4, D-065 item 2): the task bumps it
 * after each complete cycle, a supervisor of another priority samples it and judges it
 * with services/alive_core.h. One 32-bit atomic (hal/hal_atomic.h), no shared lock, so
 * neither side can block the other.
 *
 *   supervised task (the EKF, D-065 PR 3): alive_bump(&c) at the end of every complete cycle
 *   supervisor (app/comms):               alive_monitor_stalled(&m, alive_count(&c), now, window)
 *
 * Exactly one task writes a counter. Static storage owned by the caller, no heap.
 */

#include "hal/hal_atomic.h"
#include "services/alive_core.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    hal_atomic_u32_t count;
} alive_counter_t;

/* Boot only, before the supervised task runs: count 0. */
void alive_init(alive_counter_t* c);

/* Supervised task only: one more complete cycle (release store; a single writer needs no
 * read-modify-write). Wraps after 2^32 bumps; the monitor looks for a change, not an
 * order, so the wrap is harmless. */
void alive_bump(alive_counter_t* c);

/* Any task: the current count (acquire). NULL reads 0. */
uint32_t alive_count(const alive_counter_t* c);

#ifdef __cplusplus
}
#endif

#endif /* SERVICES_ALIVE_H */
