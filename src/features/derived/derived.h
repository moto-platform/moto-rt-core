#ifndef FEATURES_DERIVED_DERIVED_H
#define FEATURES_DERIVED_DERIVED_H

/*
 * Derived-signal glue, derived-task side (D-070): the module's public header together with
 * derived_core.h. QM information (D-070 item 5): the task runs below the EKF and below the
 * comms task, holds no lock, never blocks, prints nothing and allocates nothing. It reads
 * the DID samples only through services/derived_inputs (the snapshot the comms task
 * publishes); it never touches services/vehicle_signals, which is single-task.
 *
 * derived_step() and derived_get() are called from the derived task only (the one consumer
 * call site of the snapshot, and the core is not shared). A future logger runs in this task
 * or gets its own snapshot of the output table; it never reads from another task.
 *
 * Module-static state, like services/vehicle_signals.
 */

#include "features/derived/derived_core.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Boot only, before the derived task runs and after derived_inputs_init(). */
void derived_init(void);

/* Derived task, once per cycle (a few ms to tens of ms): takes the newest DID record (or
 * keeps the previous one), maps it to the core's inputs and steps the core at now_ms. */
void derived_step(uint32_t now_ms);

/* Derived task only: the last output of signal sig (a VEHICLE_CL250_DERIVED_*). False if
 * sig is out of range or out is NULL. Not-computed signals read NOT_COMPUTED. */
bool derived_get(uint32_t sig, derived_out_t* out);

/* Derived task only: a class-B ride summary (D-070 parts 2-3, D-087 item 2): CURRENT is the
 * open ride as it would close now, PREVIOUS the last closed ride. NO_DATA while there is no
 * such ride; NOT_COMPUTED for any signal that is not a computed class-B scalar. False if sig
 * is out of range or out is NULL. A ride is one tester stop epoch; there is no persistence
 * and no UDS DID yet (part 2b). */
bool derived_ride_get(derived_ride_which_t which, uint32_t sig, derived_out_t* out);

/* Derived task only: the core, for counters and the computed set (tests, diagnostics). */
const derived_core_t* derived_core(void);

#ifdef __cplusplus
}
#endif

#endif /* FEATURES_DERIVED_DERIVED_H */
