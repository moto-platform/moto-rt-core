#include "features/derived/derived.h"

#include "services/derived_inputs.h"
#include "services/vehicle_signals.h"

#include <stddef.h>

static derived_core_t core;
static derived_in_t inputs[VEHICLE_CL250_DID_COUNT];
static derived_inputs_record_t latest; /* the newest record read */

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: called by the boot setup (host program, H7) and the tests
void derived_init(void)
{
    derived_core_init(&core);
    for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
        inputs[i].value = 0.0f;
        inputs[i].stamp_ms = 0u;
        inputs[i].present = false;
        latest.did[i].value = 0.0f;
        latest.did[i].stamp_ms = 0u;
        latest.did[i].state = (uint8_t)VEHICLE_SIGNAL_NONE;
    }
    latest.pass = 0u;
    latest.published_ms = 0u;
    latest.epoch_counter = 0u;
    latest.epoch_open = 0u;
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: called by the derived task (host program, H7) and the tests
void derived_step(uint32_t now_ms)
{
    (void)derived_inputs_read(&latest); /* the one consumer call site */
    for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
        /* The core judges every DID by its own stale_after_ms from the sample stamp, the
         * limit vehicle_signals uses for STALE too, so a sample counts as present whenever
         * one exists (and a sticky STALE only ever coincides with the core's verdict). */
        inputs[i].present = latest.did[i].state != (uint8_t)VEHICLE_SIGNAL_NONE;
        inputs[i].value = latest.did[i].value;
        inputs[i].stamp_ms = latest.did[i].stamp_ms;
    }
    const derived_epoch_t epoch = {true, latest.epoch_open != 0u, latest.epoch_counter};
    derived_core_step_epoch(&core, now_ms, inputs, &epoch);
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: called by the derived task (future logger) and the tests
bool derived_get(uint32_t sig, derived_out_t* out)
{
    const derived_out_t* const o = derived_core_out(&core, sig);
    if ((o == NULL) || (out == NULL)) {
        return false;
    }
    *out = *o;
    return true;
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: called by the derived task (future logger) and the tests
bool derived_ride_get(derived_ride_which_t which, uint32_t sig, derived_out_t* out)
{
    return derived_core_ride_out(&core, which, sig, out);
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: tests and diagnostics
const derived_core_t* derived_core(void)
{
    return &core;
}
