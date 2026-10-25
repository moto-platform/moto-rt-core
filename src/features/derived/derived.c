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
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: called by the derived task (host program, H7) and the tests
void derived_step(uint32_t now_ms)
{
    (void)derived_inputs_read(&latest); /* the one consumer call site */
    for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
        /* STALE here is vehicle_signals' own DID limit; the core judges every input by the
         * stale_after_ms of the signal that uses it, from the stamp, so a sample counts
         * as present whenever one exists. */
        inputs[i].present = latest.did[i].state != (uint8_t)VEHICLE_SIGNAL_NONE;
        inputs[i].value = latest.did[i].value;
        inputs[i].stamp_ms = latest.did[i].stamp_ms;
    }
    derived_core_step(&core, now_ms, inputs);
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

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: tests and diagnostics
const derived_core_t* derived_core(void)
{
    return &core;
}
