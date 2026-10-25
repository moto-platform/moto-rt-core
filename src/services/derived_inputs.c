#include "services/derived_inputs.h"

#include "services/snapshot.h"
#include "services/vehicle_signals.h"

#include <stddef.h>
#include <string.h>

static snapshot_t snap;
static derived_inputs_record_t slots[SNAPSHOT_SLOTS];
static bool inited;       /* set at boot, before the tasks run */
static uint32_t pass_no;  /* producer only */

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: called by the boot setup (host program, H7) and the tests
void derived_inputs_init(void)
{
    (void)memset(slots, 0, sizeof slots); /* state NONE, pass 0: INVALID */
    snapshot_init(&snap);
    pass_no = 0u;
    inited = true;
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: called by app/comms (comms task) and the tests
void derived_inputs_publish(uint32_t now_ms)
{
    if ((!inited) || snapshot_producer_fault(&snap)) {
        return; /* after a producer fault the write slot may be the consumer's front */
    }
    derived_inputs_record_t* const rec = &slots[snapshot_write_index(&snap)];
    pass_no++;
    rec->pass = pass_no;
    rec->published_ms = now_ms;
    for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
        vehicle_signal_sample_t s;
        (void)memset(&s, 0, sizeof s); /* NONE */
        (void)vehicle_signals_get(i, now_ms, &s);
        rec->did[i].value = s.physical;
        rec->did[i].stamp_ms = s.timestamp_ms;
        rec->did[i].state = (uint8_t)s.state;
    }
    snapshot_publish(&snap);
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: called by features/derived (derived task) and the tests
bool derived_inputs_read(derived_inputs_record_t* out)
{
    if (out == NULL) {
        return false;
    }
    uint8_t idx = 0u;
    const bool fresh = snapshot_read_index(&snap, &idx);
    *out = slots[idx]; /* the one copy; no index outlives this call */
    return fresh;
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: diagnostics and the tests
bool derived_inputs_producer_fault(void)
{
    return snapshot_producer_fault(&snap);
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: diagnostics and the tests
bool derived_inputs_consumer_fault(void)
{
    return snapshot_consumer_fault(&snap);
}
