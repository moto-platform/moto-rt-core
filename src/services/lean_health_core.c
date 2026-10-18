#include "services/lean_health_core.h"

#include <stddef.h>

#define MS_PER_S 1000u

_Static_assert(PLATFORM_UDS_DID_LEAN_HEALTH_LENGTH == 5u, "0xFD03 layout");
_Static_assert((PLATFORM_UDS_LEAN_HEALTH_RECORD_FRESH_BYTE == 0u) &&
                   (PLATFORM_UDS_LEAN_HEALTH_OFFSET_LEARNED_BYTE == 0u) &&
                   (PLATFORM_UDS_LEAN_HEALTH_OFFSET_REJECTED_BYTE == 0u) &&
                   (PLATFORM_UDS_LEAN_HEALTH_SPEED_PRODUCER_FAULT_BYTE == 0u) &&
                   (PLATFORM_UDS_LEAN_HEALTH_OUTPUT_CONSUMER_FAULT_BYTE == 0u) &&
                   (PLATFORM_UDS_LEAN_HEALTH_SPEED_CONSUMER_FAULT_BYTE == 0u) &&
                   (PLATFORM_UDS_LEAN_HEALTH_EKF_ALIVE_OK_BYTE == 0u) &&
                   (PLATFORM_UDS_LEAN_HEALTH_RESERVED_BYTE == 0u),
               "0xFD03 flags in byte 0");
_Static_assert((PLATFORM_UDS_LEAN_HEALTH_OFFSET_REJECTS_LENGTH == 2u) &&
                   (PLATFORM_UDS_LEAN_HEALTH_UNLEARNED_S_LENGTH == 2u),
               "0xFD03 counters are u16");
_Static_assert((PLATFORM_UDS_LEAN_HEALTH_OFFSET_REJECTS_MAX == 0xFFFFu) &&
                   (PLATFORM_UDS_LEAN_HEALTH_UNLEARNED_S_MAX == 0xFFFFu) &&
                   (PLATFORM_UDS_LEAN_HEALTH_UNLEARNED_S_UNKNOWN ==
                    PLATFORM_UDS_LEAN_HEALTH_UNLEARNED_S_MAX),
               "0xFD03 counters saturate at 0xFFFF, which UNLEARNED_S reads as UNKNOWN");

static void put_be16(uint8_t* out, uint32_t value)
{
    out[0] = (uint8_t)((value >> 8u) & 0xFFu);
    out[1] = (uint8_t)(value & 0xFFu);
}

uint16_t lean_health_sat16(uint32_t v, uint32_t max)
{
    return (uint16_t)((v > max) ? max : v);
}

uint16_t lean_health_unlearned_s(lean_health_run_t* run, bool learned, uint32_t now_ms)
{
    if (run == NULL) {
        return 0u;
    }
    if (learned) {
        run->started = true;
        run->learned = true;
        return 0u;
    }
    if (!run->started || run->learned) {
        /* Anchored at the EKF's first published step, not at boot (the DID says "since
         * boot"; kept, see the header), or a learnt offset was lost: a new run. */
        run->since_ms = now_ms;
        run->saturated = false;
    }
    run->started = true;
    run->learned = false;
    const uint32_t secs = (now_ms - run->since_ms) / MS_PER_S;
    if (secs >= PLATFORM_UDS_LEAN_HEALTH_UNLEARNED_S_MAX) {
        run->saturated = true; /* sticky: the 32-bit ms difference wraps after 49.7 days */
    }
    return run->saturated ? (uint16_t)PLATFORM_UDS_LEAN_HEALTH_UNLEARNED_S_MAX
                          : (uint16_t)secs; /* below the maximum here */
}

bool lean_health_record_fresh(const lean_health_rec_t* rec, uint32_t now_ms)
{
    if ((rec == NULL) || !rec->have_record) {
        return false;
    }
    if ((rec->stamp_ms - now_ms) <= LEAN_HEALTH_FUTURE_MAX_MS) {
        return true; /* now or slightly ahead: age 0, never a wrapped age */
    }
    return (now_ms - rec->stamp_ms) <= PLATFORM_UDS_LEAN_HEALTH_MAX_AGE_MS;
}

bool lean_health_build(const lean_health_rec_t* rec, const lean_health_comms_t* comms,
                       uint32_t now_ms, uint8_t* out, uint16_t len)
{
    if ((rec == NULL) || (comms == NULL) || (out == NULL) ||
        (len != PLATFORM_UDS_DID_LEAN_HEALTH_LENGTH)) {
        return false;
    }
    const bool fresh = lean_health_record_fresh(rec, now_ms);
    uint8_t flags = 0u;
    if (fresh) {
        flags |= (uint8_t)PLATFORM_UDS_LEAN_HEALTH_RECORD_FRESH_MASK;
    }
    if (fresh && rec->ekf.offset_learned) {
        flags |= (uint8_t)PLATFORM_UDS_LEAN_HEALTH_OFFSET_LEARNED_MASK;
    }
    if (rec->ekf.offset_rejected) {
        flags |= (uint8_t)PLATFORM_UDS_LEAN_HEALTH_OFFSET_REJECTED_MASK;
    }
    if (comms->speed_producer_fault) {
        flags |= (uint8_t)PLATFORM_UDS_LEAN_HEALTH_SPEED_PRODUCER_FAULT_MASK;
    }
    if (comms->output_consumer_fault) {
        flags |= (uint8_t)PLATFORM_UDS_LEAN_HEALTH_OUTPUT_CONSUMER_FAULT_MASK;
    }
    if (rec->speed_consumer_fault) {
        flags |= (uint8_t)PLATFORM_UDS_LEAN_HEALTH_SPEED_CONSUMER_FAULT_MASK;
    }
    if (comms->ekf_alive_ok) {
        flags |= (uint8_t)PLATFORM_UDS_LEAN_HEALTH_EKF_ALIVE_OK_MASK;
    }
    out[PLATFORM_UDS_LEAN_HEALTH_RECORD_FRESH_BYTE] = flags; /* RESERVED never set */
    put_be16(&out[PLATFORM_UDS_LEAN_HEALTH_OFFSET_REJECTS_BYTE],
             rec->ekf.reject_count); /* saturated by the EKF side, see lean_health_sat16() */
    put_be16(&out[PLATFORM_UDS_LEAN_HEALTH_UNLEARNED_S_BYTE],
             fresh ? (uint32_t)rec->ekf.unlearned_s : (uint32_t)PLATFORM_UDS_LEAN_HEALTH_UNLEARNED_S_UNKNOWN);
    return true;
}
