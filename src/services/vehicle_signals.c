#include "services/vehicle_signals.h"

#include "vehicle_cl250.h"

#include <stddef.h>

typedef struct {
    bool has_sample;
    bool stale;        /* sticky until the next write */
    uint32_t raw;
    float physical;
    uint32_t timestamp_ms;
} stored_sample_t;

static stored_sample_t samples[VEHICLE_CL250_DID_COUNT];
static bool ecu_present;

void vehicle_signals_init(void)
{
    for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
        samples[i].has_sample = false;
        samples[i].stale = false;
        samples[i].raw = 0u;
        samples[i].physical = 0.0f;
        samples[i].timestamp_ms = 0u;
    }
    ecu_present = false;
}

bool vehicle_signals_write(uint32_t idx, uint32_t raw, float physical, uint32_t timestamp_ms)
{
    if (idx >= VEHICLE_CL250_DID_COUNT) {
        return false;
    }
    samples[idx].raw = raw;
    samples[idx].physical = physical;
    samples[idx].timestamp_ms = timestamp_ms;
    samples[idx].has_sample = true;
    samples[idx].stale = false;
    return true;
}

/* Age check with the sticky flag; sets it when the sample is too old. */
static bool is_stale(uint32_t idx, uint32_t age_ms)
{
    if (age_ms > (uint32_t)vehicle_cl250_dids[idx].stale_after_ms) {
        samples[idx].stale = true;
    }
    return samples[idx].stale;
}

void vehicle_signals_expire(uint32_t now_ms)
{
    for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
        if (samples[i].has_sample) {
            (void)is_stale(i, now_ms - samples[i].timestamp_ms);
        }
    }
}

bool vehicle_signals_get(uint32_t idx, uint32_t now_ms, vehicle_signal_sample_t* out)
{
    if ((idx >= VEHICLE_CL250_DID_COUNT) || (out == NULL)) {
        return false;
    }
    const stored_sample_t* s = &samples[idx];
    if (!s->has_sample) {
        out->raw = 0u;
        out->physical = 0.0f;
        out->timestamp_ms = 0u;
        out->age_ms = 0u;
        out->state = VEHICLE_SIGNAL_NONE;
        return true;
    }
    const uint32_t age = now_ms - s->timestamp_ms; /* wrap-safe */
    out->raw = s->raw;
    out->physical = s->physical;
    out->timestamp_ms = s->timestamp_ms;
    out->age_ms = age;
    out->state = is_stale(idx, age) ? VEHICLE_SIGNAL_STALE : VEHICLE_SIGNAL_VALID;
    return true;
}

void vehicle_signals_set_ecu_present(bool present)
{
    ecu_present = present;
}

bool vehicle_signals_ecu_present(void)
{
    return ecu_present;
}
