#include "services/com_core.h"

/* Half the ms counter range: an elapsed time below it is "now or later" (wrap-safe). */
#define HALF_RANGE_MS 0x80000000u

bool com_cycle_due(const com_cycle_t* cycle, uint32_t now_ms)
{
    return !cycle->started || ((uint32_t)(now_ms - cycle->next_due_ms) < HALF_RANGE_MS);
}

void com_cycle_done(com_cycle_t* cycle, uint32_t now_ms, uint32_t period_ms)
{
    if (!cycle->started || (period_ms == 0u)) {
        cycle->next_due_ms = now_ms + period_ms;
        cycle->started = true;
        return;
    }
    cycle->next_due_ms += period_ms;
    const uint32_t late = now_ms - cycle->next_due_ms;
    if (late < HALF_RANGE_MS) { /* still due: skip the missed deadlines (no burst) */
        cycle->next_due_ms += ((late / period_ms) + 1u) * period_ms;
    }
}
