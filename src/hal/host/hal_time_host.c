#include "hal/hal_time.h"
#include "hal/host/hal_time_host.h"

#include <stdbool.h>
#include <time.h>

static bool manual;
static uint32_t manual_ms;
static bool origin_set;
static uint64_t origin_ms;

static uint64_t monotonic_ms(void)
{
    struct timespec ts;
    (void)clock_gettime(CLOCK_MONOTONIC, &ts);
    return ((uint64_t)ts.tv_sec * 1000u) + ((uint64_t)ts.tv_nsec / 1000000u);
}

uint32_t hal_time_ms(void)
{
    if (manual) {
        return manual_ms;
    }
    uint64_t now = monotonic_ms();
    if (!origin_set) {
        origin_ms = now;
        origin_set = true;
    }
    return (uint32_t)(now - origin_ms); /* wraps like the target tick counter */
}

void hal_time_host_use_manual(uint32_t start_ms)
{
    manual = true;
    manual_ms = start_ms;
}

void hal_time_host_advance(uint32_t delta_ms)
{
    manual_ms += delta_ms; /* unsigned wrap is intended */
}

void hal_time_host_use_monotonic(void)
{
    manual = false;
    origin_set = false;
}

void hal_time_host_sleep_ms(uint32_t ms)
{
    struct timespec ts;
    ts.tv_sec = (time_t)(ms / 1000u);
    ts.tv_nsec = (long)(ms % 1000u) * 1000000L;
    (void)nanosleep(&ts, NULL);
}
