#include "hal/hal_time.h"
#include "hal/host/hal_time_host.h"

#include <stdbool.h>
#include <time.h>

/* Task-safe like the target counter (hal_time.h): every shared variable is accessed with
 * the compiler's __atomic builtins, so threads (and ThreadSanitizer) see no data race.
 * The monotonic origin is set once by whichever caller comes first. */
#define ORIGIN_UNSET UINT64_MAX

static bool manual;
static uint32_t manual_ms;
static uint64_t origin_ms = ORIGIN_UNSET;

static uint64_t monotonic_ms(void)
{
    struct timespec ts;
    (void)clock_gettime(CLOCK_MONOTONIC, &ts);
    return ((uint64_t)ts.tv_sec * 1000u) + ((uint64_t)ts.tv_nsec / 1000000u);
}

uint32_t hal_time_ms(void)
{
    if (__atomic_load_n(&manual, __ATOMIC_ACQUIRE)) {
        return __atomic_load_n(&manual_ms, __ATOMIC_ACQUIRE);
    }
    const uint64_t now = monotonic_ms();
    uint64_t origin = __atomic_load_n(&origin_ms, __ATOMIC_ACQUIRE);
    if (origin == ORIGIN_UNSET) {
        uint64_t expected = ORIGIN_UNSET;
        /* the first caller anchors; a loser reads the winner's origin into expected */
        origin = __atomic_compare_exchange_n(&origin_ms, &expected, now, false, __ATOMIC_ACQ_REL,
                                             __ATOMIC_ACQUIRE)
                     ? now
                     : expected;
    }
    return (uint32_t)(now - origin); /* wraps like the target tick counter */
}

void hal_time_host_use_manual(uint32_t start_ms)
{
    __atomic_store_n(&manual_ms, start_ms, __ATOMIC_RELEASE);
    __atomic_store_n(&manual, true, __ATOMIC_RELEASE);
}

void hal_time_host_advance(uint32_t delta_ms)
{
    (void)__atomic_fetch_add(&manual_ms, delta_ms, __ATOMIC_ACQ_REL); /* unsigned wrap is intended */
}

void hal_time_host_use_monotonic(void)
{
    __atomic_store_n(&origin_ms, ORIGIN_UNSET, __ATOMIC_RELEASE);
    __atomic_store_n(&manual, false, __ATOMIC_RELEASE);
}

void hal_time_host_sleep_ms(uint32_t ms)
{
    struct timespec ts;
    ts.tv_sec = (time_t)(ms / 1000u);
    ts.tv_nsec = (long)(ms % 1000u) * 1000000L;
    (void)nanosleep(&ts, NULL);
}
