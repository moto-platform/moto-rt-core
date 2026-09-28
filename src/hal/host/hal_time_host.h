#ifndef HAL_HOST_HAL_TIME_HOST_H
#define HAL_HOST_HAL_TIME_HOST_H

/*
 * Host implementation of hal/hal_time.h. Default: CLOCK_MONOTONIC, counted from the
 * first call. Tests and fast simulation switch to a manual clock that only moves when
 * hal_time_host_advance() is called.
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void hal_time_host_use_manual(uint32_t start_ms);
void hal_time_host_advance(uint32_t delta_ms);
void hal_time_host_use_monotonic(void);

/* Sleeps the calling process (host main loop pacing only; never used by services). */
void hal_time_host_sleep_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* HAL_HOST_HAL_TIME_HOST_H */
