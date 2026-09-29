#ifndef HAL_HAL_TIME_H
#define HAL_HAL_TIME_H

/*
 * Monotonic millisecond time source (HAL-free interface). The host port uses
 * CLOCK_MONOTONIC or a manual clock for tests (hal/host/hal_time_host.h); the H7 port
 * will use the tick timer. The value wraps after ~49.7 days; users compare with
 * unsigned subtraction (services/timebase.h). Only services/ include this header.
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint32_t hal_time_ms(void);

#ifdef __cplusplus
}
#endif

#endif /* HAL_HAL_TIME_H */
