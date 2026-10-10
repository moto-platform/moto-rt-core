#ifndef HAL_HAL_TIME_H
#define HAL_HAL_TIME_H

/*
 * Monotonic millisecond time source (HAL-free interface). The host port uses
 * CLOCK_MONOTONIC or a manual clock for tests (hal/host/hal_time_host.h); the H7 port
 * will use the tick timer. The value wraps after ~49.7 days; users compare with
 * unsigned subtraction (services/timebase.h). Only services/ include this header.
 *
 * Task-safe (D-065, E-17): any task may call hal_time_ms() at any priority. A port reads
 * the counter with one 32-bit access (no torn value) and keeps no state that callers
 * write, so a preempted call returns a valid, merely earlier, reading. The H7 port
 * (Q-019) reads a free-running 1 kHz counter (e.g. the FreeRTOS tick count from task
 * context, or a 32-bit timer register); an ISR caller needs the port's ISR-safe read.
 * The host port uses atomics for its manual clock and its monotonic origin.
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
