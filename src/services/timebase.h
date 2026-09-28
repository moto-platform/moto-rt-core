#ifndef SERVICES_TIMEBASE_H
#define SERVICES_TIMEBASE_H

/*
 * Millisecond timebase for services and features. Wraps hal/hal_time.h so features
 * never include the HAL. The counter wraps after ~49.7 days; always compare with
 * timebase_elapsed_ms()/timebase_expired(), never with < or >.
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint32_t timebase_now_ms(void);

/* now - since, wrap-safe for intervals below 2^32 ms. */
uint32_t timebase_elapsed_ms(uint32_t now_ms, uint32_t since_ms);

/* True once at least period_ms have passed since start_ms. */
bool timebase_expired(uint32_t now_ms, uint32_t start_ms, uint32_t period_ms);

#ifdef __cplusplus
}
#endif

#endif /* SERVICES_TIMEBASE_H */
