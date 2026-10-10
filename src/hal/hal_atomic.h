#ifndef HAL_HAL_ATOMIC_H
#define HAL_HAL_ATOMIC_H

/*
 * 32-bit atomics with ordering, for data shared between tasks of different priority
 * (D-065 item 2: services/snapshot, services/alive). HAL-free interface; only services/
 * include it. Each call is one bounded operation: no lock and no wait on another task.
 * A port may retry internally only a bounded number of times (hal/README.md).
 *
 * - load:     acquire (later reads see what the matching release made visible)
 * - store:    release (earlier writes are visible before the stored value)
 * - exchange: acquire + release, returns the previous value
 *
 * The host port (hal/host/hal_atomic_host.c) uses the compiler's __atomic builtins, which
 * ThreadSanitizer understands. The H7 port comes with the board (Q-019); its contract is
 * in hal/README.md.
 *
 * Access a hal_atomic_u32_t only through these functions; its member is not part of the
 * interface. Initialise it with hal_atomic_u32_store() before any other task runs.
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t value;
} hal_atomic_u32_t;

uint32_t hal_atomic_u32_load(const hal_atomic_u32_t* a);

void hal_atomic_u32_store(hal_atomic_u32_t* a, uint32_t v);

uint32_t hal_atomic_u32_exchange(hal_atomic_u32_t* a, uint32_t v);

#ifdef __cplusplus
}
#endif

#endif /* HAL_HAL_ATOMIC_H */
