#include "hal/hal_atomic.h"

/* Host port: the compiler's __atomic builtins (GCC and clang), lock-free for 32 bits on
 * every host this builds on, and visible to ThreadSanitizer. */

/* A preprocessor check: gcc -pedantic does not take __atomic_always_lock_free() as a
 * constant expression in _Static_assert. */
_Static_assert(sizeof(uint32_t) == sizeof(int), "uint32_t is int on the host");
#ifndef __GCC_ATOMIC_INT_LOCK_FREE
#error "the host compiler must provide the __atomic builtins (GCC or clang)"
#elif __GCC_ATOMIC_INT_LOCK_FREE != 2
#error "32-bit atomics must be always lock-free on the host"
#endif

uint32_t hal_atomic_u32_load(const hal_atomic_u32_t* a)
{
    return __atomic_load_n(&a->value, __ATOMIC_ACQUIRE);
}

void hal_atomic_u32_store(hal_atomic_u32_t* a, uint32_t v)
{
    __atomic_store_n(&a->value, v, __ATOMIC_RELEASE);
}

uint32_t hal_atomic_u32_exchange(hal_atomic_u32_t* a, uint32_t v)
{
    return __atomic_exchange_n(&a->value, v, __ATOMIC_ACQ_REL);
}
