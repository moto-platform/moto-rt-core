#include "hal/hal_atomic.h"

/* Host port: the compiler's __atomic builtins (GCC and clang), lock-free for 32 bits on
 * every host this builds on, and visible to ThreadSanitizer. */

_Static_assert(__atomic_always_lock_free(sizeof(uint32_t), 0), "32-bit atomics must be lock-free");

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
