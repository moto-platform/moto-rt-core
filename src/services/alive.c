#include "services/alive.h"

#include <stddef.h>

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: called by the EKF setup (D-065 PR 3) and the tests
void alive_init(alive_counter_t* c)
{
    if (c != NULL) {
        hal_atomic_u32_store(&c->count, 0u);
    }
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: called by the EKF task (D-065 PR 3) and the tests
void alive_bump(alive_counter_t* c)
{
    if (c != NULL) {
        hal_atomic_u32_store(&c->count, hal_atomic_u32_load(&c->count) + 1u);
    }
}

uint32_t alive_count(const alive_counter_t* c)
{
    return (c == NULL) ? 0u : hal_atomic_u32_load(&c->count);
}
