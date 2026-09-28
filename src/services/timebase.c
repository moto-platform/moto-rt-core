#include "services/timebase.h"

#include "hal/hal_time.h"

uint32_t timebase_now_ms(void)
{
    return hal_time_ms();
}

uint32_t timebase_elapsed_ms(uint32_t now_ms, uint32_t since_ms)
{
    return now_ms - since_ms; /* unsigned: wrap-safe */
}

bool timebase_expired(uint32_t now_ms, uint32_t start_ms, uint32_t period_ms)
{
    return timebase_elapsed_ms(now_ms, start_ms) >= period_ms;
}
