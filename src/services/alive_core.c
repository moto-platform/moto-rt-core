#include "services/alive_core.h"

#include <stddef.h>

void alive_monitor_init(alive_monitor_t* m, uint32_t count)
{
    if (m != NULL) {
        m->last_count = count;
        m->last_change_ms = 0u;
        m->alive = false; /* never ran yet */
    }
}

bool alive_monitor_stalled(alive_monitor_t* m, uint32_t count, uint32_t now_ms, uint32_t window_ms)
{
    if (m == NULL) {
        return true;
    }
    if (count != m->last_count) {
        m->last_count = count;
        m->last_change_ms = now_ms;
        m->alive = true;
    } else if ((uint32_t)(now_ms - m->last_change_ms) > window_ms) {
        m->alive = false; /* more than window_ms without a change (unsigned: wrap-safe) */
    } else {
        /* unchanged within the window: the verdict stands (never ran stays stalled) */
    }
    return !m->alive;
}
