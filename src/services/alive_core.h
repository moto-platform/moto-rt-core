#ifndef SERVICES_ALIVE_CORE_H
#define SERVICES_ALIVE_CORE_H

/*
 * Pure supervisor side of the alive supervision (D-064 item 4, D-065 item 2): decides
 * from successive samples of a task's alive counter (services/alive.h) whether the task
 * stalled. No HAL, RTOS or clock access; the caller passes the time.
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* The zero state is the safe one: a zeroed monitor reads stalled (safety-reviewer
 * MAJOR-1), like a monitor whose count never changed. */
typedef struct {
    uint32_t last_count;     /* count seen at the last change (or at init) */
    uint32_t last_change_ms; /* when the supervisor saw that change */
    bool alive;              /* false: never changed or stalled; true only after a change */
} alive_monitor_t;

/* Starts supervising from the count read now (the baseline, whatever its value): until
 * the count changes, the task counts as never having run, so stalled. */
void alive_monitor_init(alive_monitor_t* m, uint32_t count);

/*
 * Supervisor, every pass. A changed count is alive: false, stamped now_ms. True when the
 * count never changed since init (a registered task that never completed a cycle: no
 * grace time), or when more than window_ms passed since the last change; then true
 * until the count changes again, so a stall never ends by timer wrap. Call it at least
 * once per window: the stamp is when the supervisor saw the change, so a late supervisor
 * pass delays a stall verdict but never causes one. NULL reads as stalled.
 *
 * Max-gap only (safety-reviewer MINOR-5, accepted): one change ends a stall, so a task
 * that runs at a third of its rate, or flaps, still reads alive. For the EKF, 0x020's
 * output-age budget (D-065 item 2, PR 3) sends INVALID in that case.
 */
bool alive_monitor_stalled(alive_monitor_t* m, uint32_t count, uint32_t now_ms, uint32_t window_ms);

#ifdef __cplusplus
}
#endif

#endif /* SERVICES_ALIVE_CORE_H */
