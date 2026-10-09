#ifndef SERVICES_COM_CORE_H
#define SERVICES_COM_CORE_H

/*
 * Platform-bus send schedule (≈ AUTOSAR Com as a concept, D-006; D-056 items 3 and 7):
 * the deadline-anchored cycle shared by every rt-core sender of a cyclic platform
 * message (the republisher's 0x021 / 0x110, the heartbeat's 0x081, later the EKF's
 * 0x020 / 0x022). Pure logic: no HAL, no RTOS, no heap. services/com does the writes.
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Deadline-anchored cycle (D-056 item 3). Zero-initialised: due at once. */
typedef struct {
    uint32_t next_due_ms;
    bool started;
} com_cycle_t;

/* True at the first call and whenever now_ms reached the deadline (wrap-safe). */
bool com_cycle_due(const com_cycle_t* cycle, uint32_t now_ms);

/* The cycle's frame went out (or was dropped for good): the next deadline is one period
 * after the last one; deadlines already missed are skipped, never sent as a burst. The
 * first call anchors the schedule at now_ms. */
void com_cycle_done(com_cycle_t* cycle, uint32_t now_ms, uint32_t period_ms);

#ifdef __cplusplus
}
#endif

#endif /* SERVICES_COM_CORE_H */
