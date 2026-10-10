#ifndef SERVICES_IMU_H
#define SERVICES_IMU_H

/*
 * IMU service (D-065 item 2, PR 3a): once per EKF period, drains the hal/imu.h gyro FIFO
 * (bounded) and returns one interval-mean sample with the standstill z offset removed
 * (services/imu_core.h has the rules). A stale, saturated, lost or unconfigured period,
 * and every period before the first learnt z offset, is INVALID.
 *
 * **EKF task only** (architecture-guard on the PR 3 plan): the one caller of the hal/imu.h
 * consumer side. Never from the comms task or an ISR. The caller decides standstill from
 * its speed snapshot; this service reads no speed. This header does not bring in
 * hal/imu.h, so features see only the sample type (architecture-guard MINOR-1).
 */

#include "services/imu_core.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Static storage owned by the caller, no heap. */
typedef struct {
    imu_core_t core;
} imu_t;

/* Boot only, before the EKF task runs: no previous period, no offset. */
void imu_init(imu_t* imu);

/*
 * EKF task, once per period. Pops at most IMU_DRAIN_MAX samples; a FIFO that still holds
 * more is flushed (BACKLOG), as it is after a port fault, so no old sample leaks into
 * the next period. A call in the same ms as the previous one returns NO_NEW without
 * touching the FIFO. NULL reads INVALID (ARG).
 */
imu_sample_t imu_read(imu_t* imu, uint32_t now_ms, bool standstill);

#ifdef __cplusplus
}
#endif

#endif /* SERVICES_IMU_H */
