#include "services/imu.h"

#include "hal/imu.h"

#include <stddef.h>

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: called by the EKF setup (D-065 PR 3b) and the tests
void imu_init(imu_t* imu)
{
    if (imu != NULL) {
        imu_core_init(&imu->core);
    }
}

/* Drains the FIFO into acc: at most IMU_DRAIN_MAX samples, then one more pop to tell an
 * empty FIFO from a backlog. Stops at EMPTY or a fault; an overrun is noted and the
 * drain goes on, since the samples after it still belong to this period. */
static void drain(imu_acc_t* acc)
{
    bool done = false;
    for (uint32_t i = 0u; (i <= IMU_DRAIN_MAX) && !done; i++) {
        hal_imu_sample_t s;
        const hal_imu_status_t st = hal_imu_pop(&s);
        switch (st) {
        case HAL_IMU_OK:
            if (i < IMU_DRAIN_MAX) {
                imu_acc_add(acc, s.gyro);
            } else {
                acc->backlog = true; /* more than one period's worth: the mean is unknown */
                done = true;
            }
            break;
        case HAL_IMU_EMPTY:
            done = true;
            break;
        case HAL_IMU_OVERRUN:
            acc->overrun = true;
            break;
        default:
            acc->hal_error = true;
            done = true;
            break;
        }
    }
    if (acc->backlog || acc->hal_error) {
        hal_imu_flush();
    }
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: called by the EKF glue (D-067) and the tests
imu_offset_health_t imu_offset_health(const imu_t* imu)
{
    imu_offset_health_t h = {false, false, 0u};
    if (imu != NULL) {
        h.offset_learned = imu->core.offset_learned;
        h.offset_rejected = imu->core.offset_rejected;
        h.reject_count = imu->core.reject_count;
    }
    return h;
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: called by the EKF task (D-065 PR 3b) and the tests
imu_sample_t imu_read(imu_t* imu, uint32_t now_ms, bool standstill)
{
    imu_acc_t acc;
    imu_acc_reset(&acc);
    imu_config_t cfg = {0.0f, 0u};
    if (imu == NULL) {
        return imu_core_finish(NULL, &acc, &cfg, now_ms, standstill);
    }
    if (imu_core_new_period(&imu->core, now_ms)) {
        hal_imu_config_t hc;
        if (hal_imu_get_config(&hc) == HAL_IMU_OK) {
            cfg.gyro_rps_per_lsb = hc.gyro_rps_per_lsb;
            cfg.odr_hz = hc.odr_hz;
        } else {
            acc.hal_error = true;
        }
        drain(&acc);
    }
    return imu_core_finish(&imu->core, &acc, &cfg, now_ms, standstill);
}
