#include "hal/host/imu_host.h"

#include <stddef.h>

/*
 * Single-producer / single-consumer ring like the H7 port's (hal/README.md): the pusher
 * (test or SIL feed, standing in for the IMU ISR) only advances tail, the hal_imu_* caller
 * (the EKF task) only advances head. Both are free-running counters; level = tail - head.
 * Overruns are a producer-only counter that the consumer compares with the last value it
 * reported, so an overrun can never be lost between a read and a clear (safety-reviewer
 * MINOR-3 on PR 3a).
 * The compiler's __atomic builtins make it ThreadSanitizer-clean when the SIL feeds it
 * from another thread (architecture-guard MAJOR-5 on the PR 3 plan). The configuration
 * is set before the threads start.
 */
static hal_imu_sample_t fifo[IMU_HOST_FIFO_DEPTH];
static uint32_t head;          /* consumer */
static uint32_t tail;          /* producer */
static uint32_t overruns;      /* producer only: lost-sample events (the ISR's counter) */
static uint32_t overruns_seen; /* consumer only: the count already reported */
static bool error;
static uint32_t flushes; /* consumer */
static hal_imu_config_t config = {IMU_HOST_DEFAULT_RPS_PER_LSB, IMU_HOST_DEFAULT_ODR_HZ};

void imu_host_reset(void)
{
    __atomic_store_n(&head, 0u, __ATOMIC_SEQ_CST);
    __atomic_store_n(&tail, 0u, __ATOMIC_SEQ_CST);
    __atomic_store_n(&overruns, 0u, __ATOMIC_SEQ_CST);
    __atomic_store_n(&overruns_seen, 0u, __ATOMIC_SEQ_CST);
    __atomic_store_n(&error, false, __ATOMIC_SEQ_CST);
    __atomic_store_n(&flushes, 0u, __ATOMIC_SEQ_CST);
    config.gyro_rps_per_lsb = IMU_HOST_DEFAULT_RPS_PER_LSB;
    config.odr_hz = IMU_HOST_DEFAULT_ODR_HZ;
}

void imu_host_set_config(float gyro_rps_per_lsb, uint16_t odr_hz)
{
    config.gyro_rps_per_lsb = gyro_rps_per_lsb;
    config.odr_hz = odr_hz;
}

bool imu_host_push(const hal_imu_sample_t* s)
{
    if (s == NULL) {
        return false;
    }
    const uint32_t t = __atomic_load_n(&tail, __ATOMIC_RELAXED);
    const uint32_t h = __atomic_load_n(&head, __ATOMIC_ACQUIRE);
    if ((t - h) >= IMU_HOST_FIFO_DEPTH) {
        imu_host_inject_overrun();
        return false;
    }
    fifo[t % IMU_HOST_FIFO_DEPTH] = *s;
    __atomic_store_n(&tail, t + 1u, __ATOMIC_RELEASE);
    return true;
}

void imu_host_inject_overrun(void)
{
    __atomic_store_n(&overruns, __atomic_load_n(&overruns, __ATOMIC_RELAXED) + 1u, __ATOMIC_RELEASE);
}

void imu_host_set_error(bool e)
{
    __atomic_store_n(&error, e, __ATOMIC_RELEASE);
}

uint32_t imu_host_level(void)
{
    return __atomic_load_n(&tail, __ATOMIC_ACQUIRE) - __atomic_load_n(&head, __ATOMIC_ACQUIRE);
}

uint32_t imu_host_flush_count(void)
{
    return __atomic_load_n(&flushes, __ATOMIC_ACQUIRE);
}

hal_imu_status_t hal_imu_get_config(hal_imu_config_t* cfg)
{
    if ((cfg == NULL) || __atomic_load_n(&error, __ATOMIC_ACQUIRE)) {
        return HAL_IMU_ERR;
    }
    *cfg = config;
    return HAL_IMU_OK;
}

hal_imu_status_t hal_imu_pop(hal_imu_sample_t* out)
{
    if ((out == NULL) || __atomic_load_n(&error, __ATOMIC_ACQUIRE)) {
        return HAL_IMU_ERR;
    }
    const uint32_t o = __atomic_load_n(&overruns, __ATOMIC_ACQUIRE);
    if (o != __atomic_load_n(&overruns_seen, __ATOMIC_RELAXED)) {
        __atomic_store_n(&overruns_seen, o, __ATOMIC_RELAXED);
        return HAL_IMU_OVERRUN;
    }
    const uint32_t h = __atomic_load_n(&head, __ATOMIC_RELAXED);
    if (__atomic_load_n(&tail, __ATOMIC_ACQUIRE) == h) {
        return HAL_IMU_EMPTY;
    }
    *out = fifo[h % IMU_HOST_FIFO_DEPTH];
    __atomic_store_n(&head, h + 1u, __ATOMIC_RELEASE);
    return HAL_IMU_OK;
}

void hal_imu_flush(void)
{
    /* the overrun count first, then head: an overrun that races the flush is reported
     * (a spurious INVALID), never lost (safety-reviewer NIT-1 on PR 3a) */
    __atomic_store_n(&overruns_seen, __atomic_load_n(&overruns, __ATOMIC_ACQUIRE), __ATOMIC_RELAXED);
    __atomic_store_n(&head, __atomic_load_n(&tail, __ATOMIC_ACQUIRE), __ATOMIC_RELEASE);
    __atomic_store_n(&flushes, __atomic_load_n(&flushes, __ATOMIC_RELAXED) + 1u, __ATOMIC_RELEASE);
}
