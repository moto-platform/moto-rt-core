#ifndef HAL_HOST_IMU_HOST_H
#define HAL_HOST_IMU_HOST_H

/*
 * Host implementation of hal/imu.h (D-034): a static FIFO that tests and the SIL program
 * fill with body-axis samples, plus fault injection (overrun, bus error, a bad config).
 * Single producer (imu_host_push, the stand-in for the IMU ISR) and single consumer (the
 * hal_imu_* caller), which may run in different threads; set the configuration and the
 * error flag before the threads start.
 */

#include "hal/imu.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* FIFO depth of the fake (a real IMU FIFO holds a few hundred samples). */
#define IMU_HOST_FIFO_DEPTH 256u

/* Default configuration after imu_host_reset(): +-2000 deg/s full scale, 800 Hz. */
#define IMU_HOST_DEFAULT_RPS_PER_LSB (1.0652645e-3f) /* 2000 deg/s / 32768 LSB in rad/s */
#define IMU_HOST_DEFAULT_ODR_HZ (800u)

/* Empty FIFO, no fault, the default configuration, counters 0. */
void imu_host_reset(void);

/* Replaces the configuration that hal_imu_get_config() reports. */
void imu_host_set_config(float gyro_rps_per_lsb, uint16_t odr_hz);

/* Appends one sample. A full FIFO drops it and reports HAL_IMU_OVERRUN at the next pop,
 * like a real FIFO in stream mode; returns false then. NULL: false, nothing changes. */
bool imu_host_push(const hal_imu_sample_t* s);

/* Counts a lost-sample event: the next pop reports HAL_IMU_OVERRUN once. */
void imu_host_inject_overrun(void);

/* While set, hal_imu_get_config() and hal_imu_pop() report HAL_IMU_ERR. */
void imu_host_set_error(bool error);

/* Samples waiting in the FIFO. */
uint32_t imu_host_level(void);

/* Calls of hal_imu_flush() since the reset. */
uint32_t imu_host_flush_count(void);

#ifdef __cplusplus
}
#endif

#endif /* HAL_HOST_IMU_HOST_H */
