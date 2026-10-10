#ifndef HAL_IMU_H
#define HAL_IMU_H

/*
 * IMU gyro FIFO (HAL-free interface, D-065 item 2). The board and its IMU are open
 * (Q-019), so this interface only fixes what services/imu needs:
 *
 * - Raw gyro samples in the order the IMU produced them, already in the **body axes**
 *   (x forward, y right, z down; + rolling right, + turning right). The mounting
 *   rotation is the port's job.
 * - Sampled by the IMU itself at a fixed output data rate of at least 800 Hz, with its
 *   on-chip digital low-pass filter on (user, 2026-10-10). services/imu averages all
 *   samples of each EKF period, so the port must never decimate or point-sample.
 * - One aligned read per sample. The port's IMU ISR (FIFO watermark or data ready) and
 *   its bus transfers stay inside the port; callers never wait for the bus.
 *
 * Only services/imu includes this header (CI layering check), and only the EKF task
 * calls it (one consumer, no lock). Never from an ISR.
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* One gyro sample in raw LSB, body axes [x, y, z]. INT16_MIN and INT16_MAX are the
 * sensor's full scale: services/imu treats them as saturated. */
typedef struct {
    int16_t gyro[3];
} hal_imu_sample_t;

typedef enum {
    HAL_IMU_OK = 0,   /* a sample was delivered */
    HAL_IMU_EMPTY,    /* the FIFO is empty: no sample */
    HAL_IMU_OVERRUN,  /* samples were lost since the last pop (FIFO overflow): no sample;
                         reported once, the next pop continues with the oldest kept one */
    HAL_IMU_ERR       /* bus or sensor fault, or not initialised: no sample */
} hal_imu_status_t;

/* The running configuration, fixed after the port's init. */
typedef struct {
    float gyro_rps_per_lsb; /* scale of hal_imu_sample_t.gyro, rad/s per LSB */
    uint16_t odr_hz;        /* the IMU's output data rate */
} hal_imu_config_t;

/* HAL_IMU_OK with *cfg filled, or HAL_IMU_ERR (not initialised, NULL). */
hal_imu_status_t hal_imu_get_config(hal_imu_config_t* cfg);

/* Takes the oldest sample from the FIFO. Bounded: never waits. */
hal_imu_status_t hal_imu_pop(hal_imu_sample_t* out);

/* Discards everything in the FIFO, including a pending overrun report. Bounded. */
void hal_imu_flush(void);

#ifdef __cplusplus
}
#endif

#endif /* HAL_IMU_H */
