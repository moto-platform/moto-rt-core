#include "services/imu_core.h"

#include <math.h>
#include <stddef.h>

/* elapsed * ODR is computed on at most this many ms: above it the expected count already
 * exceeds IMU_DRAIN_MAX, so the result is the same and the product cannot overflow. */
#define IMU_ELAPSED_CAP_MS (1000u)

static imu_sample_t imu_invalid(uint32_t now_ms, uint16_t count, imu_reason_t reason)
{
    imu_sample_t s;
    s.gyro_x_rps = 0.0f;
    s.gyro_y_rps = 0.0f;
    s.gyro_z_rps = 0.0f;
    s.stamp_ms = now_ms;
    s.count = count;
    s.state = IMU_SAMPLE_INVALID;
    s.reason = reason;
    return s;
}

static void window_close(imu_core_t* c)
{
    c->win_open = false;
    c->win_count = 0u;
    c->win_sum = 0.0f;
    for (uint32_t i = 0u; i < 3u; i++) {
        c->win_min[i] = 0.0f;
        c->win_max[i] = 0.0f;
    }
}

static void window_restart(imu_core_t* c, const float mean[3], uint32_t start_ms)
{
    c->win_open = true;
    c->win_start_ms = start_ms;
    c->win_count = 1u;
    c->win_sum = mean[2];
    for (uint32_t i = 0u; i < 3u; i++) {
        c->win_min[i] = mean[i];
        c->win_max[i] = mean[i];
    }
}

/* One standstill period's raw means [x, y, z], from start_ms (the period's start) to
 * now_ms. */
static void window_add(imu_core_t* c, const float mean[3], uint32_t start_ms, uint32_t now_ms)
{
    if (!c->win_open) {
        window_restart(c, mean, start_ms);
    } else {
        float lo[3];
        float hi[3];
        bool still = true;
        for (uint32_t i = 0u; i < 3u; i++) {
            lo[i] = (mean[i] < c->win_min[i]) ? mean[i] : c->win_min[i];
            hi[i] = (mean[i] > c->win_max[i]) ? mean[i] : c->win_max[i];
            if ((hi[i] - lo[i]) > IMU_OFFSET_SPREAD_MAX_RPS) {
                still = false;
            }
        }
        if (!still) {
            /* the vehicle turned, rolled or was moved: start again from this period */
            window_restart(c, mean, start_ms);
        } else {
            c->win_count++;
            c->win_sum += mean[2];
            for (uint32_t i = 0u; i < 3u; i++) {
                c->win_min[i] = lo[i];
                c->win_max[i] = hi[i];
            }
        }
    }
    if ((now_ms - c->win_start_ms) >= IMU_OFFSET_WINDOW_MS) {
        const float offset = c->win_sum / (float)c->win_count;
        if (fabsf(offset) <= IMU_OFFSET_MAX_RPS) {
            c->offset_z_rps = offset;
            c->offset_learned = true;
            c->offset_rejected = false;
            c->learn_count++;
        } else {
            /* fail-closed: a faulty gyro or a turning vehicle; the old offset is dropped */
            c->offset_learned = false;
            c->offset_rejected = true;
            c->reject_count++;
        }
        window_close(c);
    }
}

void imu_core_init(imu_core_t* c)
{
    if (c != NULL) {
        c->last_ms = 0u;
        c->have_last = false;
        c->offset_z_rps = 0.0f;
        c->offset_learned = false;
        c->offset_rejected = false;
        c->win_start_ms = 0u;
        c->learn_count = 0u;
        c->reject_count = 0u;
        window_close(c);
    }
}

bool imu_core_new_period(const imu_core_t* c, uint32_t now_ms)
{
    return (c != NULL) && !(c->have_last && (now_ms == c->last_ms));
}

void imu_acc_reset(imu_acc_t* a)
{
    if (a != NULL) {
        a->sum[0] = 0;
        a->sum[1] = 0;
        a->sum[2] = 0;
        a->count = 0u;
        a->saturated = false;
        a->hal_error = false;
        a->overrun = false;
        a->backlog = false;
    }
}

void imu_acc_add(imu_acc_t* a, const int16_t gyro[3])
{
    if ((a == NULL) || (gyro == NULL) || (a->count >= IMU_DRAIN_MAX)) {
        return;
    }
    for (uint32_t i = 0u; i < 3u; i++) {
        if ((gyro[i] == INT16_MIN) || (gyro[i] == INT16_MAX)) {
            a->saturated = true;
        }
        a->sum[i] += (int32_t)gyro[i];
    }
    a->count++;
}

static bool config_ok(const imu_config_t* cfg)
{
    return isfinite(cfg->gyro_rps_per_lsb) && (cfg->gyro_rps_per_lsb > 0.0f) &&
           (cfg->odr_hz >= IMU_ODR_MIN_HZ) && (cfg->odr_hz <= IMU_ODR_MAX_HZ);
}

static bool count_too_low(uint32_t elapsed_ms, uint16_t odr_hz, uint16_t count)
{
    const uint32_t ms = (elapsed_ms > IMU_ELAPSED_CAP_MS) ? IMU_ELAPSED_CAP_MS : elapsed_ms;
    const uint32_t min_count = (ms * (uint32_t)odr_hz * IMU_COUNT_MIN_PERCENT) / 100000u;
    return (count == 0u) || ((uint32_t)count < min_count);
}

imu_sample_t imu_core_finish(imu_core_t* c, const imu_acc_t* a, const imu_config_t* cfg,
                             uint32_t now_ms, bool standstill)
{
    if ((c == NULL) || (a == NULL) || (cfg == NULL)) {
        return imu_invalid(now_ms, 0u, IMU_REASON_ARG);
    }
    if (!imu_core_new_period(c, now_ms)) {
        return imu_invalid(now_ms, a->count, IMU_REASON_NO_NEW);
    }
    const uint32_t start_ms = c->last_ms;
    const bool had_last = c->have_last;
    c->last_ms = now_ms;
    c->have_last = true;

    imu_reason_t reason = IMU_REASON_NONE;
    if (a->hal_error) {
        reason = IMU_REASON_HAL; /* first: a port fault also leaves the config unread */
    } else if (!config_ok(cfg)) {
        reason = IMU_REASON_CONFIG;
    } else if (a->overrun) {
        reason = IMU_REASON_OVERRUN;
    } else if (a->backlog) {
        reason = IMU_REASON_BACKLOG;
    } else if (!had_last) {
        reason = IMU_REASON_START;
    } else if (count_too_low(now_ms - start_ms, cfg->odr_hz, a->count)) {
        reason = IMU_REASON_STALE;
    } else if (a->saturated) {
        reason = IMU_REASON_SATURATED;
    } else {
        /* all checks passed: reason stays NONE */
    }
    if (reason != IMU_REASON_NONE) {
        window_close(c);
        return imu_invalid(now_ms, a->count, reason);
    }

    const float n = (float)a->count;
    float mean[3];
    bool finite = true;
    for (uint32_t i = 0u; i < 3u; i++) {
        mean[i] = ((float)a->sum[i] / n) * cfg->gyro_rps_per_lsb;
        if (!isfinite(mean[i])) {
            finite = false;
        }
    }
    if (!finite) {
        /* a finite but absurd scale (MISRA Dir 4.1) */
        window_close(c);
        return imu_invalid(now_ms, a->count, IMU_REASON_CONFIG);
    }

    if (standstill) {
        window_add(c, mean, start_ms, now_ms);
    } else {
        window_close(c);
    }
    if (!c->offset_learned) {
        return imu_invalid(now_ms, a->count,
                       c->offset_rejected ? IMU_REASON_OFFSET_RANGE : IMU_REASON_OFFSET_UNLEARNED);
    }

    imu_sample_t s;
    s.gyro_x_rps = mean[0];
    s.gyro_y_rps = mean[1];
    s.gyro_z_rps = mean[2] - c->offset_z_rps;
    s.stamp_ms = now_ms;
    s.count = a->count;
    s.state = IMU_SAMPLE_VALID;
    s.reason = IMU_REASON_NONE;
    return s;
}
