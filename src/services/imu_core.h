#ifndef SERVICES_IMU_CORE_H
#define SERVICES_IMU_CORE_H

/*
 * Pure part of services/imu (D-065 item 2, PR 3a): turns the raw gyro samples of one
 * EKF period into one interval-mean sample for the lean core, and learns the z gyro
 * offset at standstill. No HAL, RTOS or clock access: the glue (services/imu.c) drains
 * the hal/imu.h FIFO into an imu_acc_t and passes the time.
 *
 * Interval mean (safety-reviewer MAJOR-4 on PR 2, user 2026-10-10): each rate is the
 * mean of all samples in (previous stamp, stamp], never a point sample. The single
 * cylinder's firing and crank vibration (about 12-150 Hz) would alias at the 50 Hz EKF
 * rate. The mean is a boxcar over the period: zero gain at 50 Hz and its multiples and
 * |sin(pi f T) / (pi f T)| elsewhere (0.64 at 25 Hz, 0.21 at 75 Hz), on top of the IMU's
 * own digital low-pass filter (hal/imu.h).
 *
 * z offset at standstill (user, 2026-10-10): an unremoved z gyro offset b makes the lean
 * read about v * b / g low in one turn direction (lean_core README). The offset is the
 * mean of the z interval means over a window of IMU_OFFSET_WINDOW_MS during which the
 * caller reported standstill without a break and the x, y and z means each stayed within
 * IMU_OFFSET_SPREAD_MAX_RPS of themselves. It is accepted only within IMU_OFFSET_MAX_RPS
 * and is relearnt at every such window (temperature drift). Until the first accepted
 * window, and after a rejected one, every sample is INVALID (fail-closed, user).
 *
 * A wrong learn is the risk, not the offset itself: the CL250 speed has 1 km/h steps, so a
 * bike pushed and turned at walking pace can read 0. Turning it at a constant rate for the
 * whole window would be learnt as offset. The three-axis stillness makes that harder (a
 * pushed bike also rolls and pitches) and IMU_OFFSET_MAX_RPS bounds what is left: a
 * wrong offset b reads the lean about v * b / g low in one turn direction, 3.1 deg at
 * 15 m/s for 2 deg/s (architecture-guard MAJOR-1 on the PR 3 plan, user 2026-10-10).
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Output data rate the core accepts (user, 2026-10-10: at least 800 Hz with the on-chip
 * filter). The upper bound keeps one EKF gap of 40 ms within IMU_DRAIN_MAX samples. */
#define IMU_ODR_MIN_HZ (800u)
#define IMU_ODR_MAX_HZ (1600u)

/* Samples drained per read at most (bounded loop): 40 ms (lean_core's largest accepted
 * gap) at IMU_ODR_MAX_HZ. A longer backlog is flushed and the sample is INVALID. */
#define IMU_DRAIN_MAX (64u)

/* A period needs at least this share of elapsed time * ODR samples, else the sample is
 * STALE: a mean of a few samples would be close to a point sample again. */
#define IMU_COUNT_MIN_PERCENT (50u)

/* At most this share, plus the samples of IMU_SAMPLE_LATENCY_MAX_MS (rounded up), plus
 * IMU_COUNT_SLACK, else EXCESS (safety-reviewer MAJOR-1 on PR 3a): samples that reached
 * the FIFO late, e.g. batched by a large watermark, would shift the mean against its
 * stamp and make the roll rate lag at turn-in. The latency term keeps a short catch-up
 * period (a late EKF step followed by an early one) from reading EXCESS while the port
 * keeps its contract (MINOR-B on the fixes); the slack covers the 1 ms resolution of both
 * stamps. A backstop only: a batch shorter than half a period passes, so the port's
 * latency contract is the real control and is measured on HIL (MINOR-C). */
#define IMU_COUNT_MAX_PERCENT (150u)
#define IMU_SAMPLE_LATENCY_MAX_MS (2u) /* hal/imu.h port contract */
#define IMU_COUNT_SLACK (2u)

/* Stuck-at check (safety-reviewer MINOR-1 on PR 3a, MINOR-D on the fixes; provisional
 * until T1): a gyro whose raw samples stay bit-identical on all three axes for this many
 * periods in a row is frozen. Riding, engine vibration makes even one identical period
 * implausible, so 3 periods (60 ms); at standstill with the engine off a quiet part may
 * legitimately repeat, so 25 (0.5 s). A period needs at least 2 samples to count as
 * evidence either way. The state survives a port re-init until a sample changes (a
 * re-initialised part that still repeats stays FROZEN). Per-axis stuck detection on x/z
 * and the engine-off LSB spread are T1 items (cornering README). */
#define IMU_FROZEN_PERIODS_RIDING (3u)
#define IMU_FROZEN_PERIODS_STILL (25u)

/* Plausible scale (safety-reviewer MINOR-2 on PR 3a, MINOR-A on the fixes): the full
 * scale implied by the reported rad/s per LSB must lie within 450..2600 deg/s, a margin
 * around the nominal 500..2000 deg/s settings, whose datasheet sensitivities give 499.5
 * (BMI270, 65.6 LSB/(deg/s)) up to 2294 deg/s (LSM6DSO, 70 mdps/LSB). A driver that
 * reports a scale for a full scale other than the sensor's would read the rates several
 * times off; the port also reads its full-scale and ODR registers back at init. */
#define IMU_SCALE_MIN_RPS_PER_LSB (2.3968450e-4f) /* 450 deg/s / 32768 LSB */
#define IMU_SCALE_MAX_RPS_PER_LSB (1.3848437e-3f) /* 2600 deg/s / 32768 LSB */

/* z offset learning (user, 2026-10-10; provisional until T1): 2 s window, x/y/z means
 * each within 1 deg/s, accepted offset at most 2 deg/s. More means a faulty gyro or a
 * vehicle that is turning; the rt-core IMU must therefore have a zero-rate offset well
 * below 2 deg/s (e.g. LSM6DSO about 1, BMI270 about 0.5 deg/s; not an MPU-6050). */
#define IMU_OFFSET_WINDOW_MS (2000u)
#define IMU_OFFSET_SPREAD_MAX_RPS (0.017453292f) /* 1 deg/s */
#define IMU_OFFSET_MAX_RPS (0.034906585f)        /* 2 deg/s */

/* Sample state. An allow-list: zero and any unknown value are unusable. */
typedef enum {
    IMU_SAMPLE_INVALID = 0,
    IMU_SAMPLE_VALID = 1
} imu_state_t;

/* Why a sample is INVALID: the first failing check, in imu_core_finish() order. */
typedef enum {
    IMU_REASON_NONE = 0,     /* VALID */
    IMU_REASON_ARG,          /* NULL argument: nothing changed */
    IMU_REASON_NO_NEW,       /* same ms as the previous period's end: nothing changed */
    IMU_REASON_HAL,          /* the port reported a fault */
    IMU_REASON_CONFIG,       /* scale or ODR outside its range */
    IMU_REASON_OVERRUN,      /* samples of this period were lost */
    IMU_REASON_BACKLOG,      /* more than IMU_DRAIN_MAX samples waited: flushed */
    IMU_REASON_START,        /* first period: no previous stamp */
    IMU_REASON_STALE,        /* no new sample, or too few for the elapsed time */
    IMU_REASON_EXCESS,       /* more samples than the elapsed time allows: late/batched */
    IMU_REASON_SATURATED,    /* a sample at the sensor's full scale */
    IMU_REASON_FROZEN,       /* raw samples bit-identical for IMU_FROZEN_PERIODS_* periods */
    IMU_REASON_OFFSET_UNLEARNED, /* no z offset learnt yet */
    IMU_REASON_OFFSET_RANGE  /* the last window's z offset was beyond IMU_OFFSET_MAX_RPS */
} imu_reason_t;

/* One period's sample. All rates 0 when INVALID. */
typedef struct {
    float gyro_x_rps;   /* body x (roll rate), interval mean */
    float gyro_y_rps;   /* body y (pitch rate), interval mean */
    float gyro_z_rps;   /* body z, interval mean minus the learnt offset */
    uint32_t stamp_ms;  /* the period's end: the time of the read */
    uint16_t count;     /* raw samples in the mean */
    imu_state_t state;
    imu_reason_t reason;
} imu_sample_t;

/* Running configuration, copied from hal_imu_config_t by the glue. */
typedef struct {
    float gyro_rps_per_lsb;
    uint16_t odr_hz;
} imu_config_t;

/* One period's raw samples, filled by the glue. */
typedef struct {
    int32_t sum[3];  /* |sum| <= IMU_DRAIN_MAX * 32768: no overflow */
    uint16_t count;
    int16_t min[3];  /* smallest raw sample per axis (valid while count > 0) */
    int16_t max[3];  /* largest raw sample per axis */
    bool saturated;  /* a sample at INT16_MIN or INT16_MAX on any axis */
    bool hal_error;  /* set by the glue */
    bool overrun;    /* set by the glue */
    bool backlog;    /* set by the glue */
} imu_acc_t;

/* Core state, owned by the glue (static). The zero state is safe: no previous stamp and
 * no offset learnt, so the first samples are INVALID. */
typedef struct {
    uint32_t last_ms;       /* end of the previous period */
    bool have_last;
    float offset_z_rps;     /* learnt z offset */
    bool offset_learned;
    bool offset_rejected;   /* the last window was rejected (reason OFFSET_RANGE) */
    uint32_t frozen_periods; /* periods in a row with bit-identical raw samples */
    bool win_open;          /* a learning window is running */
    uint32_t win_start_ms;  /* start of its first period */
    uint32_t win_count;     /* z means in it */
    float win_sum;          /* sum of the z means, rad/s */
    float win_min[3];       /* x, y, z: smallest mean in the window */
    float win_max[3];       /* x, y, z: largest mean in the window */
    uint32_t learn_count;   /* accepted windows since init (diagnostics/tests) */
    uint32_t reject_count;  /* rejected windows since init */
} imu_core_t;

/* No previous stamp, no offset, no window. */
void imu_core_init(imu_core_t* c);

/* False when now_ms is the end of the previous period (a catch-up call in the same ms):
 * the glue then returns imu_core_finish()'s NO_NEW without draining the FIFO, so no
 * sample is taken out of its period (architecture-guard MINOR-5). NULL reads false. */
bool imu_core_new_period(const imu_core_t* c, uint32_t now_ms);

/* Empty accumulator, no flags. */
void imu_acc_reset(imu_acc_t* a);

/* Adds one raw sample [x, y, z]. Ignored once count reached IMU_DRAIN_MAX (the glue never
 * adds more; this keeps the sums bounded for any caller). */
void imu_acc_add(imu_acc_t* a, const int16_t gyro[3]);

/*
 * Ends the period at now_ms. Checks in order, the first failure wins: arguments, the same
 * ms as the previous end (NO_NEW: nothing changed, the period goes on), HAL fault,
 * config, overrun, backlog, first period, stale (count 0 or below IMU_COUNT_MIN_PERCENT
 * of (now - previous stamp) * ODR), excess (above IMU_COUNT_MAX_PERCENT of it plus
 * the latency and the slack), saturation, frozen; then the means, the offset learning
 * (with standstill) and the offset state. standstill: the caller saw a VALID speed of 0
 * that is within its age limit. Every result but ARG and NO_NEW ends the period, so the next one
 * starts at now_ms. A failure before the means, or a period without standstill, closes the
 * learning window; the offset states (UNLEARNED, RANGE) do not, since learning is how
 * they end.
 */
imu_sample_t imu_core_finish(imu_core_t* c, const imu_acc_t* a, const imu_config_t* cfg,
                             uint32_t now_ms, bool standstill);

#ifdef __cplusplus
}
#endif

#endif /* SERVICES_IMU_CORE_H */
