#ifndef FEATURES_CORNERING_LEAN_CORE_H
#define FEATURES_CORNERING_LEAN_CORE_H

/*
 * Pure lean (roll) estimator of rt-core's cornering Layer 2 (D-065 item 1): a 2-state
 * EKF over [roll, roll-rate gyro bias]. No HAL, RTOS, services or clock access: the
 * glue (D-065 PR 3) passes the IMU sample, the speed snapshot and their stamps, and
 * maps the output to the gen/ 0x020 fields.
 *
 * Model (pitch about 0, a documented limit):
 *   predict  roll += (p - bias) * dt, with p the body roll-rate gyro
 *   measure  the coordinated turn tan(roll) = v * psi_dot / g, with psi_dot the earth
 *            yaw rate. At pitch 0 the body z gyro reads r = psi_dot * cos(roll), so the
 *            measurement is z = r with h(x) = (g / v) * sin(roll). (atan(v * r / g)
 *            would read atan(sin(roll)), 35 deg for a 45 deg lean.)
 * Body axes: x forward, y right, z down; roll is positive leaning right, and r is
 * positive turning right, matching 0x020's LEAN_ANGLE sign.
 *
 * Output rules (D-065 item 1, safety-reviewer MAJOR-1 on defs v0.9.0): every invalid
 * condition (non-finite input or state, failed innovation gate, large roll variance,
 * unusable speed) gives LEAN_OUT_INVALID with roll 0 and quality 0. The core never
 * produces CLAMPED: the glue clamps the returned copy to the gen/ clamp limit, so the
 * clamp can never reach the EKF state.
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Constants. Physics and unit constants are exact; the speed age and margin come from
 * gen/ platform_limits.h (lean_core.c). Every value marked provisional (D-065 item 1)
 * is revisited with the T1/T4 measurements; each comment gives the rationale.
 */
#define LEAN_GRAVITY_MPS2 (9.80665f)     /* standard gravity */
#define LEAN_DEG_PER_RAD (57.295780f)
#define LEAN_ROLL_MAX_RAD (1.5707964f)   /* 90 deg: h = sin(roll) is monotone only below it */

/* Gyro sample plausibility: 2000 deg/s, the largest common MEMS full scale. A larger
 * reading is a corrupt sample; services/imu (PR 3) also rejects saturation at the real
 * sensor's range. */
#define LEAN_RATE_MAX_RPS (34.906586f)

/* Gap between IMU samples (provisional): two nominal EKF periods (20 ms, D-064 item 4),
 * so one missed cycle is predicted over; a longer gap loses the roll integration and
 * reinitialises the filter. */
#define LEAN_DT_MAX_MS (40u)

/* How far the speed stamp may lie after the IMU stamp (provisional): the IMU sample can
 * be up to one EKF period plus jitter older than the newest speed write, so a stamp up to
 * this far ahead reads as age 0; further ahead it is unusable, never a wrapped age. */
#define LEAN_SPEED_FUTURE_MAX_MS (40u)

/* Minimum speed (provisional), checked against speed - age * margin: below about
 * 18 km/h the steady coordinated turn no longer holds (counter-steering at walking pace,
 * feet down) and g/v amplifies the yaw-rate noise. */
#define LEAN_SPEED_MIN_MPS (5.0f)

/* Process noise (provisional): roll-rate gyro white noise including engine vibration,
 * about 0.6 deg/s/sqrt(Hz) (rad^2/s); bias random walk (rad^2/s^3). */
#define LEAN_Q_ROLL (1.0e-4f)
#define LEAN_Q_BIAS (1.0e-7f)

/* Measurement noise on the yaw rate (provisional): about 4 deg/s, the z gyro noise plus
 * the departure from a steady coordinated turn in transients ((rad/s)^2). */
#define LEAN_R_YAW (5.0e-3f)

/* Initial covariance (provisional): roll (30 deg)^2, bias (2 deg/s)^2. Large enough that
 * a fresh filter reads INVALID (variance) until measurements converge it. */
#define LEAN_P0_ROLL (0.27415568f)
#define LEAN_P0_BIAS (1.2184697e-3f)

/* Transient model error (provisional): in a turn-in the yaw rate lags the roll by about
 * the steering response, so the steady-turn measurement is off by about
 * H0 * lag * roll rate; it is added to R. Above LEAN_BIAS_LEARN_RATE_MAX_RPS of roll rate
 * the bias is not updated, so a turn-in never leaks into it (safety-reviewer MAJOR-2). */
#define LEAN_YAW_LAG_S (0.2f)
#define LEAN_BIAS_LEARN_RATE_MAX_RPS (0.17453292f) /* 10 deg/s */
/* The freeze outlasts the last high-rate step by three lags, while the lagging yaw rate
 * settles (safety-reviewer MINOR-1 on the fixes). */
#define LEAN_BIAS_HOLD_MS (600u)

/* Innovation gate: normalised innovation squared above 9 (3 sigma, 1 DOF) rejects the
 * update. A reject run longer than LEAN_GATE_REJECT_MAX_MS (provisional: five EKF
 * cycles, longer than a steering transient's model error at 3 sigma) means the filter
 * diverged: INVALID and reinit. */
#define LEAN_GATE_NIS (9.0f)
#define LEAN_GATE_REJECT_MAX_MS (100u)

/* Bias plausibility (provisional): a roll-rate bias estimate above 5 deg/s is a diverged
 * filter, not a MEMS gyro after its power-on offset. */
#define LEAN_BIAS_MAX_RPS (0.087266463f)

/* QUALITY from the roll standard deviation (provisional): 100 % at or below 1 deg,
 * linear down to 1 % at 5 deg; above 5 deg INVALID (VARIANCE). ESTIMATED never sends 0
 * (0 = no confidence). */
#define LEAN_SIGMA_GOOD_RAD (0.017453292f)
#define LEAN_SIGMA_MAX_RAD (0.087266463f)
#define LEAN_QUALITY_MIN (1u)
#define LEAN_QUALITY_MAX (100u)

/* QUALITY cap while a gate reject run is open (user, 2026-10-10; architecture-guard
 * MAJOR-1 on the plan): the roll then rests on the gyro alone, so ESTIMATED stays but
 * reads at most 50 %, until an accepted update or the run's end (INVALID, reinit). */
#define LEAN_QUALITY_REJECT_MAX (50u)

/* One IMU sample: body rates in rad/s and its timebase stamp. Contract for the producer
 * (services/imu, PR 3; safety-reviewer MAJOR-4): each rate is the mean over
 * (previous stamp, stamp], both filtered alike, never a point sample, since the
 * single cylinder's vibration would alias at the EKF rate; the z offset is removed at
 * standstill (it is not estimated here). */
typedef struct {
    float gyro_x_rps; /* body x (roll-rate) gyro, + rolling right */
    float gyro_z_rps; /* body z gyro, + turning right; NOT the earth yaw rate (header) */
    uint32_t stamp_ms;
} lean_imu_t;

/* Speed snapshot state. An allow-list: zero and any unknown value are unusable. */
typedef enum {
    LEAN_SPEED_UNUSABLE = 0,
    LEAN_SPEED_VALID = 1
} lean_speed_state_t;

/* The speed as it crosses tasks (D-065 item 2, architecture-guard BLOCKER-1 on the
 * plan): physical value, absolute stamp and state. The core computes the age itself at
 * every step, so a stalled producer makes the lean INVALID within the max age. */
typedef struct {
    float speed_mps;
    uint32_t stamp_ms; /* timebase ms of the sample (its request's send time, D-051) */
    lean_speed_state_t state;
} lean_speed_t;

/* Output state. The zero value is the safe one. Its values differ from the gen/ 0x020
 * choices (ESTIMATED 0, INVALID 3): the glue maps it with a switch, never a cast, to
 * ESTIMATED (or CLAMPED after its output clamp) or INVALID, never RESERVED. */
typedef enum {
    LEAN_OUT_INVALID = 0,
    LEAN_OUT_ESTIMATED = 1
} lean_out_state_t;

/* Why a step read INVALID: the first failing check, in step order (diagnostics/tests). */
typedef enum {
    LEAN_REASON_NONE = 0,  /* ESTIMATED */
    LEAN_REASON_ARG,       /* NULL argument */
    LEAN_REASON_INPUT,     /* non-finite or out-of-range gyro sample: step skipped */
    LEAN_REASON_DT,        /* first step, a gap above the limit or a backward stamp: reinit */
    LEAN_REASON_DUPLICATE, /* same stamp as the last sample: skipped, state kept */
    LEAN_REASON_FILTER,    /* non-finite or implausible filter state: reinit */
    LEAN_REASON_GATE,      /* innovation gate rejected too long: diverged, reinit */
    LEAN_REASON_VARIANCE,  /* roll variance too large (also: not yet converged) */
    LEAN_REASON_SPEED,     /* no usable speed: no measurement */
    LEAN_REASON_ROLL_RANGE /* |roll| at or beyond 90 deg: outside the model, restart */
} lean_reason_t;

/* One step's result, returned by value: clamping it never touches the filter. */
typedef struct {
    float roll_deg;         /* 0 when INVALID */
    uint8_t quality;        /* 0..100 % from the roll variance; 0 when INVALID */
    lean_out_state_t state;
    lean_reason_t reason;
} lean_out_t;

/* Filter state, owned by the caller (static). A zeroed object is not ready: its first
 * step reinitialises it and reads INVALID, so P = 0 never passes as certainty. */
typedef struct {
    float roll;             /* rad */
    float bias;             /* rad/s, roll-rate gyro bias */
    float p00;              /* covariance: var(roll), rad^2 */
    float p01;              /* cov(roll, bias) */
    float p11;              /* var(bias), (rad/s)^2 */
    uint32_t last_ms;       /* stamp of the last accepted IMU sample */
    uint32_t reject_since_ms; /* first gate reject of the current run */
    uint32_t fast_ms;       /* last update step with |roll rate| above the bias-learn limit */
    bool ready;             /* set by init/reinit; false in the zero state */
    bool have_last;         /* last_ms holds a stamp */
    bool rejecting;         /* a gate reject run is open (held by steps without a usable speed) */
    bool bias_hold;         /* bias frozen: fast roll now or within LEAN_BIAS_HOLD_MS of fast_ms */
} lean_core_t;

/* Resets the filter: roll 0, bias 0, the initial covariance, no last stamp. */
void lean_core_init(lean_core_t* c);

/*
 * One EKF cycle for an IMU sample, with the latest speed snapshot. Checks, first
 * failing reason wins: NULL arguments; the gyro sample (non-finite or beyond the rate
 * limit: no predict, the last stamp is kept so the gap is caught next step); the gap
 * since the last sample (first step, or outside 1 ms..LEAN_DT_MAX_MS: reinit, no
 * predict); predict; filter health (reinit); the speed (state, finite, age, minimum
 * speed) and, with a usable speed, the measurement update behind the innovation gate
 * (a reject run longer than LEAN_GATE_REJECT_MAX_MS: diverged, reinit; an accepted
 * update ends the run; a step without a usable speed holds it, so its age keeps
 * counting) and filter health again, or without a speed the roll prior reset once p00
 * exceeds LEAN_P0_ROLL; the roll range (restart); the roll variance; the speed. A
 * duplicate stamp (dt 0) is skipped like a bad sample, state kept (the glue calls the
 * core only for a new sample, so this is a guard, not a path).
 * Bounded: no loop, a fixed number of float operations.
 */
lean_out_t lean_core_step(lean_core_t* c, const lean_imu_t* imu, const lean_speed_t* speed);

/* QUALITY (0..100 %) for a roll standard deviation in rad: LEAN_QUALITY_MAX at or below
 * LEAN_SIGMA_GOOD_RAD, linear (truncated) down to LEAN_QUALITY_MIN at LEAN_SIGMA_MAX_RAD;
 * 0 for a negative, non-finite or larger value. lean_core_step() caps it at
 * LEAN_QUALITY_REJECT_MAX while a gate reject run is open. */
uint8_t lean_core_quality(float roll_sigma_rad);

#ifdef __cplusplus
}
#endif

#endif /* FEATURES_CORNERING_LEAN_CORE_H */
