/*
 * Pure 2-state roll EKF (D-065 item 1). See lean_core.h for the model, the check order
 * and the output rules; features/cornering/README.md for the rationale and limits.
 */
#include "features/cornering/lean_core.h"

#include "platform_limits.h"

#include <math.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

/* isfinite() and the NaN-false comparisons below need IEEE semantics, and sqrtf() must
 * not set errno (D-065 item 1: -fno-math-errno, never -ffast-math). */
#if defined(__FAST_MATH__)
#error "lean_core.c must not be built with -ffast-math"
#endif
#if defined(__GNUC__) && !defined(__NO_MATH_ERRNO__)
#error "lean_core.c needs -fno-math-errno"
#endif

#define LEAN_MS_TO_S (0.001f)

/* The speed's own uncertainty grows with its age (vehicle_speed rule, gen/). */
#define LEAN_SPEED_MAX_AGE_MS PLATFORM_LIMIT_VEHICLE_SPEED_MAX_AGE_MS
#define LEAN_SPEED_ACCEL_MARGIN_MPS2 PLATFORM_LIMIT_VEHICLE_SPEED_ACCEL_MARGIN_MPS2

static void reinit(lean_core_t* c)
{
    c->roll = 0.0f;
    c->bias = 0.0f;
    c->p00 = LEAN_P0_ROLL;
    c->p01 = 0.0f;
    c->p11 = LEAN_P0_BIAS;
    c->last_ms = 0u;
    c->reject_since_ms = 0u;
    c->ready = true;
    c->have_last = false;
    c->rejecting = false;
}

/* Reinit, keeping now_ms as the last stamp so the next sample can predict. */
static void restart(lean_core_t* c, uint32_t now_ms)
{
    reinit(c);
    c->last_ms = now_ms;
    c->have_last = true;
}

static lean_out_t invalid(lean_reason_t reason)
{
    const lean_out_t out = {0.0f, 0u, LEAN_OUT_INVALID, reason};
    return out;
}

static bool finite(float x)
{
    return isfinite(x) != 0;
}

static bool rate_ok(float rate_rps)
{
    bool ok = false;
    if (finite(rate_rps)) {
        ok = fabsf(rate_rps) < LEAN_RATE_MAX_RPS;
    }
    return ok;
}

/* Finite state, positive variances, positive semi-definite covariance, plausible bias.
 * Checked in the order predict propagates a non-finite value (bias into roll; p11 into
 * p01 and p00; p01 into p00), so each check is reachable and tested. */
static bool healthy(const lean_core_t* c)
{
    bool ok = finite(c->bias) && finite(c->roll) && finite(c->p11) && finite(c->p01) &&
              finite(c->p00);
    if (ok) {
        const float det = (c->p00 * c->p11) - (c->p01 * c->p01);
        ok = (c->p00 > 0.0f) && (c->p11 > 0.0f) && (det >= 0.0f) &&
             (fabsf(c->bias) <= LEAN_BIAS_MAX_RPS);
    }
    return ok;
}

/*
 * Age of a speed stamp, wrap-safe without signed casts (E-17): up to the max age behind
 * now it is that age; up to LEAN_SPEED_FUTURE_MAX_MS ahead it is 0; anything else
 * (older, or further ahead, which unsigned subtraction would turn into a huge age) is
 * unusable.
 */
static bool speed_age_ms(uint32_t now_ms, uint32_t stamp_ms, uint32_t* age_ms)
{
    const uint32_t behind = now_ms - stamp_ms;
    const uint32_t ahead = stamp_ms - now_ms;
    bool ok = true;
    if (behind <= LEAN_SPEED_MAX_AGE_MS) {
        *age_ms = behind;
    } else if (ahead <= LEAN_SPEED_FUTURE_MAX_MS) {
        *age_ms = 0u;
    } else {
        ok = false;
    }
    return ok;
}

/* A usable speed: VALID, finite, fresh, and at least the minimum even after the
 * age-grown margin. sigma_v is that margin (m/s), used as the speed's uncertainty. */
static bool speed_usable(const lean_speed_t* s, uint32_t now_ms, float* sigma_v)
{
    uint32_t age_ms = 0u;
    bool ok = false;
    if ((s->state == LEAN_SPEED_VALID) && finite(s->speed_mps)) {
        ok = speed_age_ms(now_ms, s->stamp_ms, &age_ms);
    }
    if (ok) {
        const float age_s = (float)age_ms * LEAN_MS_TO_S;
        const float margin = age_s * LEAN_SPEED_ACCEL_MARGIN_MPS2;
        ok = (s->speed_mps - margin) >= LEAN_SPEED_MIN_MPS;
        *sigma_v = margin;
    }
    return ok;
}

static void predict(lean_core_t* c, float gyro_x_rps, float dt_s)
{
    const float p00 = c->p00;
    const float p01 = c->p01;
    const float p11 = c->p11;
    c->roll += (gyro_x_rps - c->bias) * dt_s;
    /* P = F P F' + Q, F = [1 -dt; 0 1], Q = diag(q_roll dt, q_bias dt) */
    c->p00 = ((p00 - (2.0f * dt_s * p01)) + (dt_s * dt_s * p11)) + (LEAN_Q_ROLL * dt_s);
    c->p01 = p01 - (dt_s * p11);
    c->p11 = p11 + (LEAN_Q_BIAS * dt_s);
}

/*
 * Measurement z = gyro_z, h = (g/v) sin(roll), H = [(g/v) cos(roll), 0]; the speed's
 * uncertainty enters through dh/dv = -h/v. Behind the innovation gate; Joseph-form
 * covariance update. Needs a healthy state (p00 > 0) and v >= LEAN_SPEED_MIN_MPS, so
 * every division is by a positive value. Returns false when the reject run is longer
 * than LEAN_GATE_REJECT_MAX_MS (diverged).
 */
static bool measure(lean_core_t* c, float gyro_z_rps, float v_mps, float sigma_v,
                    uint32_t now_ms)
{
    const float g_v = LEAN_GRAVITY_MPS2 / v_mps;
    const float h = g_v * sinf(c->roll);
    const float h0 = g_v * cosf(c->roll);
    const float dh_dv = -h / v_mps;
    const float r_eff = LEAN_R_YAW + ((dh_dv * dh_dv) * (sigma_v * sigma_v));
    const float s = ((h0 * h0) * c->p00) + r_eff;
    const float nu = gyro_z_rps - h;
    bool ok = true;

    if ((nu * nu) > (LEAN_GATE_NIS * s)) {
        if (!c->rejecting) {
            c->rejecting = true;
            c->reject_since_ms = now_ms;
        } else if ((now_ms - c->reject_since_ms) > LEAN_GATE_REJECT_MAX_MS) {
            ok = false;
        } else {
            /* run still within its window: gyro propagation only */
        }
    } else {
        const float p00 = c->p00;
        const float p01 = c->p01;
        const float p11 = c->p11;
        const float k0 = (p00 * h0) / s;
        const float k1 = (p01 * h0) / s;
        const float a = 1.0f - (k0 * h0);
        const float b = -(k1 * h0);
        c->rejecting = false;
        c->roll += k0 * nu;
        c->bias += k1 * nu;
        /* P = A P A' + R K K', A = I - K H = [a 0; b 1] */
        c->p00 = ((a * a) * p00) + (r_eff * (k0 * k0));
        c->p01 = (a * ((b * p00) + p01)) + (r_eff * (k0 * k1));
        c->p11 = ((((b * b) * p00) + ((2.0f * b) * p01)) + p11) + (r_eff * (k1 * k1));
    }
    return ok;
}

uint8_t lean_core_quality(float roll_sigma_rad)
{
    uint8_t q = 0u;
    if (finite(roll_sigma_rad) && (roll_sigma_rad >= 0.0f) && (roll_sigma_rad <= LEAN_SIGMA_MAX_RAD)) {
        q = (uint8_t)LEAN_QUALITY_MAX;
        if (roll_sigma_rad > LEAN_SIGMA_GOOD_RAD) {
            const float frac = (LEAN_SIGMA_MAX_RAD - roll_sigma_rad) /
                               (LEAN_SIGMA_MAX_RAD - LEAN_SIGMA_GOOD_RAD);
            const float span = (float)LEAN_QUALITY_MAX - (float)LEAN_QUALITY_MIN;
            const float scaled = frac * span; /* in [0, span) */
            q = (uint8_t)((uint8_t)scaled + (uint8_t)LEAN_QUALITY_MIN);
        }
    }
    return q;
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: called by the EKF task glue (D-065 PR 3) and the tests
void lean_core_init(lean_core_t* c)
{
    if (c != NULL) {
        reinit(c);
    }
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: called by the EKF task glue (D-065 PR 3) and the tests
lean_out_t lean_core_step(lean_core_t* c, const lean_imu_t* imu, const lean_speed_t* speed)
{
    lean_out_t out;
    float sigma_v = 0.0f;
    bool have_speed = false;

    if ((c == NULL) || (imu == NULL) || (speed == NULL)) {
        return invalid(LEAN_REASON_ARG);
    }
    if (!c->ready) {
        reinit(c); /* a zeroed object: never read P = 0 as certainty */
    }
    if (!rate_ok(imu->gyro_x_rps) || !rate_ok(imu->gyro_z_rps)) {
        return invalid(LEAN_REASON_INPUT); /* last_ms kept: the gap is seen next step */
    }
    if (!c->have_last) {
        restart(c, imu->stamp_ms);
        return invalid(LEAN_REASON_DT);
    }
    {
        const uint32_t dt_ms = imu->stamp_ms - c->last_ms;
        if ((dt_ms == 0u) || (dt_ms > LEAN_DT_MAX_MS)) {
            restart(c, imu->stamp_ms);
            return invalid(LEAN_REASON_DT);
        }
        c->last_ms = imu->stamp_ms;
        predict(c, imu->gyro_x_rps, (float)dt_ms * LEAN_MS_TO_S);
    }
    if (!healthy(c)) {
        restart(c, imu->stamp_ms);
        return invalid(LEAN_REASON_FILTER);
    }
    have_speed = speed_usable(speed, imu->stamp_ms, &sigma_v);
    if (have_speed) {
        if (!measure(c, imu->gyro_z_rps, speed->speed_mps, sigma_v, imu->stamp_ms)) {
            restart(c, imu->stamp_ms);
            return invalid(LEAN_REASON_GATE);
        }
        if (!healthy(c)) {
            restart(c, imu->stamp_ms);
            return invalid(LEAN_REASON_FILTER);
        }
    }
    {
        /* healthy: p00 > 0, so sqrtf gets a positive finite value (Dir 4.11) */
        const float sigma = sqrtf(c->p00);
        if (sigma > LEAN_SIGMA_MAX_RAD) {
            return invalid(LEAN_REASON_VARIANCE);
        }
        if (!have_speed) {
            return invalid(LEAN_REASON_SPEED);
        }
        if (fabsf(c->roll) >= LEAN_ROLL_MAX_RAD) {
            return invalid(LEAN_REASON_ROLL_RANGE);
        }
        out.roll_deg = c->roll * LEAN_DEG_PER_RAD;
        out.quality = lean_core_quality(sigma);
        if (c->rejecting && (out.quality > (uint8_t)LEAN_QUALITY_REJECT_MAX)) {
            out.quality = (uint8_t)LEAN_QUALITY_REJECT_MAX;
        }
        out.state = LEAN_OUT_ESTIMATED;
        out.reason = LEAN_REASON_NONE;
    }
    return out;
}
