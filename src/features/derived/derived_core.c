/*
 * Pure derived-signal core (D-070). See derived_core.h for the sampling rule and
 * features/derived/README.md for the rationale. The formulas follow the Python reference
 * (vehicle-server src/vehicle_server/derived/formulas.py), input by input in the same order, so a
 * flag is the one the reference gives when more than one input is unusable.
 */
#include "features/derived/derived_core.h"

#include "features/derived/derived_gear.h"
#include "vehicle_cl250_constants.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/* isfinite() and the NaN-false comparisons need IEEE semantics, logf() must not set errno
 * (like lean_core.c: -fno-math-errno, never -ffast-math). */
#if defined(__FAST_MATH__)
#error "derived_core.c must not be built with -ffast-math"
#endif
#if defined(__GNUC__) && !defined(__NO_MATH_ERRNO__)
#error "derived_core.c needs -fno-math-errno"
#endif

#define RING_SPEED (0u)
#define RING_FUEL_RATE (1u)
#define RING_COOLANT (2u)
#define RING_OIL (3u)
#define RING_ACCEL (4u)
#define RING_NONE (0xFFu)

#define KIND_NONE (0u)
#define KIND_POINT (1u)
#define KIND_INTEGRAL (2u)
#define KIND_EVENT (3u)
#define KIND_RIDE (4u)

#define HALF_RANGE (0x7FFFFFFFu)

_Static_assert(VEHICLE_CL250_DERIVED_COUNT <= 255u, "signal indices are small");

typedef struct {
    derived_core_t* c;
    uint32_t now;
    uint32_t sig;
    const vehicle_cl250_derived_t* spec;
} ctx_t;

/* ---- time helpers (wrap-safe) -------------------------------------------------------- */

/* Age of a stamp: a stamp ahead of now (half range or more) is age 0, never a wrapped age. */
static uint32_t age_of(uint32_t now, uint32_t stamp)
{
    const uint32_t a = now - stamp;
    return (a > HALF_RANGE) ? 0u : a;
}

/* True when a is at or after b. */
static bool at_or_after(uint32_t a, uint32_t b)
{
    return (uint32_t)(a - b) <= HALF_RANGE;
}

static uint32_t later(uint32_t a, uint32_t b)
{
    return at_or_after(a, b) ? a : b;
}

static bool is_zero(float x)
{
    return (x <= 0.0f) && (x >= 0.0f);
}

static bool is_finite_f(float x)
{
    return isfinite(x) != 0;
}

/* ---- which signal has which formula ---------------------------------------------------- */

static uint32_t kind_of(uint32_t sig)
{
    uint32_t kind = KIND_POINT;
    switch (sig) {
    case VEHICLE_CL250_DERIVED_TRIP_DISTANCE:
    case VEHICLE_CL250_DERIVED_TRIP_TIME:
    case VEHICLE_CL250_DERIVED_IDLE_TIME:
        kind = KIND_INTEGRAL;
        break;
    case VEHICLE_CL250_DERIVED_STOP_EVENT:
        kind = KIND_EVENT;
        break;
    case VEHICLE_CL250_DERIVED_GEARBOX_RATIO_MEASURED:
    case VEHICLE_CL250_DERIVED_GEAR_ESTIMATE:
    case VEHICLE_CL250_DERIVED_VEHICLE_ACCEL:
    case VEHICLE_CL250_DERIVED_VEHICLE_JERK:
    case VEHICLE_CL250_DERIVED_ENGINE_TORQUE:
    case VEHICLE_CL250_DERIVED_DEMAND_TORQUE:
    case VEHICLE_CL250_DERIVED_TORQUE_GAP:
    case VEHICLE_CL250_DERIVED_ENGINE_POWER:
    case VEHICLE_CL250_DERIVED_AIR_DENSITY:
    case VEHICLE_CL250_DERIVED_POWER_CORRECTION_FACTOR:
    case VEHICLE_CL250_DERIVED_AIR_FUEL_RATIO:
    case VEHICLE_CL250_DERIVED_FUEL_RATE_MAF:
    case VEHICLE_CL250_DERIVED_FUEL_CONSUMPTION_INSTANT:
    case VEHICLE_CL250_DERIVED_FUEL_CONSUMPTION_AVG:
    case VEHICLE_CL250_DERIVED_FUEL_RANGE:
    case VEHICLE_CL250_DERIVED_BSFC:
    case VEHICLE_CL250_DERIVED_FUEL_PER_STROKE:
    case VEHICLE_CL250_DERIVED_TOTAL_FUEL_TRIM:
    case VEHICLE_CL250_DERIVED_THROTTLE_CHANNEL_RESIDUAL:
    case VEHICLE_CL250_DERIVED_THROTTLE_COMMAND_RESIDUAL:
    case VEHICLE_CL250_DERIVED_GRIP_CHANNEL_RESIDUAL:
    case VEHICLE_CL250_DERIVED_ALTITUDE_CHANGE:
    case VEHICLE_CL250_DERIVED_COOLANT_RATE:
    case VEHICLE_CL250_DERIVED_OIL_TEMP_RATE:
        kind = KIND_POINT;
        break;
    default:
        /* a class-B ride summary, or no formula yet */
        kind = (derived_ride_slot(sig) != DERIVED_RIDE_NO_SLOT) ? KIND_RIDE : KIND_NONE;
        break;
    }
    return kind;
}

/* ---- sample rings ---------------------------------------------------------------------- */

static derived_sample_t* ring_data(derived_core_t* c, uint32_t id, uint32_t* cap)
{
    derived_sample_t* d = NULL;
    switch (id) {
    case RING_SPEED:
        d = c->speed_win;
        *cap = DERIVED_WIN_SPEED_CAP;
        break;
    case RING_FUEL_RATE:
        d = c->fuel_rate_win;
        *cap = DERIVED_WIN_FUEL_RATE_CAP;
        break;
    case RING_COOLANT:
        d = c->coolant_win;
        *cap = DERIVED_WIN_COOLANT_CAP;
        break;
    case RING_OIL:
        d = c->oil_win;
        *cap = DERIVED_WIN_OIL_CAP;
        break;
    default:
        d = c->accel_win;
        *cap = DERIVED_WIN_ACCEL_CAP;
        break;
    }
    return d;
}

static void ring_push(derived_core_t* c, uint32_t id, float value, uint32_t stamp)
{
    uint32_t cap = 0u;
    derived_sample_t* const d = ring_data(c, id, &cap);
    derived_ring_t* const r = &c->ring[id];
    r->head = (uint16_t)(((uint32_t)r->head + 1u) % cap);
    d[r->head].value = value;
    d[r->head].stamp_ms = stamp;
    if ((uint32_t)r->count < cap) {
        r->count++;
    }
}

static uint32_t ring_of_did(uint32_t did)
{
    uint32_t id = RING_NONE;
    switch (did) {
    case (uint32_t)VEHICLE_CL250_IDX_VEHICLE_SPEED:
        id = RING_SPEED;
        break;
    case (uint32_t)VEHICLE_CL250_IDX_ENGINE_FUEL_RATE:
        id = RING_FUEL_RATE;
        break;
    case (uint32_t)VEHICLE_CL250_IDX_COOLANT_TEMP:
        id = RING_COOLANT;
        break;
    case (uint32_t)VEHICLE_CL250_IDX_ENGINE_OIL_TEMP:
        id = RING_OIL;
        break;
    default:
        id = RING_NONE;
        break;
    }
    return id;
}

typedef struct {
    uint32_t n;
    float mean;
    float slope_per_s;
    bool slope_ok;
} win_t;

/*
 * Samples of ring `id` in (now - window_ms, now], newest first, each times `scale`: their
 * count, mean and least-squares slope per second. The loop is bounded by the ring size.
 */
static void win_collect(derived_core_t* c, uint32_t id, uint32_t now, uint32_t window_ms,
                        float scale, bool want_slope, win_t* w)
{
    uint32_t cap = 0u;
    const derived_sample_t* const d = ring_data(c, id, &cap);
    const derived_ring_t* const r = &c->ring[id];
    float sum = 0.0f;
    uint32_t n = 0u;
    for (uint32_t k = 0u; k < (uint32_t)r->count; k++) {
        const derived_sample_t* const s = &d[((uint32_t)r->head + cap - k) % cap];
        if (age_of(now, s->stamp_ms) >= window_ms) {
            break; /* older ones are older still */
        }
        sum += s->value * scale;
        n++;
    }
    w->n = n;
    w->mean = (n > 0u) ? (sum / (float)n) : 0.0f;
    w->slope_per_s = 0.0f;
    w->slope_ok = false;
    if (want_slope && (n >= DERIVED_MIN_WINDOW_SAMPLES)) {
        float mean_x = 0.0f;
        for (uint32_t k = 0u; k < n; k++) {
            mean_x -= (float)age_of(now, d[((uint32_t)r->head + cap - k) % cap].stamp_ms);
        }
        mean_x /= (float)n;
        float var = 0.0f;
        float cov = 0.0f;
        for (uint32_t k = 0u; k < n; k++) {
            const derived_sample_t* const s = &d[((uint32_t)r->head + cap - k) % cap];
            const float dx = -(float)age_of(now, s->stamp_ms) - mean_x;
            var += dx * dx;
            cov += dx * ((s->value * scale) - w->mean);
        }
        if (var > 0.0f) {
            w->slope_per_s = (cov / var) * DERIVED_MS_PER_S;
            w->slope_ok = true;
        }
    }
}

/* ---- input access (fresh, declared) ---------------------------------------------------- */

static bool declares(const vehicle_cl250_derived_t* spec, uint8_t kind, uint32_t index)
{
    bool found = false;
    for (uint32_t i = 0u; (i < (uint32_t)spec->input_count) && (i < VEHICLE_CL250_DERIVED_INPUT_MAX); i++) {
        if ((spec->inputs[i].kind == kind) && ((uint32_t)spec->inputs[i].index == index)) {
            found = true;
        }
    }
    return found;
}

/* A DID sample is usable at `now` within `limit` ms. */
static derived_flag_t did_fresh(const derived_core_t* c, uint32_t did, uint32_t now, uint32_t limit,
                                float* v)
{
    derived_flag_t f = DERIVED_FLAG_NONE;
    if (did >= VEHICLE_CL250_DID_COUNT) {
        f = DERIVED_FLAG_MATH_ERROR;
    } else {
        const derived_did_t* const d = &c->did[did];
        if ((!d->present) || d->lapsed || (age_of(now, d->stamp_ms) > limit)) {
            f = DERIVED_FLAG_STALE_INPUT;
        } else if (!is_finite_f(d->value)) {
            f = DERIVED_FLAG_MATH_ERROR;
        } else {
            *v = d->value;
        }
    }
    return f;
}

/* A declared DID input, with that DID's own stale_after_ms from the gen/ DID table (D-070). */
static derived_flag_t did_value(const ctx_t* x, uint32_t did, float* v)
{
    derived_flag_t f = DERIVED_FLAG_MATH_ERROR; /* an undeclared input is a bug: fail closed */
    if (declares(x->spec, VEHICLE_CL250_DERIVED_INPUT_DID, did)) {
        f = did_fresh(x->c, did, x->now, (uint32_t)vehicle_cl250_dids[did].stale_after_ms, v);
    }
    return f;
}

/* A declared derived input: fresh by the producer's stale_after_ms, then valid. */
static derived_flag_t derived_value(const ctx_t* x, uint32_t sig, float* v)
{
    derived_flag_t f = DERIVED_FLAG_MATH_ERROR;
    if ((sig < VEHICLE_CL250_DERIVED_COUNT) && declares(x->spec, VEHICLE_CL250_DERIVED_INPUT_DERIVED, sig)) {
        const derived_out_t* const o = &x->c->out[sig];
        if (((o->status != DERIVED_VALID) && (o->status != DERIVED_INVALID)) ||
            (age_of(x->now, o->stamp_ms) > (uint32_t)vehicle_cl250_derived[sig].stale_after_ms)) {
            f = DERIVED_FLAG_STALE_INPUT;
        } else if (o->status == DERIVED_INVALID) {
            f = DERIVED_FLAG_INVALID_INPUT;
        } else {
            f = DERIVED_FLAG_NONE;
            *v = o->value;
        }
    }
    return f;
}

/* The window of a DID input: the newest sample must be fresh. */
static derived_flag_t did_window(const ctx_t* x, uint32_t did, float scale, bool want_slope, win_t* w)
{
    float dummy = 0.0f;
    derived_flag_t f = did_value(x, did, &dummy);
    if (f == DERIVED_FLAG_NONE) {
        const uint32_t id = ring_of_did(did);
        if ((id == RING_NONE) || (x->spec->window_ms == 0u)) {
            f = DERIVED_FLAG_MATH_ERROR;
        } else {
            win_collect(x->c, id, x->now, (uint32_t)x->spec->window_ms, scale, want_slope, w);
        }
    }
    return f;
}

/* ---- formulas ---------------------------------------------------------------------------- */

static float air_density(float pressure_kpa, float temperature_c)
{
    return (pressure_kpa * DERIVED_PA_PER_KPA) /
           (DERIVED_R_DRY_AIR * (temperature_c + DERIVED_KELVIN_OFFSET));
}

static derived_flag_t f_vehicle_accel(const ctx_t* x, float* out)
{
    win_t w;
    derived_flag_t f = did_window(x, (uint32_t)VEHICLE_CL250_IDX_VEHICLE_SPEED, DERIVED_KMH_TO_MPS, true, &w);
    if (f == DERIVED_FLAG_NONE) {
        if (w.slope_ok) {
            *out = w.slope_per_s;
        } else {
            f = DERIVED_FLAG_INSUFFICIENT_SAMPLES;
        }
    }
    return f;
}

static derived_flag_t f_vehicle_jerk(const ctx_t* x, float* out)
{
    float dummy = 0.0f;
    derived_flag_t f = derived_value(x, VEHICLE_CL250_DERIVED_VEHICLE_ACCEL, &dummy);
    if (f == DERIVED_FLAG_NONE) {
        win_t w;
        win_collect(x->c, RING_ACCEL, x->now, (uint32_t)x->spec->window_ms, 1.0f, true, &w);
        if (w.slope_ok) {
            *out = w.slope_per_s;
        } else {
            f = DERIVED_FLAG_INSUFFICIENT_SAMPLES;
        }
    }
    return f;
}

static derived_flag_t f_gearbox_ratio(const ctx_t* x, float* out)
{
    float speed_kmh = 0.0f;
    float rpm = 0.0f;
    derived_flag_t f = did_value(x, (uint32_t)VEHICLE_CL250_IDX_VEHICLE_SPEED, &speed_kmh);
    if (f == DERIVED_FLAG_NONE) {
        const float speed = speed_kmh * DERIVED_KMH_TO_MPS;
        if (speed <= 0.0f) {
            f = DERIVED_FLAG_STANDSTILL;
        } else {
            const float wheel_rpm = (speed / (2.0f * DERIVED_PI * VEHICLE_CL250_REAR_WHEEL_RADIUS_DYNAMIC_M)) * DERIVED_SECONDS_PER_MINUTE;
            const float reduction = VEHICLE_CL250_PRIMARY_REDUCTION_RATIO * VEHICLE_CL250_SECONDARY_REDUCTION_RATIO;
            f = did_value(x, (uint32_t)VEHICLE_CL250_IDX_ENGINE_SPEED, &rpm);
            if (f == DERIVED_FLAG_NONE) {
                *out = rpm / (wheel_rpm * reduction);
            }
        }
    }
    return f;
}

static derived_flag_t f_gear_estimate(const ctx_t* x, float* out)
{
    float ratio = 0.0f;
    const derived_flag_t f = derived_value(x, VEHICLE_CL250_DERIVED_GEARBOX_RATIO_MEASURED, &ratio);
    if (f == DERIVED_FLAG_NONE) {
        *out = (float)derived_gear_estimate(ratio);
    }
    return f;
}

/* a * b / 100 of two DIDs (torques: percent of the reference torque). */
static derived_flag_t torque_of(const ctx_t* x, uint32_t pct_did, float* out)
{
    float pct = 0.0f;
    float ref = 0.0f;
    derived_flag_t f = did_value(x, pct_did, &pct);
    if (f == DERIVED_FLAG_NONE) {
        f = did_value(x, (uint32_t)VEHICLE_CL250_IDX_REFERENCE_TORQUE, &ref);
    }
    if (f == DERIVED_FLAG_NONE) {
        *out = (pct * ref) / DERIVED_PERCENT;
    }
    return f;
}

static derived_flag_t f_torque_gap(const ctx_t* x, float* out)
{
    float demand = 0.0f;
    float actual = 0.0f;
    derived_flag_t f = derived_value(x, VEHICLE_CL250_DERIVED_DEMAND_TORQUE, &demand);
    if (f == DERIVED_FLAG_NONE) {
        f = derived_value(x, VEHICLE_CL250_DERIVED_ENGINE_TORQUE, &actual);
    }
    if (f == DERIVED_FLAG_NONE) {
        *out = demand - actual;
    }
    return f;
}

static derived_flag_t f_engine_power(const ctx_t* x, float* out)
{
    float torque = 0.0f;
    float rpm = 0.0f;
    derived_flag_t f = derived_value(x, VEHICLE_CL250_DERIVED_ENGINE_TORQUE, &torque);
    if (f == DERIVED_FLAG_NONE) {
        f = did_value(x, (uint32_t)VEHICLE_CL250_IDX_ENGINE_SPEED, &rpm);
    }
    if (f == DERIVED_FLAG_NONE) {
        *out = ((torque * rpm) * DERIVED_RPM_TO_RAD_PER_S) / DERIVED_G_PER_KG;
    }
    return f;
}

static derived_flag_t f_air_density(const ctx_t* x, float* out)
{
    float p = 0.0f;
    float t = 0.0f;
    derived_flag_t f = did_value(x, (uint32_t)VEHICLE_CL250_IDX_BAROMETRIC_PRESSURE, &p);
    if (f == DERIVED_FLAG_NONE) {
        f = did_value(x, (uint32_t)VEHICLE_CL250_IDX_INTAKE_AIR_TEMP, &t);
    }
    if (f == DERIVED_FLAG_NONE) {
        *out = air_density(p, t);
    }
    return f;
}

static derived_flag_t f_power_correction(const ctx_t* x, float* out)
{
    float rho = 0.0f;
    const derived_flag_t f = derived_value(x, VEHICLE_CL250_DERIVED_AIR_DENSITY, &rho);
    if (f == DERIVED_FLAG_NONE) {
        const float reference = air_density(DERIVED_J1349_REFERENCE_PRESSURE_KPA, DERIVED_J1349_REFERENCE_TEMPERATURE_C);
        *out = reference / rho;
    }
    return f;
}

static derived_flag_t f_air_fuel_ratio(const ctx_t* x, float* out)
{
    float lambda = 0.0f;
    const derived_flag_t f = did_value(x, (uint32_t)VEHICLE_CL250_IDX_COMMANDED_EQUIV_RATIO, &lambda);
    if (f == DERIVED_FLAG_NONE) {
        *out = lambda * VEHICLE_CL250_STOICHIOMETRIC_AFR;
    }
    return f;
}

static derived_flag_t f_fuel_rate_maf(const ctx_t* x, float* out)
{
    float lambda = 0.0f;
    float maf = 0.0f;
    derived_flag_t f = did_value(x, (uint32_t)VEHICLE_CL250_IDX_COMMANDED_EQUIV_RATIO, &lambda);
    if (f == DERIVED_FLAG_NONE) {
        const float afr = lambda * VEHICLE_CL250_STOICHIOMETRIC_AFR;
        if (afr <= 0.0f) {
            f = DERIVED_FLAG_INVALID_INPUT;
        } else {
            f = did_value(x, (uint32_t)VEHICLE_CL250_IDX_MAF_RATE, &maf);
            if (f == DERIVED_FLAG_NONE) {
                *out = (maf * DERIVED_GPS_TO_KG_PER_H) / (afr * VEHICLE_CL250_FUEL_DENSITY_KG_PER_L);
            }
        }
    }
    return f;
}

static derived_flag_t f_fuel_consumption_instant(const ctx_t* x, float* out)
{
    float speed = 0.0f;
    float rate = 0.0f;
    derived_flag_t f = did_value(x, (uint32_t)VEHICLE_CL250_IDX_VEHICLE_SPEED, &speed);
    if (f == DERIVED_FLAG_NONE) {
        if (speed <= 0.0f) {
            f = DERIVED_FLAG_STANDSTILL;
        } else {
            f = did_value(x, (uint32_t)VEHICLE_CL250_IDX_ENGINE_FUEL_RATE, &rate);
            if (f == DERIVED_FLAG_NONE) {
                *out = (rate / speed) * DERIVED_PERCENT;
            }
        }
    }
    return f;
}

static derived_flag_t f_fuel_consumption_avg(const ctx_t* x, float* out)
{
    win_t rate;
    win_t speed;
    derived_flag_t f = did_window(x, (uint32_t)VEHICLE_CL250_IDX_ENGINE_FUEL_RATE, 1.0f, false, &rate);
    if (f == DERIVED_FLAG_NONE) {
        f = did_window(x, (uint32_t)VEHICLE_CL250_IDX_VEHICLE_SPEED, 1.0f, false, &speed);
    }
    if (f == DERIVED_FLAG_NONE) {
        if ((rate.n < DERIVED_MIN_WINDOW_SAMPLES) || (speed.n < DERIVED_MIN_WINDOW_SAMPLES)) {
            f = DERIVED_FLAG_INSUFFICIENT_SAMPLES;
        } else if (speed.mean <= 0.0f) {
            f = DERIVED_FLAG_STANDSTILL;
        } else {
            *out = (DERIVED_PERCENT * rate.mean) / speed.mean;
        }
    }
    return f;
}

static derived_flag_t f_fuel_range(const ctx_t* x, float* out)
{
    float avg = 0.0f;
    float level = 0.0f;
    derived_flag_t f = derived_value(x, VEHICLE_CL250_DERIVED_FUEL_CONSUMPTION_AVG, &avg);
    if (f == DERIVED_FLAG_NONE) {
        if (avg <= 0.0f) {
            f = DERIVED_FLAG_NO_CONSUMPTION;
        } else {
            f = did_value(x, (uint32_t)VEHICLE_CL250_IDX_FUEL_LEVEL, &level);
            if (f == DERIVED_FLAG_NONE) {
                const float litres = (level / DERIVED_PERCENT) * VEHICLE_CL250_FUEL_TANK_CAPACITY_L;
                *out = (litres / avg) * DERIVED_PERCENT;
            }
        }
    }
    return f;
}

static derived_flag_t f_bsfc(const ctx_t* x, float* out)
{
    float power = 0.0f;
    float rate = 0.0f;
    derived_flag_t f = derived_value(x, VEHICLE_CL250_DERIVED_ENGINE_POWER, &power);
    if (f == DERIVED_FLAG_NONE) {
        if (power <= 0.0f) {
            f = DERIVED_FLAG_NO_POWER;
        } else {
            f = did_value(x, (uint32_t)VEHICLE_CL250_IDX_ENGINE_FUEL_RATE, &rate);
            if (f == DERIVED_FLAG_NONE) {
                const float mass_rate = rate * VEHICLE_CL250_FUEL_DENSITY_KG_PER_L;
                *out = (mass_rate * DERIVED_G_PER_KG) / power;
            }
        }
    }
    return f;
}

static derived_flag_t f_fuel_per_stroke(const ctx_t* x, float* out)
{
    float rpm = 0.0f;
    float rate = 0.0f;
    derived_flag_t f = did_value(x, (uint32_t)VEHICLE_CL250_IDX_ENGINE_SPEED, &rpm);
    if (f == DERIVED_FLAG_NONE) {
        if (rpm <= 0.0f) {
            f = DERIVED_FLAG_ENGINE_OFF;
        } else {
            f = did_value(x, (uint32_t)VEHICLE_CL250_IDX_ENGINE_FUEL_RATE, &rate);
            if (f == DERIVED_FLAG_NONE) {
                const float mg_per_hour = (rate * VEHICLE_CL250_FUEL_DENSITY_KG_PER_L) * DERIVED_MG_PER_KG;
                *out = mg_per_hour / ((rpm * DERIVED_SECONDS_PER_MINUTE) / DERIVED_REVOLUTIONS_PER_INJECTION);
            }
        }
    }
    return f;
}

/* a +/- b of two DIDs: sum = true adds, else a - b. */
static derived_flag_t two_dids(const ctx_t* x, uint32_t a_did, uint32_t b_did, bool sum, float* out)
{
    float a = 0.0f;
    float b = 0.0f;
    derived_flag_t f = did_value(x, a_did, &a);
    if (f == DERIVED_FLAG_NONE) {
        f = did_value(x, b_did, &b);
    }
    if (f == DERIVED_FLAG_NONE) {
        *out = sum ? (a + b) : (a - b);
    }
    return f;
}

static derived_flag_t f_throttle_channel(const ctx_t* x, float* out)
{
    float a = 0.0f;
    float b = 0.0f;
    float r = 0.0f;
    derived_flag_t f = did_value(x, (uint32_t)VEHICLE_CL250_IDX_THROTTLE_POS, &a);
    if (f == DERIVED_FLAG_NONE) {
        f = did_value(x, (uint32_t)VEHICLE_CL250_IDX_ABSOLUTE_THROTTLE_POS_B, &b);
    }
    if (f == DERIVED_FLAG_NONE) {
        f = did_value(x, (uint32_t)VEHICLE_CL250_IDX_RELATIVE_THROTTLE_POS, &r);
    }
    if (f == DERIVED_FLAG_NONE) {
        float m = fabsf(a - b);
        m = fmaxf(m, fabsf(a - r));
        m = fmaxf(m, fabsf(b - r));
        *out = m;
    }
    return f;
}

static derived_flag_t f_altitude_change(const ctx_t* x, float* out)
{
    float p = 0.0f;
    float t = 0.0f;
    derived_flag_t f = did_value(x, (uint32_t)VEHICLE_CL250_IDX_BAROMETRIC_PRESSURE, &p);
    if (f == DERIVED_FLAG_NONE) {
        f = did_value(x, (uint32_t)VEHICLE_CL250_IDX_INTAKE_AIR_TEMP, &t);
    }
    if (f == DERIVED_FLAG_NONE) {
        const float temperature = t + DERIVED_KELVIN_OFFSET;
        if ((!x->c->baro_ref_set) || (p <= 0.0f) || (temperature <= 0.0f)) {
            f = DERIVED_FLAG_INVALID_INPUT;
        } else {
            const float scale_height = (DERIVED_R_DRY_AIR * temperature) / DERIVED_STANDARD_GRAVITY;
            *out = scale_height * logf(x->c->baro_ref / p);
        }
    }
    return f;
}

static derived_flag_t f_temp_rate(const ctx_t* x, uint32_t did, float* out)
{
    win_t w;
    derived_flag_t f = did_window(x, did, 1.0f, true, &w);
    if (f == DERIVED_FLAG_NONE) {
        if (w.slope_ok) {
            *out = w.slope_per_s * DERIVED_SECONDS_PER_MINUTE;
        } else {
            f = DERIVED_FLAG_INSUFFICIENT_SAMPLES;
        }
    }
    return f;
}

/* Dispatch of the point formulas (gen index to formula). */
static derived_flag_t eval_point(const ctx_t* x, float* out)
{
    derived_flag_t f = DERIVED_FLAG_MATH_ERROR;
    switch (x->sig) {
    case VEHICLE_CL250_DERIVED_GEARBOX_RATIO_MEASURED:
        f = f_gearbox_ratio(x, out);
        break;
    case VEHICLE_CL250_DERIVED_GEAR_ESTIMATE:
        f = f_gear_estimate(x, out);
        break;
    case VEHICLE_CL250_DERIVED_VEHICLE_ACCEL:
        f = f_vehicle_accel(x, out);
        break;
    case VEHICLE_CL250_DERIVED_VEHICLE_JERK:
        f = f_vehicle_jerk(x, out);
        break;
    case VEHICLE_CL250_DERIVED_ENGINE_TORQUE:
        f = torque_of(x, (uint32_t)VEHICLE_CL250_IDX_ACTUAL_TORQUE_PCT, out);
        break;
    case VEHICLE_CL250_DERIVED_DEMAND_TORQUE:
        f = torque_of(x, (uint32_t)VEHICLE_CL250_IDX_DEMAND_TORQUE_PCT, out);
        break;
    case VEHICLE_CL250_DERIVED_TORQUE_GAP:
        f = f_torque_gap(x, out);
        break;
    case VEHICLE_CL250_DERIVED_ENGINE_POWER:
        f = f_engine_power(x, out);
        break;
    case VEHICLE_CL250_DERIVED_AIR_DENSITY:
        f = f_air_density(x, out);
        break;
    case VEHICLE_CL250_DERIVED_POWER_CORRECTION_FACTOR:
        f = f_power_correction(x, out);
        break;
    case VEHICLE_CL250_DERIVED_AIR_FUEL_RATIO:
        f = f_air_fuel_ratio(x, out);
        break;
    case VEHICLE_CL250_DERIVED_FUEL_RATE_MAF:
        f = f_fuel_rate_maf(x, out);
        break;
    case VEHICLE_CL250_DERIVED_FUEL_CONSUMPTION_INSTANT:
        f = f_fuel_consumption_instant(x, out);
        break;
    case VEHICLE_CL250_DERIVED_FUEL_CONSUMPTION_AVG:
        f = f_fuel_consumption_avg(x, out);
        break;
    case VEHICLE_CL250_DERIVED_FUEL_RANGE:
        f = f_fuel_range(x, out);
        break;
    case VEHICLE_CL250_DERIVED_BSFC:
        f = f_bsfc(x, out);
        break;
    case VEHICLE_CL250_DERIVED_FUEL_PER_STROKE:
        f = f_fuel_per_stroke(x, out);
        break;
    case VEHICLE_CL250_DERIVED_TOTAL_FUEL_TRIM:
        f = two_dids(x, (uint32_t)VEHICLE_CL250_IDX_SHORT_TERM_FUEL_TRIM_B1,
                     (uint32_t)VEHICLE_CL250_IDX_LONG_TERM_FUEL_TRIM_B1, true, out);
        break;
    case VEHICLE_CL250_DERIVED_THROTTLE_CHANNEL_RESIDUAL:
        f = f_throttle_channel(x, out);
        break;
    case VEHICLE_CL250_DERIVED_THROTTLE_COMMAND_RESIDUAL:
        f = two_dids(x, (uint32_t)VEHICLE_CL250_IDX_COMMANDED_THROTTLE,
                     (uint32_t)VEHICLE_CL250_IDX_THROTTLE_POS, false, out);
        break;
    case VEHICLE_CL250_DERIVED_GRIP_CHANNEL_RESIDUAL:
        f = two_dids(x, (uint32_t)VEHICLE_CL250_IDX_ACCEL_PEDAL_POS_D,
                     (uint32_t)VEHICLE_CL250_IDX_RELATIVE_ACCEL_PEDAL_POS, false, out);
        break;
    case VEHICLE_CL250_DERIVED_ALTITUDE_CHANGE:
        f = f_altitude_change(x, out);
        break;
    case VEHICLE_CL250_DERIVED_COOLANT_RATE:
        f = f_temp_rate(x, (uint32_t)VEHICLE_CL250_IDX_COOLANT_TEMP, out);
        break;
    case VEHICLE_CL250_DERIVED_OIL_TEMP_RATE:
        f = f_temp_rate(x, (uint32_t)VEHICLE_CL250_IDX_ENGINE_OIL_TEMP, out);
        break;
    default:
        f = DERIVED_FLAG_MATH_ERROR;
        break;
    }
    return f;
}

/* ---- integrals (trip distance, trip time, idle time) -------------------------------------- */

static uint32_t integral_slot(uint32_t sig)
{
    uint32_t slot = 2u;
    if (sig == VEHICLE_CL250_DERIVED_TRIP_DISTANCE) {
        slot = 0u;
    } else if (sig == VEHICLE_CL250_DERIVED_TRIP_TIME) {
        slot = 1u;
    } else {
        slot = 2u;
    }
    return slot;
}

/* The DIDs an integral reads, in the reference's order; returns their count. */
static uint32_t integral_inputs(uint32_t sig, uint32_t* ids)
{
    uint32_t n = 1u;
    if (sig == VEHICLE_CL250_DERIVED_TRIP_DISTANCE) {
        ids[0] = (uint32_t)VEHICLE_CL250_IDX_VEHICLE_SPEED;
    } else if (sig == VEHICLE_CL250_DERIVED_TRIP_TIME) {
        ids[0] = (uint32_t)VEHICLE_CL250_IDX_ENGINE_SPEED;
    } else {
        ids[0] = (uint32_t)VEHICLE_CL250_IDX_VEHICLE_SPEED;
        ids[1] = (uint32_t)VEHICLE_CL250_IDX_ENGINE_SPEED;
        n = 2u;
    }
    return n;
}

/* The rate (unit per second) held from a breakpoint; v[] in the order of integral_inputs. */
static double integral_rate(uint32_t sig, const float* v)
{
    double rate = 0.0;
    if (sig == VEHICLE_CL250_DERIVED_TRIP_DISTANCE) {
        rate = (double)(v[0] / DERIVED_SECONDS_PER_HOUR);
    } else if (sig == VEHICLE_CL250_DERIVED_TRIP_TIME) {
        rate = (v[0] > 0.0f) ? 1.0 : 0.0;
    } else {
        rate = (is_zero(v[0]) && (v[1] > 0.0f)) ? 1.0 : 0.0;
    }
    return rate;
}

/*
 * A new sample of any input is a breakpoint b (the newest new stamp of this step). The
 * segment held since the last usable breakpoint ends there, at most as long as the
 * shortest remaining freshness of its inputs (a gap adds nothing); a new segment starts
 * when every input is fresh at b. The reference does the same over the whole session.
 */
static void integral_update(derived_core_t* c, uint32_t sig)
{
    uint32_t ids[2] = {0u, 0u};
    const uint32_t n = integral_inputs(sig, ids);
    derived_integral_t* const g = &c->integral[integral_slot(sig)];
    bool any = false;
    uint32_t b = 0u;
    for (uint32_t k = 0u; k < n; k++) {
        const derived_did_t* const d = &c->did[ids[k]];
        if (d->fresh_now) {
            b = any ? later(b, d->stamp_ms) : d->stamp_ms;
            any = true;
        }
    }
    if (any) {
        c->evals_step++;
        if (g->seg_active) {
            const uint32_t run = (uint32_t)(b - g->seg_start_ms);
            int32_t span = (run > HALF_RANGE) ? 0 : (int32_t)run;
            if (span > g->seg_remaining_ms) {
                span = g->seg_remaining_ms;
            }
            if (span < 0) {
                span = 0;
            }
            g->prefix += g->rate * ((double)span / 1000.0);
            g->seg_active = false;
        }
        float v[2] = {0.0f, 0.0f};
        bool usable = true;
        int32_t remaining = 0;
        for (uint32_t k = 0u; k < n; k++) {
            float one = 0.0f;
            const uint32_t limit_ms = (uint32_t)vehicle_cl250_dids[ids[k]].stale_after_ms;
            if (did_fresh(c, ids[k], b, limit_ms, &one) != DERIVED_FLAG_NONE) {
                usable = false;
            } else {
                v[k] = one;
                const int32_t left = (int32_t)limit_ms - (int32_t)age_of(b, c->did[ids[k]].stamp_ms);
                remaining = (k == 0u) ? left : ((left < remaining) ? left : remaining);
            }
        }
        if (usable) {
            g->rate = integral_rate(sig, v);
            g->seg_start_ms = b;
            g->seg_remaining_ms = remaining;
            g->seg_active = true;
        }
    }
}

static derived_flag_t eval_integral(const ctx_t* x, float* out)
{
    uint32_t ids[2] = {0u, 0u};
    const uint32_t n = integral_inputs(x->sig, ids);
    const derived_integral_t* const g = &x->c->integral[integral_slot(x->sig)];
    derived_flag_t f = DERIVED_FLAG_NONE;
    for (uint32_t k = 0u; k < n; k++) {
        float dummy = 0.0f;
        if (f == DERIVED_FLAG_NONE) {
            f = did_value(x, ids[k], &dummy);
        }
    }
    if (f == DERIVED_FLAG_NONE) {
        double total = g->prefix;
        if (g->seg_active) {
            const uint32_t run = (uint32_t)(x->now - g->seg_start_ms);
            int32_t elapsed = (run > HALF_RANGE) ? 0 : (int32_t)run;
            if (elapsed > g->seg_remaining_ms) {
                elapsed = g->seg_remaining_ms;
            }
            if (elapsed < 0) {
                elapsed = 0;
            }
            total += g->rate * ((double)elapsed / 1000.0);
        }
        *out = (float)total;
    }
    return f;
}

/* ---- storing an output ---------------------------------------------------------------------- */

static void store(derived_core_t* c, uint32_t sig, uint32_t stamp, derived_flag_t flag, float value)
{
    const vehicle_cl250_derived_t* const spec = &vehicle_cl250_derived[sig];
    derived_out_t* const o = &c->out[sig];
    derived_flag_t f = flag;
    if (f == DERIVED_FLAG_NONE) {
        if (!is_finite_f(value)) {
            f = DERIVED_FLAG_MATH_ERROR;
        } else if ((value < spec->min) || (value > spec->max)) {
            f = DERIVED_FLAG_OUT_OF_RANGE; /* never clamped */
        } else {
            /* in range */
        }
    }
    o->unranged = (flag == DERIVED_FLAG_NONE) ? value : 0.0f;
    o->flag = f;
    o->status = (f == DERIVED_FLAG_NONE) ? DERIVED_VALID : DERIVED_INVALID;
    o->value = (f == DERIVED_FLAG_NONE) ? value : 0.0f;
    o->stamp_ms = stamp;
    o->seq++;
    if ((sig == VEHICLE_CL250_DERIVED_VEHICLE_ACCEL) && (f == DERIVED_FLAG_NONE)) {
        ring_push(c, RING_ACCEL, value, stamp);
    }
    if (c->ride.open && (f == DERIVED_FLAG_NONE)) {
        derived_ride_class_a(&c->ride.acc, sig, value); /* the ride's last-valid / count / max */
    }
}

/* ---- stop event --------------------------------------------------------------------------------- */

static void stop_step(derived_core_t* c, uint32_t sig)
{
    const derived_did_t* const speed = &c->did[VEHICLE_CL250_IDX_VEHICLE_SPEED];
    if (speed->fresh_now && is_finite_f(speed->value)) {
        derived_stop_t* const st = &c->stop;
        const uint32_t t = speed->stamp_ms;
        const uint32_t speed_limit =
            (uint32_t)vehicle_cl250_dids[VEHICLE_CL250_IDX_VEHICLE_SPEED].stale_after_ms; /* D-087 */
        c->evals_step++;
        if (st->has_prev && (((uint32_t)(t - st->prev_ms) > speed_limit))) {
            st->started = false; /* nothing is followed across a gap */
            st->moving = false;
        }
        st->prev_ms = t;
        st->has_prev = true;
        if (st->started) {
            if (speed->value > 0.0f) {
                const uint32_t duration_ms = t - st->start_ms;
                store(c, sig, t, DERIVED_FLAG_NONE, (float)duration_ms / DERIVED_MS_PER_S);
                st->started = false;
            }
        } else if (is_zero(speed->value) && st->moving) {
            float rpm = 0.0f;
            const uint32_t rpm_limit =
                (uint32_t)vehicle_cl250_dids[VEHICLE_CL250_IDX_ENGINE_SPEED].stale_after_ms; /* D-087 */
            if ((did_fresh(c, (uint32_t)VEHICLE_CL250_IDX_ENGINE_SPEED, t, rpm_limit, &rpm) == DERIVED_FLAG_NONE) &&
                (rpm > 0.0f)) {
                st->started = true;
                st->start_ms = t;
            }
        } else {
            /* standing without having moved, or still moving: nothing */
        }
        st->moving = speed->value > 0.0f;
    }
}

/* ---- the step ------------------------------------------------------------------------------------ */

static void record_sample(derived_core_t* c, uint32_t i, float value, uint32_t stamp)
{
    const uint32_t ring = ring_of_did(i);
    if (ring != RING_NONE) {
        ring_push(c, ring, value, stamp);
    }
    if ((i == (uint32_t)VEHICLE_CL250_IDX_BAROMETRIC_PRESSURE) && (!c->baro_ref_set) && is_finite_f(value) &&
        (value > 0.0f)) {
        c->baro_ref = value;
        c->baro_ref_set = true;
    }
}

static void ingest(derived_core_t* c, uint32_t now, const derived_in_t* in)
{
    for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
        derived_did_t* const d = &c->did[i];
        d->fresh_now = false;
        if (d->ignore_valid && in[i].present && (in[i].stamp_ms == d->ignore_stamp_ms)) {
            d->present = false; /* the old epoch's last sample, held by the producer */
        } else if (!in[i].present) {
            d->present = false;
        } else {
            d->ignore_valid = false;
            const bool is_new = (!d->present) || (in[i].stamp_ms != d->stamp_ms);
            d->present = true;
            d->value = in[i].value;
            d->stamp_ms = in[i].stamp_ms;
            if (is_new) {
                d->fresh_now = true;
                d->lapsed = false;
                if (!d->seen) {
                    d->seen = true;
                    d->first_ms = in[i].stamp_ms;
                }
                record_sample(c, i, in[i].value, in[i].stamp_ms);
                if (c->ride.open) {
                    derived_ride_sample(&c->ride.acc, i, in[i].value, in[i].stamp_ms);
                }
            }
            const uint32_t a = now - d->stamp_ms;
            if ((a > c->max_stale_ms) && (a <= HALF_RANGE)) {
                d->lapsed = true; /* a wrapped counter can never make it fresh again */
            }
        }
    }
}

/* Arms the grid: start = the latest first-sample time of the inputs (a derived input: the
 * producer's grid start). False while an input has never been seen. */
static bool try_arm(derived_core_t* c, uint32_t sig)
{
    const vehicle_cl250_derived_t* const spec = &vehicle_cl250_derived[sig];
    derived_sched_t* const s = &c->sched[sig];
    bool complete = true;
    uint32_t start = 0u;
    bool first = true;
    for (uint32_t k = 0u; (k < (uint32_t)spec->input_count) && (k < VEHICLE_CL250_DERIVED_INPUT_MAX); k++) {
        const uint32_t idx = spec->inputs[k].index;
        uint32_t t = 0u;
        if (spec->inputs[k].kind == VEHICLE_CL250_DERIVED_INPUT_DID) {
            complete = complete && (idx < VEHICLE_CL250_DID_COUNT) && c->did[idx].seen;
            t = c->did[idx].first_ms;
        } else {
            complete = complete && (idx < VEHICLE_CL250_DERIVED_COUNT) && c->sched[idx].armed;
            t = c->sched[idx].start_ms;
        }
        if (complete) {
            start = first ? t : later(start, t);
            first = false;
        }
    }
    if (complete && !first) {
        s->armed = true;
        s->start_ms = start;
        s->next_ms = start;
    }
    return s->armed;
}

static void grid_step(derived_core_t* c, uint32_t sig, uint32_t now)
{
    derived_sched_t* const s = &c->sched[sig];
    const vehicle_cl250_derived_t* const spec = &vehicle_cl250_derived[sig];
    if ((s->armed || try_arm(c, sig)) && at_or_after(now, s->next_ms)) {
        const uint32_t period = (uint32_t)spec->period_ms;
        const uint32_t tick = s->next_ms + (((now - s->next_ms) / period) * period);
        const ctx_t x = {c, now, sig, spec};
        float value = 0.0f;
        const derived_flag_t f = (kind_of(sig) == KIND_INTEGRAL) ? eval_integral(&x, &value) : eval_point(&x, &value);
        store(c, sig, tick, f, value);
        s->next_ms = tick + period;
        c->evals_step++;
    }
}

/* A new ride starts from a fresh class-A state (the reference evaluates every ride on its own):
 * grids, integrals, windows, the altitude reference and the stop event restart, and the DID
 * samples held now (the old epoch's) are ignored until a new sample arrives. Sequence
 * counters keep counting. */
static void reset_class_a(derived_core_t* c)
{
    for (uint32_t sig = 0u; sig < VEHICLE_CL250_DERIVED_COUNT; sig++) {
        if (c->sched[sig].computed && (kind_of(sig) != KIND_RIDE)) {
            derived_out_t* const o = &c->out[sig];
            o->status = DERIVED_NO_DATA;
            o->flag = DERIVED_FLAG_NONE;
            o->value = 0.0f;
            o->unranged = 0.0f;
            c->sched[sig].armed = false;
            c->sched[sig].start_ms = 0u;
            c->sched[sig].next_ms = 0u;
        }
    }
    for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
        derived_did_t* const d = &c->did[i];
        if (d->present) {
            d->ignore_valid = true;
            d->ignore_stamp_ms = d->stamp_ms;
        }
        d->present = false;
        d->seen = false;
        d->fresh_now = false;
        d->lapsed = false;
        d->first_ms = 0u;
    }
    (void)memset(c->integral, 0, sizeof c->integral);
    (void)memset(&c->stop, 0, sizeof c->stop);
    (void)memset(c->ring, 0, sizeof c->ring);
    c->baro_ref = 0.0f;
    c->baro_ref_set = false;
}

/*
 * The ride follows the epoch (D-087 item 2): a changed counter (!=, any jump, wrap-safe) or
 * a closed epoch closes the open ride at once, then an open epoch with no ride opens one.
 * Close + reopen in one record (counter + 1, open still true) is therefore two events in
 * this order. A power-down close is not handled (no persistence yet, D-045).
 */
static void epoch_step(derived_core_t* c, uint32_t now, const derived_epoch_t* e)
{
    derived_ride_t* const r = &c->ride;
    r->last_now_ms = now;
    if ((e != NULL) && e->known) {
        if (r->open && ((!e->open) || (e->counter != r->counter))) {
            derived_ride_close(r, now);
        }
        if ((!r->open) && e->open) {
            reset_class_a(c);
            derived_ride_begin(r, e->counter);
        }
    }
}

void derived_core_step_epoch(derived_core_t* c, uint32_t now_ms, const derived_in_t* in,
                             const derived_epoch_t* epoch)
{
    if ((c == NULL) || (in == NULL)) {
        return;
    }
    c->evals_step = 0u;
    epoch_step(c, now_ms, epoch);
    ingest(c, now_ms, in);
    for (uint32_t sig = 0u; sig < VEHICLE_CL250_DERIVED_COUNT; sig++) {
        if (c->sched[sig].computed) {
            const uint32_t kind = kind_of(sig);
            if (kind == KIND_EVENT) {
                stop_step(c, sig);
            } else if (kind != KIND_RIDE) {
                if (kind == KIND_INTEGRAL) {
                    integral_update(c, sig);
                }
                grid_step(c, sig, now_ms);
            } else {
                /* class B: accumulated from the samples and outputs above, read at the close */
            }
        }
    }
    if (c->evals_step > c->evals_max) {
        c->evals_max = c->evals_step;
    }
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: the class-A-only step, called by the tests
void derived_core_step(derived_core_t* c, uint32_t now_ms, const derived_in_t* in)
{
    derived_core_step_epoch(c, now_ms, in, NULL);
}

/* ---- init: which signals are computed ---------------------------------------------------------- */

static bool fits(uint32_t window_ms, uint32_t period_ms, uint32_t cap)
{
    return (window_ms != 0u) && (period_ms != 0u) && (((window_ms / period_ms) + 2u) <= cap);
}

static bool sizing_ok(uint32_t sig)
{
    const vehicle_cl250_derived_t* const s = &vehicle_cl250_derived[sig];
    const uint32_t w = s->window_ms;
    bool ok = true;
    switch (sig) {
    case VEHICLE_CL250_DERIVED_VEHICLE_ACCEL:
        ok = fits(w, vehicle_cl250_dids[VEHICLE_CL250_IDX_VEHICLE_SPEED].poll_period_ms, DERIVED_WIN_SPEED_CAP);
        break;
    case VEHICLE_CL250_DERIVED_VEHICLE_JERK:
        ok = fits(w, vehicle_cl250_derived[VEHICLE_CL250_DERIVED_VEHICLE_ACCEL].period_ms, DERIVED_WIN_ACCEL_CAP);
        break;
    case VEHICLE_CL250_DERIVED_FUEL_CONSUMPTION_AVG:
        ok = fits(w, vehicle_cl250_dids[VEHICLE_CL250_IDX_VEHICLE_SPEED].poll_period_ms, DERIVED_WIN_SPEED_CAP) &&
             fits(w, vehicle_cl250_dids[VEHICLE_CL250_IDX_ENGINE_FUEL_RATE].poll_period_ms, DERIVED_WIN_FUEL_RATE_CAP);
        break;
    case VEHICLE_CL250_DERIVED_COOLANT_RATE:
        ok = fits(w, vehicle_cl250_dids[VEHICLE_CL250_IDX_COOLANT_TEMP].poll_period_ms, DERIVED_WIN_COOLANT_CAP);
        break;
    case VEHICLE_CL250_DERIVED_OIL_TEMP_RATE:
        ok = fits(w, vehicle_cl250_dids[VEHICLE_CL250_IDX_ENGINE_OIL_TEMP].poll_period_ms, DERIVED_WIN_OIL_CAP);
        break;
    default:
        ok = true;
        break;
    }
    return ok;
}

static derived_nc_reason_t classify(const derived_core_t* c, uint32_t sig)
{
    const vehicle_cl250_derived_t* const s = &vehicle_cl250_derived[sig];
    const bool ride = (s->derived_class == VEHICLE_CL250_DERIVED_CLASS_RIDE);
    derived_nc_reason_t r = DERIVED_NC_NONE;
    if ((s->derived_class != VEHICLE_CL250_DERIVED_CLASS_LIVE) && (!ride)) {
        r = DERIVED_NC_CLASS;
    } else if (!s->rt_core_ready) {
        r = DERIVED_NC_NOT_READY;
    } else if (ride && ((s->elements != 1u) || (s->axis_count != 0u) || s->per_gear)) {
        r = DERIVED_NC_ELEMENTS; /* binned or per-gear: more than a scalar state */
    } else if (kind_of(sig) == KIND_NONE) {
        r = DERIVED_NC_NO_FORMULA;
    } else if ((s->input_count == 0u) || (s->input_count > VEHICLE_CL250_DERIVED_INPUT_MAX) ||
               ((!ride) && (!s->event) && (s->period_ms == 0u)) ||
               ((kind_of(sig) == KIND_EVENT) ? (!s->event) : s->event)) {
        r = DERIVED_NC_GEN;
    } else {
        for (uint32_t k = 0u; k < (uint32_t)s->input_count; k++) {
            const uint32_t idx = s->inputs[k].index;
            const uint8_t kind = s->inputs[k].kind;
            if (kind == VEHICLE_CL250_DERIVED_INPUT_DID) {
                if (idx >= VEHICLE_CL250_DID_COUNT) {
                    r = DERIVED_NC_GEN;
                }
            } else if (kind == VEHICLE_CL250_DERIVED_INPUT_DERIVED) {
                /* the producer must come earlier in the table and be computed itself */
                if ((idx >= sig) || (!c->sched[idx].computed)) {
                    r = DERIVED_NC_INPUT;
                }
            } else {
                r = DERIVED_NC_INPUT; /* a sensor: none is computed yet */
            }
        }
        if ((r == DERIVED_NC_NONE) && (!sizing_ok(sig))) {
            r = DERIVED_NC_SIZING;
        }
    }
    return r;
}

void derived_core_init(derived_core_t* c)
{
    if (c == NULL) {
        return;
    }
    (void)memset(c, 0, sizeof *c);
    derived_ride_init(&c->ride);
    uint32_t max_stale = 0u;
    for (uint32_t sig = 0u; sig < VEHICLE_CL250_DERIVED_COUNT; sig++) {
        const derived_nc_reason_t r = classify(c, sig);
        c->sched[sig].reason = r;
        c->sched[sig].computed = (r == DERIVED_NC_NONE);
        c->out[sig].status = (r == DERIVED_NC_NONE) ? DERIVED_NO_DATA : DERIVED_NOT_COMPUTED;
        c->out[sig].flag = DERIVED_FLAG_NONE;
        if (r == DERIVED_NC_NONE) {
            c->computed_count++;
            if ((uint32_t)vehicle_cl250_derived[sig].stale_after_ms > max_stale) {
                max_stale = vehicle_cl250_derived[sig].stale_after_ms;
            }
        }
    }
    /* the DIDs' own limits (every DID input, events included, D-087) */
    for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
        if ((uint32_t)vehicle_cl250_dids[i].stale_after_ms > max_stale) {
            max_stale = vehicle_cl250_dids[i].stale_after_ms;
        }
    }
    c->max_stale_ms = max_stale;
}

const derived_out_t* derived_core_out(const derived_core_t* c, uint32_t sig)
{
    return ((c == NULL) || (sig >= VEHICLE_CL250_DERIVED_COUNT)) ? NULL : &c->out[sig];
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: read by the tests and diagnostics
bool derived_core_is_computed(const derived_core_t* c, uint32_t sig)
{
    return (c != NULL) && (sig < VEHICLE_CL250_DERIVED_COUNT) && c->sched[sig].computed;
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: read by the tests and diagnostics
derived_nc_reason_t derived_core_reason(const derived_core_t* c, uint32_t sig)
{
    return ((c == NULL) || (sig >= VEHICLE_CL250_DERIVED_COUNT)) ? DERIVED_NC_GEN : c->sched[sig].reason;
}

bool derived_core_ride_out(const derived_core_t* c, derived_ride_which_t which, uint32_t sig,
                           derived_out_t* out)
{
    if ((c == NULL) || (out == NULL) || (sig >= VEHICLE_CL250_DERIVED_COUNT)) {
        return false;
    }
    const uint32_t slot = derived_ride_slot(sig);
    (void)memset(out, 0, sizeof *out);
    if ((slot == DERIVED_RIDE_NO_SLOT) || (!c->sched[sig].computed)) {
        out->status = DERIVED_NOT_COMPUTED;
    } else if (which == DERIVED_RIDE_PREVIOUS) {
        if (c->ride.have_prev) {
            *out = c->ride.prev[slot];
        } else {
            out->status = DERIVED_NO_DATA;
        }
    } else if (c->ride.open) {
        derived_ride_finalize(&c->ride.acc, sig, out);
        out->stamp_ms = c->ride.last_now_ms;
    } else {
        out->status = DERIVED_NO_DATA;
    }
    return true;
}
