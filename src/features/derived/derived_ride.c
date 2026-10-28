/*
 * Class-B ride accumulators and closed values (D-070, D-087). See derived_ride.h. The
 * formulas follow vehicle-server src/vehicle_server/derived/summary.py.
 */
#include "features/derived/derived_ride.h"

#include "vehicle_cl250.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#if defined(__FAST_MATH__)
#error "derived_ride.c must not be built with -ffast-math"
#endif

/* The reference's unit conversion of the fuel integral: L/h to L/s (summary.py
 * SECONDS_PER_HOUR) and the consumption factor (LITRES_PER_100KM_FACTOR); ms to s. */
#define RIDE_SECONDS_PER_HOUR (3600.0)
#define RIDE_L_PER_100KM (100.0)
#define RIDE_MS_PER_S (1000.0)

static const uint16_t slot_signal[DERIVED_RIDE_SLOTS] = {
    VEHICLE_CL250_DERIVED_RIDE_DISTANCE,          VEHICLE_CL250_DERIVED_RIDE_TIME,
    VEHICLE_CL250_DERIVED_RIDE_IDLE_TIME,         VEHICLE_CL250_DERIVED_RIDE_STOP_COUNT,
    VEHICLE_CL250_DERIVED_RIDE_FUEL_USED,         VEHICLE_CL250_DERIVED_RIDE_FUEL_CONSUMPTION,
    VEHICLE_CL250_DERIVED_RIDE_LTFT_MEAN,         VEHICLE_CL250_DERIVED_RIDE_STFT_STD,
    VEHICLE_CL250_DERIVED_RIDE_COOLANT_RATE_MAX,  VEHICLE_CL250_DERIVED_RIDE_START_COOLANT_TEMP,
    VEHICLE_CL250_DERIVED_RIDE_START_INTAKE_TEMP, VEHICLE_CL250_DERIVED_RIDE_END_COOLANT_TEMP,
};

uint32_t derived_ride_slot(uint32_t sig)
{
    uint32_t slot = DERIVED_RIDE_NO_SLOT;
    for (uint32_t i = 0u; i < DERIVED_RIDE_SLOTS; i++) {
        if ((uint32_t)slot_signal[i] == sig) {
            slot = i;
        }
    }
    return slot;
}

void derived_ride_init(derived_ride_t* r)
{
    if (r == NULL) {
        return;
    }
    (void)memset(r, 0, sizeof *r);
    for (uint32_t i = 0u; i < DERIVED_RIDE_SLOTS; i++) {
        r->prev[i].status = DERIVED_NO_DATA;
        r->prev[i].flag = DERIVED_FLAG_NONE;
    }
}

void derived_ride_begin(derived_ride_t* r, uint16_t counter)
{
    if (r == NULL) {
        return;
    }
    (void)memset(&r->acc, 0, sizeof r->acc);
    r->counter = counter;
    r->open = true;
}

static bool ride_finite(float x)
{
    return isfinite(x) != 0;
}

/* The fuel integral: a sample ends the open segment (at most the DID's own stale_after_ms
 * long, so a gap adds nothing) and starts the next one. */
static void fuel_sample(derived_ride_acc_t* a, float value, uint32_t stamp_ms)
{
    const int32_t limit = (int32_t)vehicle_cl250_dids[VEHICLE_CL250_IDX_ENGINE_FUEL_RATE].stale_after_ms;
    if (a->fuel_seg_active) {
        const uint32_t run = stamp_ms - a->fuel_seg_start_ms;
        int32_t span = (run > 0x7FFFFFFFu) ? 0 : (int32_t)run;
        if (span > a->fuel_seg_remaining_ms) {
            span = a->fuel_seg_remaining_ms;
        }
        if (span < 0) {
            span = 0;
        }
        a->fuel_prefix += a->fuel_rate * ((double)span / RIDE_MS_PER_S);
        a->fuel_seg_active = false;
    }
    a->fuel_seen = true;
    if (ride_finite(value)) {
        a->fuel_rate = (double)value / RIDE_SECONDS_PER_HOUR;
        a->fuel_seg_start_ms = stamp_ms;
        a->fuel_seg_remaining_ms = limit; /* the sample is fresh at its own stamp */
        a->fuel_seg_active = true;
    }
}

void derived_ride_sample(derived_ride_acc_t* a, uint32_t did, float value, uint32_t stamp_ms)
{
    if (a == NULL) {
        return;
    }
    switch (did) {
    case (uint32_t)VEHICLE_CL250_IDX_ENGINE_FUEL_RATE:
        fuel_sample(a, value, stamp_ms);
        break;
    case (uint32_t)VEHICLE_CL250_IDX_LONG_TERM_FUEL_TRIM_B1:
        a->ltft_n++;
        a->ltft_sum += (double)value;
        break;
    case (uint32_t)VEHICLE_CL250_IDX_SHORT_TERM_FUEL_TRIM_B1: {
        a->stft_n++;
        const double delta = (double)value - a->stft_mean;
        a->stft_mean += delta / (double)a->stft_n;
        a->stft_m2 += delta * ((double)value - a->stft_mean);
        break;
    }
    case (uint32_t)VEHICLE_CL250_IDX_COOLANT_TEMP:
        if (!a->has_coolant) {
            a->coolant_first = value;
            a->has_coolant = true;
        }
        a->coolant_last = value;
        break;
    case (uint32_t)VEHICLE_CL250_IDX_INTAKE_AIR_TEMP:
        if (!a->has_intake) {
            a->intake_first = value;
            a->has_intake = true;
        }
        break;
    default:
        break; /* no summary reads this DID */
    }
}

void derived_ride_class_a(derived_ride_acc_t* a, uint32_t sig, float value)
{
    if (a == NULL) {
        return;
    }
    switch (sig) {
    case VEHICLE_CL250_DERIVED_TRIP_DISTANCE:
        a->trip_distance = value;
        a->has_trip_distance = true;
        break;
    case VEHICLE_CL250_DERIVED_TRIP_TIME:
        a->trip_time = value;
        a->has_trip_time = true;
        break;
    case VEHICLE_CL250_DERIVED_IDLE_TIME:
        a->idle_time = value;
        a->has_idle_time = true;
        break;
    case VEHICLE_CL250_DERIVED_STOP_EVENT:
        if (a->stop_count < UINT32_MAX) {
            a->stop_count++;
        }
        break;
    case VEHICLE_CL250_DERIVED_COOLANT_RATE:
        if ((!a->has_coolant_rate) || (value > a->coolant_rate_max)) {
            a->coolant_rate_max = value;
        }
        a->has_coolant_rate = true;
        break;
    default:
        break;
    }
}

/* Raw result of a summary: a flag, or NONE with a value. */
typedef struct {
    derived_flag_t flag;
    double value;
} raw_t;

static raw_t raw_value(double v)
{
    const raw_t r = {DERIVED_FLAG_NONE, v};
    return r;
}

static raw_t raw_flag(derived_flag_t f)
{
    const raw_t r = {f, 0.0};
    return r;
}

static raw_t last_valid(bool has, float v)
{
    return has ? raw_value((double)v) : raw_flag(DERIVED_FLAG_INVALID_INPUT);
}

/* The range check and the output row, as the reference's _checked(). */
static void conclude(uint32_t sig, raw_t raw, derived_out_t* out)
{
    const vehicle_cl250_derived_t* const spec = &vehicle_cl250_derived[sig];
    derived_flag_t f = raw.flag;
    const float value = (float)raw.value;
    if (f == DERIVED_FLAG_NONE) {
        if (!ride_finite(value)) {
            f = DERIVED_FLAG_MATH_ERROR;
        } else if ((value < spec->min) || (value > spec->max)) {
            f = DERIVED_FLAG_OUT_OF_RANGE; /* never clamped */
        } else {
            /* in range */
        }
    }
    out->unranged = (raw.flag == DERIVED_FLAG_NONE) ? value : 0.0f;
    out->flag = f;
    out->status = (f == DERIVED_FLAG_NONE) ? DERIVED_VALID : DERIVED_INVALID;
    out->value = (f == DERIVED_FLAG_NONE) ? value : 0.0f;
}

static raw_t raw_simple(const derived_ride_acc_t* a, uint32_t sig);

/* ride_fuel_consumption reads two finalized class-B results (invalid or out of range is
 * "invalid input"), distance <= 0 is standstill. */
static raw_t consumption(const derived_ride_acc_t* a)
{
    derived_out_t used;
    derived_out_t dist;
    (void)memset(&used, 0, sizeof used);
    (void)memset(&dist, 0, sizeof dist);
    conclude(VEHICLE_CL250_DERIVED_RIDE_FUEL_USED, raw_simple(a, VEHICLE_CL250_DERIVED_RIDE_FUEL_USED), &used);
    conclude(VEHICLE_CL250_DERIVED_RIDE_DISTANCE, raw_simple(a, VEHICLE_CL250_DERIVED_RIDE_DISTANCE), &dist);
    raw_t r = raw_flag(DERIVED_FLAG_INVALID_INPUT);
    if ((used.status == DERIVED_VALID) && (dist.status == DERIVED_VALID)) {
        if (dist.value <= 0.0f) {
            r = raw_flag(DERIVED_FLAG_STANDSTILL);
        } else {
            r = raw_value(((double)used.value / (double)dist.value) * RIDE_L_PER_100KM);
        }
    }
    return r;
}

static raw_t raw_simple(const derived_ride_acc_t* a, uint32_t sig)
{
    raw_t r = raw_flag(DERIVED_FLAG_INVALID_INPUT);
    switch (sig) {
    case VEHICLE_CL250_DERIVED_RIDE_DISTANCE:
        r = last_valid(a->has_trip_distance, a->trip_distance);
        break;
    case VEHICLE_CL250_DERIVED_RIDE_TIME:
        r = last_valid(a->has_trip_time, a->trip_time);
        break;
    case VEHICLE_CL250_DERIVED_RIDE_IDLE_TIME:
        r = last_valid(a->has_idle_time, a->idle_time);
        break;
    case VEHICLE_CL250_DERIVED_RIDE_STOP_COUNT:
        r = raw_value((double)a->stop_count);
        break;
    case VEHICLE_CL250_DERIVED_RIDE_FUEL_USED:
        r = a->fuel_seen ? raw_value(a->fuel_prefix) : raw_flag(DERIVED_FLAG_INVALID_INPUT);
        break;
    case VEHICLE_CL250_DERIVED_RIDE_LTFT_MEAN:
        if (a->ltft_n > 0u) {
            r = raw_value(a->ltft_sum / (double)a->ltft_n);
        }
        break;
    case VEHICLE_CL250_DERIVED_RIDE_STFT_STD:
        if (a->stft_n >= 2u) {
            r = raw_value(sqrt(a->stft_m2 / (double)a->stft_n));
        } else if (a->stft_n == 1u) {
            r = raw_flag(DERIVED_FLAG_INSUFFICIENT_SAMPLES);
        } else {
            r = raw_flag(DERIVED_FLAG_INVALID_INPUT);
        }
        break;
    case VEHICLE_CL250_DERIVED_RIDE_COOLANT_RATE_MAX:
        r = last_valid(a->has_coolant_rate, a->coolant_rate_max);
        break;
    case VEHICLE_CL250_DERIVED_RIDE_START_COOLANT_TEMP:
        r = last_valid(a->has_coolant, a->coolant_first);
        break;
    case VEHICLE_CL250_DERIVED_RIDE_START_INTAKE_TEMP:
        r = last_valid(a->has_intake, a->intake_first);
        break;
    default: /* VEHICLE_CL250_DERIVED_RIDE_END_COOLANT_TEMP */
        r = last_valid(a->has_coolant, a->coolant_last);
        break;
    }
    return r;
}

static raw_t raw_of(const derived_ride_acc_t* a, uint32_t sig)
{
    return (sig == VEHICLE_CL250_DERIVED_RIDE_FUEL_CONSUMPTION) ? consumption(a) : raw_simple(a, sig);
}

void derived_ride_finalize(const derived_ride_acc_t* a, uint32_t sig, derived_out_t* out)
{
    if ((a == NULL) || (out == NULL)) {
        return;
    }
    if (derived_ride_slot(sig) == DERIVED_RIDE_NO_SLOT) {
        out->status = DERIVED_NOT_COMPUTED;
        out->flag = DERIVED_FLAG_NONE;
        out->value = 0.0f;
        out->unranged = 0.0f;
        return;
    }
    conclude(sig, raw_of(a, sig), out);
}

void derived_ride_close(derived_ride_t* r, uint32_t now_ms)
{
    if (r == NULL) {
        return;
    }
    for (uint32_t i = 0u; i < DERIVED_RIDE_SLOTS; i++) {
        derived_out_t* const o = &r->prev[i];
        derived_ride_finalize(&r->acc, (uint32_t)slot_signal[i], o);
        o->stamp_ms = now_ms;
        o->seq++;
    }
    r->have_prev = true;
    r->open = false;
    r->closed_count++;
}
