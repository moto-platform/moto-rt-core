#ifndef FEATURES_DERIVED_DERIVED_CORE_H
#define FEATURES_DERIVED_DERIVED_CORE_H

/*
 * Pure derived-signal core of rt-core (D-070 items 1, 2 and 5, part 1: class A signals
 * whose inputs are DIDs or other computed derived signals). No HAL, RTOS, services or clock
 * access: the glue (derived.c) hands in the DID samples (value, stamp, present) and the
 * time; the core evaluates every computed signal on its gen/ period grid and keeps the last
 * output per signal.
 *
 * QM information (D-070 item 5): nothing here feeds 0x020, safety-node, the heartbeat or
 * the anomaly safety net. The module is not safety-critical, but it shares a CPU with the
 * EKF: no blocking, every loop is bounded by a compile-time size, static memory only.
 *
 * Sampling rule (D-070 item 1, the moto-server reference engine.py / context.py): an
 * input's value at time t is its last sample at or before t, and counts only while its age
 * is within its own stale_after_ms: for a DID input the DID's (vehicle_cl250_dids[], D-070),
 * for a derived input the producer's (vehicle_cl250_derived[]); the event signal keeps the
 * reference's polls-of-a-DID limit (open point, README). A held value is never
 * presented as fresh. The output of a signal is on its own period_ms grid, which starts at
 * the latest first-sample time of its inputs (a derived input: the producer's grid start).
 * A result outside the gen/ [min, max] is invalid (OUT_OF_RANGE), never clamped.
 *
 * Which signals are computed is decided at init from the gen/ table (class A, rt_core_ready,
 * inputs only DIDs or computed derived signals earlier in the table, a formula in
 * derived_core.c, window buffers large enough); the others report NOT_COMPUTED with a reason.
 * Evaluation order is the gen/ index order (every derived input has a lower index).
 *
 * Time: evaluation happens at `now_ms`; the output is stamped with the grid tick it
 * answers (the latest tick at or before now). Called every millisecond (the tests) it is
 * the reference to the last tick; called every few ms (the glue) a tick is evaluated up to a
 * step late, and a step that spans several ticks of one signal evaluates the latest one only
 * (bounded work, no catch-up). Likewise at most one new sample per DID is seen per step.
 */

#include "vehicle_cl250.h"
#include "vehicle_cl250_derived.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Why an output is invalid. The numbering is the one of the golden vectors
 * (tests/host/vectors/derived_vectors.h), which test_derived_core checks. */
typedef enum {
    DERIVED_FLAG_NONE = 0,
    DERIVED_FLAG_STALE_INPUT = 1,
    DERIVED_FLAG_INVALID_INPUT = 2,
    DERIVED_FLAG_INSUFFICIENT_SAMPLES = 3,
    DERIVED_FLAG_OUT_OF_RANGE = 4,
    DERIVED_FLAG_MATH_ERROR = 5, /* a non-finite input or result (the reference: out_of_range) */
    DERIVED_FLAG_STANDSTILL = 6,
    DERIVED_FLAG_STRAIGHT_LINE = 7, /* sensor-input signals only: not computed in part 1 */
    DERIVED_FLAG_NO_CONSUMPTION = 8,
    DERIVED_FLAG_NO_POWER = 9,
    DERIVED_FLAG_ENGINE_OFF = 10
} derived_flag_t;

typedef enum {
    DERIVED_NOT_COMPUTED = 0, /* never evaluated: see derived_core_reason() */
    DERIVED_NO_DATA,          /* computed, nothing evaluated yet (or no event yet) */
    DERIVED_VALID,
    DERIVED_INVALID           /* evaluated, `flag` says why */
} derived_status_t;

/* Why a signal is not computed. */
typedef enum {
    DERIVED_NC_NONE = 0, /* it is computed */
    DERIVED_NC_CLASS,    /* not class A (ride summaries come with class B) */
    DERIVED_NC_NOT_READY,/* gen/ rt_core_ready is false */
    DERIVED_NC_NO_FORMULA,
    DERIVED_NC_INPUT,    /* a sensor input, or a derived input that is not computed */
    DERIVED_NC_SIZING,   /* its window does not fit the compile-time sample buffer */
    DERIVED_NC_GEN       /* the gen/ entry is inconsistent (no period, no window) */
} derived_nc_reason_t;

/* One DID sample as the glue hands it in (services/vehicle_signals through the snapshot). */
typedef struct {
    float value;
    uint32_t stamp_ms; /* timebase ms of the sample (its request's send time) */
    bool present;      /* false: no sample since boot */
} derived_in_t;

typedef struct {
    float value;       /* the result when VALID, else 0 */
    float unranged;    /* the formula result before the range check (diagnostics, tests) */
    uint32_t stamp_ms; /* the grid tick, or the event time */
    uint32_t seq;      /* counts evaluations / events of this signal */
    derived_status_t status;
    derived_flag_t flag;
} derived_out_t;

/*
 * Compile-time sizing (static RAM). Capacity = window_ms / poll period + 2 samples of the
 * DID (or of the derived producer). gen/ does not export window or period as constant
 * expressions, so the numbers below are checked against the gen/ table at init: a signal
 * whose window needs more is NOT_COMPUTED (DERIVED_NC_SIZING), never silently truncated.
 * The caps derive from the gen/ table fields `poll_period_ms` (vehicle_cl250_dids[]; for the
 * accel ring `period_ms` of the producer) and `window_ms` (vehicle_cl250_derived[]) and are
 * checked against them at init (SIZING). Values for the pinned defs: SPEED 60000/110+2 = 547, FUEL_RATE 60000/2000+2 = 32,
 * COOLANT 60000/800+2 = 77, OIL 60000/5000+2 = 14, VEHICLE_ACCEL output 440/110+2 = 6.
 */
#define DERIVED_WIN_SPEED_CAP (548u)
#define DERIVED_WIN_FUEL_RATE_CAP (32u)
#define DERIVED_WIN_COOLANT_CAP (77u)
#define DERIVED_WIN_OIL_CAP (14u)
#define DERIVED_WIN_ACCEL_CAP (8u)

/*
 * Algorithm parameters and unit conversions ported from the reference
 * (moto-server src/moto_server/derived/formulas.py and context.py). Vehicle facts come
 * from gen/ vehicle_cl250_constants.h only. Each "algorithm parameter" is provisional
 * (D-070 item 6); the others are unit conversions or physics constants.
 */
#define DERIVED_PI (3.14159265f)                       /* mathematical constant */
#define DERIVED_KMH_TO_MPS (0.27777778f)               /* 1 / 3.6, formulas.py:39 */
#define DERIVED_RPM_TO_RAD_PER_S (0.10471976f)         /* 2 pi / 60, formulas.py:40 */
#define DERIVED_R_DRY_AIR (287.058f)                   /* J/(kg K), formulas.py:43 */
#define DERIVED_KELVIN_OFFSET (273.15f)                /* formulas.py:44 */
#define DERIVED_STANDARD_GRAVITY (9.80665f)            /* formulas.py:47 */
#define DERIVED_MG_PER_KG (1.0e6f)                     /* formulas.py:49 */
#define DERIVED_G_PER_KG (1000.0f)                     /* formulas.py:50 */
#define DERIVED_SECONDS_PER_MINUTE (60.0f)             /* formulas.py:51 */
#define DERIVED_REVOLUTIONS_PER_INJECTION (2.0f)       /* four-stroke, formulas.py:54 */
#define DERIVED_GPS_TO_KG_PER_H (3.6f)                 /* g/s -> kg/h, formulas.py:395 */
#define DERIVED_SECONDS_PER_HOUR (3600.0f)             /* km/h -> km/s, formulas.py:296 */
#define DERIVED_PA_PER_KPA (1000.0f)                   /* formulas.py:155 */
#define DERIVED_PERCENT (100.0f)                       /* formulas.py:342 */
#define DERIVED_MS_PER_S (1000.0f)                     /* formulas.py:168 */
/* SAE J1349 reference conditions of the power correction: algorithm parameters, provisional
 * (D-070 item 6), formulas.py:56-57. */
#define DERIVED_J1349_REFERENCE_PRESSURE_KPA (99.0f)
#define DERIVED_J1349_REFERENCE_TEMPERATURE_C (25.0f)
/* A DID is considered gone after this many of its poll periods: the age limit of the
 * event signals, which have no stale_after_ms in gen/. Algorithm parameter, provisional
 * (D-070 item 6), context.py:27 DID_AGE_LIMIT_POLLS. */
#define DERIVED_DID_AGE_LIMIT_POLLS (3u)
/* A slope or a window mean needs at least this many samples: algorithm parameter,
 * provisional (D-070 item 6), formulas.py:160 and :417. */
#define DERIVED_MIN_WINDOW_SAMPLES (2u)

typedef struct {
    float value;
    uint32_t stamp_ms;
} derived_sample_t;

typedef struct {
    uint16_t head;  /* index of the newest sample */
    uint16_t count;
} derived_ring_t;

typedef struct {
    float value;
    uint32_t stamp_ms;
    uint32_t first_ms; /* stamp of the first sample seen */
    bool present;
    bool seen;
    bool fresh_now;    /* a new sample arrived in this step */
    bool lapsed;       /* older than every limit: stays unusable until a new sample */
} derived_did_t;

typedef struct {
    uint32_t start_ms;
    uint32_t next_ms;
    bool armed;
    bool computed;
    derived_nc_reason_t reason;
} derived_sched_t;

/* Integral of a rate along the time axis (trip distance, trip time, idle time). */
typedef struct {
    double prefix;
    double rate;
    uint32_t seg_start_ms;
    int32_t seg_remaining_ms;
    bool seg_active;
} derived_integral_t;

typedef struct {
    bool has_prev;
    bool moving;
    bool started;
    uint32_t prev_ms;
    uint32_t start_ms;
} derived_stop_t;

typedef struct {
    derived_out_t out[VEHICLE_CL250_DERIVED_COUNT];
    derived_sched_t sched[VEHICLE_CL250_DERIVED_COUNT];
    derived_did_t did[VEHICLE_CL250_DID_COUNT];
    derived_integral_t integral[3];
    derived_stop_t stop;
    float baro_ref;  /* first positive barometric sample: the altitude reference */
    bool baro_ref_set;
    uint32_t max_stale_ms; /* longest limit of any computed signal: the lapse bound */
    uint32_t computed_count;
    derived_ring_t ring[5];
    derived_sample_t speed_win[DERIVED_WIN_SPEED_CAP];
    derived_sample_t fuel_rate_win[DERIVED_WIN_FUEL_RATE_CAP];
    derived_sample_t coolant_win[DERIVED_WIN_COOLANT_CAP];
    derived_sample_t oil_win[DERIVED_WIN_OIL_CAP];
    derived_sample_t accel_win[DERIVED_WIN_ACCEL_CAP];
    uint32_t evals_step; /* formula evaluations (and event/breakpoint updates) in the last step */
    uint32_t evals_max;  /* the largest evals_step so far */
} derived_core_t;

/* Boot: decides the computed set from the gen/ table and clears every state. */
void derived_core_init(derived_core_t* c);

/*
 * One step at now_ms with the current DID samples (in[VEHICLE_CL250_DID_COUNT]). Bounded:
 * a fixed pass over the DIDs and over the gen/ table, every window loop limited by its
 * buffer. NULL arguments do nothing.
 */
void derived_core_step(derived_core_t* c, uint32_t now_ms, const derived_in_t* in);

/* The last output of signal `sig` (a VEHICLE_CL250_DERIVED_*), or NULL when out of range. */
const derived_out_t* derived_core_out(const derived_core_t* c, uint32_t sig);

bool derived_core_is_computed(const derived_core_t* c, uint32_t sig);

/* DERIVED_NC_NONE for a computed signal, else the reason; DERIVED_NC_GEN for a bad index. */
derived_nc_reason_t derived_core_reason(const derived_core_t* c, uint32_t sig);

#ifdef __cplusplus
}
#endif

#endif /* FEATURES_DERIVED_DERIVED_CORE_H */
