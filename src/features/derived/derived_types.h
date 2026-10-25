#ifndef FEATURES_DERIVED_DERIVED_TYPES_H
#define FEATURES_DERIVED_DERIVED_TYPES_H

/*
 * Types shared by the derived core (class A, derived_core.h) and the ride summaries (class B,
 * derived_ride.h): flags, status, the input sample, the output row and the epoch the glue
 * hands in. Pure data, no includes beyond the C library.
 */

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
    DERIVED_NC_CLASS,    /* neither class A nor class B */
    DERIVED_NC_NOT_READY,/* gen/ rt_core_ready is false */
    DERIVED_NC_NO_FORMULA,
    DERIVED_NC_INPUT,    /* a sensor input, or a derived input that is not computed */
    DERIVED_NC_SIZING,   /* its window does not fit the compile-time sample buffer */
    DERIVED_NC_ELEMENTS, /* binned, multi-element or per-gear: needs more than a scalar state */
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
 * The tester's D-075 stop epoch as the glue read it from services/derived_inputs (D-087
 * item 2): a ride is one epoch. `counter` changes whenever a new epoch starts (wrap-safe,
 * any jump size); `open` is false while the ECU is absent. `known` is false when the caller
 * has no epoch information (the part-1 class-A-only step): no ride then.
 */
typedef struct {
    bool known;
    bool open;
    uint16_t counter;
} derived_epoch_t;

/* Which ride record derived_core_ride_out() / derived_ride_get() reads. */
typedef enum {
    DERIVED_RIDE_CURRENT = 0, /* the open ride, as it would close now */
    DERIVED_RIDE_PREVIOUS     /* the last closed ride */
} derived_ride_which_t;

#ifdef __cplusplus
}
#endif

#endif /* FEATURES_DERIVED_DERIVED_TYPES_H */
