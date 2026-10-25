#ifndef FEATURES_DERIVED_DERIVED_RIDE_H
#define FEATURES_DERIVED_DERIVED_RIDE_H

/*
 * Class-B ride summaries (D-070 item 2, parts 2-3; D-087 item 2). A ride is one D-075 stop
 * epoch of the vehicle tester: the core opens it when the epoch starts and closes it when the
 * epoch ends or another one starts. This file holds the pure accumulators and the closed
 * values; the epoch handling and the class-A reset are in derived_core.c.
 *
 * Computed signals (decided from the gen/ table: class B, rt_core_ready, scalar, inputs only
 * DIDs or computed signals, and a formula here): ride_distance, ride_time, ride_idle_time,
 * ride_stop_count, ride_fuel_used, ride_fuel_consumption, ride_ltft_mean, ride_stft_std,
 * ride_coolant_rate_max, ride_start_coolant_temp, ride_start_intake_temp,
 * ride_end_coolant_temp. Binned, per-gear, sensor-input, window-record (W1-W5) and the other
 * class-B signals are NOT_COMPUTED with a reason.
 *
 * The state is a handful of scalars for the open ride plus one closed record (the previous
 * ride) with one output row per computed signal: no heap, no unbounded loop. The time
 * integral and the means keep `double` accumulators (a long ride would lose small
 * increments in float32; the M7 has a double-precision FPU); results are float32.
 *
 * Semantics follow the moto-server reference (derived/summary.py): "last valid" signals take
 * the last valid class-A output of the ride; means and the first/last values are over every
 * DID sample of the ride; the fuel integral holds a sample until the next one but never
 * beyond that DID's own stale_after_ms (D-087), and the segment still open at the close adds
 * nothing. A power-down close is not handled here (no persistence yet, D-045): the epoch
 * flag is the only ride signal.
 */

#include "features/derived/derived_types.h"
#include "vehicle_cl250_derived.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DERIVED_RIDE_SLOTS (12u)
#define DERIVED_RIDE_NO_SLOT (0xFFu)

typedef struct {
    float trip_distance; /* last valid class-A outputs of the ride */
    float trip_time;
    float idle_time;
    float coolant_rate_max;
    float coolant_first;
    float coolant_last;
    float intake_first;
    uint32_t stop_count;
    uint32_t ltft_n;
    uint32_t stft_n;
    double ltft_sum;
    double stft_mean; /* Welford */
    double stft_m2;
    double fuel_prefix; /* closed segments of the fuel integral, litres */
    double fuel_rate;   /* L/s of the open segment */
    uint32_t fuel_seg_start_ms;
    int32_t fuel_seg_remaining_ms;
    bool fuel_seg_active;
    bool fuel_seen;
    bool has_trip_distance;
    bool has_trip_time;
    bool has_idle_time;
    bool has_coolant_rate;
    bool has_coolant;
    bool has_intake;
} derived_ride_acc_t;

typedef struct {
    derived_ride_acc_t acc; /* the open ride's running state */
    derived_out_t prev[DERIVED_RIDE_SLOTS]; /* the last closed ride, per slot */
    uint32_t last_now_ms; /* time of the last core step (stamps CURRENT reads) */
    uint32_t closed_count;
    uint16_t counter; /* epoch counter of the open ride */
    bool open;
    bool have_prev;
} derived_ride_t;

/* Slot of a class-B signal this file computes (0..DERIVED_RIDE_SLOTS-1), else NO_SLOT. */
uint32_t derived_ride_slot(uint32_t sig);

/* Boot: no ride, no closed record. */
void derived_ride_init(derived_ride_t* r);

/* A new ride: fresh accumulators. */
void derived_ride_begin(derived_ride_t* r, uint16_t counter);

/* A new DID sample of the open ride (once per sample, in time order). */
void derived_ride_sample(derived_ride_acc_t* a, uint32_t did, float value, uint32_t stamp_ms);

/* A valid class-A output of the open ride (the signals the summaries read; others ignored). */
void derived_ride_class_a(derived_ride_acc_t* a, uint32_t sig, float value);

/* The value `sig` would close with now. status VALID or INVALID (flag), unranged, value;
 * stamp and seq are the caller's. Not a computed class-B slot: NOT_COMPUTED. */
void derived_ride_finalize(const derived_ride_acc_t* a, uint32_t sig, derived_out_t* out);

/* Closes the open ride into `prev` at now_ms. */
void derived_ride_close(derived_ride_t* r, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* FEATURES_DERIVED_DERIVED_RIDE_H */
