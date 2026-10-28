#ifndef FEATURES_DERIVED_DERIVED_GEAR_H
#define FEATURES_DERIVED_DERIVED_GEAR_H

/*
 * Gear estimate from a measured gearbox ratio (D-070, catalog section 5): pure, no state.
 * Kept in its own file so it can move to services/ when a second feature needs a gear
 * (architecture-guard MAJOR on the C3 plan); until then it is private to features/derived.
 * The ratios come from gen/ vehicle_cl250_constants.h only.
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* A measured ratio matches a gear when it is within this relative distance of the gear's
 * ratio. Algorithm parameter, provisional (D-070 item 6); same value as the reference
 * (vehicle-server derived/formulas.py:62 GEAR_MATCH_TOLERANCE). */
#define DERIVED_GEAR_MATCH_TOLERANCE (0.05f)

/* The gear 1..VEHICLE_CL250_GEAR_COUNT whose ratio is nearest to `measured_ratio`
 * (relative distance, the lower gear on a tie), or 0 when none is within the tolerance
 * or the ratio is not a finite positive number. */
uint8_t derived_gear_estimate(float measured_ratio);

#ifdef __cplusplus
}
#endif

#endif /* FEATURES_DERIVED_DERIVED_GEAR_H */
