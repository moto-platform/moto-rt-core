#include "features/derived/derived_gear.h"

#include "vehicle_cl250_constants.h"

#include <math.h>
#include <stdbool.h>

#if defined(__FAST_MATH__)
#error "derived_gear.c must not be built with -ffast-math"
#endif

/* The table lists the six gen/ ratios; a gearbox with another gear count must extend it. */
_Static_assert(VEHICLE_CL250_GEAR_COUNT == 6u, "gear table covers exactly the gen/ gear count");

uint8_t derived_gear_estimate(float measured_ratio)
{
    static const float gear_ratio[VEHICLE_CL250_GEAR_COUNT] = {
        VEHICLE_CL250_GEAR_RATIO_1, VEHICLE_CL250_GEAR_RATIO_2, VEHICLE_CL250_GEAR_RATIO_3,
        VEHICLE_CL250_GEAR_RATIO_4, VEHICLE_CL250_GEAR_RATIO_5, VEHICLE_CL250_GEAR_RATIO_6,
    };
    uint8_t gear = 0u;
    if ((isfinite(measured_ratio) != 0) && (measured_ratio > 0.0f)) {
        float best = 0.0f;
        uint8_t best_gear = 0u;
        for (uint8_t i = 0u; i < (uint8_t)VEHICLE_CL250_GEAR_COUNT; i++) {
            const float dist = fabsf((measured_ratio / gear_ratio[i]) - 1.0f);
            if ((best_gear == 0u) || (dist < best)) {
                best = dist;
                best_gear = (uint8_t)(i + 1u);
            }
        }
        if (best <= DERIVED_GEAR_MATCH_TOLERANCE) {
            gear = best_gear;
        }
    }
    return gear;
}
