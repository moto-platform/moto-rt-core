#ifndef FEATURES_VEHICLE_REPUBLISH_CORE_H
#define FEATURES_VEHICLE_REPUBLISH_CORE_H

/*
 * Pure logic of the platform-bus republisher (ISSUES D-5, D-056): vehicle signal samples
 * to the gen/ platform messages 0x021 VehicleSpeed and 0x110 VehicleEngine, the E2E
 * protection of 0x021 and the cycle schedule. No HAL, RTOS or CAN port access: the glue
 * (vehicle_republish.c) reads services/vehicle_signals and writes services/can_if.
 *
 * Every ID, layout, scale, range, cycle time and E2E DataID comes from gen/ (platform.h,
 * platform_e2e.h, vehicle_cl250.h); this file names no CAN literal (invariant 2).
 */

#include "platform.h"
#include "platform_e2e.h"
#include "services/vehicle_signals.h"
#include "vehicle_cl250.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* An age past the range of VEHICLE_SPEED_AGE: the generated encode saturates at the
 * signal's maximum (2550 ms, D-056), so an INVALID speed never reads as fresh. */
#define VEHICLE_REPUBLISH_AGE_SATURATED_MS UINT32_MAX

/* 0x021 from the 0xF40D sample (D-056 item 2): VALID only for a VALID sample whose value
 * is inside the signal's physical range (non-finite values fail that check), value 0 and
 * AGE saturated otherwise; no clamping. */
void vehicle_republish_speed_msg(const vehicle_signal_sample_t* speed,
                                 struct platform_vehicle_speed_t* msg);

/* 0x110 from the engine DIDs, indexed like vehicle_cl250_dids[], and the ECU presence:
 * per signal the same VALID rule, value 0 when INVALID. */
void vehicle_republish_engine_msg(const vehicle_signal_sample_t samples[VEHICLE_CL250_DID_COUNT],
                                  bool ecu_present, struct platform_vehicle_engine_t* msg);

/*
 * Packs 0x021 and protects it with the next E2E counter. *next receives the state after
 * this frame; the caller copies it to its committed state only when the port accepted
 * the frame (D-056 item 5), so a refused write leaves no counter gap. False if packing
 * failed (out untouched beyond its length).
 */
bool vehicle_republish_speed_frame(const struct platform_vehicle_speed_t* msg,
                                   const moto_e2e_tx_state_t* committed,
                                   moto_e2e_tx_state_t* next,
                                   uint8_t out[PLATFORM_VEHICLE_SPEED_LENGTH]);

/* Packs 0x110 (no E2E: state range). */
bool vehicle_republish_engine_frame(const struct platform_vehicle_engine_t* msg,
                                    uint8_t out[PLATFORM_VEHICLE_ENGINE_LENGTH]);

#ifdef __cplusplus
}
#endif

#endif /* FEATURES_VEHICLE_REPUBLISH_CORE_H */
