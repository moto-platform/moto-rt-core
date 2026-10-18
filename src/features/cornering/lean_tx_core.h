#ifndef FEATURES_CORNERING_LEAN_TX_CORE_H
#define FEATURES_CORNERING_LEAN_TX_CORE_H

/*
 * Pure 0x020 EkfLean builder (D-065 items 2-3, PR 3b): turns the EKF's last output
 * record, as the comms task sees it, into the gen/ message. No HAL, RTOS or clock.
 *
 * Rules, in order (the first failing one gives INVALID with LEAN 0 and QUALITY 0, never
 * RESERVED; D-065 item 1, safety-reviewer MAJOR-1 on defs v0.9.0):
 *  1. a snapshot fault on either side;
 *  2. the output's age: a stamp up to LEAN_TX_FUTURE_MAX_MS after now is age 0 (the EKF
 *     preempts the comms pass between its clock read and its snapshot read;
 *     architecture-guard MAJOR-2 on the PR 3 plan), a stamp further ahead or older than
 *     LEAN_TX_AGE_MAX_MS is INVALID: a stale output still passes E2E, so the receivers'
 *     60 ms timeout alone would not catch a replayed lean;
 *  3. the core state, mapped by a switch: only LEAN_OUT_ESTIMATED goes on;
 *  4. a finite roll strictly inside the model bound of 90 deg, and a QUALITY within
 *     LEAN_QUALITY_MIN and the gen/ range;
 *  5. the clamp: |roll| above gen/ PLATFORM_LIMIT_LEAN_ANGLE_CLAMP_MAX_DEG goes out as
 *     that limit with its sign and state CLAMPED (D-065 item 3). Only this copy is
 *     clamped; the EKF state never sees it.
 */

#include "features/cornering/lean_core.h"
#include "platform.h"
#include "platform_e2e.h"
#include "platform_limits.h"
#include "services/lean_health_core.h"
#include "vehicle_cl250.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Release jitter of the EKF and comms tasks together (provisional until HIL, Q-019). */
#define LEAN_TX_JITTER_MS (5u)

/* Output age budget (D-065 item 2): one EKF cycle, plus one comms pass at most (the
 * vehicle client's step bound, D-053), plus jitter: 20 + 10 + 5 = 35 ms. */
#define LEAN_TX_AGE_MAX_MS \
    (PLATFORM_EKF_LEAN_CYCLE_TIME_MS + VEHICLE_CL250_CLIENT_STEP_MAX_MS + LEAN_TX_JITTER_MS)

/* How far the output stamp may lie after the comms pass's now: one comms pass. */
#define LEAN_TX_FUTURE_MAX_MS (VEHICLE_CL250_CLIENT_STEP_MAX_MS)

/* The EKF's output as it crosses to the comms task (services/snapshot). The zero record
 * is INVALID (state 0, stamp 0). */
typedef struct {
    lean_out_t out;
    uint32_t stamp_ms; /* the IMU stamp of the step that produced out */
    bool fault;        /* an EKF-side snapshot fault is latched (architecture-guard MAJOR-4) */
    /* The EKF-side fields of DID 0xFD03 LEAN_HEALTH (D-067, E-17 (a)): diagnostic only,
     * never read by lean_tx_msg(). They travel in this record, through the same SPSC
     * snapshot, so no other shared state exists. */
    lean_health_ekf_t health;
} lean_record_t;

/* Fills *msg from rec as seen at now_ms (see the rules above). comms_fault: the comms
 * side's own snapshot fault. A NULL rec reads INVALID; NULL msg does nothing. */
void lean_tx_msg(const lean_record_t* rec, bool comms_fault, uint32_t now_ms,
                 struct platform_ekf_lean_t* msg);

/* Packs msg and protects it with the gen/ E2E on a copy of the committed state: *next
 * holds the advanced state, to be committed only when the port accepts the frame (D-056
 * item 5). False on NULL or a packing/E2E failure. */
bool lean_tx_frame(const struct platform_ekf_lean_t* msg, const moto_e2e_tx_state_t* committed,
                   moto_e2e_tx_state_t* next, uint8_t out[PLATFORM_EKF_LEAN_LENGTH]);

#ifdef __cplusplus
}
#endif

#endif /* FEATURES_CORNERING_LEAN_TX_CORE_H */
