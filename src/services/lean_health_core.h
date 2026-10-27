#ifndef SERVICES_LEAN_HEALTH_CORE_H
#define SERVICES_LEAN_HEALTH_CORE_H

/*
 * Pure side of DID 0xFD03 LEAN_HEALTH (D-067, ISSUES E-17 (a)): builds the 5-byte record
 * of gen/ platform_uds.h from the EKF's last published health fields and the comms
 * task's own flags, and tracks the unlearned-run duration on the EKF side. No HAL, RTOS
 * or clock access: the time is passed in. Diagnostic only (QM): no safety decision reads
 * any of it.
 *
 * Layout, flags, widths and maxima all come from gen/ (PLATFORM_UDS_LEAN_HEALTH_*), the
 * rules from the DID in vehicle-defs uds/dids.yaml (read the comment block above
 * max_age_ms: it is the safety review's race-freedom contract):
 *   - EKF-side fields (offset learned / rejected, reject count, unlearned seconds, the
 *     speed-consumer fault latch) travel in the EKF's existing lean record through the
 *     SPSC snapshot; the comms task copies them while lean_tx_step() reads the record
 *     (services/lean_health.h), and this builder only ever sees that copy.
 *   - RECORD_FRESH follows lean_tx_core's age rule (a stamp more than
 *     LEAN_HEALTH_FUTURE_MAX_MS ahead of now is not fresh, never a wrapped age) with the
 *     DID's wider window PLATFORM_UDS_LEAN_HEALTH_MAX_AGE_MS.
 *   - No record since boot, or an older one: RECORD_FRESH and OFFSET_LEARNED clear,
 *     UNLEARNED_S = UNKNOWN, the other EKF fields their last values (0 before the first
 *     record). RESERVED is always 0. Counters saturate, never wrap.
 */

#include "platform_uds.h"
#include "services/lean_health_types.h"
#include "vehicle_cl250.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* How far a record stamp may lie after now: one comms pass, the same value as
 * LEAN_TX_FUTURE_MAX_MS (static-asserted in lean_tx.c; a service cannot include a
 * feature). */
#define LEAN_HEALTH_FUTURE_MAX_MS (VEHICLE_CL250_CLIENT_STEP_MAX_MS)

/* lean_health_ekf_t (the EKF-side fields of the lean record, written by the EKF task each
 * step) lives in services/lean_health_types.h. */

/* The comms task's copy of the last lean record (services/lean_health.h). The zero
 * state is "no record since boot". */
typedef struct {
    bool have_record;            /* a record has been published since boot */
    uint32_t stamp_ms;           /* the record's IMU stamp */
    lean_health_ekf_t ekf;       /* its EKF-side fields */
    bool speed_consumer_fault;   /* the EKF task's speed-snapshot fault latch */
} lean_health_rec_t;

/* Comms-task state read at request time. */
typedef struct {
    bool speed_producer_fault;   /* the comms task's own speed-snapshot publish latch */
    bool output_consumer_fault;  /* the comms task's own output-snapshot read latch */
    bool ekf_alive_ok;           /* an EKF is registered AND its alive supervision passes */
} lean_health_comms_t;

/* The EKF's unlearned-run tracker (EKF task only). The zero state is "not started". */
typedef struct {
    bool started;
    bool learned;       /* the offset was learnt at the last call */
    bool saturated;     /* this run reached the maximum: latched until the run restarts */
    uint32_t since_ms;  /* start of the current run without a learnt offset */
} lean_health_run_t;

/* v clamped to max. */
uint16_t lean_health_sat16(uint32_t v, uint32_t max);

/* EKF task, once per published step: whole seconds of the current run without a learnt
 * offset, saturating at the gen/ maximum (latched, so a run longer than the 2^32 ms
 * timer wrap stays saturated); 0 while one is learnt. The run starts at the first call
 * (the EKF's first published step) and again whenever a learnt offset is lost. A NULL
 * tracker reads 0.
 * Deviation from the DID text, kept on purpose (safety-reviewer MINOR-2): uds/dids.yaml
 * says "since boot", this anchors at the EKF's first step, one EKF cycle or more later.
 * Aligning the wording is a defs choice (ISSUES E-17). */
uint16_t lean_health_unlearned_s(lean_health_run_t* run, bool learned, uint32_t now_ms);

/* True when rec is a record the EKF published within the DID's window as of now_ms. */
bool lean_health_record_fresh(const lean_health_rec_t* rec, uint32_t now_ms);

/* Fills out (len must be PLATFORM_UDS_DID_LEAN_HEALTH_LENGTH) from rec and comms as of
 * now_ms. False on NULL or a length that differs from gen/ (refuse rather than guess);
 * out is then untouched. */
bool lean_health_build(const lean_health_rec_t* rec, const lean_health_comms_t* comms,
                       uint32_t now_ms, uint8_t* out, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* SERVICES_LEAN_HEALTH_CORE_H */
