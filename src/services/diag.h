#ifndef SERVICES_DIAG_H
#define SERVICES_DIAG_H

/*
 * Diagnostic state service (≈ AUTOSAR Dem, concept only, D-006): rt-core's own DTC
 * memory, and the vehicle-tester status and step counters that the platform UDS server
 * reports (0xFD00, 0xFD02; D-040, D-055). The UDS client glue writes the status, the
 * step counters and its DTC results; the UDS server glue reads them, clears the DTCs and
 * reports its own monitors (VEHICLE_TESTER_LATCHED supervision, VEHICLE_BUS_OFF_LATCHED
 * from services/can_sm). Neither feature includes the other, and diag includes no other
 * service.
 *
 * DTCs are the gen/ table (platform_uds.h, platform_uds_dtcs[]), indexed by
 * platform_uds_dtc_index_t. A monitor reports its test result every pass
 * (level-triggered): a failed test sets TEST_FAILED and CONFIRMED_DTC, a passed test
 * clears TEST_FAILED only. Only bits in PLATFORM_UDS_DTC_STATUS_AVAILABILITY_MASK are
 * ever set. diag_dtc_clear_all() (UDS 0x14) zeroes every status byte; a condition that
 * is still active sets its bits again on the monitor's next report, so a clear can hide
 * an active fault for one pass at most. Clearing never changes the monitored condition
 * itself (for example the UDS client latch, D-039).
 *
 * Fail-safe vehicle-tester status (D-040, safety review MAJOR-1): the status carries the
 * time it was written. None since diag_init(), or one older than
 * PLATFORM_UDS_VEHICLE_TESTER_STATUS_MAX_AGE_MS, reads as FAULT = NOT_RUNNING with every
 * flag clear, and diag_supervise() then fails VEHICLE_TESTER_LATCHED. A client that
 * never opened or stopped therefore never looks healthy.
 *
 * Step counters (D-055): the client's D-053 counters with the time they were written.
 * None since diag_init() reads as zero, not fresh; older than
 * PLATFORM_UDS_RT_CORE_HEALTH_MAX_AGE_MS reads as the last values, not fresh (sticky
 * until the next write, like the status). Freshness is their only health signal.
 *
 * RAM only: the memory is lost on reset until the H7 flash driver exists (D-040).
 * Static storage, no heap. Main loop only (not ISR-safe).
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* State of rt-core's vehicle-bus UDS client, as the 0xFD00 DID reports it. `fault` is
 * a PLATFORM_UDS_VEHICLE_TESTER_STATUS_FAULT_* value from gen/. */
typedef struct {
    bool ecu_present;
    bool session_up;
    bool latched;
    uint8_t fault;
} diag_vehicle_tester_t;

/* The vehicle UDS client's D-053 step counters, as the 0xFD02 DID reports them. */
typedef struct {
    uint32_t overruns;   /* saturating */
    uint32_t gap_max_ms; /* saturating */
    bool fresh;          /* written within PLATFORM_UDS_RT_CORE_HEALTH_MAX_AGE_MS */
} diag_client_steps_t;

/* Clears every DTC status; the vehicle tester reads NOT_RUNNING and the step counters
 * zero and not fresh until their first write. */
void diag_init(uint32_t now_ms);

/* A monitor's test result for DTC index idx (ignored if out of range). */
void diag_dtc_report(uint32_t idx, bool failed);

/* DTCStatusMask of DTC index idx; 0 if out of range. */
uint8_t diag_dtc_status(uint32_t idx);

/* ClearDiagnosticInformation for all groups: every status byte to 0. */
void diag_dtc_clear_all(void);

/* Latest vehicle-tester status (writer: the UDS client glue, every step). */
void diag_set_vehicle_tester(const diag_vehicle_tester_t* status, uint32_t now_ms);

/* The status as of now_ms: NOT_RUNNING with every flag clear if it is missing or older
 * than PLATFORM_UDS_VEHICLE_TESTER_STATUS_MAX_AGE_MS. */
diag_vehicle_tester_t diag_vehicle_tester(uint32_t now_ms);

/* Latest step counters (writer: the UDS client glue, every step, latched or not). */
void diag_set_client_steps(uint32_t overruns, uint32_t gap_max_ms, uint32_t now_ms);

/* The step counters as of now_ms; `fresh` false if none was written since init or the
 * last write is older than PLATFORM_UDS_RT_CORE_HEALTH_MAX_AGE_MS (wrap-safe). */
diag_client_steps_t diag_client_steps(uint32_t now_ms);

/* Every pass (the UDS server glue): fails VEHICLE_TESTER_LATCHED while the status is
 * missing or stale, including MAX_AGE_MS after init with no status at all, and marks
 * stale step counters as not fresh until their next write. */
void diag_supervise(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* SERVICES_DIAG_H */
