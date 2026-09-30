#ifndef SERVICES_DIAG_H
#define SERVICES_DIAG_H

/*
 * Diagnostic state service (≈ AUTOSAR Dem, concept only, D-006): rt-core's own DTC
 * memory and the vehicle-tester status that the platform UDS server reports (D-040).
 * Features write it, the UDS server reads and clears it; neither includes the other.
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

/* Clears every DTC status; the vehicle tester reads NOT_RUNNING until its first status. */
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

/* Every pass (the UDS server glue): fails VEHICLE_TESTER_LATCHED while the status is
 * missing or stale, including MAX_AGE_MS after init with no status at all. */
void diag_supervise(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* SERVICES_DIAG_H */
