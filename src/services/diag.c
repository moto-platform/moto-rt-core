#include "services/diag.h"

#include "platform_uds.h"
#include "uds_iso14229.h"

#include <stddef.h>

#define DTC_FAILED_BITS \
    ((uint8_t)((UDS_DTC_STATUS_TEST_FAILED | UDS_DTC_STATUS_CONFIRMED_DTC) & \
               PLATFORM_UDS_DTC_STATUS_AVAILABILITY_MASK))
#define DTC_ACTIVE_BIT \
    ((uint8_t)(UDS_DTC_STATUS_TEST_FAILED & PLATFORM_UDS_DTC_STATUS_AVAILABILITY_MASK))

static uint8_t dtc_status[PLATFORM_UDS_DTC_COUNT];
static diag_vehicle_tester_t vehicle_tester;
static uint32_t tester_stamp_ms; /* last status, or diag_init() while none arrived */
static bool tester_seen;
/* Sticky once supervise saw the status missing or stale, until the next status: the ms
 * counter wrap (~49.7 days) cannot make an old status fresh again. */
static bool tester_lost;

static bool tester_fresh(uint32_t now_ms)
{
    return tester_seen && !tester_lost &&
           ((uint32_t)(now_ms - tester_stamp_ms) < PLATFORM_UDS_VEHICLE_TESTER_STATUS_MAX_AGE_MS);
}

void diag_init(uint32_t now_ms)
{
    diag_dtc_clear_all();
    vehicle_tester.ecu_present = false;
    vehicle_tester.session_up = false;
    vehicle_tester.latched = false;
    vehicle_tester.fault = (uint8_t)PLATFORM_UDS_VEHICLE_TESTER_STATUS_FAULT_NOT_RUNNING;
    tester_stamp_ms = now_ms;
    tester_seen = false;
    tester_lost = false;
}

void diag_dtc_report(uint32_t idx, bool failed)
{
    if (idx >= PLATFORM_UDS_DTC_COUNT) {
        return;
    }
    if (failed) {
        dtc_status[idx] = (uint8_t)(dtc_status[idx] | DTC_FAILED_BITS);
    } else {
        dtc_status[idx] = (uint8_t)(dtc_status[idx] & (uint8_t)~DTC_ACTIVE_BIT);
    }
}

uint8_t diag_dtc_status(uint32_t idx)
{
    return (idx < PLATFORM_UDS_DTC_COUNT) ? dtc_status[idx] : 0u;
}

void diag_dtc_clear_all(void)
{
    for (uint32_t i = 0u; i < PLATFORM_UDS_DTC_COUNT; i++) {
        dtc_status[i] = 0u;
    }
}

void diag_set_vehicle_tester(const diag_vehicle_tester_t* status, uint32_t now_ms)
{
    if (status != NULL) {
        vehicle_tester = *status;
        tester_stamp_ms = now_ms;
        tester_seen = true;
        tester_lost = false;
    }
}

diag_vehicle_tester_t diag_vehicle_tester(uint32_t now_ms)
{
    if (tester_fresh(now_ms)) {
        return vehicle_tester;
    }
    const diag_vehicle_tester_t not_running = {
        false, false, false, (uint8_t)PLATFORM_UDS_VEHICLE_TESTER_STATUS_FAULT_NOT_RUNNING};
    return not_running;
}

void diag_supervise(uint32_t now_ms)
{
    if ((uint32_t)(now_ms - tester_stamp_ms) >= PLATFORM_UDS_VEHICLE_TESTER_STATUS_MAX_AGE_MS) {
        tester_lost = true; /* no status since init, or the last one is too old */
    }
    if (tester_lost) {
        diag_dtc_report(PLATFORM_UDS_DTC_IDX_VEHICLE_TESTER_LATCHED, true);
    }
}
