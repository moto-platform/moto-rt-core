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
static diag_client_steps_t client_steps; /* `fresh` unused here: computed on read */
static uint32_t steps_stamp_ms;
static bool steps_seen;
static bool steps_lost; /* sticky like tester_lost, until the next write */

static bool tester_fresh(uint32_t now_ms)
{
    return tester_seen && !tester_lost &&
           ((uint32_t)(now_ms - tester_stamp_ms) < PLATFORM_UDS_VEHICLE_TESTER_STATUS_MAX_AGE_MS);
}

static bool steps_fresh(uint32_t now_ms)
{
    return steps_seen && !steps_lost &&
           ((uint32_t)(now_ms - steps_stamp_ms) < PLATFORM_UDS_RT_CORE_HEALTH_MAX_AGE_MS);
}

void diag_init(uint32_t now_ms)
{
    diag_dtc_clear_all();
    client_steps.overruns = 0u;
    client_steps.gap_max_ms = 0u;
    client_steps.fresh = false;
    steps_stamp_ms = now_ms;
    steps_seen = false;
    steps_lost = false;
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

void diag_set_client_steps(uint32_t overruns, uint32_t gap_max_ms, uint32_t now_ms)
{
    client_steps.overruns = overruns;
    client_steps.gap_max_ms = gap_max_ms;
    steps_stamp_ms = now_ms;
    steps_seen = true;
    steps_lost = false;
}

diag_client_steps_t diag_client_steps(uint32_t now_ms)
{
    diag_client_steps_t out = client_steps;
    out.fresh = steps_fresh(now_ms);
    return out;
}

void diag_supervise(uint32_t now_ms)
{
    if ((uint32_t)(now_ms - steps_stamp_ms) >= PLATFORM_UDS_RT_CORE_HEALTH_MAX_AGE_MS) {
        steps_lost = true; /* the ms counter wrap cannot make old counters fresh again */
    }
    if ((uint32_t)(now_ms - tester_stamp_ms) >= PLATFORM_UDS_VEHICLE_TESTER_STATUS_MAX_AGE_MS) {
        tester_lost = true; /* no status since init, or the last one is too old */
    }
    if (tester_lost) {
        diag_dtc_report(PLATFORM_UDS_DTC_IDX_VEHICLE_TESTER_LATCHED, true);
    }
}
