/*
 * L0 tests for the diagnostic state service (services/diag): DTC status bytes with
 * level-triggered reports (ISO 14229-1 Annex D status bits), ClearDiagnosticInformation
 * semantics, the vehicle-tester status and the client step counters of 0xFD02 (D-055).
 * No requirement IDs yet (Q-006).
 */
#include "platform_uds.h"
#include "services/diag.h"
#include "uds_iso14229.h"

#include <unity.h>

#define FAILED_BITS \
    ((uint8_t)((UDS_DTC_STATUS_TEST_FAILED | UDS_DTC_STATUS_CONFIRMED_DTC) & \
               PLATFORM_UDS_DTC_STATUS_AVAILABILITY_MASK))
#define TEST_FAILED_BIT \
    ((uint8_t)(UDS_DTC_STATUS_TEST_FAILED & PLATFORM_UDS_DTC_STATUS_AVAILABILITY_MASK))
#define CONFIRMED_BIT \
    ((uint8_t)(UDS_DTC_STATUS_CONFIRMED_DTC & PLATFORM_UDS_DTC_STATUS_AVAILABILITY_MASK))

#define T0 5000u
#define MAX_AGE PLATFORM_UDS_VEHICLE_TESTER_STATUS_MAX_AGE_MS
#define STEPS_MAX_AGE PLATFORM_UDS_RT_CORE_HEALTH_MAX_AGE_MS

void setUp(void)
{
    diag_init(T0);
}

void tearDown(void)
{
}

static void test_init_clears_every_dtc_and_the_tester_status(void)
{
    diag_dtc_report(PLATFORM_UDS_DTC_IDX_VEHICLE_ECU_COMM_LOST, true);
    const diag_vehicle_tester_t t = {true, true, true,
                                     PLATFORM_UDS_VEHICLE_TESTER_STATUS_FAULT_GUARD};
    diag_set_vehicle_tester(&t, T0);
    diag_init(T0);
    for (uint32_t i = 0u; i < PLATFORM_UDS_DTC_COUNT; i++) {
        TEST_ASSERT_EQUAL_HEX8(0u, diag_dtc_status(i));
    }
    const diag_vehicle_tester_t r = diag_vehicle_tester(T0);
    TEST_ASSERT_FALSE(r.ecu_present);
    TEST_ASSERT_FALSE(r.session_up);
    TEST_ASSERT_FALSE(r.latched);
    TEST_ASSERT_EQUAL_UINT8(PLATFORM_UDS_VEHICLE_TESTER_STATUS_FAULT_NOT_RUNNING, r.fault);
}

static void test_failed_report_sets_test_failed_and_confirmed_within_the_availability_mask(void)
{
    for (uint32_t i = 0u; i < PLATFORM_UDS_DTC_COUNT; i++) {
        diag_dtc_report(i, true);
        TEST_ASSERT_EQUAL_HEX8(FAILED_BITS, diag_dtc_status(i));
        TEST_ASSERT_EQUAL_HEX8(
            0u, (uint8_t)(diag_dtc_status(i) & (uint8_t)~PLATFORM_UDS_DTC_STATUS_AVAILABILITY_MASK));
    }
}

static void test_dtc_reports_are_independent_per_index(void)
{
    diag_dtc_report(PLATFORM_UDS_DTC_IDX_VEHICLE_TESTER_LATCHED, true);
    TEST_ASSERT_EQUAL_HEX8(0u, diag_dtc_status(PLATFORM_UDS_DTC_IDX_VEHICLE_ECU_COMM_LOST));
    TEST_ASSERT_EQUAL_HEX8(FAILED_BITS,
                           diag_dtc_status(PLATFORM_UDS_DTC_IDX_VEHICLE_TESTER_LATCHED));
}

static void test_passed_report_clears_test_failed_only_and_confirmed_stays(void)
{
    diag_dtc_report(PLATFORM_UDS_DTC_IDX_VEHICLE_ECU_COMM_LOST, true);
    diag_dtc_report(PLATFORM_UDS_DTC_IDX_VEHICLE_ECU_COMM_LOST, false);
    const uint8_t st = diag_dtc_status(PLATFORM_UDS_DTC_IDX_VEHICLE_ECU_COMM_LOST);
    TEST_ASSERT_EQUAL_HEX8(0u, (uint8_t)(st & TEST_FAILED_BIT));
    TEST_ASSERT_EQUAL_HEX8(CONFIRMED_BIT, (uint8_t)(st & CONFIRMED_BIT));
}

static void test_passed_report_on_a_clean_dtc_leaves_it_clean(void)
{
    diag_dtc_report(PLATFORM_UDS_DTC_IDX_VEHICLE_ECU_COMM_LOST, false);
    TEST_ASSERT_EQUAL_HEX8(0u, diag_dtc_status(PLATFORM_UDS_DTC_IDX_VEHICLE_ECU_COMM_LOST));
}

static void test_clear_all_zeroes_every_status_byte(void)
{
    for (uint32_t i = 0u; i < PLATFORM_UDS_DTC_COUNT; i++) {
        diag_dtc_report(i, true);
    }
    diag_dtc_clear_all();
    for (uint32_t i = 0u; i < PLATFORM_UDS_DTC_COUNT; i++) {
        TEST_ASSERT_EQUAL_HEX8(0u, diag_dtc_status(i));
    }
}

static void test_clear_does_not_touch_the_vehicle_tester_status(void)
{
    const diag_vehicle_tester_t t = {true, false, true,
                                     PLATFORM_UDS_VEHICLE_TESTER_STATUS_FAULT_GATE};
    diag_set_vehicle_tester(&t, T0);
    diag_dtc_clear_all();
    TEST_ASSERT_TRUE(diag_vehicle_tester(T0).latched);
    TEST_ASSERT_EQUAL_UINT8(PLATFORM_UDS_VEHICLE_TESTER_STATUS_FAULT_GATE,
                            diag_vehicle_tester(T0).fault);
}

static void test_a_still_failing_report_after_clear_sets_the_bits_again(void)
{
    diag_dtc_report(PLATFORM_UDS_DTC_IDX_VEHICLE_TESTER_LATCHED, true);
    diag_dtc_clear_all();
    TEST_ASSERT_EQUAL_HEX8(0u, diag_dtc_status(PLATFORM_UDS_DTC_IDX_VEHICLE_TESTER_LATCHED));
    diag_dtc_report(PLATFORM_UDS_DTC_IDX_VEHICLE_TESTER_LATCHED, true);
    TEST_ASSERT_EQUAL_HEX8(FAILED_BITS,
                           diag_dtc_status(PLATFORM_UDS_DTC_IDX_VEHICLE_TESTER_LATCHED));
}

static void test_out_of_range_index_is_ignored_and_reads_zero(void)
{
    diag_dtc_report(PLATFORM_UDS_DTC_COUNT, true);
    diag_dtc_report(UINT32_MAX, true);
    for (uint32_t i = 0u; i < PLATFORM_UDS_DTC_COUNT; i++) {
        TEST_ASSERT_EQUAL_HEX8(0u, diag_dtc_status(i));
    }
    TEST_ASSERT_EQUAL_HEX8(0u, diag_dtc_status(PLATFORM_UDS_DTC_COUNT));
    TEST_ASSERT_EQUAL_HEX8(0u, diag_dtc_status(UINT32_MAX));
}

static void test_vehicle_tester_status_round_trips(void)
{
    const diag_vehicle_tester_t t = {true, true, false,
                                     PLATFORM_UDS_VEHICLE_TESTER_STATUS_FAULT_FOREIGN_TESTER};
    diag_set_vehicle_tester(&t, T0);
    const diag_vehicle_tester_t r = diag_vehicle_tester(T0);
    TEST_ASSERT_TRUE(r.ecu_present);
    TEST_ASSERT_TRUE(r.session_up);
    TEST_ASSERT_FALSE(r.latched);
    TEST_ASSERT_EQUAL_UINT8(PLATFORM_UDS_VEHICLE_TESTER_STATUS_FAULT_FOREIGN_TESTER, r.fault);
}

static void test_null_vehicle_tester_set_is_ignored(void)
{
    const diag_vehicle_tester_t t = {true, false, true,
                                     PLATFORM_UDS_VEHICLE_TESTER_STATUS_FAULT_GUARD};
    diag_set_vehicle_tester(&t, T0);
    diag_set_vehicle_tester(NULL, T0);
    const diag_vehicle_tester_t r = diag_vehicle_tester(T0);
    TEST_ASSERT_TRUE(r.ecu_present);
    TEST_ASSERT_TRUE(r.latched);
    TEST_ASSERT_EQUAL_UINT8(PLATFORM_UDS_VEHICLE_TESTER_STATUS_FAULT_GUARD, r.fault);
}

static const diag_vehicle_tester_t healthy = {true, true, false,
                                              PLATFORM_UDS_VEHICLE_TESTER_STATUS_FAULT_NONE};

static void assert_not_running(uint32_t now)
{
    const diag_vehicle_tester_t r = diag_vehicle_tester(now);
    TEST_ASSERT_FALSE(r.ecu_present);
    TEST_ASSERT_FALSE(r.session_up);
    TEST_ASSERT_FALSE(r.latched);
    TEST_ASSERT_EQUAL_UINT8(PLATFORM_UDS_VEHICLE_TESTER_STATUS_FAULT_NOT_RUNNING, r.fault);
}

static void test_vehicle_tester_reads_not_running_before_the_first_status(void)
{
    assert_not_running(T0);
    assert_not_running(T0 + 1u);
}

static void test_a_fresh_status_is_returned_until_max_age(void)
{
    diag_set_vehicle_tester(&healthy, T0 + 10u);
    const diag_vehicle_tester_t r = diag_vehicle_tester(T0 + 10u + MAX_AGE - 1u);
    TEST_ASSERT_TRUE(r.ecu_present);
    TEST_ASSERT_TRUE(r.session_up);
    TEST_ASSERT_EQUAL_UINT8(PLATFORM_UDS_VEHICLE_TESTER_STATUS_FAULT_NONE, r.fault);
}

static void test_a_status_older_than_max_age_reads_not_running(void)
{
    diag_set_vehicle_tester(&healthy, T0 + 10u);
    assert_not_running(T0 + 10u + MAX_AGE);
}

static void test_supervise_fails_the_latched_dtc_only_after_max_age_without_any_status(void)
{
    diag_supervise(T0 + MAX_AGE - 1u);
    TEST_ASSERT_EQUAL_HEX8(0u, diag_dtc_status(PLATFORM_UDS_DTC_IDX_VEHICLE_TESTER_LATCHED));
    diag_supervise(T0 + MAX_AGE);
    TEST_ASSERT_EQUAL_HEX8(FAILED_BITS, diag_dtc_status(PLATFORM_UDS_DTC_IDX_VEHICLE_TESTER_LATCHED));
    TEST_ASSERT_EQUAL_HEX8(0u, diag_dtc_status(PLATFORM_UDS_DTC_IDX_VEHICLE_ECU_COMM_LOST));
}

static void test_supervise_keeps_a_fresh_status_healthy(void)
{
    for (uint32_t t = T0; t < (T0 + (3u * MAX_AGE)); t += 50u) {
        diag_set_vehicle_tester(&healthy, t);
        diag_supervise(t + 1u);
    }
    TEST_ASSERT_EQUAL_HEX8(0u, diag_dtc_status(PLATFORM_UDS_DTC_IDX_VEHICLE_TESTER_LATCHED));
}

static void test_supervise_fails_the_dtc_when_the_status_goes_stale_and_it_survives_a_clear(void)
{
    diag_set_vehicle_tester(&healthy, T0);
    diag_supervise(T0 + MAX_AGE - 1u);
    TEST_ASSERT_EQUAL_HEX8(0u, diag_dtc_status(PLATFORM_UDS_DTC_IDX_VEHICLE_TESTER_LATCHED));
    diag_supervise(T0 + MAX_AGE);
    TEST_ASSERT_EQUAL_HEX8(FAILED_BITS, diag_dtc_status(PLATFORM_UDS_DTC_IDX_VEHICLE_TESTER_LATCHED));
    diag_dtc_clear_all();
    diag_supervise(T0 + MAX_AGE + 1u); /* level-triggered: set again on the next pass */
    TEST_ASSERT_EQUAL_HEX8(FAILED_BITS, diag_dtc_status(PLATFORM_UDS_DTC_IDX_VEHICLE_TESTER_LATCHED));
}

static void test_a_stale_status_stays_not_running_across_the_clock_wrap_until_a_new_set(void)
{
    diag_set_vehicle_tester(&healthy, T0);
    diag_supervise(T0 + MAX_AGE); /* supervise sees it stale */
    /* 2^32 + 10 ms after the write the 32-bit clock reads T0 + 10: raw age 10 ms */
    assert_not_running(T0 + 10u);
    diag_supervise(T0 + 10u);
    assert_not_running(T0 + 11u);
    diag_set_vehicle_tester(&healthy, T0 + 20u);
    TEST_ASSERT_TRUE(diag_vehicle_tester(T0 + 21u).session_up);
}

static void test_a_new_status_after_staleness_makes_it_fresh_again(void)
{
    diag_set_vehicle_tester(&healthy, T0);
    diag_supervise(T0 + MAX_AGE);
    assert_not_running(T0 + MAX_AGE);
    diag_set_vehicle_tester(&healthy, T0 + MAX_AGE + 5u);
    TEST_ASSERT_TRUE(diag_vehicle_tester(T0 + MAX_AGE + 6u).ecu_present);
}

/* ------------------------------------------------------------------ step counters */

static void test_step_counters_read_zero_and_not_fresh_before_the_first_write(void)
{
    const diag_client_steps_t r = diag_client_steps(T0 + 1u);
    TEST_ASSERT_EQUAL_UINT32(0u, r.overruns);
    TEST_ASSERT_EQUAL_UINT32(0u, r.gap_max_ms);
    TEST_ASSERT_FALSE(r.fresh);
    TEST_ASSERT_FALSE(diag_client_steps(T0).fresh); /* a stamp equal to init is no write */
}

static void test_step_counters_round_trip_and_stay_fresh_until_max_age(void)
{
    diag_set_client_steps(7u, 0xFFFFFFFFu, T0 + 10u);
    diag_client_steps_t r = diag_client_steps(T0 + 10u + STEPS_MAX_AGE - 1u);
    TEST_ASSERT_EQUAL_UINT32(7u, r.overruns);
    TEST_ASSERT_EQUAL_UINT32(0xFFFFFFFFu, r.gap_max_ms);
    TEST_ASSERT_TRUE(r.fresh);
    r = diag_client_steps(T0 + 10u + STEPS_MAX_AGE);
    TEST_ASSERT_FALSE(r.fresh);
    TEST_ASSERT_EQUAL_UINT32(7u, r.overruns); /* last values kept, only not fresh */
}

static void test_stale_step_counters_stay_not_fresh_across_the_clock_wrap_until_a_new_write(void)
{
    diag_set_client_steps(1u, 12u, T0);
    diag_supervise(T0 + STEPS_MAX_AGE); /* supervise sees them stale */
    /* 2^32 + 10 ms after the write the 32-bit clock reads T0 + 10: raw age 10 ms */
    TEST_ASSERT_FALSE(diag_client_steps(T0 + 10u).fresh);
    diag_set_client_steps(2u, 12u, T0 + 20u);
    TEST_ASSERT_TRUE(diag_client_steps(T0 + 21u).fresh);
    TEST_ASSERT_EQUAL_UINT32(2u, diag_client_steps(T0 + 21u).overruns);
}

static void test_supervise_keeps_regularly_written_step_counters_fresh(void)
{
    for (uint32_t t = T0; t < (T0 + (3u * STEPS_MAX_AGE)); t += 50u) {
        diag_set_client_steps(0u, 10u, t);
        diag_supervise(t + 1u);
        TEST_ASSERT_TRUE(diag_client_steps(t + 1u).fresh);
    }
}

static void test_clear_and_init_of_the_step_counters(void)
{
    diag_set_client_steps(3u, 40u, T0 + 1u);
    diag_dtc_clear_all(); /* 0x14 never touches them */
    TEST_ASSERT_EQUAL_UINT32(3u, diag_client_steps(T0 + 2u).overruns);
    diag_init(T0 + 2u);
    TEST_ASSERT_EQUAL_UINT32(0u, diag_client_steps(T0 + 3u).overruns);
    TEST_ASSERT_FALSE(diag_client_steps(T0 + 3u).fresh);
}

/* ------------------------------------------------------------------ fault summary (D-064) */

static void test_fault_onsets_count_passed_to_failed_transitions_only(void)
{
    diag_init(0u);
    TEST_ASSERT_FALSE(diag_fault_active());
    TEST_ASSERT_EQUAL_UINT32(0u, diag_fault_onsets());
    diag_dtc_report(PLATFORM_UDS_DTC_IDX_VEHICLE_ECU_COMM_LOST, false);
    TEST_ASSERT_EQUAL_UINT32(0u, diag_fault_onsets());
    for (uint32_t i = 0u; i < 5u; i++) { /* level-triggered: every pass */
        diag_dtc_report(PLATFORM_UDS_DTC_IDX_VEHICLE_ECU_COMM_LOST, true);
    }
    TEST_ASSERT_TRUE(diag_fault_active());
    TEST_ASSERT_EQUAL_UINT32(1u, diag_fault_onsets());
    diag_dtc_report(PLATFORM_UDS_DTC_IDX_VEHICLE_BUS_OFF_LATCHED, true);
    TEST_ASSERT_EQUAL_UINT32(2u, diag_fault_onsets());
    diag_dtc_report(PLATFORM_UDS_DTC_IDX_VEHICLE_ECU_COMM_LOST, false);
    TEST_ASSERT_TRUE(diag_fault_active()); /* U0001-88 still failed */
    diag_dtc_report(PLATFORM_UDS_DTC_IDX_VEHICLE_BUS_OFF_LATCHED, false);
    TEST_ASSERT_FALSE(diag_fault_active());
    diag_dtc_report(PLATFORM_UDS_DTC_IDX_VEHICLE_ECU_COMM_LOST, true); /* a new onset */
    TEST_ASSERT_EQUAL_UINT32(3u, diag_fault_onsets());
    diag_dtc_report(PLATFORM_UDS_DTC_COUNT, true); /* out of range: ignored */
    TEST_ASSERT_EQUAL_UINT32(3u, diag_fault_onsets());
}

static void test_a_clear_keeps_the_fault_summary_and_init_resets_it(void)
{
    diag_init(0u);
    diag_dtc_report(PLATFORM_UDS_DTC_IDX_VEHICLE_TESTER_LATCHED, true);
    diag_dtc_clear_all();
    TEST_ASSERT_EQUAL_HEX8(0u, diag_dtc_status(PLATFORM_UDS_DTC_IDX_VEHICLE_TESTER_LATCHED));
    TEST_ASSERT_TRUE(diag_fault_active()); /* a clear hides no active fault */
    diag_dtc_report(PLATFORM_UDS_DTC_IDX_VEHICLE_TESTER_LATCHED, true);
    TEST_ASSERT_EQUAL_UINT32(1u, diag_fault_onsets()); /* and is no new onset */
    diag_init(0u);
    TEST_ASSERT_FALSE(diag_fault_active());
    TEST_ASSERT_EQUAL_UINT32(0u, diag_fault_onsets());
}

static void test_supervise_of_a_missing_status_is_one_onset(void)
{
    diag_init(0u);
    for (uint32_t t = 0u; t < 3u * PLATFORM_UDS_VEHICLE_TESTER_STATUS_MAX_AGE_MS; t++) {
        diag_supervise(t);
    }
    TEST_ASSERT_TRUE(diag_fault_active());
    TEST_ASSERT_EQUAL_UINT32(1u, diag_fault_onsets());
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_init_clears_every_dtc_and_the_tester_status);
    RUN_TEST(test_failed_report_sets_test_failed_and_confirmed_within_the_availability_mask);
    RUN_TEST(test_dtc_reports_are_independent_per_index);
    RUN_TEST(test_passed_report_clears_test_failed_only_and_confirmed_stays);
    RUN_TEST(test_passed_report_on_a_clean_dtc_leaves_it_clean);
    RUN_TEST(test_clear_all_zeroes_every_status_byte);
    RUN_TEST(test_clear_does_not_touch_the_vehicle_tester_status);
    RUN_TEST(test_a_still_failing_report_after_clear_sets_the_bits_again);
    RUN_TEST(test_out_of_range_index_is_ignored_and_reads_zero);
    RUN_TEST(test_vehicle_tester_status_round_trips);
    RUN_TEST(test_null_vehicle_tester_set_is_ignored);
    RUN_TEST(test_vehicle_tester_reads_not_running_before_the_first_status);
    RUN_TEST(test_a_fresh_status_is_returned_until_max_age);
    RUN_TEST(test_a_status_older_than_max_age_reads_not_running);
    RUN_TEST(test_supervise_fails_the_latched_dtc_only_after_max_age_without_any_status);
    RUN_TEST(test_supervise_keeps_a_fresh_status_healthy);
    RUN_TEST(test_supervise_fails_the_dtc_when_the_status_goes_stale_and_it_survives_a_clear);
    RUN_TEST(test_a_stale_status_stays_not_running_across_the_clock_wrap_until_a_new_set);
    RUN_TEST(test_a_new_status_after_staleness_makes_it_fresh_again);
    RUN_TEST(test_step_counters_read_zero_and_not_fresh_before_the_first_write);
    RUN_TEST(test_step_counters_round_trip_and_stay_fresh_until_max_age);
    RUN_TEST(test_stale_step_counters_stay_not_fresh_across_the_clock_wrap_until_a_new_write);
    RUN_TEST(test_supervise_keeps_regularly_written_step_counters_fresh);
    RUN_TEST(test_clear_and_init_of_the_step_counters);
    RUN_TEST(test_fault_onsets_count_passed_to_failed_transitions_only);
    RUN_TEST(test_a_clear_keeps_the_fault_summary_and_init_resets_it);
    RUN_TEST(test_supervise_of_a_missing_status_is_one_onset);
    return UNITY_END();
}
