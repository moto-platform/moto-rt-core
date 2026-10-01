/*
 * SIL tests for the UDS client (features/uds/uds_client, Ç3) end to end on the host
 * platform layer (D-034): the real client, vehicle ISO-TP link, can_if guard and
 * vehicle_signals service against the simulated CL250 ECU on the in-process bus, with a
 * manual ms clock. The ECU needs the extended session for its reads (D-019).
 *
 * Every test checks every frame the tester put on the bus: 29-bit
 * VEHICLE_CL250_REQUEST_ID, DLC 8, a Single Frame that passes the generated D-020 frame
 * gate, and one of session / tester present / 0x22 for a gen/ DID. No Flow Control is
 * ever sent (Q-020). No requirement IDs yet (Q-006).
 */
#include "app/host/sim_ecu.h"
#include "features/uds/uds_client.h"
#include "hal/host/can_port_host.h"
#include "hal/host/hal_time_host.h"
#include "platform_uds.h"
#include "services/can_if.h"
#include "services/diag.h"
#include "services/timebase.h"
#include "services/vehicle_signals.h"
#include "platform_limits.h"
#include "uds_iso14229.h"
#include "vehicle_cl250.h"

#include <string.h>
#include <unity.h>

#define SNIFF_MAX 16384u

static vbus_t bus;
static uint8_t node_vehicle, node_sniff;
static sim_ecu_t ecu;
static uds_client_t client;

typedef struct {
    can_frame_t f;
    uint32_t t;
} sniffed_t;
static sniffed_t sniffed[SNIFF_MAX];
static uint32_t sniff_count;

void setUp(void)
{
    hal_time_host_use_manual(1000u);
    vbus_init(&bus);
    TEST_ASSERT_TRUE(vbus_attach(&bus, &node_vehicle));
    TEST_ASSERT_TRUE(vbus_attach(&bus, &node_sniff));
    TEST_ASSERT_TRUE(can_port_host_bind_vbus(CAN_PORT_VEHICLE, &bus, node_vehicle));
    TEST_ASSERT_TRUE(sim_ecu_init(&ecu, &bus));
    ecu.require_session = true;
    can_if_init();
    diag_init(timebase_now_ms());
    vehicle_signals_init();
    memset(&client, 0, sizeof client);
    TEST_ASSERT_EQUAL(ISOTP_OK, uds_client_open(&client));
    sniff_count = 0u;
}

void tearDown(void)
{
    can_port_host_unbind_all();
    hal_time_host_use_monotonic();
}

static void sniff(void)
{
    can_frame_t f;
    while (vbus_recv(&bus, node_sniff, &f) == CAN_PORT_OK) {
        TEST_ASSERT_LESS_THAN_UINT32(SNIFF_MAX, sniff_count);
        sniffed[sniff_count].f = f;
        sniffed[sniff_count].t = timebase_now_ms();
        sniff_count++;
    }
}

static bool all_valid(uint32_t now)
{
    for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
        vehicle_signal_sample_t s;
        TEST_ASSERT_TRUE(vehicle_signals_get(i, now, &s));
        if (s.state != VEHICLE_SIGNAL_VALID) {
            return false;
        }
    }
    return true;
}

/* One main-loop pass per ms, like app/host/main.c. With every_ms_valid, every DID must
 * be VALID after each pass. */
static void run(uint32_t ms, bool every_ms_valid)
{
    for (uint32_t i = 0u; i < ms; i++) {
        const uint32_t now = timebase_now_ms();
        (void)can_if_dispatch(CAN_PORT_VEHICLE, 64u);
        sim_ecu_step(&ecu, now);
        (void)can_if_dispatch(CAN_PORT_VEHICLE, 64u);
        uds_client_step(&client);
        sniff();
        if (every_ms_valid) {
            TEST_ASSERT_TRUE_MESSAGE(all_valid(now), "a DID went stale in healthy operation");
        }
        hal_time_host_advance(1u);
    }
}

static bool is_tester(const sniffed_t* s)
{
    return s->f.id == VEHICLE_CL250_REQUEST_ID;
}

/* The tester's frames: gate, format, and only the three request kinds. */
static void assert_tester_frames_ok(void)
{
    const uint8_t pad = VEHICLE_CL250_PADDING_BYTE;
    for (uint32_t i = 0u; i < sniff_count; i++) {
        const can_frame_t* f = &sniffed[i].f;
        if (!is_tester(&sniffed[i])) {
            continue;
        }
        TEST_ASSERT_TRUE(f->extended);
        TEST_ASSERT_EQUAL_UINT8(VEHICLE_CL250_FRAME_DLC, f->dlc);
        TEST_ASSERT_TRUE_MESSAGE(vehicle_cl250_frame_allowed(f->data, f->dlc), "D-020 frame gate");
        TEST_ASSERT_EQUAL_HEX8_MESSAGE(0u, (uint8_t)(f->data[0] & 0xF0u), "not a Single Frame (FC?)");
        const uint8_t n = f->data[0];
        const bool session = (n == 2u) && (f->data[1] == VEHICLE_CL250_SESSION_SID) &&
                             (f->data[2] == VEHICLE_CL250_SESSION_SUBFUNCTION);
        const bool tp = (n == 2u) && (f->data[1] == VEHICLE_CL250_TESTER_PRESENT_SID) &&
                        (f->data[2] == VEHICLE_CL250_TESTER_PRESENT_SUBFUNCTION);
        const bool read = (n == 3u) && (f->data[1] == UDS_SID_READ_DATA_BY_IDENTIFIER) &&
                          (vehicle_cl250_find((uint16_t)(((uint16_t)f->data[2] << 8u) |
                                                         f->data[3])) != NULL);
        TEST_ASSERT_TRUE_MESSAGE(session || tp || read, "unexpected tester request");
        for (uint8_t b = (uint8_t)(n + 1u); b < 8u; b++) {
            TEST_ASSERT_EQUAL_HEX8(pad, f->data[b]);
        }
    }
    TEST_ASSERT_EQUAL_UINT32(0u, can_if_tx_refused_count(CAN_PORT_VEHICLE));
}

static uint32_t tester_count(uint8_t sid, uint32_t from_t)
{
    uint32_t n = 0u;
    for (uint32_t i = 0u; i < sniff_count; i++) {
        if (is_tester(&sniffed[i]) && (sniffed[i].t >= from_t) && (sniffed[i].f.data[1] == sid)) {
            n++;
        }
    }
    return n;
}

/* True if the tester sent a 0x22 for did at t (sample stamps are request times, D-051). */
static bool read_request_at(uint16_t did, uint32_t t)
{
    for (uint32_t i = 0u; i < sniff_count; i++) {
        const uint8_t* d = sniffed[i].f.data;
        if (is_tester(&sniffed[i]) && (sniffed[i].t == t) && (d[0] == 3u) &&
            (d[1] == UDS_SID_READ_DATA_BY_IDENTIFIER) &&
            ((uint16_t)(((uint16_t)d[2] << 8u) | d[3]) == did)) {
            return true;
        }
    }
    return false;
}

/* ------------------------------------------------------------------------- */
/* Healthy operation                                                          */
/* ------------------------------------------------------------------------- */

static void test_sil_session_first_then_every_did_valid_with_the_gen_formula(void)
{
    run(300u, false);
    TEST_ASSERT_GREATER_THAN_UINT32(0u, sniff_count);
    const uint8_t p = VEHICLE_CL250_PADDING_BYTE;
    const uint8_t first[8] = {0x02u, VEHICLE_CL250_SESSION_SID, VEHICLE_CL250_SESSION_SUBFUNCTION,
                              p, p, p, p, p};
    TEST_ASSERT_TRUE(is_tester(&sniffed[0]));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(first, sniffed[0].f.data, 8u);
    TEST_ASSERT_TRUE(uds_client_session_up(&client));
    TEST_ASSERT_TRUE(uds_client_ecu_present(&client));
    TEST_ASSERT_TRUE(vehicle_signals_ecu_present());
    const uint32_t now = timebase_now_ms();
    TEST_ASSERT_TRUE(all_valid(now));
    for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
        const vehicle_cl250_did_t* e = &vehicle_cl250_dids[i];
        vehicle_signal_sample_t s;
        TEST_ASSERT_TRUE(vehicle_signals_get(i, now, &s));
        const float expect =
            ((float)s.raw * (float)e->factor_num) / (float)e->factor_den + (float)e->offset;
        TEST_ASSERT_FLOAT_WITHIN(0.001f, expect, s.physical);
        TEST_ASSERT_TRUE((s.physical >= e->min) && (s.physical <= e->max));
    }
    assert_tester_frames_ok();
}

static void test_sil_every_did_stays_valid_and_tester_present_keeps_its_period(void)
{
    run(300u, false);
    const uint32_t t0 = timebase_now_ms();
    run(10000u, true); /* stale_after_ms is never exceeded in healthy operation */
    const uint32_t tps = tester_count(VEHICLE_CL250_TESTER_PRESENT_SID, t0);
    TEST_ASSERT_UINT32_WITHIN(1u, 10000u / VEHICLE_CL250_TESTER_PRESENT_PERIOD_MS, tps);
    uint32_t last = 0u;
    bool have = false;
    for (uint32_t i = 0u; i < sniff_count; i++) {
        if (is_tester(&sniffed[i]) && (sniffed[i].f.data[1] == VEHICLE_CL250_TESTER_PRESENT_SID)) {
            if (have) {
                TEST_ASSERT_GREATER_OR_EQUAL_UINT32(VEHICLE_CL250_TESTER_PRESENT_PERIOD_MS,
                                                    sniffed[i].t - last);
            }
            last = sniffed[i].t;
            have = true;
        }
    }
    const uds_client_stats_t* st = uds_client_stats(&client);
    TEST_ASSERT_EQUAL_UINT32(0u, st->timeouts);
    TEST_ASSERT_EQUAL_UINT32(0u, st->nrc);
    TEST_ASSERT_EQUAL_UINT32(1u, st->session_starts);
    TEST_ASSERT_FALSE(uds_client_failed(&client));
    TEST_ASSERT_EQUAL_UINT32(1u, tester_count(VEHICLE_CL250_SESSION_SID, 0u));
    assert_tester_frames_ok();
}

/* ------------------------------------------------------------------------- */
/* ECU off, session loss                                                      */
/* ------------------------------------------------------------------------- */

static void test_sil_ecu_off_goes_absent_and_stale_then_recovers(void)
{
    run(1000u, false);
    TEST_ASSERT_TRUE(all_valid(timebase_now_ms()));
    ecu.silent = true; /* ignition off */
    const uint32_t off = timebase_now_ms();
    uint32_t max_stale = 0u;
    for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
        if (vehicle_cl250_dids[i].stale_after_ms > max_stale) {
            max_stale = vehicle_cl250_dids[i].stale_after_ms;
        }
    }
    run(max_stale + 1u, false);
    for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
        vehicle_signal_sample_t s;
        TEST_ASSERT_TRUE(vehicle_signals_get(i, timebase_now_ms(), &s));
        TEST_ASSERT_EQUAL(VEHICLE_SIGNAL_STALE, s.state);
    }
    run(VEHICLE_CL250_ECU_ABSENT_TIMEOUT_MS, false);
    TEST_ASSERT_FALSE(uds_client_ecu_present(&client));
    TEST_ASSERT_FALSE(vehicle_signals_ecu_present());
    TEST_ASSERT_FALSE(uds_client_session_up(&client));
    /* While absent: only session attempts and tester present, no DID reads. */
    const uint32_t absent_from = timebase_now_ms();
    run(5000u, false);
    TEST_ASSERT_EQUAL_UINT32(0u, tester_count(UDS_SID_READ_DATA_BY_IDENTIFIER, absent_from));
    TEST_ASSERT_UINT32_WITHIN(1u, 5000u / VEHICLE_CL250_SESSION_RETRY_INTERVAL_MS,
                              tester_count(VEHICLE_CL250_SESSION_SID, absent_from));

    /* Ignition on: the ECU is back in its default session. */
    ecu.silent = false;
    sim_ecu_drop_session(&ecu);
    run(VEHICLE_CL250_SESSION_RETRY_INTERVAL_MS + 200u, false);
    TEST_ASSERT_TRUE(uds_client_session_up(&client));
    TEST_ASSERT_TRUE(uds_client_ecu_present(&client));
    TEST_ASSERT_TRUE(all_valid(timebase_now_ms()));
    TEST_ASSERT_EQUAL_UINT32(1u, uds_client_stats(&client)->session_losses);
    TEST_ASSERT_GREATER_THAN_UINT32(off, absent_from);
    assert_tester_frames_ok();
}

static void test_sil_unread_sample_stays_stale_across_the_counter_wrap(void)
{
    const uint32_t idx = VEHICLE_CL250_IDX_ENGINE_SPEED;
    run(500u, false);
    vehicle_signal_sample_t s;
    TEST_ASSERT_TRUE(vehicle_signals_get(idx, timebase_now_ms(), &s));
    TEST_ASSERT_EQUAL(VEHICLE_SIGNAL_VALID, s.state);
    const uint32_t ts = s.timestamp_ms;
    ecu.silent = true;
    run(vehicle_cl250_dids[idx].stale_after_ms + 50u, false); /* nobody reads meanwhile */
    /* Jump to 2^32 ms after the sample, plus 10 ms: the raw age reads 10 ms. */
    hal_time_host_advance((ts + 10u) - timebase_now_ms());
    TEST_ASSERT_EQUAL_UINT32(ts + 10u, timebase_now_ms());
    TEST_ASSERT_TRUE(vehicle_signals_get(idx, timebase_now_ms(), &s));
    TEST_ASSERT_EQUAL(VEHICLE_SIGNAL_STALE, s.state); /* the client's step expired it */
}

static void test_sil_session_drop_is_detected_by_nrc_and_reestablished(void)
{
    run(VEHICLE_CL250_SESSION_RETRY_INTERVAL_MS + 500u, false);
    TEST_ASSERT_TRUE(all_valid(timebase_now_ms()));
    sim_ecu_drop_session(&ecu); /* e.g. an ECU reset: reads now get NRC 0x7F */
    run(5u, false);
    const uds_client_stats_t* st = uds_client_stats(&client);
    TEST_ASSERT_EQUAL_UINT32(1u, st->session_losses);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(1u, st->nrc);
    run(2000u, true); /* re-established at once; nothing goes stale */
    TEST_ASSERT_TRUE(uds_client_session_up(&client));
    TEST_ASSERT_EQUAL_UINT32(2u, st->session_starts);
    TEST_ASSERT_EQUAL_UINT32(0u, st->timeouts);
    assert_tester_frames_ok();
}

/* ------------------------------------------------------------------------- */
/* NRC 0x78, other NRCs                                                       */
/* ------------------------------------------------------------------------- */

/*
 * 0x78 bursts that end within the DID's poll period are waited out: 0xF411 (TPS,
 * 200 ms) is answered after 0x78 at 0, 60 and 120 ms, at 180 ms, past the base timeout.
 * Its sample is stamped with the request's send time, not the arrival (D-051). An
 * answer later than the period makes the DID faulty (next test).
 */
static void test_sil_response_pending_bursts_within_the_period_are_waited_out(void)
{
    const uint32_t idx = VEHICLE_CL250_IDX_THROTTLE_POS;
    const uint32_t answer_ms = 180u;
    TEST_ASSERT_GREATER_THAN_UINT32(VEHICLE_CL250_RESPONSE_TIMEOUT_BASE_MS, answer_ms);
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(vehicle_cl250_dids[idx].poll_period_ms, answer_ms + 2u);
    run(300u, false);
    ecu.pending_count = 3u;
    ecu.pending_interval_ms = 60u; /* 0x78 at 0, 60, 120 ms, the answer at 180 ms */
    ecu.pending_did = VEHICLE_CL250_DID_THROTTLE_POS;
    const uint32_t reads_before = uds_client_stats(&client)->reads_ok;
    run(3000u, false);
    const uds_client_stats_t* st = uds_client_stats(&client);
    TEST_ASSERT_EQUAL_UINT32(0u, st->timeouts);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(3u, st->response_pending);
    TEST_ASSERT_GREATER_THAN_UINT32(reads_before + 5u, st->reads_ok);
    vehicle_signal_sample_t s;
    TEST_ASSERT_TRUE(vehicle_signals_get(idx, timebase_now_ms(), &s));
    TEST_ASSERT_TRUE_MESSAGE(read_request_at(VEHICLE_CL250_DID_THROTTLE_POS, s.timestamp_ms),
                             "the sample is not stamped with its request's send time");
    ecu.pending_count = 0u;
    run(1000u, false);
    TEST_ASSERT_TRUE(all_valid(timebase_now_ms()));
    assert_tester_frames_ok();
}

static void test_sil_endless_response_pending_ends_at_the_gen_max(void)
{
    run(300u, false);
    ecu.pending_count = 200u;
    ecu.pending_interval_ms = 50u; /* 0x78 for 10 s */
    const uint32_t t0 = timebase_now_ms();
    run(VEHICLE_CL250_RESPONSE_TIMEOUT_MAX_MS + 100u, false);
    const uds_client_stats_t* st = uds_client_stats(&client);
    TEST_ASSERT_EQUAL_UINT32(1u, st->timeouts);
    /* After the cap the slot is free again: another read went out. */
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(2u, tester_count(UDS_SID_READ_DATA_BY_IDENTIFIER, t0));
    assert_tester_frames_ok();
}

static void test_sil_nrc_on_one_did_leaves_the_others_valid(void)
{
    ecu.nrc_enabled = true;
    ecu.nrc_did = VEHICLE_CL250_DID_THROTTLE_POS;
    ecu.nrc_code = UDS_NRC_REQUEST_OUT_OF_RANGE;
    run(3000u, false);
    const uint32_t now = timebase_now_ms();
    for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
        vehicle_signal_sample_t s;
        TEST_ASSERT_TRUE(vehicle_signals_get(i, now, &s));
        TEST_ASSERT_EQUAL((i == VEHICLE_CL250_IDX_THROTTLE_POS) ? VEHICLE_SIGNAL_NONE
                                                                : VEHICLE_SIGNAL_VALID,
                          s.state);
    }
    const uds_client_stats_t* st = uds_client_stats(&client);
    TEST_ASSERT_EQUAL_UINT32(0u, st->timeouts);
    TEST_ASSERT_EQUAL_UINT32(0u, st->did_skips);
    TEST_ASSERT_UINT32_WITHIN(2u, 3000u / vehicle_cl250_dids[VEHICLE_CL250_IDX_THROTTLE_POS].poll_period_ms,
                              st->nrc);
    TEST_ASSERT_TRUE(uds_client_session_up(&client));
    assert_tester_frames_ok();
}

/*
 * D-051 (ISSUES E-7): 0xF40D answered after 0x78 at 0, 60 and 120 ms, at 180 ms, later
 * than its 100 ms period. Before D-051 it kept its priority and the 0x78 extension and
 * was due again when its answer landed, so RPM (then 50/150 ms) was STALE all the
 * time. Now a late answer makes it faulty: its next reads end at the base timeout and
 * it is skipped after MAX_CONSECUTIVE_TIMEOUTS, so only the fresh attempt after each
 * cooldown holds the slot past RPM's stale_after_ms. Every speed sample is stamped
 * with a speed request's send time, so its age is never underestimated.
 */
static void test_sil_speed_answered_after_its_period_keeps_rpm_fresh_and_its_age_honest(void)
{
    const uint32_t speed = VEHICLE_CL250_IDX_VEHICLE_SPEED;
    const uint32_t rpm = VEHICLE_CL250_IDX_ENGINE_SPEED;
    const uint32_t duration = 3u * VEHICLE_CL250_DID_SKIP_COOLDOWN_MS;
    run(1000u, false);
    TEST_ASSERT_TRUE(all_valid(timebase_now_ms()));
    ecu.pending_count = 3u;
    ecu.pending_interval_ms = 60u;
    ecu.pending_did = VEHICLE_CL250_DID_VEHICLE_SPEED;
    uint32_t rpm_stale_ms = 0u;
    uint32_t speed_stale_ms = 0u;
    uint32_t speed_samples = 0u;
    vehicle_signal_sample_t s;
    TEST_ASSERT_TRUE(vehicle_signals_get(speed, timebase_now_ms(), &s));
    uint32_t last_stamp = s.timestamp_ms;
    for (uint32_t i = 0u; i < duration; i++) {
        run(1u, false);
        const uint32_t now = timebase_now_ms();
        TEST_ASSERT_TRUE(vehicle_signals_get(rpm, now, &s));
        if (s.state != VEHICLE_SIGNAL_VALID) {
            rpm_stale_ms++;
        }
        TEST_ASSERT_TRUE(vehicle_signals_get(speed, now, &s));
        if (s.state != VEHICLE_SIGNAL_VALID) {
            speed_stale_ms++;
        }
        if (s.timestamp_ms != last_stamp) {
            last_stamp = s.timestamp_ms;
            speed_samples++;
            TEST_ASSERT_TRUE_MESSAGE(read_request_at(VEHICLE_CL250_DID_VEHICLE_SPEED, last_stamp),
                                     "a speed sample is not stamped with a speed request");
        }
    }
    const uds_client_stats_t* st = uds_client_stats(&client);
    /* Measured 2026-10-01 with defs v0.3.2 (RPM 100/300 ms): RPM STALE 0 of 15000 ms
     * (245 with v0.3.1's 50/150, 14897 without D-051); speed STALE 14460 ms with one
     * sample per fresh attempt (fail-safe), 3 skips; 0 skips and 82 samples without D-051. */
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(duration / 20u, rpm_stale_ms);
    TEST_ASSERT_GREATER_THAN_UINT32(0u, st->did_skips);
    TEST_ASSERT_GREATER_THAN_UINT32(0u, speed_samples);
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(st->did_skips + 1u, speed_samples); /* fresh attempts only */
    TEST_ASSERT_GREATER_THAN_UINT32(duration / 2u, speed_stale_ms);
    TEST_ASSERT_TRUE(uds_client_session_up(&client));
    assert_tester_frames_ok();
}

/*
 * The ECU stops giving 0xF40D (NRC on that DID only) while everything else keeps
 * running. The last speed sample is VALID up to PLATFORM_LIMIT_VEHICLE_SPEED_MAX_AGE_MS
 * (D-048: = its stale_after_ms) and STALE from 301 ms on; the other DIDs stay VALID.
 */
static void test_sil_speed_sample_goes_stale_at_301_ms_when_the_ecu_stops_giving_it(void)
{
    const uint32_t idx = VEHICLE_CL250_IDX_VEHICLE_SPEED;
    const uint32_t max_age = PLATFORM_LIMIT_VEHICLE_SPEED_MAX_AGE_MS;
    vehicle_signal_sample_t s;
    run(1000u, false);
    TEST_ASSERT_TRUE(all_valid(timebase_now_ms()));
    ecu.nrc_enabled = true;
    ecu.nrc_did = VEHICLE_CL250_DID_VEHICLE_SPEED;
    ecu.nrc_code = UDS_NRC_REQUEST_OUT_OF_RANGE;
    TEST_ASSERT_TRUE(vehicle_signals_get(idx, timebase_now_ms(), &s));
    const uint32_t last = s.timestamp_ms;
    /* D-051: the age counts from the request's send time, not from the answer. */
    TEST_ASSERT_TRUE(read_request_at(VEHICLE_CL250_DID_VEHICLE_SPEED, last));
    while (timebase_now_ms() != (last + max_age)) {
        run(1u, false);
    }
    TEST_ASSERT_TRUE(vehicle_signals_get(idx, timebase_now_ms(), &s));
    TEST_ASSERT_EQUAL_UINT32(last, s.timestamp_ms); /* no new sample came */
    TEST_ASSERT_EQUAL(VEHICLE_SIGNAL_VALID, s.state);
    run(1u, false);
    TEST_ASSERT_TRUE(vehicle_signals_get(idx, timebase_now_ms(), &s));
    TEST_ASSERT_EQUAL_UINT32(max_age + 1u, s.age_ms);
    TEST_ASSERT_EQUAL(VEHICLE_SIGNAL_STALE, s.state);
    for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
        if (i != idx) {
            TEST_ASSERT_TRUE(vehicle_signals_get(i, timebase_now_ms(), &s));
            TEST_ASSERT_EQUAL(VEHICLE_SIGNAL_VALID, s.state);
        }
    }
    TEST_ASSERT_EQUAL_UINT32(0u, uds_client_stats(&client)->timeouts);
    assert_tester_frames_ok();
}

/* ------------------------------------------------------------------------- */
/* Q-020: a segmented answer cannot be received                               */
/* ------------------------------------------------------------------------- */

static void test_sil_segmented_answer_is_service_unavailable_without_flow_control(void)
{
    ecu.segmented_enabled = true;
    ecu.segmented_did = VEHICLE_CL250_DID_COOLANT_TEMP;
    run(9000u, false);
    const uds_client_stats_t* st = uds_client_stats(&client);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(1u, st->unavailable);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(1u, st->did_skips);
    TEST_ASSERT_FALSE(uds_client_failed(&client)); /* the dropped FC does not latch */

    /* Each First Frame: nothing from the tester for N_Cr + N_Bs, then polling resumes.
     * The DID is retried after the skip cooldown and fails the same way. */
    const uint32_t wait = ISOTP_DEFAULT_N_CR_MS + isotp_link_n_bs_ms(&client.link);
    uint32_t ffs = 0u;
    for (uint32_t i = 0u; i < sniff_count; i++) {
        if (is_tester(&sniffed[i]) || ((sniffed[i].f.data[0] & 0xF0u) != 0x10u)) {
            continue;
        }
        ffs++;
        const uint32_t t_ff = sniffed[i].t;
        uint32_t j = i + 1u;
        while ((j < sniff_count) && !is_tester(&sniffed[j])) {
            j++;
        }
        TEST_ASSERT_LESS_THAN_UINT32(sniff_count, j);
        TEST_ASSERT_GREATER_OR_EQUAL_UINT32(wait, sniffed[j].t - t_ff);
    }
    TEST_ASSERT_EQUAL_UINT32(2u, ffs);

    vehicle_signal_sample_t s;
    TEST_ASSERT_TRUE(vehicle_signals_get(VEHICLE_CL250_IDX_COOLANT_TEMP, timebase_now_ms(), &s));
    TEST_ASSERT_EQUAL(VEHICLE_SIGNAL_NONE, s.state);
    TEST_ASSERT_TRUE(vehicle_signals_get(VEHICLE_CL250_IDX_ENGINE_SPEED, timebase_now_ms(), &s));
    TEST_ASSERT_EQUAL(VEHICLE_SIGNAL_VALID, s.state);
    assert_tester_frames_ok(); /* includes: no FC ever left the tester */
}

/* ------------------------------------------------------------------------- */
/* Fail-closed and arguments                                                  */
/* ------------------------------------------------------------------------- */

static void test_sil_guard_refusal_latches_the_client(void)
{
    run(500u, false);
    TEST_ASSERT_FALSE(uds_client_failed(&client));
    /* Something writes a forbidden frame (ECU reset) on the vehicle port: the guard
     * refuses it, and the client stops, since the bus is no longer under its control. */
    can_frame_t f;
    memset(&f, 0, sizeof f);
    f.id = VEHICLE_CL250_REQUEST_ID;
    f.extended = true;
    f.dlc = 8u;
    const uint8_t reset[8] = {0x02u, 0x11u, 0x01u, 0xAAu, 0xAAu, 0xAAu, 0xAAu, 0xAAu};
    memcpy(f.data, reset, 8u);
    TEST_ASSERT_EQUAL(CAN_PORT_ERR_REFUSED, can_if_write(CAN_PORT_VEHICLE, &f));
    run(1u, false);
    TEST_ASSERT_TRUE(uds_client_failed(&client));
    TEST_ASSERT_EQUAL(UDS_CLIENT_FAULT_GUARD, uds_client_fault(&client));
    TEST_ASSERT_FALSE(uds_client_session_up(&client));
    const uint32_t mark = sniff_count;
    run(5000u, false);
    for (uint32_t i = mark; i < sniff_count; i++) {
        TEST_ASSERT_FALSE(is_tester(&sniffed[i]));
    }
    TEST_ASSERT_FALSE(all_valid(timebase_now_ms()));
}

static void test_sil_a_busy_link_does_not_latch_the_client(void)
{
    /* The link is busy with a message the client did not queue: the client waits. */
    const uint8_t tp[2] = {VEHICLE_CL250_TESTER_PRESENT_SID, VEHICLE_CL250_TESTER_PRESENT_SUBFUNCTION};
    TEST_ASSERT_EQUAL(ISOTP_OK, isotp_link_send(&client.link, tp, 2u));
    run(300u, false);
    TEST_ASSERT_FALSE(uds_client_failed(&client));
    TEST_ASSERT_EQUAL(UDS_CLIENT_FAULT_NONE, uds_client_fault(&client));
    TEST_ASSERT_TRUE(all_valid(timebase_now_ms()));
    assert_tester_frames_ok();
}

/* The mailbox stays full (DLC unplugged, ECU browned out: nobody ACKs), then frees. */
static void blocked_tx_then_recovery(uint32_t blocked_ms)
{
    run(1000u, false);
    TEST_ASSERT_TRUE(all_valid(timebase_now_ms()));
    vbus_set_tx_blocked(&bus, node_vehicle, true);
    const uint32_t mark = sniff_count;
    run(blocked_ms, false);
    for (uint32_t i = mark; i < sniff_count; i++) {
        TEST_ASSERT_FALSE(is_tester(&sniffed[i])); /* nothing got out */
    }
    TEST_ASSERT_FALSE(uds_client_failed(&client));
    vbus_set_tx_blocked(&bus, node_vehicle, false);
    const uint32_t released = timebase_now_ms();
    const uint32_t mark2 = sniff_count;
    run(1u, false);
    uint32_t burst = 0u;
    for (uint32_t i = mark2; i < sniff_count; i++) {
        burst += is_tester(&sniffed[i]) ? 1u : 0u;
    }
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(1u, burst); /* at most the one stuck request */
    run(VEHICLE_CL250_SESSION_RETRY_INTERVAL_MS + 1000u, false);
    TEST_ASSERT_TRUE(all_valid(timebase_now_ms()));
    TEST_ASSERT_TRUE(uds_client_session_up(&client));
    TEST_ASSERT_FALSE(uds_client_failed(&client));
    TEST_ASSERT_GREATER_THAN_UINT32(0u, tester_count(UDS_SID_READ_DATA_BY_IDENTIFIER, released));
    assert_tester_frames_ok();
}

static void test_sil_short_tx_blockage_recovers_without_a_latch(void)
{
    blocked_tx_then_recovery(500u);
}

static void test_sil_long_tx_blockage_goes_absent_and_recovers_without_a_latch(void)
{
    blocked_tx_then_recovery(5000u);
    TEST_ASSERT_EQUAL_UINT32(1u, uds_client_stats(&client)->session_losses);
}

static void foreign_frame_latches(uint32_t id, bool extended)
{
    run(500u, false);
    can_frame_t f;
    memset(&f, 0, sizeof f);
    f.id = id;
    f.extended = extended;
    f.dlc = 8u;
    const uint8_t read_rpm[8] = {0x03u, UDS_SID_READ_DATA_BY_IDENTIFIER,
                                 (uint8_t)(VEHICLE_CL250_DID_ENGINE_SPEED >> 8u),
                                 (uint8_t)(VEHICLE_CL250_DID_ENGINE_SPEED & 0xFFu),
                                 0xAAu, 0xAAu, 0xAAu, 0xAAu};
    memcpy(f.data, read_rpm, 8u);
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_send(&bus, node_sniff, &f)); /* a second tester */
    run(1u, false);
    TEST_ASSERT_TRUE(uds_client_failed(&client));
    TEST_ASSERT_EQUAL(UDS_CLIENT_FAULT_FOREIGN_TESTER, uds_client_fault(&client));
    const uint32_t mark = sniff_count;
    run(3000u, false);
    for (uint32_t i = mark; i < sniff_count; i++) {
        TEST_ASSERT_FALSE(is_tester(&sniffed[i]));
    }
}

static void test_sil_a_second_tester_on_the_request_id_latches_the_client(void)
{
    foreign_frame_latches(VEHICLE_CL250_REQUEST_ID, true);
}

static void test_sil_a_second_tester_on_the_fallback_id_latches_the_client(void)
{
    foreign_frame_latches(VEHICLE_CL250_FALLBACK_REQUEST_ID, false);
}

static void dummy_rx(void* ctx, const can_frame_t* frame)
{
    (void)ctx;
    (void)frame;
}

static void test_open_fails_closed_without_room_for_the_foreign_watch(void)
{
    static uds_client_t fresh;
    client.open = false; /* retire the setUp client: its receivers go with can_if_init() */
    can_if_init();
    /* Leave two free receiver slots: the link and one watch ID fit, the second does not. */
    for (uint32_t i = 0u; i + 2u < CAN_IF_MAX_RECEIVERS; i++) {
        TEST_ASSERT_EQUAL(CAN_IF_OK, can_if_register_rx(CAN_PORT_PLATFORM, 0x100u + i, false,
                                                        dummy_rx, NULL));
    }
    memset(&fresh, 0, sizeof fresh);
    TEST_ASSERT_EQUAL(ISOTP_ERR_ARG, uds_client_open(&fresh));
    TEST_ASSERT_TRUE(uds_client_failed(&fresh));
    uds_client_step(&fresh);
    run(100u, false);
    TEST_ASSERT_EQUAL_UINT32(0u, sniff_count);
}

static void test_open_and_getters_handle_bad_arguments(void)
{
    static uds_client_t other;
    TEST_ASSERT_EQUAL(ISOTP_ERR_ARG, uds_client_open(NULL));
    TEST_ASSERT_EQUAL(ISOTP_ERR_BUSY, uds_client_open(&client));
    memset(&other, 0, sizeof other);
    TEST_ASSERT_EQUAL(ISOTP_ERR_ARG, uds_client_open(&other)); /* one vehicle link only */
    TEST_ASSERT_TRUE(uds_client_failed(&other));
    TEST_ASSERT_EQUAL(UDS_CLIENT_FAULT_NONE, uds_client_fault(&other));
    TEST_ASSERT_EQUAL(UDS_CLIENT_FAULT_NONE, uds_client_fault(NULL));
    TEST_ASSERT_FALSE(isotp_link_tx_ready(NULL));
    TEST_ASSERT_TRUE(isotp_link_tx_ready(&client.link));
    TEST_ASSERT_FALSE(uds_client_session_up(&other));
    TEST_ASSERT_FALSE(uds_client_ecu_present(&other));
    uds_client_step(&other); /* not open: no-op */
    uds_client_step(NULL);
    TEST_ASSERT_TRUE(uds_client_failed(NULL));
    TEST_ASSERT_FALSE(uds_client_session_up(NULL));
    TEST_ASSERT_FALSE(uds_client_ecu_present(NULL));
    TEST_ASSERT_NULL(uds_client_stats(NULL));
    TEST_ASSERT_NOT_NULL(uds_client_stats(&client));
    TEST_ASSERT_FALSE(isotp_link_rx_busy(NULL));
    TEST_ASSERT_EQUAL_UINT32(0u, isotp_link_n_bs_ms(NULL));
    TEST_ASSERT_EQUAL_UINT32(ISOTP_DEFAULT_N_BS_MS, isotp_link_n_bs_ms(&client.link));
}

/* ------------------------------------------------------------------------- */
/* Functional watch, diagnostics (D-039, D-040)                               */
/* ------------------------------------------------------------------------- */

static void inject_on_watch_id(uint32_t i)
{
    can_frame_t f;
    memset(&f, 0, sizeof f);
    f.id = vehicle_cl250_functional_watch[i].id;
    f.extended = vehicle_cl250_functional_watch[i].extended;
    f.dlc = 8u;
    const uint8_t read_rpm[8] = {0x03u, UDS_SID_READ_DATA_BY_IDENTIFIER,
                                 (uint8_t)(VEHICLE_CL250_DID_ENGINE_SPEED >> 8u),
                                 (uint8_t)(VEHICLE_CL250_DID_ENGINE_SPEED & 0xFFu),
                                 0xAAu, 0xAAu, 0xAAu, 0xAAu};
    memcpy(f.data, read_rpm, 8u);
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_send(&bus, node_sniff, &f)); /* a generic OBD dongle */
}

static void test_sil_a_frame_on_every_functional_watch_id_latches_the_client(void)
{
    TEST_ASSERT_GREATER_THAN_UINT32(0u, VEHICLE_CL250_FUNCTIONAL_WATCH_COUNT);
    for (uint32_t i = 0u; i < VEHICLE_CL250_FUNCTIONAL_WATCH_COUNT; i++) {
        if (i > 0u) {
            tearDown(); /* a fresh client for the next watch ID */
            setUp();
        }
        run(500u, false);
        TEST_ASSERT_FALSE(uds_client_failed(&client));
        inject_on_watch_id(i);
        run(1u, false);
        TEST_ASSERT_TRUE(uds_client_failed(&client));
        TEST_ASSERT_EQUAL(UDS_CLIENT_FAULT_FOREIGN_TESTER, uds_client_fault(&client));
        const uint32_t mark = sniff_count;
        run(3000u, false);
        for (uint32_t k = mark; k < sniff_count; k++) {
            TEST_ASSERT_FALSE(is_tester(&sniffed[k])); /* fail-closed: nothing more is sent */
        }
    }
}

static void test_sil_a_latch_is_reported_to_diag_and_clearing_never_releases_it(void)
{
    const uint8_t failed = (uint8_t)(UDS_DTC_STATUS_TEST_FAILED & PLATFORM_UDS_DTC_STATUS_AVAILABILITY_MASK);
    run(500u, false);
    TEST_ASSERT_EQUAL_HEX8(0u, diag_dtc_status(PLATFORM_UDS_DTC_IDX_VEHICLE_TESTER_LATCHED));
    TEST_ASSERT_FALSE(diag_vehicle_tester(timebase_now_ms()).latched);
    TEST_ASSERT_EQUAL_UINT8(PLATFORM_UDS_VEHICLE_TESTER_STATUS_FAULT_NONE, diag_vehicle_tester(timebase_now_ms()).fault);

    inject_on_watch_id(0u);
    run(2u, false);
    TEST_ASSERT_EQUAL_HEX8(failed, (uint8_t)(diag_dtc_status(PLATFORM_UDS_DTC_IDX_VEHICLE_TESTER_LATCHED) & failed));
    TEST_ASSERT_TRUE(diag_vehicle_tester(timebase_now_ms()).latched);
    TEST_ASSERT_EQUAL_UINT8(PLATFORM_UDS_VEHICLE_TESTER_STATUS_FAULT_FOREIGN_TESTER,
                            diag_vehicle_tester(timebase_now_ms()).fault);

    /* a UDS 0x14 clear: the client's next pass reports the still-active latch again */
    diag_dtc_clear_all();
    TEST_ASSERT_EQUAL_HEX8(0u, diag_dtc_status(PLATFORM_UDS_DTC_IDX_VEHICLE_TESTER_LATCHED));
    uds_client_step(&client);
    TEST_ASSERT_EQUAL_HEX8(failed, (uint8_t)(diag_dtc_status(PLATFORM_UDS_DTC_IDX_VEHICLE_TESTER_LATCHED) & failed));
    TEST_ASSERT_TRUE(uds_client_failed(&client));
    TEST_ASSERT_EQUAL(UDS_CLIENT_FAULT_FOREIGN_TESTER, uds_client_fault(&client));
    TEST_ASSERT_TRUE(diag_vehicle_tester(timebase_now_ms()).latched);
}

static void test_sil_ecu_comm_lost_dtc_needs_the_absence_timeout_since_open(void)
{
    const uint8_t failed = (uint8_t)(UDS_DTC_STATUS_TEST_FAILED & PLATFORM_UDS_DTC_STATUS_AVAILABILITY_MASK);
    const uint8_t confirmed = (uint8_t)(UDS_DTC_STATUS_CONFIRMED_DTC & PLATFORM_UDS_DTC_STATUS_AVAILABILITY_MASK);
    ecu.silent = true; /* the ECU never answers */
    run(VEHICLE_CL250_ECU_ABSENT_TIMEOUT_MS, false); /* steps at open + 0 .. TIMEOUT - 1 */
    TEST_ASSERT_EQUAL_HEX8(0u, diag_dtc_status(PLATFORM_UDS_DTC_IDX_VEHICLE_ECU_COMM_LOST));
    TEST_ASSERT_FALSE(diag_vehicle_tester(timebase_now_ms()).ecu_present);
    run(1u, false); /* the step at open + TIMEOUT */
    TEST_ASSERT_EQUAL_HEX8(failed, (uint8_t)(diag_dtc_status(PLATFORM_UDS_DTC_IDX_VEHICLE_ECU_COMM_LOST) & failed));
    TEST_ASSERT_EQUAL_HEX8(confirmed, (uint8_t)(diag_dtc_status(PLATFORM_UDS_DTC_IDX_VEHICLE_ECU_COMM_LOST) & confirmed));
    TEST_ASSERT_FALSE(uds_client_failed(&client)); /* absence is not a latch */

    /* the ECU is back: the active bit clears, the confirmed bit stays */
    ecu.silent = false;
    sim_ecu_drop_session(&ecu);
    run(VEHICLE_CL250_SESSION_RETRY_INTERVAL_MS + 200u, false);
    TEST_ASSERT_TRUE(uds_client_ecu_present(&client));
    TEST_ASSERT_TRUE(diag_vehicle_tester(timebase_now_ms()).ecu_present);
    const uint8_t st = diag_dtc_status(PLATFORM_UDS_DTC_IDX_VEHICLE_ECU_COMM_LOST);
    TEST_ASSERT_EQUAL_HEX8(0u, (uint8_t)(st & failed));
    TEST_ASSERT_EQUAL_HEX8(confirmed, (uint8_t)(st & confirmed));
}

static void test_sil_an_answering_ecu_never_sets_the_comm_lost_dtc(void)
{
    run(VEHICLE_CL250_ECU_ABSENT_TIMEOUT_MS + 2000u, false);
    TEST_ASSERT_EQUAL_HEX8(0u, diag_dtc_status(PLATFORM_UDS_DTC_IDX_VEHICLE_ECU_COMM_LOST));
    TEST_ASSERT_TRUE(diag_vehicle_tester(timebase_now_ms()).ecu_present);
    TEST_ASSERT_TRUE(diag_vehicle_tester(timebase_now_ms()).session_up);
    TEST_ASSERT_FALSE(diag_vehicle_tester(timebase_now_ms()).latched);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_sil_session_first_then_every_did_valid_with_the_gen_formula);
    RUN_TEST(test_sil_every_did_stays_valid_and_tester_present_keeps_its_period);
    RUN_TEST(test_sil_ecu_off_goes_absent_and_stale_then_recovers);
    RUN_TEST(test_sil_unread_sample_stays_stale_across_the_counter_wrap);
    RUN_TEST(test_sil_session_drop_is_detected_by_nrc_and_reestablished);
    RUN_TEST(test_sil_response_pending_bursts_within_the_period_are_waited_out);
    RUN_TEST(test_sil_speed_answered_after_its_period_keeps_rpm_fresh_and_its_age_honest);
    RUN_TEST(test_sil_endless_response_pending_ends_at_the_gen_max);
    RUN_TEST(test_sil_nrc_on_one_did_leaves_the_others_valid);
    RUN_TEST(test_sil_speed_sample_goes_stale_at_301_ms_when_the_ecu_stops_giving_it);
    RUN_TEST(test_sil_segmented_answer_is_service_unavailable_without_flow_control);
    RUN_TEST(test_sil_guard_refusal_latches_the_client);
    RUN_TEST(test_sil_a_busy_link_does_not_latch_the_client);
    RUN_TEST(test_sil_short_tx_blockage_recovers_without_a_latch);
    RUN_TEST(test_sil_long_tx_blockage_goes_absent_and_recovers_without_a_latch);
    RUN_TEST(test_sil_a_second_tester_on_the_request_id_latches_the_client);
    RUN_TEST(test_sil_a_second_tester_on_the_fallback_id_latches_the_client);
    RUN_TEST(test_open_fails_closed_without_room_for_the_foreign_watch);
    RUN_TEST(test_open_and_getters_handle_bad_arguments);
    RUN_TEST(test_sil_a_frame_on_every_functional_watch_id_latches_the_client);
    RUN_TEST(test_sil_a_latch_is_reported_to_diag_and_clearing_never_releases_it);
    RUN_TEST(test_sil_ecu_comm_lost_dtc_needs_the_absence_timeout_since_open);
    RUN_TEST(test_sil_an_answering_ecu_never_sets_the_comm_lost_dtc);
    return UNITY_END();
}
