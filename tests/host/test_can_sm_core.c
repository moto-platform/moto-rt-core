/*
 * L0 tests for services/can_sm_core (Ç1): ISO 11898-1 error states, bus-off recovery with
 * backoff, the bus-off latch (D-030, D-054) and the N_As TX confirmation timeout. Pure
 * logic: the controller snapshot and the clock are set by each test.
 */
#include "services/can_sm_core.h"
#include "vehicle_cl250.h"

#include <string.h>
#include <unity.h>

/* Test configuration: a small backoff cap so the doubling reaches it quickly. */
#define INITIAL_MS 1000u
#define MAX_MS 8000u
#define TX_TIMEOUT_MS 1000u
#define LATCH_AFTER 5u
#define T0 5000u

static can_sm_core_t sm;
static can_sm_input_t in;
static can_sm_actions_t act;

static can_sm_config_t config(uint32_t latch_after)
{
    const can_sm_config_t cfg = {INITIAL_MS, MAX_MS, TX_TIMEOUT_MS, latch_after};
    return cfg;
}

static void step(uint32_t now)
{
    can_sm_core_step(&sm, now, &in, &act);
}

/* The controller reports a new bus-off (the ISR counter moves, the flag is set). */
static void go_bus_off(uint32_t now)
{
    in.bus_off = true;
    in.bus_off_events++;
    step(now);
}

/* The controller rejoined (what a recovery does) and the next step sees it. */
static void go_bus_on(uint32_t now)
{
    in.bus_off = false;
    step(now);
}

void setUp(void)
{
    memset(&in, 0, sizeof in);
    memset(&act, 0, sizeof act);
    const can_sm_config_t cfg = config(LATCH_AFTER);
    TEST_ASSERT_TRUE(can_sm_core_init(&sm, &cfg));
    step(T0); /* primes the counters */
}

void tearDown(void)
{
}

static void test_init_refuses_an_invalid_config_and_stays_latched(void)
{
    can_sm_core_t s;
    const can_sm_config_t zero_initial = {0u, MAX_MS, TX_TIMEOUT_MS, 0u};
    const can_sm_config_t max_below = {INITIAL_MS, INITIAL_MS - 1u, TX_TIMEOUT_MS, 0u};
    const can_sm_config_t zero_timeout = {INITIAL_MS, MAX_MS, 0u, 0u};
    TEST_ASSERT_FALSE(can_sm_core_init(&s, NULL));
    TEST_ASSERT_EQUAL(CAN_SM_LATCHED, can_sm_core_state(&s));
    TEST_ASSERT_FALSE(can_sm_core_tx_allowed(&s));
    TEST_ASSERT_FALSE(can_sm_core_init(&s, &zero_initial));
    TEST_ASSERT_FALSE(can_sm_core_tx_allowed(&s));
    TEST_ASSERT_FALSE(can_sm_core_init(&s, &max_below));
    TEST_ASSERT_FALSE(can_sm_core_tx_allowed(&s));
    TEST_ASSERT_FALSE(can_sm_core_init(&s, &zero_timeout));
    TEST_ASSERT_FALSE(can_sm_core_tx_allowed(&s));
    TEST_ASSERT_FALSE(can_sm_core_init(NULL, &zero_timeout));

    /* a latched core never asks for a recovery, whatever the controller says */
    can_sm_input_t bus_off = {0u, 0u, false, true, 1u, 0u, 0u};
    can_sm_core_step(&s, T0, &bus_off, &act);
    can_sm_core_step(&s, T0 + 100000u, &bus_off, &act);
    TEST_ASSERT_FALSE(act.recover);
    TEST_ASSERT_EQUAL(CAN_SM_LATCHED, can_sm_core_state(&s));
}

static void test_init_starts_error_active_with_tx_allowed(void)
{
    TEST_ASSERT_EQUAL(CAN_SM_ERROR_ACTIVE, can_sm_core_state(&sm));
    TEST_ASSERT_TRUE(can_sm_core_tx_allowed(&sm));
    TEST_ASSERT_EQUAL_UINT32(INITIAL_MS, sm.backoff_ms);
    TEST_ASSERT_EQUAL_UINT32(0u, can_sm_core_abort_seq(&sm));
    TEST_ASSERT_FALSE(act.abort_tx);
    TEST_ASSERT_FALSE(act.recover);
}

static void test_error_passive_above_127_from_tec_rec_or_the_flag(void)
{
    in.tec = 127u;
    in.rec = 127u;
    step(T0 + 1u);
    TEST_ASSERT_EQUAL(CAN_SM_ERROR_ACTIVE, can_sm_core_state(&sm));
    in.tec = 128u;
    step(T0 + 2u);
    TEST_ASSERT_EQUAL(CAN_SM_ERROR_PASSIVE, can_sm_core_state(&sm));
    TEST_ASSERT_TRUE(can_sm_core_tx_allowed(&sm)); /* an error-passive node still sends */
    in.tec = 0u;
    in.rec = 128u;
    step(T0 + 3u);
    TEST_ASSERT_EQUAL(CAN_SM_ERROR_PASSIVE, can_sm_core_state(&sm));
    in.rec = 0u;
    in.error_passive = true;
    step(T0 + 4u);
    TEST_ASSERT_EQUAL(CAN_SM_ERROR_PASSIVE, can_sm_core_state(&sm));
    in.error_passive = false;
    step(T0 + 5u);
    TEST_ASSERT_EQUAL(CAN_SM_ERROR_ACTIVE, can_sm_core_state(&sm));
    TEST_ASSERT_EQUAL_UINT8(128u, can_sm_core_stats(&sm)->tec_max);
    TEST_ASSERT_EQUAL_UINT8(128u, can_sm_core_stats(&sm)->rec_max);
}

static void test_events_before_the_first_step_are_not_counted(void)
{
    can_sm_core_t s;
    const can_sm_config_t cfg = config(LATCH_AFTER);
    TEST_ASSERT_TRUE(can_sm_core_init(&s, &cfg));
    can_sm_input_t old = {0u, 0u, false, false, 7u, 0u, 42u};
    can_sm_core_step(&s, T0, &old, &act);
    TEST_ASSERT_EQUAL(CAN_SM_ERROR_ACTIVE, can_sm_core_state(&s));
    TEST_ASSERT_EQUAL_UINT32(0u, can_sm_core_stats(&s)->bus_off_events);
    TEST_ASSERT_FALSE(act.abort_tx);
}

static void test_a_port_already_bus_off_at_the_first_step_counts_one_event(void)
{
    can_sm_core_t s;
    const can_sm_config_t cfg = config(LATCH_AFTER);
    TEST_ASSERT_TRUE(can_sm_core_init(&s, &cfg));
    can_sm_input_t off = {0u, 0u, false, true, 3u, 1u, 0u};
    can_sm_core_step(&s, T0, &off, &act);
    TEST_ASSERT_EQUAL(CAN_SM_BUS_OFF, can_sm_core_state(&s));
    TEST_ASSERT_EQUAL_UINT32(1u, can_sm_core_stats(&s)->bus_off_events);
    TEST_ASSERT_TRUE(act.abort_tx);
    off.tx_pending = 0u; /* the abort took */
    /* still bus-off with no new event: not counted again */
    can_sm_core_step(&s, T0 + 1u, &off, &act);
    TEST_ASSERT_EQUAL_UINT32(1u, can_sm_core_stats(&s)->bus_off_events);
    TEST_ASSERT_FALSE(act.abort_tx);
}

static void test_bus_off_aborts_tx_and_recovers_after_the_initial_backoff(void)
{
    in.tx_pending = 1u;
    const uint32_t seq = can_sm_core_abort_seq(&sm);
    go_bus_off(T0 + 10u);
    TEST_ASSERT_TRUE(act.abort_tx);
    TEST_ASSERT_FALSE(act.recover);
    TEST_ASSERT_NOT_EQUAL(seq, can_sm_core_abort_seq(&sm));
    TEST_ASSERT_EQUAL(CAN_SM_BUS_OFF, can_sm_core_state(&sm));
    TEST_ASSERT_FALSE(can_sm_core_tx_allowed(&sm));
    TEST_ASSERT_EQUAL_UINT32(1u, can_sm_core_stats(&sm)->bus_off_events);
    in.tx_pending = 0u; /* the abort took */

    step(T0 + 10u + INITIAL_MS - 1u);
    TEST_ASSERT_FALSE(act.recover);
    TEST_ASSERT_FALSE(act.abort_tx);
    step(T0 + 10u + INITIAL_MS);
    TEST_ASSERT_TRUE(act.recover);
    TEST_ASSERT_EQUAL_UINT32(1u, can_sm_core_stats(&sm)->recover_attempts);
    TEST_ASSERT_EQUAL(CAN_SM_BUS_OFF, can_sm_core_state(&sm)); /* until the controller says so */

    go_bus_on(T0 + 10u + INITIAL_MS + 3u);
    TEST_ASSERT_EQUAL(CAN_SM_ERROR_ACTIVE, can_sm_core_state(&sm));
    TEST_ASSERT_TRUE(can_sm_core_tx_allowed(&sm));
}

static void test_failed_recoveries_double_the_backoff_up_to_the_cap(void)
{
    /* the bus stays broken: the controller never leaves bus-off */
    go_bus_off(T0);
    const uint32_t waits[] = {1000u, 2000u, 4000u, 8000u, 8000u, 8000u};
    uint32_t t = T0;
    for (uint32_t i = 0u; i < (sizeof waits / sizeof waits[0]); i++) {
        step(t + waits[i] - 1u);
        TEST_ASSERT_FALSE_MESSAGE(act.recover, "recovery before its backoff");
        t += waits[i];
        step(t);
        TEST_ASSERT_TRUE_MESSAGE(act.recover, "no recovery at its backoff");
    }
    TEST_ASSERT_EQUAL_UINT32(MAX_MS, sm.backoff_ms);
    TEST_ASSERT_EQUAL_UINT32(6u, can_sm_core_stats(&sm)->recover_attempts);
    TEST_ASSERT_EQUAL_UINT32(1u, can_sm_core_stats(&sm)->bus_off_events);
}

static void test_repeated_bus_offs_after_recovery_keep_doubling(void)
{
    /* each recovery works, but the port falls bus-off again at once */
    uint32_t t = T0;
    const uint32_t waits[] = {1000u, 2000u, 4000u, 8000u};
    for (uint32_t i = 0u; i < (sizeof waits / sizeof waits[0]); i++) {
        go_bus_off(t);
        step(t + waits[i] - 1u);
        TEST_ASSERT_FALSE(act.recover);
        t += waits[i];
        step(t);
        TEST_ASSERT_TRUE(act.recover);
        go_bus_on(t + 1u);
        t += 2u;
    }
    TEST_ASSERT_EQUAL(CAN_SM_ERROR_ACTIVE, can_sm_core_state(&sm));
    TEST_ASSERT_EQUAL_UINT32(4u, can_sm_core_stats(&sm)->bus_off_events);
}

static void test_backoff_returns_to_initial_after_max_ms_on_the_bus(void)
{
    go_bus_off(T0);
    step(T0 + INITIAL_MS);
    TEST_ASSERT_TRUE(act.recover);
    TEST_ASSERT_EQUAL_UINT32(2u * INITIAL_MS, sm.backoff_ms);
    const uint32_t on = T0 + INITIAL_MS + 1u;
    go_bus_on(on);
    step(on + MAX_MS - 1u);
    TEST_ASSERT_EQUAL_UINT32(2u * INITIAL_MS, sm.backoff_ms);
    step(on + MAX_MS);
    TEST_ASSERT_EQUAL_UINT32(INITIAL_MS, sm.backoff_ms);
}

static void test_backoff_is_not_reset_by_a_short_time_on_the_bus(void)
{
    go_bus_off(T0);
    step(T0 + INITIAL_MS);
    const uint32_t on = T0 + INITIAL_MS + 1u;
    go_bus_on(on);
    step(on + MAX_MS - 1u);
    go_bus_off(on + MAX_MS - 1u); /* one ms short of stable */
    step(on + MAX_MS - 1u + (2u * INITIAL_MS) - 1u);
    TEST_ASSERT_FALSE(act.recover);
    step(on + MAX_MS - 1u + (2u * INITIAL_MS));
    TEST_ASSERT_TRUE(act.recover);
}

static void test_the_fifth_bus_off_latches_and_nothing_recovers_it(void)
{
    uint32_t t = T0;
    for (uint32_t i = 1u; i < LATCH_AFTER; i++) {
        go_bus_off(t);
        TEST_ASSERT_EQUAL(CAN_SM_BUS_OFF, can_sm_core_state(&sm));
        t += MAX_MS;
        step(t);
        TEST_ASSERT_TRUE(act.recover);
        go_bus_on(t + 1u);
        t += 2u;
    }
    in.tx_pending = 1u;
    go_bus_off(t);
    TEST_ASSERT_EQUAL(CAN_SM_LATCHED, can_sm_core_state(&sm));
    TEST_ASSERT_TRUE(act.abort_tx);
    TEST_ASSERT_FALSE(can_sm_core_tx_allowed(&sm));
    TEST_ASSERT_EQUAL_UINT32(LATCH_AFTER, can_sm_core_stats(&sm)->bus_off_events);

    /* no recovery ever; the controller coming back does not revive the port */
    for (uint32_t k = 1u; k <= 10u; k++) {
        step(t + (k * MAX_MS));
        TEST_ASSERT_FALSE(act.recover);
    }
    in.bus_off = false;
    in.tx_pending = 0u;
    step(t + (20u * MAX_MS));
    TEST_ASSERT_EQUAL(CAN_SM_LATCHED, can_sm_core_state(&sm));
    TEST_ASSERT_FALSE(act.abort_tx);
    /* a frame left pending on a latched port is aborted (fail-closed) */
    in.tx_pending = 1u;
    step(t + (21u * MAX_MS));
    TEST_ASSERT_TRUE(act.abort_tx);
    TEST_ASSERT_FALSE(act.recover);
}

static void test_the_vehicle_backoff_and_latch_values_from_gen(void)
{
    /* D-030 / D-054 with the gen/ schedule: 4 recoveries (1, 2, 4, 8 s), then the latch */
    can_sm_core_t s;
    const can_sm_config_t cfg = {VEHICLE_CL250_BUS_OFF_BACKOFF_INITIAL_MS,
                                 VEHICLE_CL250_BUS_OFF_BACKOFF_MAX_MS, TX_TIMEOUT_MS, 5u};
    TEST_ASSERT_TRUE(can_sm_core_init(&s, &cfg));
    can_sm_input_t x;
    memset(&x, 0, sizeof x);
    can_sm_core_step(&s, 0u, &x, &act);
    uint32_t t = 0u;
    uint32_t wait = VEHICLE_CL250_BUS_OFF_BACKOFF_INITIAL_MS;
    for (uint32_t i = 1u; i <= 4u; i++) {
        x.bus_off = true;
        x.bus_off_events++;
        can_sm_core_step(&s, t, &x, &act);
        can_sm_core_step(&s, t + wait - 1u, &x, &act);
        TEST_ASSERT_FALSE(act.recover);
        can_sm_core_step(&s, t + wait, &x, &act);
        TEST_ASSERT_TRUE(act.recover);
        x.bus_off = false;
        can_sm_core_step(&s, t + wait + 1u, &x, &act);
        t += wait + 2u;
        wait *= 2u;
    }
    x.bus_off = true;
    x.bus_off_events++;
    can_sm_core_step(&s, t, &x, &act);
    TEST_ASSERT_EQUAL(CAN_SM_LATCHED, can_sm_core_state(&s));
}

static void test_no_latch_when_latch_after_is_zero(void)
{
    can_sm_core_t s;
    const can_sm_config_t cfg = config(0u);
    TEST_ASSERT_TRUE(can_sm_core_init(&s, &cfg));
    can_sm_input_t x;
    memset(&x, 0, sizeof x);
    can_sm_core_step(&s, 0u, &x, &act);
    uint32_t t = 0u;
    for (uint32_t i = 0u; i < 100u; i++) {
        x.bus_off = true;
        x.bus_off_events++;
        can_sm_core_step(&s, t, &x, &act);
        t += MAX_MS;
        can_sm_core_step(&s, t, &x, &act);
        TEST_ASSERT_TRUE(act.recover);
        x.bus_off = false;
        can_sm_core_step(&s, t + 1u, &x, &act);
        t += 2u;
    }
    TEST_ASSERT_EQUAL(CAN_SM_ERROR_ACTIVE, can_sm_core_state(&s));
    TEST_ASSERT_EQUAL_UINT32(100u, can_sm_core_stats(&s)->bus_off_events);
}

static void test_several_events_in_one_step_all_count_and_can_latch(void)
{
    in.bus_off = true;
    in.bus_off_events += 3u;
    step(T0 + 1u);
    TEST_ASSERT_EQUAL_UINT32(3u, can_sm_core_stats(&sm)->bus_off_events);
    TEST_ASSERT_EQUAL(CAN_SM_BUS_OFF, can_sm_core_state(&sm));
    in.bus_off_events += 2u; /* back in bus-off twice before a step saw the bus on */
    step(T0 + 2u);
    TEST_ASSERT_EQUAL(CAN_SM_LATCHED, can_sm_core_state(&sm));
}

static void test_an_event_while_still_bus_off_keeps_the_attempt_timer(void)
{
    go_bus_off(T0);
    step(T0 + INITIAL_MS);
    TEST_ASSERT_TRUE(act.recover);
    /* the controller rejoined and fell bus-off again between two steps */
    go_bus_off(T0 + INITIAL_MS + 5u);
    TEST_ASSERT_TRUE(act.abort_tx);
    TEST_ASSERT_EQUAL_UINT32(2u, can_sm_core_stats(&sm)->bus_off_events);
    step(T0 + INITIAL_MS + (2u * INITIAL_MS) - 1u);
    TEST_ASSERT_FALSE(act.recover);
    step(T0 + INITIAL_MS + (2u * INITIAL_MS));
    TEST_ASSERT_TRUE(act.recover);
}

static void test_the_bus_off_counter_wraps(void)
{
    can_sm_core_t s;
    const can_sm_config_t cfg = config(LATCH_AFTER);
    TEST_ASSERT_TRUE(can_sm_core_init(&s, &cfg));
    can_sm_input_t x;
    memset(&x, 0, sizeof x);
    x.bus_off_events = UINT32_MAX - 1u;
    can_sm_core_step(&s, T0, &x, &act);
    x.bus_off = true;
    x.bus_off_events = 1u; /* UINT32_MAX, 0, 1: three events */
    can_sm_core_step(&s, T0 + 1u, &x, &act);
    TEST_ASSERT_EQUAL_UINT32(3u, can_sm_core_stats(&s)->bus_off_events);
}

static void test_recovery_timing_across_the_ms_counter_wrap(void)
{
    can_sm_core_t s;
    const can_sm_config_t cfg = config(LATCH_AFTER);
    TEST_ASSERT_TRUE(can_sm_core_init(&s, &cfg));
    can_sm_input_t x;
    memset(&x, 0, sizeof x);
    const uint32_t t = UINT32_MAX - 500u;
    can_sm_core_step(&s, t, &x, &act);
    x.bus_off = true;
    x.bus_off_events = 1u;
    can_sm_core_step(&s, t, &x, &act);
    can_sm_core_step(&s, t + INITIAL_MS - 1u, &x, &act); /* wrapped */
    TEST_ASSERT_FALSE(act.recover);
    can_sm_core_step(&s, t + INITIAL_MS, &x, &act);
    TEST_ASSERT_TRUE(act.recover);
}

static void test_backoff_doubling_never_overflows(void)
{
    can_sm_core_t s;
    const can_sm_config_t cfg = {0x80000001u, UINT32_MAX, TX_TIMEOUT_MS, 0u};
    TEST_ASSERT_TRUE(can_sm_core_init(&s, &cfg));
    can_sm_input_t x;
    memset(&x, 0, sizeof x);
    can_sm_core_step(&s, 0u, &x, &act);
    x.bus_off = true;
    x.bus_off_events = 1u;
    can_sm_core_step(&s, 0u, &x, &act);
    can_sm_core_step(&s, 0x80000001u, &x, &act);
    TEST_ASSERT_TRUE(act.recover);
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, s.backoff_ms);
}

static void test_n_as_aborts_a_frame_not_sent_within_the_timeout(void)
{
    in.tx_pending = 1u;
    step(T0 + 1u); /* the timer starts here */
    TEST_ASSERT_FALSE(act.abort_tx);
    step(T0 + 1u + TX_TIMEOUT_MS - 1u);
    TEST_ASSERT_FALSE(act.abort_tx);
    const uint32_t seq = can_sm_core_abort_seq(&sm);
    step(T0 + 1u + TX_TIMEOUT_MS);
    TEST_ASSERT_TRUE(act.abort_tx);
    TEST_ASSERT_NOT_EQUAL(seq, can_sm_core_abort_seq(&sm));
    TEST_ASSERT_EQUAL_UINT32(1u, can_sm_core_stats(&sm)->tx_timeouts);
    TEST_ASSERT_EQUAL(CAN_SM_ERROR_ACTIVE, can_sm_core_state(&sm)); /* N_As is not a bus fault */
    TEST_ASSERT_TRUE(can_sm_core_tx_allowed(&sm));

    /* a new frame pending later gets a fresh timer */
    in.tx_pending = 0u;
    step(T0 + 2000u);
    in.tx_pending = 1u;
    step(T0 + 2100u);
    step(T0 + 2100u + TX_TIMEOUT_MS - 1u);
    TEST_ASSERT_FALSE(act.abort_tx);
    step(T0 + 2100u + TX_TIMEOUT_MS);
    TEST_ASSERT_TRUE(act.abort_tx);
}

static void test_n_as_restarts_on_tx_progress(void)
{
    in.tx_pending = 3u;
    step(T0 + 1u);
    in.tx_done++; /* one frame went out, others still queued */
    in.tx_pending = 2u;
    step(T0 + 900u);
    step(T0 + 1u + TX_TIMEOUT_MS);
    TEST_ASSERT_FALSE(act.abort_tx);
    step(T0 + 900u + TX_TIMEOUT_MS - 1u);
    TEST_ASSERT_FALSE(act.abort_tx);
    step(T0 + 900u + TX_TIMEOUT_MS);
    TEST_ASSERT_TRUE(act.abort_tx);
}

static void test_n_as_stops_when_nothing_is_pending(void)
{
    in.tx_pending = 1u;
    step(T0 + 1u);
    in.tx_pending = 0u;
    in.tx_done++;
    step(T0 + 500u);
    step(T0 + 1u + (5u * TX_TIMEOUT_MS));
    TEST_ASSERT_FALSE(act.abort_tx);
    TEST_ASSERT_EQUAL_UINT32(0u, can_sm_core_stats(&sm)->tx_timeouts);
}

static void test_n_as_across_the_ms_counter_wrap(void)
{
    can_sm_core_t s;
    const can_sm_config_t cfg = config(LATCH_AFTER);
    TEST_ASSERT_TRUE(can_sm_core_init(&s, &cfg));
    can_sm_input_t x;
    memset(&x, 0, sizeof x);
    const uint32_t t = UINT32_MAX - 10u;
    can_sm_core_step(&s, t, &x, &act);
    x.tx_pending = 1u;
    can_sm_core_step(&s, t, &x, &act);
    can_sm_core_step(&s, t + TX_TIMEOUT_MS - 1u, &x, &act);
    TEST_ASSERT_FALSE(act.abort_tx);
    can_sm_core_step(&s, t + TX_TIMEOUT_MS, &x, &act);
    TEST_ASSERT_TRUE(act.abort_tx);
}

static void test_counters_saturate(void)
{
    sm.stats.bus_off_events = UINT32_MAX - 1u;
    sm.stats.recover_attempts = UINT32_MAX;
    sm.stats.tx_timeouts = UINT32_MAX;
    sm.cfg.latch_after = 0u;
    go_bus_off(T0 + 1u);
    in.bus_off_events += 4u;
    step(T0 + 2u);
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, sm.stats.bus_off_events);
    step(T0 + 2u + INITIAL_MS);
    TEST_ASSERT_TRUE(act.recover);
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, sm.stats.recover_attempts);
    go_bus_on(T0 + 3u + INITIAL_MS);
    in.tx_pending = 1u;
    step(T0 + 4u + INITIAL_MS);
    step(T0 + 4u + INITIAL_MS + TX_TIMEOUT_MS);
    TEST_ASSERT_TRUE(act.abort_tx);
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, sm.stats.tx_timeouts);
}

static void test_a_bus_off_seen_by_its_flag_first_is_counted_once(void)
{
    /* safety MAJOR-1: the snapshot sees PSR.BO before the ISR counted the entry */
    in.bus_off = true;
    step(T0 + 1u);
    TEST_ASSERT_TRUE(act.abort_tx);
    TEST_ASSERT_EQUAL_UINT32(1u, can_sm_core_stats(&sm)->bus_off_events);
    in.bus_off_events++; /* the ISR catches up */
    step(T0 + 2u);
    TEST_ASSERT_FALSE(act.abort_tx);
    TEST_ASSERT_EQUAL_UINT32(1u, can_sm_core_stats(&sm)->bus_off_events);
    /* a later, real bus-off still counts */
    in.bus_off_events++;
    step(T0 + 3u);
    TEST_ASSERT_TRUE(act.abort_tx);
    TEST_ASSERT_EQUAL_UINT32(2u, can_sm_core_stats(&sm)->bus_off_events);
}

static void test_a_flag_credit_takes_one_of_several_counted_events(void)
{
    in.bus_off = true;
    step(T0 + 1u);
    in.bus_off_events += 2u; /* the flagged entry plus a new one */
    step(T0 + 2u);
    TEST_ASSERT_TRUE(act.abort_tx);
    TEST_ASSERT_EQUAL_UINT32(2u, can_sm_core_stats(&sm)->bus_off_events);
}

static void test_a_flag_credit_ends_when_the_port_is_back_on_the_bus(void)
{
    /* the ISR never counted the flagged entry: the credit must not eat a later event */
    in.bus_off = true;
    step(T0 + 1u);
    step(T0 + 1u + INITIAL_MS);
    TEST_ASSERT_TRUE(act.recover);
    go_bus_on(T0 + 2u + INITIAL_MS);
    go_bus_off(T0 + 3u + INITIAL_MS);
    TEST_ASSERT_TRUE(act.abort_tx);
    TEST_ASSERT_EQUAL_UINT32(2u, can_sm_core_stats(&sm)->bus_off_events);
}

static void test_a_flag_credit_does_not_eat_a_short_bus_off_after_recovery(void)
{
    /* the ISR never counted the flagged entry; later a bus-off that the driver itself
     * recovered between two steps (e.g. a kernel restart-ms) moves only the counter */
    in.bus_off = true;
    step(T0 + 1u);
    step(T0 + 1u + INITIAL_MS);
    go_bus_on(T0 + 2u + INITIAL_MS);
    in.bus_off_events++;
    step(T0 + 3u + INITIAL_MS);
    TEST_ASSERT_TRUE(act.abort_tx);
    TEST_ASSERT_EQUAL_UINT32(2u, can_sm_core_stats(&sm)->bus_off_events);
}

static void test_recovery_waits_while_a_frame_is_still_pending(void)
{
    /* safety m2: an abort that did not take is repeated; no rejoin with a stale frame */
    in.tx_pending = 1u;
    go_bus_off(T0 + 1u);
    TEST_ASSERT_TRUE(act.abort_tx);
    step(T0 + 1u + INITIAL_MS);
    TEST_ASSERT_TRUE(act.abort_tx);
    TEST_ASSERT_FALSE(act.recover);
    step(T0 + 2u + INITIAL_MS + 500u);
    TEST_ASSERT_TRUE(act.abort_tx);
    TEST_ASSERT_FALSE(act.recover);
    TEST_ASSERT_EQUAL_UINT32(2u, can_sm_core_stats(&sm)->recover_deferred);
    in.tx_pending = 0u;
    step(T0 + 3u + INITIAL_MS + 500u);
    TEST_ASSERT_FALSE(act.abort_tx);
    TEST_ASSERT_TRUE(act.recover);
    TEST_ASSERT_EQUAL_UINT32(2u, can_sm_core_stats(&sm)->recover_deferred);
    TEST_ASSERT_EQUAL_UINT32(1u, can_sm_core_stats(&sm)->recover_attempts);
}

static void test_force_abort_bumps_the_sequence(void)
{
    const uint32_t seq = can_sm_core_abort_seq(&sm);
    can_sm_core_force_abort(&sm);
    TEST_ASSERT_NOT_EQUAL(seq, can_sm_core_abort_seq(&sm));
    can_sm_core_force_abort(NULL);
}

static void test_null_arguments_are_safe(void)
{
    can_sm_core_step(&sm, T0, &in, NULL);
    act.abort_tx = true;
    act.recover = true;
    can_sm_core_step(NULL, T0, &in, &act);
    TEST_ASSERT_FALSE(act.abort_tx);
    TEST_ASSERT_FALSE(act.recover);
    can_sm_core_step(&sm, T0, NULL, &act);
    TEST_ASSERT_EQUAL(CAN_SM_LATCHED, can_sm_core_state(NULL));
    TEST_ASSERT_FALSE(can_sm_core_tx_allowed(NULL));
    TEST_ASSERT_EQUAL_UINT32(0u, can_sm_core_abort_seq(NULL));
    TEST_ASSERT_NULL(can_sm_core_stats(NULL));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_init_refuses_an_invalid_config_and_stays_latched);
    RUN_TEST(test_init_starts_error_active_with_tx_allowed);
    RUN_TEST(test_error_passive_above_127_from_tec_rec_or_the_flag);
    RUN_TEST(test_events_before_the_first_step_are_not_counted);
    RUN_TEST(test_a_port_already_bus_off_at_the_first_step_counts_one_event);
    RUN_TEST(test_bus_off_aborts_tx_and_recovers_after_the_initial_backoff);
    RUN_TEST(test_failed_recoveries_double_the_backoff_up_to_the_cap);
    RUN_TEST(test_repeated_bus_offs_after_recovery_keep_doubling);
    RUN_TEST(test_backoff_returns_to_initial_after_max_ms_on_the_bus);
    RUN_TEST(test_backoff_is_not_reset_by_a_short_time_on_the_bus);
    RUN_TEST(test_the_fifth_bus_off_latches_and_nothing_recovers_it);
    RUN_TEST(test_the_vehicle_backoff_and_latch_values_from_gen);
    RUN_TEST(test_no_latch_when_latch_after_is_zero);
    RUN_TEST(test_several_events_in_one_step_all_count_and_can_latch);
    RUN_TEST(test_an_event_while_still_bus_off_keeps_the_attempt_timer);
    RUN_TEST(test_the_bus_off_counter_wraps);
    RUN_TEST(test_recovery_timing_across_the_ms_counter_wrap);
    RUN_TEST(test_backoff_doubling_never_overflows);
    RUN_TEST(test_n_as_aborts_a_frame_not_sent_within_the_timeout);
    RUN_TEST(test_n_as_restarts_on_tx_progress);
    RUN_TEST(test_n_as_stops_when_nothing_is_pending);
    RUN_TEST(test_n_as_across_the_ms_counter_wrap);
    RUN_TEST(test_counters_saturate);
    RUN_TEST(test_a_bus_off_seen_by_its_flag_first_is_counted_once);
    RUN_TEST(test_a_flag_credit_takes_one_of_several_counted_events);
    RUN_TEST(test_a_flag_credit_ends_when_the_port_is_back_on_the_bus);
    RUN_TEST(test_a_flag_credit_does_not_eat_a_short_bus_off_after_recovery);
    RUN_TEST(test_recovery_waits_while_a_frame_is_still_pending);
    RUN_TEST(test_force_abort_bumps_the_sequence);
    RUN_TEST(test_null_arguments_are_safe);
    return UNITY_END();
}
