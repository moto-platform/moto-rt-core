/*
 * L0 tests for the republisher glue (features/vehicle_republish, ISSUES D-5, D-056) on the
 * host CAN port bound to the in-process bus, with a manual clock: what reaches a platform
 * receiver, when, and with which E2E status.
 */
#include "features/vehicle_republish/vehicle_republish.h"
#include "hal/host/can_port_host.h"
#include "hal/host/hal_time_host.h"
#include "services/can_if.h"
#include "services/can_sm.h"
#include "services/timebase.h"
#include "services/vehicle_signals.h"

#include <string.h>
#include <unity.h>

static vbus_t bus_p;
static vbus_t bus_v;
static uint8_t node_dut;
static uint8_t node_peer;
static uint8_t node_vehicle;
static vehicle_republish_t rep;
static moto_e2e_rx_state_t rx;

void setUp(void)
{
    hal_time_host_use_manual(5000u);
    vbus_init(&bus_p);
    vbus_init(&bus_v);
    TEST_ASSERT_TRUE(vbus_attach(&bus_p, &node_dut));
    TEST_ASSERT_TRUE(vbus_attach(&bus_p, &node_peer));
    TEST_ASSERT_TRUE(vbus_attach(&bus_v, &node_vehicle));
    TEST_ASSERT_TRUE(can_port_host_bind_vbus(CAN_PORT_PLATFORM, &bus_p, node_dut));
    TEST_ASSERT_TRUE(can_port_host_bind_vbus(CAN_PORT_VEHICLE, &bus_v, node_vehicle));
    can_if_init();
    vehicle_signals_init();
    TEST_ASSERT_TRUE(vehicle_republish_open(&rep));
    TEST_ASSERT_EQUAL(CAN_PORT_OK, can_if_apply_filters(CAN_PORT_PLATFORM));
    can_sm_step(CAN_PORT_PLATFORM);
    moto_e2e_rx_init(&rx);
}

void tearDown(void)
{
    can_port_host_unbind_all();
    hal_time_host_use_monotonic();
}

/* One comms pass of the platform side (app/comms order), then one ms. */
static void pass(void)
{
    can_sm_step(CAN_PORT_PLATFORM);
    vehicle_republish_step(&rep);
    hal_time_host_advance(1u);
}

typedef struct {
    uint32_t speed;
    uint32_t engine;
    uint32_t e2e_ok;
    uint32_t e2e_initial;
    uint32_t e2e_other;
    can_frame_t last_speed;
    can_frame_t last_engine;
} seen_t;

/* Drains the receiver; 0x021 goes through the receiver's E2E check, as CONN/LINUX do it. */
static void drain(seen_t* s)
{
    can_frame_t f;
    while (vbus_recv(&bus_p, node_peer, &f) == CAN_PORT_OK) {
        if (f.id == PLATFORM_VEHICLE_SPEED_FRAME_ID) {
            s->speed++;
            s->last_speed = f;
            const moto_e2e_status_t st =
                moto_e2e_check(PLATFORM_VEHICLE_SPEED_E2E_DATA_ID, PLATFORM_VEHICLE_SPEED_E2E_MAX_DELTA_COUNTER,
                               PLATFORM_VEHICLE_SPEED_E2E_TIMEOUT_MS, f.data, f.dlc, &rx, timebase_now_ms());
            s->e2e_ok += (st == MOTO_E2E_OK) ? 1u : 0u;
            s->e2e_initial += (st == MOTO_E2E_INITIAL) ? 1u : 0u;
            s->e2e_other += ((st != MOTO_E2E_OK) && (st != MOTO_E2E_INITIAL)) ? 1u : 0u;
        } else if (f.id == PLATFORM_VEHICLE_ENGINE_FRAME_ID) {
            s->engine++;
            s->last_engine = f;
        } else {
            TEST_FAIL_MESSAGE("unexpected frame on the platform bus");
        }
    }
}

static void run(seen_t* s, uint32_t ms)
{
    for (uint32_t i = 0u; i < ms; i++) {
        pass();
        drain(s);
    }
}

/* The bytes the core builds for the current samples, with the receiver-side counter. */
static void expected_speed(uint8_t out[PLATFORM_VEHICLE_SPEED_LENGTH], uint8_t counter)
{
    vehicle_signal_sample_t s;
    TEST_ASSERT_TRUE(vehicle_signals_get(VEHICLE_CL250_IDX_VEHICLE_SPEED, timebase_now_ms() - 1u, &s));
    struct platform_vehicle_speed_t msg;
    vehicle_republish_speed_msg(&s, &msg);
    moto_e2e_tx_state_t c = {counter};
    moto_e2e_tx_state_t n;
    TEST_ASSERT_TRUE(vehicle_republish_speed_frame(&msg, &c, &n, out));
}

static void test_both_messages_go_out_at_their_gen_cycle_times_from_the_first_pass(void)
{
    seen_t s;
    memset(&s, 0, sizeof s);
    pass();
    drain(&s);
    TEST_ASSERT_EQUAL_UINT32(1u, s.speed); /* at once, INVALID: no sample yet */
    TEST_ASSERT_EQUAL_UINT32(1u, s.engine);
    run(&s, 1000u - 1u);
    TEST_ASSERT_EQUAL_UINT32(1000u / PLATFORM_VEHICLE_SPEED_CYCLE_TIME_MS, s.speed);
    TEST_ASSERT_EQUAL_UINT32(1000u / PLATFORM_VEHICLE_ENGINE_CYCLE_TIME_MS, s.engine);
    TEST_ASSERT_EQUAL_UINT32(1u, s.e2e_initial);
    TEST_ASSERT_EQUAL_UINT32(s.speed - 1u, s.e2e_ok);
    TEST_ASSERT_EQUAL_UINT32(0u, s.e2e_other);
    TEST_ASSERT_EQUAL_UINT32(PLATFORM_VEHICLE_SPEED_LENGTH, s.last_speed.dlc);
    TEST_ASSERT_EQUAL_UINT32(PLATFORM_VEHICLE_ENGINE_LENGTH, s.last_engine.dlc);
    TEST_ASSERT_EQUAL_UINT32(s.speed, rep.speed.sent);
    TEST_ASSERT_EQUAL_UINT32(0u, rep.speed.retried + rep.speed.dropped + rep.engine.retried + rep.engine.dropped);
    TEST_ASSERT_EQUAL_UINT32(0u, vbus_frame_count(&bus_v)); /* nothing on the vehicle bus */
}

static void test_the_frames_carry_the_current_samples(void)
{
    seen_t s;
    memset(&s, 0, sizeof s);
    pass(); /* the INVALID first frames */
    drain(&s);
    const uint32_t now = timebase_now_ms();
    float phys;
    const uint8_t speed_byte = 88u;
    TEST_ASSERT_TRUE(vehicle_cl250_decode(&vehicle_cl250_dids[VEHICLE_CL250_IDX_VEHICLE_SPEED], &speed_byte, 1u, &phys));
    TEST_ASSERT_TRUE(vehicle_signals_write(VEHICLE_CL250_IDX_VEHICLE_SPEED, speed_byte, phys, now));
    TEST_ASSERT_TRUE(vehicle_signals_write(VEHICLE_CL250_IDX_ENGINE_SPEED, 17000u, 4250.0f, now));
    vehicle_signals_set_ecu_present(true);
    run(&s, PLATFORM_VEHICLE_SPEED_CYCLE_TIME_MS);
    TEST_ASSERT_EQUAL_UINT32(2u, s.speed);
    uint8_t want[PLATFORM_VEHICLE_SPEED_LENGTH];
    expected_speed(want, 1u);
    TEST_ASSERT_EQUAL_MEMORY(want, s.last_speed.data, sizeof want);

    struct platform_vehicle_engine_t em;
    vehicle_signal_sample_t samples[VEHICLE_CL250_DID_COUNT];
    for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
        TEST_ASSERT_TRUE(vehicle_signals_get(i, timebase_now_ms() - 1u, &samples[i]));
    }
    vehicle_republish_engine_msg(samples, true, &em);
    TEST_ASSERT_EQUAL_UINT8(PLATFORM_VEHICLE_ENGINE_ENGINE_SPEED_VALID_VALID_CHOICE, em.engine_speed_valid);
    uint8_t ewant[PLATFORM_VEHICLE_ENGINE_LENGTH];
    TEST_ASSERT_TRUE(vehicle_republish_engine_frame(&em, ewant));
    TEST_ASSERT_EQUAL_MEMORY(ewant, s.last_engine.data, sizeof ewant);

    /* the speed sample goes STALE after its stale_after_ms: the frame turns INVALID */
    run(&s, vehicle_cl250_dids[VEHICLE_CL250_IDX_VEHICLE_SPEED].stale_after_ms);
    vehicle_signal_sample_t now_s;
    TEST_ASSERT_TRUE(vehicle_signals_get(VEHICLE_CL250_IDX_VEHICLE_SPEED, timebase_now_ms(), &now_s));
    TEST_ASSERT_EQUAL(VEHICLE_SIGNAL_STALE, now_s.state);
    run(&s, PLATFORM_VEHICLE_SPEED_CYCLE_TIME_MS);
    struct platform_vehicle_speed_t inv;
    vehicle_republish_speed_msg(&now_s, &inv);
    uint8_t iwant[PLATFORM_VEHICLE_SPEED_LENGTH];
    moto_e2e_tx_state_t c = {(uint8_t)((s.speed - 1u) & MOTO_E2E_COUNTER_MASK)};
    moto_e2e_tx_state_t n;
    TEST_ASSERT_TRUE(vehicle_republish_speed_frame(&inv, &c, &n, iwant));
    TEST_ASSERT_EQUAL_MEMORY(iwant, s.last_speed.data, sizeof iwant);
}

static void test_a_full_tx_is_retried_next_pass_without_a_counter_gap(void)
{
    seen_t s;
    memset(&s, 0, sizeof s);
    run(&s, PLATFORM_VEHICLE_SPEED_CYCLE_TIME_MS); /* one frame of each */
    vbus_set_tx_blocked(&bus_p, node_dut, true);
    run(&s, 4u); /* due at the first of these passes, refused in all 4 */
    TEST_ASSERT_EQUAL_UINT32(1u, s.speed);
    TEST_ASSERT_EQUAL_UINT32(4u, rep.speed.retried);
    vbus_set_tx_blocked(&bus_p, node_dut, false);
    run(&s, 1u); /* the retry goes out in the very next pass */
    TEST_ASSERT_EQUAL_UINT32(2u, s.speed);
    TEST_ASSERT_EQUAL_UINT32(1u, s.e2e_ok); /* consecutive counter: OK */
    TEST_ASSERT_EQUAL_UINT32(0u, s.e2e_other);
    TEST_ASSERT_EQUAL_UINT32(0u, rep.speed.dropped);
}

static void test_a_bus_off_drops_cycles_and_a_late_receiver_resyncs(void)
{
    seen_t s;
    memset(&s, 0, sizeof s);
    run(&s, PLATFORM_VEHICLE_SPEED_CYCLE_TIME_MS);
    vbus_set_bus_off(&bus_p, node_dut, true);
    run(&s, 5u * PLATFORM_VEHICLE_SPEED_CYCLE_TIME_MS);
    TEST_ASSERT_EQUAL_UINT32(1u, s.speed);
    TEST_ASSERT_EQUAL_UINT32(5u, rep.speed.dropped); /* one per cycle, no retry storm */
    TEST_ASSERT_EQUAL_UINT32(5u, rep.engine.dropped);
    vbus_recover(&bus_p, node_dut); /* back on the bus */
    run(&s, PLATFORM_VEHICLE_SPEED_CYCLE_TIME_MS);
    TEST_ASSERT_EQUAL_UINT32(2u, s.speed);
    TEST_ASSERT_EQUAL_UINT32(2u, s.e2e_initial); /* the gap exceeded the timeout: resync */
    TEST_ASSERT_EQUAL_UINT32(0u, s.e2e_other);
}

static void test_after_a_stall_only_the_newest_speed_frame_goes_out(void)
{
    seen_t s;
    memset(&s, 0, sizeof s);
    run(&s, PLATFORM_VEHICLE_SPEED_CYCLE_TIME_MS);
    vbus_set_tx_stalled(&bus_p, node_dut, true);
    run(&s, 10u * PLATFORM_VEHICLE_SPEED_CYCLE_TIME_MS + 5u);
    TEST_ASSERT_EQUAL_UINT32(1u, s.speed);
    TEST_ASSERT_GREATER_THAN_UINT32(0u, vbus_tx_replaced(&bus_p, node_dut));
    TEST_ASSERT_EQUAL_UINT32(0u, can_sm_stats(CAN_PORT_PLATFORM)->tx_timeouts);
    vbus_set_tx_stalled(&bus_p, node_dut, false);
    run(&s, 1u);
    TEST_ASSERT_EQUAL_UINT32(2u, s.speed); /* one frame, at most one cycle old */
    TEST_ASSERT_EQUAL_UINT32(0u, s.e2e_ok); /* never OK across the gap: INITIAL */
    TEST_ASSERT_EQUAL_UINT32(2u, s.e2e_initial);
    TEST_ASSERT_EQUAL_UINT32(0u, s.e2e_other);
}

static void test_a_replace_that_comes_too_late_leaves_no_counter_gap(void)
{
    seen_t s;
    memset(&s, 0, sizeof s);
    run(&s, PLATFORM_VEHICLE_SPEED_CYCLE_TIME_MS); /* t0: frame 0 */
    vbus_set_tx_stalled(&bus_p, node_dut, true);
    run(&s, PLATFORM_VEHICLE_SPEED_CYCLE_TIME_MS); /* t0+50: frame 1 waits in its buffer */
    vbus_set_tx_cancel_late(&bus_p, node_dut, true);
    vbus_set_tx_stalled(&bus_p, node_dut, false);
    vbus_set_tx_stalled(&bus_p, node_dut, true); /* frame 1 went out at the un-stall */
    drain(&s);
    TEST_ASSERT_EQUAL_UINT32(2u, s.speed);
    run(&s, PLATFORM_VEHICLE_SPEED_CYCLE_TIME_MS); /* frame 2 waits */
    run(&s, PLATFORM_VEHICLE_SPEED_CYCLE_TIME_MS); /* replace too late: frame 2 out, frame 3 next pass */
    vbus_set_tx_stalled(&bus_p, node_dut, false); /* frame 3 out */
    run(&s, 1u); /* t0+200: frame 4 on a free bus */
    TEST_ASSERT_EQUAL_UINT32(5u, s.speed);
    TEST_ASSERT_EQUAL_UINT32(4u, s.e2e_ok); /* frames 1 to 4 consecutive: no counter gap */
    TEST_ASSERT_EQUAL_UINT32(0u, s.e2e_other);
}

/* Writes a 0xF40D sample whose age is age_ms when the next 0x021 goes out, and runs to it. */
static vehicle_signal_sample_t speed_at_next_frame(seen_t* s, uint32_t age_ms)
{
    const uint32_t due = rep.speed_cycle.next_due_ms;
    const uint8_t byte = 120u;
    float phys;
    TEST_ASSERT_TRUE(vehicle_cl250_decode(&vehicle_cl250_dids[VEHICLE_CL250_IDX_VEHICLE_SPEED], &byte, 1u, &phys));
    TEST_ASSERT_TRUE(vehicle_signals_write(VEHICLE_CL250_IDX_VEHICLE_SPEED, byte, phys, due - age_ms));
    const uint32_t before = s->speed;
    while (s->speed == before) {
        pass();
        drain(s);
    }
    vehicle_signal_sample_t got;
    TEST_ASSERT_TRUE(vehicle_signals_get(VEHICLE_CL250_IDX_VEHICLE_SPEED, due, &got));
    TEST_ASSERT_EQUAL_UINT32(age_ms, got.age_ms);
    return got;
}

static void test_the_speed_is_valid_up_to_stale_after_ms_and_invalid_one_ms_later(void)
{
    const uint32_t stale = vehicle_cl250_dids[VEHICLE_CL250_IDX_VEHICLE_SPEED].stale_after_ms;
    seen_t s;
    memset(&s, 0, sizeof s);
    run(&s, 1u);
    vehicle_signal_sample_t at = speed_at_next_frame(&s, stale);
    TEST_ASSERT_EQUAL(VEHICLE_SIGNAL_VALID, at.state);
    struct platform_vehicle_speed_t msg;
    vehicle_republish_speed_msg(&at, &msg);
    TEST_ASSERT_EQUAL_UINT8(PLATFORM_VEHICLE_SPEED_VEHICLE_SPEED_VALID_VALID_CHOICE, msg.vehicle_speed_valid);
    uint8_t want[PLATFORM_VEHICLE_SPEED_LENGTH];
    expected_speed(want, (uint8_t)((s.speed - 1u) & MOTO_E2E_COUNTER_MASK));
    TEST_ASSERT_EQUAL_MEMORY(want, s.last_speed.data, sizeof want);

    at = speed_at_next_frame(&s, stale + 1u);
    TEST_ASSERT_EQUAL(VEHICLE_SIGNAL_STALE, at.state);
    vehicle_republish_speed_msg(&at, &msg);
    TEST_ASSERT_EQUAL_UINT8(PLATFORM_VEHICLE_SPEED_VEHICLE_SPEED_VALID_INVALID_CHOICE, msg.vehicle_speed_valid);
    expected_speed(want, (uint8_t)((s.speed - 1u) & MOTO_E2E_COUNTER_MASK));
    TEST_ASSERT_EQUAL_MEMORY(want, s.last_speed.data, sizeof want);
}

static void test_a_tx_full_longer_than_a_cycle_sends_one_frame_and_keeps_the_grid(void)
{
    const uint32_t p = PLATFORM_VEHICLE_SPEED_CYCLE_TIME_MS;
    seen_t s;
    memset(&s, 0, sizeof s);
    run(&s, 1u);
    const uint32_t anchor = rep.speed_cycle.next_due_ms - p;
    run(&s, p - 1u);
    vbus_set_tx_blocked(&bus_p, node_dut, true);
    run(&s, (2u * p) + 20u); /* due at +p, still refused past +2p and +3p */
    vbus_set_tx_blocked(&bus_p, node_dut, false);
    run(&s, 1u);
    TEST_ASSERT_EQUAL_UINT32(2u, s.speed); /* one frame, no burst */
    /* the counter is consecutive, but 170 ms exceed the 150 ms timeout: INITIAL, never OK */
    TEST_ASSERT_EQUAL_UINT32(2u, s.e2e_initial);
    TEST_ASSERT_EQUAL_UINT32(0u, s.e2e_other);
    TEST_ASSERT_EQUAL_UINT32(0u, (rep.speed_cycle.next_due_ms - anchor) % p); /* grid kept */
    TEST_ASSERT_TRUE((rep.speed_cycle.next_due_ms - timebase_now_ms()) <= p);
}

static void test_a_refused_open_sends_nothing_not_even_through_the_fifo(void)
{
    vehicle_republish_t other;
    TEST_ASSERT_FALSE(vehicle_republish_open(&other)); /* the port is sealed */
    TEST_ASSERT_FALSE(other.opened);
    for (uint32_t i = 0u; i < 3u * PLATFORM_VEHICLE_SPEED_CYCLE_TIME_MS; i++) {
        can_sm_step(CAN_PORT_PLATFORM);
        vehicle_republish_step(&other);
        hal_time_host_advance(1u);
    }
    TEST_ASSERT_EQUAL_UINT32(0u, vbus_frame_count(&bus_p));
    TEST_ASSERT_EQUAL_UINT32(0u, other.speed.sent + other.engine.sent);
}

static void test_the_counters_saturate_on_a_dead_bus(void)
{
    seen_t s;
    memset(&s, 0, sizeof s);
    run(&s, 1u);
    rep.speed.retried = UINT32_MAX - 1u;
    vbus_set_tx_blocked(&bus_p, node_dut, true);
    run(&s, PLATFORM_VEHICLE_SPEED_CYCLE_TIME_MS + 5u);
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, rep.speed.retried); /* never wraps to 0 */
}

static void test_open_registers_the_dedicated_buffer_once_and_before_the_seal(void)
{
    vehicle_republish_t other;
    TEST_ASSERT_FALSE(vehicle_republish_open(&other)); /* sealed in setUp */
    TEST_ASSERT_FALSE(vehicle_republish_open(NULL));
    can_if_init();
    TEST_ASSERT_TRUE(vehicle_republish_open(&other));
    TEST_ASSERT_FALSE(vehicle_republish_open(&other)); /* already listed */
    vehicle_republish_step(NULL); /* harmless */
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_both_messages_go_out_at_their_gen_cycle_times_from_the_first_pass);
    RUN_TEST(test_the_frames_carry_the_current_samples);
    RUN_TEST(test_a_full_tx_is_retried_next_pass_without_a_counter_gap);
    RUN_TEST(test_a_bus_off_drops_cycles_and_a_late_receiver_resyncs);
    RUN_TEST(test_after_a_stall_only_the_newest_speed_frame_goes_out);
    RUN_TEST(test_a_replace_that_comes_too_late_leaves_no_counter_gap);
    RUN_TEST(test_the_speed_is_valid_up_to_stale_after_ms_and_invalid_one_ms_later);
    RUN_TEST(test_a_tx_full_longer_than_a_cycle_sends_one_frame_and_keeps_the_grid);
    RUN_TEST(test_a_refused_open_sends_nothing_not_even_through_the_fifo);
    RUN_TEST(test_the_counters_saturate_on_a_dead_bus);
    RUN_TEST(test_open_registers_the_dedicated_buffer_once_and_before_the_seal);
    return UNITY_END();
}
