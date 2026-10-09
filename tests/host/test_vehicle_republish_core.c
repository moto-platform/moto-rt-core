/*
 * L0 tests for features/vehicle_republish/vehicle_republish_core (ISSUES D-5, D-056):
 * the sample -> 0x021 / 0x110 mapping and E2E (the cycle schedule: test_com_core). Every expected value
 * comes from gen/ (encode, is_in_range, choices, E2E check), never a hand-written scale.
 */
#include "features/vehicle_republish/vehicle_republish_core.h"

#include <math.h>
#include <string.h>
#include <unity.h>

void setUp(void) {}
void tearDown(void) {}

static vehicle_signal_sample_t sample(vehicle_signal_state_t state, float physical, uint32_t age)
{
    vehicle_signal_sample_t s;
    memset(&s, 0, sizeof s);
    s.state = state;
    s.physical = physical;
    s.age_ms = age;
    return s;
}

/* The raw maximum of VEHICLE_SPEED_AGE (2550 ms), found from gen/ is_in_range(). */
static uint8_t age_raw_max(void)
{
    uint8_t raw = 0u;
    while ((raw < UINT8_MAX) && platform_vehicle_speed_vehicle_speed_age_is_in_range((uint8_t)(raw + 1u))) {
        raw++;
    }
    return raw;
}

/* ------------------------------------------------------------------ 0x021 mapping */

static void test_a_valid_speed_sample_is_sent_valid_with_its_value_and_age(void)
{
    const vehicle_signal_sample_t s = sample(VEHICLE_SIGNAL_VALID, 87.0f, 120u);
    struct platform_vehicle_speed_t msg;
    vehicle_republish_speed_msg(&s, &msg);
    TEST_ASSERT_EQUAL_UINT8(PLATFORM_VEHICLE_SPEED_VEHICLE_SPEED_VALID_VALID_CHOICE, msg.vehicle_speed_valid);
    TEST_ASSERT_EQUAL_UINT16(platform_vehicle_speed_vehicle_speed_encode(87.0f), msg.vehicle_speed);
    TEST_ASSERT_EQUAL_UINT8(platform_vehicle_speed_vehicle_speed_age_encode(120.0f), msg.vehicle_speed_age);
    TEST_ASSERT_NOT_EQUAL(0u, msg.vehicle_speed);
}

static void test_none_and_stale_speed_are_invalid_with_value_0_and_a_saturated_age(void)
{
    const vehicle_signal_state_t states[2] = {VEHICLE_SIGNAL_NONE, VEHICLE_SIGNAL_STALE};
    for (uint32_t i = 0u; i < 2u; i++) {
        const vehicle_signal_sample_t s = sample(states[i], 50.0f, 10u);
        struct platform_vehicle_speed_t msg;
        vehicle_republish_speed_msg(&s, &msg);
        TEST_ASSERT_EQUAL_UINT8(PLATFORM_VEHICLE_SPEED_VEHICLE_SPEED_VALID_INVALID_CHOICE, msg.vehicle_speed_valid);
        TEST_ASSERT_EQUAL_UINT16(0u, msg.vehicle_speed);
        TEST_ASSERT_EQUAL_UINT8(age_raw_max(), msg.vehicle_speed_age); /* never reads as fresh */
    }
    struct platform_vehicle_speed_t msg;
    vehicle_republish_speed_msg(NULL, &msg);
    TEST_ASSERT_EQUAL_UINT8(PLATFORM_VEHICLE_SPEED_VEHICLE_SPEED_VALID_INVALID_CHOICE, msg.vehicle_speed_valid);
}

static void test_speed_at_the_255_kmh_limit_is_valid_and_beyond_it_invalid_not_clamped(void)
{
    struct platform_vehicle_speed_t msg;
    vehicle_signal_sample_t s = sample(VEHICLE_SIGNAL_VALID, vehicle_cl250_dids[VEHICLE_CL250_IDX_VEHICLE_SPEED].max, 50u);
    vehicle_republish_speed_msg(&s, &msg); /* 255 km/h: the DID range = the 0x021 range (D-048) */
    TEST_ASSERT_EQUAL_UINT8(PLATFORM_VEHICLE_SPEED_VEHICLE_SPEED_VALID_VALID_CHOICE, msg.vehicle_speed_valid);
    TEST_ASSERT_TRUE(platform_vehicle_speed_vehicle_speed_is_in_range(msg.vehicle_speed));
    TEST_ASSERT_FALSE(platform_vehicle_speed_vehicle_speed_is_in_range((uint16_t)(msg.vehicle_speed + 1u)));

    const float bad[5] = {vehicle_cl250_dids[VEHICLE_CL250_IDX_VEHICLE_SPEED].max + 1.0f, -1.0f, NAN,
                          INFINITY, -INFINITY};
    for (uint32_t i = 0u; i < 5u; i++) {
        s = sample(VEHICLE_SIGNAL_VALID, bad[i], 50u);
        vehicle_republish_speed_msg(&s, &msg);
        TEST_ASSERT_EQUAL_UINT8(PLATFORM_VEHICLE_SPEED_VEHICLE_SPEED_VALID_INVALID_CHOICE, msg.vehicle_speed_valid);
        TEST_ASSERT_EQUAL_UINT16(0u, msg.vehicle_speed);
        TEST_ASSERT_EQUAL_UINT8(age_raw_max(), msg.vehicle_speed_age);
    }
}

static void test_the_age_saturates_and_never_wraps_to_fresh(void)
{
    const uint32_t ages[5] = {2549u, 2550u, 2556u, 99999u, UINT32_MAX};
    for (uint32_t i = 0u; i < 5u; i++) {
        const vehicle_signal_sample_t s = sample(VEHICLE_SIGNAL_VALID, 10.0f, ages[i]);
        struct platform_vehicle_speed_t msg;
        vehicle_republish_speed_msg(&s, &msg);
        TEST_ASSERT_EQUAL_UINT8(age_raw_max(), msg.vehicle_speed_age);
    }
    const vehicle_signal_sample_t fresh = sample(VEHICLE_SIGNAL_VALID, 10.0f, 0u);
    struct platform_vehicle_speed_t msg;
    vehicle_republish_speed_msg(&fresh, &msg);
    TEST_ASSERT_EQUAL_UINT8(0u, msg.vehicle_speed_age);
}

static uint8_t age_raw(uint32_t age)
{
    const vehicle_signal_sample_t s = sample(VEHICLE_SIGNAL_VALID, 10.0f, age);
    struct platform_vehicle_speed_t msg;
    vehicle_republish_speed_msg(&s, &msg);
    return msg.vehicle_speed_age;
}

static void test_the_age_rounds_to_the_nearest_step(void)
{
    /* D-056 item 2: nearest 10 ms; 294 reads 290, 295 reads 300 (safety review MINOR-3) */
    TEST_ASSERT_EQUAL_UINT8(age_raw(290u), age_raw(294u));
    TEST_ASSERT_EQUAL_UINT8(age_raw(300u), age_raw(295u));
    TEST_ASSERT_EQUAL_UINT8((uint8_t)(age_raw(290u) + 1u), age_raw(300u));
    TEST_ASSERT_EQUAL_UINT8(platform_vehicle_speed_vehicle_speed_age_encode(300.0f), age_raw(300u));
}

static void test_every_speed_did_value_maps_to_its_exact_raw_value(void)
{
    const vehicle_cl250_did_t* d = &vehicle_cl250_dids[VEHICLE_CL250_IDX_VEHICLE_SPEED];
    uint16_t last = 0u;
    for (uint32_t raw = 0u; raw <= UINT8_MAX; raw++) {
        const uint8_t byte = (uint8_t)raw;
        float phys;
        TEST_ASSERT_TRUE(vehicle_cl250_decode(d, &byte, 1u, &phys));
        const vehicle_signal_sample_t s = sample(VEHICLE_SIGNAL_VALID, phys, 0u);
        struct platform_vehicle_speed_t msg;
        vehicle_republish_speed_msg(&s, &msg);
        TEST_ASSERT_EQUAL_UINT8(PLATFORM_VEHICLE_SPEED_VEHICLE_SPEED_VALID_VALID_CHOICE, msg.vehicle_speed_valid);
        if (raw > 0u) { /* evenly spaced: one km/h is the same raw step everywhere */
            TEST_ASSERT_EQUAL_UINT16(platform_vehicle_speed_vehicle_speed_encode(1.0f), (uint16_t)(msg.vehicle_speed - last));
        }
        last = msg.vehicle_speed;
    }
}

/* ------------------------------------------------------------------ 0x110 mapping */

static void all_valid(vehicle_signal_sample_t samples[VEHICLE_CL250_DID_COUNT])
{
    samples[VEHICLE_CL250_IDX_ENGINE_SPEED] = sample(VEHICLE_SIGNAL_VALID, 4250.25f, 30u);
    samples[VEHICLE_CL250_IDX_VEHICLE_SPEED] = sample(VEHICLE_SIGNAL_VALID, 60.0f, 30u);
    samples[VEHICLE_CL250_IDX_THROTTLE_POS] = sample(VEHICLE_SIGNAL_VALID, 15.0f * 100.0f / 255.0f, 30u);
    samples[VEHICLE_CL250_IDX_COOLANT_TEMP] = sample(VEHICLE_SIGNAL_VALID, 85.0f, 30u);
    samples[VEHICLE_CL250_IDX_BATTERY_VOLTAGE] = sample(VEHICLE_SIGNAL_VALID, 0.010f, 30u);
}

static void test_valid_engine_samples_are_sent_with_their_valid_bits_and_ecu_presence(void)
{
    vehicle_signal_sample_t s[VEHICLE_CL250_DID_COUNT];
    all_valid(s);
    struct platform_vehicle_engine_t msg;
    vehicle_republish_engine_msg(s, true, &msg);
    TEST_ASSERT_EQUAL_UINT16(platform_vehicle_engine_engine_speed_encode(4250.25f), msg.engine_speed);
    TEST_ASSERT_EQUAL_UINT16(platform_vehicle_engine_battery_voltage_encode(0.010f), msg.battery_voltage);
    TEST_ASSERT_EQUAL_UINT8(platform_vehicle_engine_coolant_temp_encode(85.0f), msg.coolant_temp);
    TEST_ASSERT_EQUAL_UINT8(platform_vehicle_engine_throttle_pos_encode(s[VEHICLE_CL250_IDX_THROTTLE_POS].physical), msg.throttle_pos);
    TEST_ASSERT_EQUAL_UINT8(PLATFORM_VEHICLE_ENGINE_ENGINE_SPEED_VALID_VALID_CHOICE, msg.engine_speed_valid);
    TEST_ASSERT_EQUAL_UINT8(PLATFORM_VEHICLE_ENGINE_BATTERY_VOLTAGE_VALID_VALID_CHOICE, msg.battery_voltage_valid);
    TEST_ASSERT_EQUAL_UINT8(PLATFORM_VEHICLE_ENGINE_COOLANT_TEMP_VALID_VALID_CHOICE, msg.coolant_temp_valid);
    TEST_ASSERT_EQUAL_UINT8(PLATFORM_VEHICLE_ENGINE_THROTTLE_POS_VALID_VALID_CHOICE, msg.throttle_pos_valid);
    TEST_ASSERT_EQUAL_UINT8(PLATFORM_VEHICLE_ENGINE_ECU_PRESENT_PRESENT_CHOICE, msg.ecu_present);
    vehicle_republish_engine_msg(s, false, &msg);
    TEST_ASSERT_EQUAL_UINT8(PLATFORM_VEHICLE_ENGINE_ECU_PRESENT_ABSENT_CHOICE, msg.ecu_present);
}

static void test_each_engine_signal_goes_invalid_on_its_own(void)
{
    const vehicle_cl250_did_index_t idx[4] = {VEHICLE_CL250_IDX_ENGINE_SPEED, VEHICLE_CL250_IDX_BATTERY_VOLTAGE,
                                             VEHICLE_CL250_IDX_COOLANT_TEMP, VEHICLE_CL250_IDX_THROTTLE_POS};
    const vehicle_signal_state_t bad_state[3] = {VEHICLE_SIGNAL_NONE, VEHICLE_SIGNAL_STALE, VEHICLE_SIGNAL_VALID};
    for (uint32_t k = 0u; k < 4u; k++) {
        for (uint32_t b = 0u; b < 3u; b++) {
            vehicle_signal_sample_t s[VEHICLE_CL250_DID_COUNT];
            all_valid(s);
            /* VALID but outside every signal's physical range (and NaN for the last one) */
            s[idx[k]] = sample(bad_state[b], (b == 2u) ? -1000.0f : 1.0f, 30u);
            if ((b == 2u) && (k == 3u)) {
                s[idx[k]].physical = NAN;
            }
            struct platform_vehicle_engine_t msg;
            vehicle_republish_engine_msg(s, true, &msg);
            const uint8_t valid[4] = {msg.engine_speed_valid, msg.battery_voltage_valid, msg.coolant_temp_valid,
                                      msg.throttle_pos_valid};
            const uint32_t value[4] = {msg.engine_speed, msg.battery_voltage, msg.coolant_temp, msg.throttle_pos};
            for (uint32_t j = 0u; j < 4u; j++) {
                TEST_ASSERT_EQUAL_UINT8((j == k) ? 0u : 1u, valid[j]); /* INVALID_CHOICE 0, VALID_CHOICE 1 */
                if (j == k) {
                    TEST_ASSERT_EQUAL_UINT32(0u, value[j]);
                }
            }
        }
    }
    TEST_ASSERT_EQUAL_UINT8(0u, PLATFORM_VEHICLE_ENGINE_ENGINE_SPEED_VALID_INVALID_CHOICE);
    TEST_ASSERT_EQUAL_UINT8(1u, PLATFORM_VEHICLE_ENGINE_ENGINE_SPEED_VALID_VALID_CHOICE);
}

/* The DBC defines 0x110 with the J1979 source scaling: every DID raw value must arrive
 * unchanged (D-056: the gen/ encode rounds; it truncated 38863 battery values before). */
static void check_raw_passthrough(vehicle_cl250_did_index_t idx, uint32_t max_raw)
{
    const vehicle_cl250_did_t* d = &vehicle_cl250_dids[idx];
    for (uint32_t raw = 0u; raw <= max_raw; raw++) {
        const uint8_t bytes[2] = {(uint8_t)(raw >> 8), (uint8_t)raw};
        float phys;
        const bool ok = (d->length == 2u) ? vehicle_cl250_decode(d, bytes, 2u, &phys)
                                          : vehicle_cl250_decode(d, &bytes[1], 1u, &phys);
        if (!ok) {
            continue; /* outside the DID's [min, max]: never stored */
        }
        vehicle_signal_sample_t s[VEHICLE_CL250_DID_COUNT];
        all_valid(s);
        s[idx] = sample(VEHICLE_SIGNAL_VALID, phys, 0u);
        struct platform_vehicle_engine_t msg;
        vehicle_republish_engine_msg(s, true, &msg);
        uint32_t got = 0u;
        if (idx == VEHICLE_CL250_IDX_ENGINE_SPEED) {
            got = msg.engine_speed;
        } else if (idx == VEHICLE_CL250_IDX_BATTERY_VOLTAGE) {
            got = msg.battery_voltage;
        } else if (idx == VEHICLE_CL250_IDX_COOLANT_TEMP) {
            got = msg.coolant_temp;
        } else {
            got = msg.throttle_pos;
        }
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(raw, got, "DID raw value changed on 0x110");
    }
}

static void test_every_engine_did_raw_value_arrives_unchanged_on_0x110(void)
{
    check_raw_passthrough(VEHICLE_CL250_IDX_ENGINE_SPEED, UINT16_MAX);
    check_raw_passthrough(VEHICLE_CL250_IDX_BATTERY_VOLTAGE, UINT16_MAX);
    check_raw_passthrough(VEHICLE_CL250_IDX_COOLANT_TEMP, UINT8_MAX);
    check_raw_passthrough(VEHICLE_CL250_IDX_THROTTLE_POS, UINT8_MAX);
}

/* ------------------------------------------------------------------ frames, E2E */

static void test_speed_frames_pass_the_receiver_e2e_check_and_the_counter_moves_only_when_committed(void)
{
    const vehicle_signal_sample_t s = sample(VEHICLE_SIGNAL_VALID, 42.0f, 80u);
    struct platform_vehicle_speed_t msg;
    vehicle_republish_speed_msg(&s, &msg);
    moto_e2e_tx_state_t committed;
    moto_e2e_tx_init(&committed);
    moto_e2e_rx_state_t rx;
    moto_e2e_rx_init(&rx);
    uint32_t now = 1000u;
    for (uint32_t i = 0u; i < 40u; i++) { /* more than two counter wraps */
        uint8_t data[PLATFORM_VEHICLE_SPEED_LENGTH];
        moto_e2e_tx_state_t next;
        TEST_ASSERT_TRUE(vehicle_republish_speed_frame(&msg, &committed, &next, data));
        TEST_ASSERT_EQUAL_UINT8((uint8_t)((committed.counter + 1u) & MOTO_E2E_COUNTER_MASK), next.counter);
        const moto_e2e_status_t st =
            moto_e2e_check(PLATFORM_VEHICLE_SPEED_E2E_DATA_ID, PLATFORM_VEHICLE_SPEED_E2E_MAX_DELTA_COUNTER,
                           PLATFORM_VEHICLE_SPEED_E2E_TIMEOUT_MS, data, sizeof data, &rx, now);
        TEST_ASSERT_EQUAL((i == 0u) ? MOTO_E2E_INITIAL : MOTO_E2E_OK, st);
        committed = next; /* the port accepted it */
        now += PLATFORM_VEHICLE_SPEED_CYCLE_TIME_MS;
    }
    /* a refused write does not commit: the retry carries the same counter, no gap */
    uint8_t a[PLATFORM_VEHICLE_SPEED_LENGTH];
    uint8_t b[PLATFORM_VEHICLE_SPEED_LENGTH];
    moto_e2e_tx_state_t na;
    moto_e2e_tx_state_t nb;
    TEST_ASSERT_TRUE(vehicle_republish_speed_frame(&msg, &committed, &na, a));
    TEST_ASSERT_TRUE(vehicle_republish_speed_frame(&msg, &committed, &nb, b));
    TEST_ASSERT_EQUAL_MEMORY(a, b, sizeof a);
    /* a corrupted byte or the wrong DataID is caught */
    b[2] ^= 0x01u;
    TEST_ASSERT_EQUAL(MOTO_E2E_WRONG_CRC,
                      moto_e2e_check(PLATFORM_VEHICLE_SPEED_E2E_DATA_ID, PLATFORM_VEHICLE_SPEED_E2E_MAX_DELTA_COUNTER,
                                     PLATFORM_VEHICLE_SPEED_E2E_TIMEOUT_MS, b, sizeof b, &rx, now));
    TEST_ASSERT_EQUAL(MOTO_E2E_WRONG_CRC,
                      moto_e2e_check(PLATFORM_EKF_LEAN_E2E_DATA_ID, PLATFORM_VEHICLE_SPEED_E2E_MAX_DELTA_COUNTER,
                                     PLATFORM_VEHICLE_SPEED_E2E_TIMEOUT_MS, a, sizeof a, &rx, now));
}

static void test_frame_builders_refuse_null_arguments(void)
{
    struct platform_vehicle_speed_t sm;
    struct platform_vehicle_engine_t em;
    memset(&sm, 0, sizeof sm);
    memset(&em, 0, sizeof em);
    moto_e2e_tx_state_t c = {0u};
    moto_e2e_tx_state_t n = {0u};
    uint8_t data[8];
    TEST_ASSERT_FALSE(vehicle_republish_speed_frame(NULL, &c, &n, data));
    TEST_ASSERT_FALSE(vehicle_republish_speed_frame(&sm, NULL, &n, data));
    TEST_ASSERT_FALSE(vehicle_republish_speed_frame(&sm, &c, NULL, data));
    TEST_ASSERT_FALSE(vehicle_republish_speed_frame(&sm, &c, &n, NULL));
    TEST_ASSERT_FALSE(vehicle_republish_engine_frame(NULL, data));
    TEST_ASSERT_FALSE(vehicle_republish_engine_frame(&em, NULL));
    TEST_ASSERT_TRUE(vehicle_republish_engine_frame(&em, data));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_a_valid_speed_sample_is_sent_valid_with_its_value_and_age);
    RUN_TEST(test_none_and_stale_speed_are_invalid_with_value_0_and_a_saturated_age);
    RUN_TEST(test_speed_at_the_255_kmh_limit_is_valid_and_beyond_it_invalid_not_clamped);
    RUN_TEST(test_the_age_saturates_and_never_wraps_to_fresh);
    RUN_TEST(test_the_age_rounds_to_the_nearest_step);
    RUN_TEST(test_every_speed_did_value_maps_to_its_exact_raw_value);
    RUN_TEST(test_valid_engine_samples_are_sent_with_their_valid_bits_and_ecu_presence);
    RUN_TEST(test_each_engine_signal_goes_invalid_on_its_own);
    RUN_TEST(test_every_engine_did_raw_value_arrives_unchanged_on_0x110);
    RUN_TEST(test_speed_frames_pass_the_receiver_e2e_check_and_the_counter_moves_only_when_committed);
    RUN_TEST(test_frame_builders_refuse_null_arguments);
    return UNITY_END();
}
