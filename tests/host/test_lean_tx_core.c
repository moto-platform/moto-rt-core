/*
 * L0 tests for the pure 0x020 builder (features/cornering/lean_tx_core, D-065 items 2-3,
 * PR 3b): the output clamp to gen/ PLATFORM_LIMIT_LEAN_ANGLE_CLAMP_MAX_DEG (E-17,
 * safety-reviewer MAJOR-1 on defs v0.9.0), the state switch, QUALITY, the output age
 * budget with a future stamp (architecture-guard MAJOR-2), the snapshot faults
 * (MAJOR-4), and pack + E2E.
 */
#include "features/cornering/lean_core.h"
#include "features/cornering/lean_tx_core.h"
#include "moto_e2e.h"
#include "platform.h"
#include "platform_e2e.h"

#include <math.h>
#include <string.h>
#include <unity.h>

#define NOW (1000u)
#define DT_MS (20u)
#define DEG2RAD (0.017453292f)

void setUp(void) {}
void tearDown(void) {}

static lean_record_t rec(float roll_deg, uint8_t quality, lean_out_state_t state, uint32_t stamp)
{
    lean_record_t r;
    (void)memset(&r, 0, sizeof r);
    r.out.roll_deg = roll_deg;
    r.out.quality = quality;
    r.out.state = state;
    r.out.reason = (state == LEAN_OUT_ESTIMATED) ? LEAN_REASON_NONE : LEAN_REASON_SPEED;
    r.stamp_ms = stamp;
    return r;
}

static struct platform_ekf_lean_t msg_for(const lean_record_t* r, bool comms_fault, uint32_t now)
{
    struct platform_ekf_lean_t m;
    (void)memset(&m, 0xA5, sizeof m); /* every field must be written */
    lean_tx_msg(r, comms_fault, now, &m);
    return m;
}

static void assert_msg(const struct platform_ekf_lean_t* m, uint8_t state, int16_t lean, uint8_t q)
{
    TEST_ASSERT_EQUAL_UINT8(state, m->lean_angle_state);
    TEST_ASSERT_EQUAL_INT16(lean, m->lean_angle);
    TEST_ASSERT_EQUAL_UINT8(q, m->lean_angle_quality);
    TEST_ASSERT_NOT_EQUAL(PLATFORM_EKF_LEAN_LEAN_ANGLE_STATE_RESERVED_CHOICE, m->lean_angle_state);
}

static void assert_invalid_msg(const struct platform_ekf_lean_t* m)
{
    assert_msg(m, PLATFORM_EKF_LEAN_LEAN_ANGLE_STATE_INVALID_CHOICE, 0, 0u);
}

static void test_the_clamp_boundaries_E_17(void)
{
    const struct {
        float roll;
        uint8_t state;
        int16_t lean;
    } cases[] = {
        {54.99f, PLATFORM_EKF_LEAN_LEAN_ANGLE_STATE_ESTIMATED_CHOICE, 5499},
        {55.00f, PLATFORM_EKF_LEAN_LEAN_ANGLE_STATE_ESTIMATED_CHOICE, 5500},
        {55.01f, PLATFORM_EKF_LEAN_LEAN_ANGLE_STATE_CLAMPED_CHOICE, 5500},
        {-55.01f, PLATFORM_EKF_LEAN_LEAN_ANGLE_STATE_CLAMPED_CHOICE, -5500},
        {-55.00f, PLATFORM_EKF_LEAN_LEAN_ANGLE_STATE_ESTIMATED_CHOICE, -5500},
        {-54.99f, PLATFORM_EKF_LEAN_LEAN_ANGLE_STATE_ESTIMATED_CHOICE, -5499},
        {60.0f, PLATFORM_EKF_LEAN_LEAN_ANGLE_STATE_CLAMPED_CHOICE, 5500},
        {89.99f, PLATFORM_EKF_LEAN_LEAN_ANGLE_STATE_CLAMPED_CHOICE, 5500},
        {-89.99f, PLATFORM_EKF_LEAN_LEAN_ANGLE_STATE_CLAMPED_CHOICE, -5500},
        {0.0f, PLATFORM_EKF_LEAN_LEAN_ANGLE_STATE_ESTIMATED_CHOICE, 0},
    };
    for (uint32_t i = 0u; i < (sizeof cases / sizeof cases[0]); i++) {
        const lean_record_t r = rec(cases[i].roll, 87u, LEAN_OUT_ESTIMATED, NOW);
        const struct platform_ekf_lean_t m = msg_for(&r, false, NOW);
        assert_msg(&m, cases[i].state, cases[i].lean, 87u); /* QUALITY kept when clamped */
    }
}

static void test_the_model_bound_and_non_finite_rolls_are_invalid(void)
{
    const float bad[] = {90.0f, -90.0f, 180.0f, -180.0f, NAN, INFINITY, -INFINITY};
    for (uint32_t i = 0u; i < (sizeof bad / sizeof bad[0]); i++) {
        const lean_record_t r = rec(bad[i], 90u, LEAN_OUT_ESTIMATED, NOW);
        const struct platform_ekf_lean_t m = msg_for(&r, false, NOW);
        assert_invalid_msg(&m);
    }
}

static void test_the_state_maps_by_switch_and_never_to_reserved(void)
{
    const lean_record_t inv = rec(10.0f, 90u, LEAN_OUT_INVALID, NOW);
    struct platform_ekf_lean_t m = msg_for(&inv, false, NOW);
    assert_invalid_msg(&m);
    /* values the core never produces: the wire choice values and an unknown one */
    const int odd[] = {2, 3, 7, -1};
    for (uint32_t i = 0u; i < (sizeof odd / sizeof odd[0]); i++) {
        lean_record_t r = rec(10.0f, 90u, LEAN_OUT_ESTIMATED, NOW);
        r.out.state = (lean_out_state_t)odd[i];
        m = msg_for(&r, false, NOW);
        assert_invalid_msg(&m);
    }
    /* a sweep over every roll and quality: never RESERVED */
    for (int d = -1000; d <= 1000; d += 7) {
        for (uint32_t q = 0u; q <= 255u; q += 17u) {
            const lean_record_t r = rec((float)d / 10.0f, (uint8_t)q, LEAN_OUT_ESTIMATED, NOW);
            m = msg_for(&r, false, NOW);
            TEST_ASSERT_NOT_EQUAL(PLATFORM_EKF_LEAN_LEAN_ANGLE_STATE_RESERVED_CHOICE, m.lean_angle_state);
        }
    }
}

static void test_quality_must_be_within_1_and_the_gen_range(void)
{
    const uint8_t bad[] = {0u, 101u, 255u};
    for (uint32_t i = 0u; i < (sizeof bad / sizeof bad[0]); i++) {
        const lean_record_t r = rec(10.0f, bad[i], LEAN_OUT_ESTIMATED, NOW);
        const struct platform_ekf_lean_t m = msg_for(&r, false, NOW);
        assert_invalid_msg(&m);
    }
    const lean_record_t lo = rec(10.0f, 1u, LEAN_OUT_ESTIMATED, NOW);
    struct platform_ekf_lean_t m = msg_for(&lo, false, NOW);
    assert_msg(&m, PLATFORM_EKF_LEAN_LEAN_ANGLE_STATE_ESTIMATED_CHOICE, 1000, 1u);
    const lean_record_t hi = rec(10.0f, 100u, LEAN_OUT_ESTIMATED, NOW);
    m = msg_for(&hi, false, NOW);
    assert_msg(&m, PLATFORM_EKF_LEAN_LEAN_ANGLE_STATE_ESTIMATED_CHOICE, 1000, 100u);
}

static void test_an_output_older_than_the_budget_is_invalid(void)
{
    TEST_ASSERT_EQUAL_UINT32(35u, LEAN_TX_AGE_MAX_MS); /* 20 + 10 + 5 < 60 */
    lean_record_t r = rec(10.0f, 90u, LEAN_OUT_ESTIMATED, NOW - LEAN_TX_AGE_MAX_MS);
    struct platform_ekf_lean_t m = msg_for(&r, false, NOW);
    assert_msg(&m, PLATFORM_EKF_LEAN_LEAN_ANGLE_STATE_ESTIMATED_CHOICE, 1000, 90u);
    r.stamp_ms = NOW - LEAN_TX_AGE_MAX_MS - 1u;
    m = msg_for(&r, false, NOW);
    assert_invalid_msg(&m);
    r.stamp_ms = 0u; /* the boot record */
    m = msg_for(&r, false, NOW);
    assert_invalid_msg(&m);
    /* across the timebase wrap */
    r.stamp_ms = 0xFFFFFFF0u;
    m = msg_for(&r, false, 5u); /* age 21 ms */
    assert_msg(&m, PLATFORM_EKF_LEAN_LEAN_ANGLE_STATE_ESTIMATED_CHOICE, 1000, 90u);
}

static void test_a_slightly_future_stamp_is_age_zero_MAJOR_2(void)
{
    lean_record_t r = rec(-20.0f, 90u, LEAN_OUT_ESTIMATED, NOW + LEAN_TX_FUTURE_MAX_MS);
    struct platform_ekf_lean_t m = msg_for(&r, false, NOW);
    assert_msg(&m, PLATFORM_EKF_LEAN_LEAN_ANGLE_STATE_ESTIMATED_CHOICE, -2000, 90u);
    r.stamp_ms = NOW + LEAN_TX_FUTURE_MAX_MS + 1u;
    m = msg_for(&r, false, NOW);
    assert_invalid_msg(&m);
    r.stamp_ms = NOW + 0x80000000u; /* never a wrapped, huge or negative age */
    m = msg_for(&r, false, NOW);
    assert_invalid_msg(&m);
}

static void test_snapshot_faults_and_null_read_invalid_MAJOR_4(void)
{
    lean_record_t r = rec(10.0f, 90u, LEAN_OUT_ESTIMATED, NOW);
    struct platform_ekf_lean_t m = msg_for(&r, true, NOW);
    assert_invalid_msg(&m);
    r.fault = true;
    m = msg_for(&r, false, NOW);
    assert_invalid_msg(&m);
    m = msg_for(NULL, false, NOW);
    assert_invalid_msg(&m);
    lean_tx_msg(&r, false, NOW, NULL); /* no crash */
}

static void test_the_frame_is_packed_and_e2e_protected_on_a_copy(void)
{
    const lean_record_t r = rec(-55.01f, 77u, LEAN_OUT_ESTIMATED, NOW);
    const struct platform_ekf_lean_t m = msg_for(&r, false, NOW);
    moto_e2e_tx_state_t committed;
    moto_e2e_tx_init(&committed);
    const moto_e2e_tx_state_t before = committed;
    moto_e2e_tx_state_t next;
    uint8_t data[PLATFORM_EKF_LEAN_LENGTH];
    TEST_ASSERT_TRUE(lean_tx_frame(&m, &committed, &next, data));
    TEST_ASSERT_EQUAL_MEMORY(&before, &committed, sizeof committed); /* committed untouched */

    moto_e2e_rx_state_t rx;
    (void)memset(&rx, 0, sizeof rx);
    moto_e2e_rx_init(&rx);
    /* the first frame resyncs the receiver, the next one (counter + 1) is OK */
    TEST_ASSERT_EQUAL(MOTO_E2E_INITIAL, moto_e2e_check(PLATFORM_EKF_LEAN_E2E_DATA_ID,
                                                       PLATFORM_EKF_LEAN_E2E_MAX_DELTA_COUNTER,
                                                       PLATFORM_EKF_LEAN_E2E_TIMEOUT_MS, data,
                                                       sizeof data, &rx, NOW));
    moto_e2e_tx_state_t after = next;
    uint8_t second[PLATFORM_EKF_LEAN_LENGTH];
    TEST_ASSERT_TRUE(lean_tx_frame(&m, &next, &after, second));
    TEST_ASSERT_EQUAL(MOTO_E2E_OK, moto_e2e_check(PLATFORM_EKF_LEAN_E2E_DATA_ID,
                                                  PLATFORM_EKF_LEAN_E2E_MAX_DELTA_COUNTER,
                                                  PLATFORM_EKF_LEAN_E2E_TIMEOUT_MS, second,
                                                  sizeof second, &rx, NOW + 20u));
    /* the bytes are the gen/ packing of the expected signals (rt_core has no unpack) */
    struct platform_ekf_lean_t expect;
    (void)platform_ekf_lean_init(&expect);
    expect.lean_angle_state = PLATFORM_EKF_LEAN_LEAN_ANGLE_STATE_CLAMPED_CHOICE;
    expect.lean_angle = -5500;
    expect.lean_angle_quality = 77u;
    moto_e2e_tx_state_t next2;
    uint8_t ref[PLATFORM_EKF_LEAN_LENGTH];
    TEST_ASSERT_TRUE(lean_tx_frame(&expect, &committed, &next2, ref));
    TEST_ASSERT_EQUAL_MEMORY(ref, data, sizeof data);
    TEST_ASSERT_EQUAL_MEMORY(&next2, &next, sizeof next);
    /* a corrupted byte fails the receiver's check */
    data[3] ^= 0x01u;
    TEST_ASSERT_FALSE(moto_e2e_is_valid(moto_e2e_check(PLATFORM_EKF_LEAN_E2E_DATA_ID,
                                                       PLATFORM_EKF_LEAN_E2E_MAX_DELTA_COUNTER,
                                                       PLATFORM_EKF_LEAN_E2E_TIMEOUT_MS, data,
                                                       sizeof data, &rx, NOW + 40u)));
    TEST_ASSERT_FALSE(lean_tx_frame(NULL, &committed, &next, data));
    TEST_ASSERT_FALSE(lean_tx_frame(&m, NULL, &next, data));
    TEST_ASSERT_FALSE(lean_tx_frame(&m, &committed, NULL, data));
    TEST_ASSERT_FALSE(lean_tx_frame(&m, &committed, &next, NULL));
}

static void test_clamping_a_real_core_output_leaves_the_filter_unchanged(void)
{
    /* a 60 deg steady turn at 15 m/s through the real core: estimated unclamped (PR 2) */
    lean_core_t c;
    lean_core_init(&c);
    const float roll = 60.0f * DEG2RAD;
    const float v = 15.0f;
    lean_out_t out = {0};
    uint32_t t = NOW;
    for (uint32_t i = 0u; i < 200u; i++) {
        const lean_imu_t s = {0.0f, LEAN_GRAVITY_MPS2 * sinf(roll) / v, t};
        const lean_speed_t sp = {v, t, LEAN_SPEED_VALID};
        out = lean_core_step(&c, &s, &sp);
        t += DT_MS;
    }
    TEST_ASSERT_EQUAL(LEAN_OUT_ESTIMATED, out.state);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 60.0f, out.roll_deg);
    const lean_core_t before = c;
    lean_record_t r;
    (void)memset(&r, 0, sizeof r);
    r.out = out;
    r.stamp_ms = t - DT_MS;
    const struct platform_ekf_lean_t m = msg_for(&r, false, t - DT_MS);
    assert_msg(&m, PLATFORM_EKF_LEAN_LEAN_ANGLE_STATE_CLAMPED_CHOICE, 5500, out.quality);
    TEST_ASSERT_EQUAL_MEMORY(&before, &c, sizeof c);
    TEST_ASSERT_TRUE(r.out.roll_deg > 55.0f); /* the record itself is not clamped either */
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_the_clamp_boundaries_E_17);
    RUN_TEST(test_the_model_bound_and_non_finite_rolls_are_invalid);
    RUN_TEST(test_the_state_maps_by_switch_and_never_to_reserved);
    RUN_TEST(test_quality_must_be_within_1_and_the_gen_range);
    RUN_TEST(test_an_output_older_than_the_budget_is_invalid);
    RUN_TEST(test_a_slightly_future_stamp_is_age_zero_MAJOR_2);
    RUN_TEST(test_snapshot_faults_and_null_read_invalid_MAJOR_4);
    RUN_TEST(test_the_frame_is_packed_and_e2e_protected_on_a_copy);
    RUN_TEST(test_clamping_a_real_core_output_leaves_the_filter_unchanged);
    return UNITY_END();
}
