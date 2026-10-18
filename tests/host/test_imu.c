/*
 * L0 tests for the IMU service glue (services/imu, D-065 item 2, PR 3a) on the host IMU
 * fake (hal/host/imu_host): the bounded drain, the backlog and fault flushes, NO_NEW,
 * and a two-thread run (feed thread = the IMU ISR stand-in, consumer = the EKF task),
 * also under ThreadSanitizer (host-tsan).
 */
#include "hal/host/imu_host.h"
#include "services/imu.h"

#include <pthread.h>
#include <sched.h>
#include <stdint.h>
#include <unity.h>

#define PERIOD_MS (20u)
#define N_PERIOD (16u) /* 20 ms at the fake's default 800 Hz */

static imu_t imu;
static uint32_t now;

void setUp(void)
{
    imu_host_reset();
    imu_init(&imu);
    now = 500u;
}

void tearDown(void) {}

/* n samples around [x, y, z] with -1/+1 LSB of noise (never frozen; even n keeps the
 * mean). */
static void push(uint32_t n, int16_t x, int16_t y, int16_t z)
{
    for (uint32_t i = 0u; i < n; i++) {
        const int16_t d = ((i % 2u) == 0u) ? -1 : 1;
        const hal_imu_sample_t s = {{(int16_t)(x + d), (int16_t)(y + d), (int16_t)(z + d)}};
        TEST_ASSERT_TRUE(imu_host_push(&s));
    }
}

static imu_sample_t read_period(uint32_t n, int16_t z, bool standstill)
{
    push(n, 0, 0, z);
    now += PERIOD_MS;
    return imu_read(&imu, now, standstill);
}

/* START, then 2 s of standstill with z offset zr: learnt. */
static void learn(int16_t zr)
{
    TEST_ASSERT_EQUAL(IMU_REASON_START, read_period(N_PERIOD, zr, true).reason);
    for (uint32_t i = 0u; i < 100u; i++) {
        (void)read_period(N_PERIOD, zr, true);
    }
}

static void test_a_drained_period_is_valid_after_the_offset_was_learnt(void)
{
    learn(30);
    push(N_PERIOD, 100, -100, 30 + 50);
    now += PERIOD_MS;
    const imu_sample_t s = imu_read(&imu, now, false);
    TEST_ASSERT_EQUAL(IMU_SAMPLE_VALID, s.state);
    TEST_ASSERT_EQUAL_UINT16(N_PERIOD, s.count);
    TEST_ASSERT_EQUAL_FLOAT(100.0f * IMU_HOST_DEFAULT_RPS_PER_LSB, s.gyro_x_rps);
    TEST_ASSERT_EQUAL_FLOAT(-100.0f * IMU_HOST_DEFAULT_RPS_PER_LSB, s.gyro_y_rps);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-6f, 50.0f * IMU_HOST_DEFAULT_RPS_PER_LSB, s.gyro_z_rps);
    TEST_ASSERT_EQUAL_UINT32(0u, imu_host_level());
    TEST_ASSERT_EQUAL_UINT32(0u, imu_host_flush_count());
}

static void test_an_empty_fifo_is_stale(void)
{
    learn(0);
    TEST_ASSERT_EQUAL(IMU_REASON_STALE, read_period(0u, 0, false).reason);
}

static void test_the_drain_is_bounded_and_a_backlog_is_flushed(void)
{
    learn(0);
    /* exactly IMU_DRAIN_MAX samples (80 ms at 800 Hz): all taken, not a backlog */
    push(IMU_DRAIN_MAX, 0, 0, 0);
    now += 80u;
    imu_sample_t s = imu_read(&imu, now, false);
    TEST_ASSERT_EQUAL(IMU_REASON_NONE, s.reason);
    TEST_ASSERT_EQUAL_UINT16(IMU_DRAIN_MAX, s.count);
    /* one more: flushed, INVALID, and nothing of it leaks into the next period */
    push(IMU_DRAIN_MAX + 1u, 0, 0, 0);
    push(50u, 0, 0, 0);
    now += 40u;
    s = imu_read(&imu, now, false);
    TEST_ASSERT_EQUAL(IMU_REASON_BACKLOG, s.reason);
    TEST_ASSERT_EQUAL_UINT32(1u, imu_host_flush_count());
    TEST_ASSERT_EQUAL_UINT32(0u, imu_host_level());
    TEST_ASSERT_EQUAL(IMU_REASON_NONE, read_period(N_PERIOD, 0, false).reason);
}

static void test_an_overrun_is_invalid_and_the_rest_is_still_drained(void)
{
    learn(0);
    push(N_PERIOD, 0, 0, 0);
    imu_host_inject_overrun();
    now += PERIOD_MS;
    TEST_ASSERT_EQUAL(IMU_REASON_OVERRUN, imu_read(&imu, now, false).reason);
    TEST_ASSERT_EQUAL_UINT32(0u, imu_host_level());
    TEST_ASSERT_EQUAL_UINT32(0u, imu_host_flush_count());
    TEST_ASSERT_EQUAL(IMU_REASON_NONE, read_period(N_PERIOD, 0, false).reason);
    /* NULL is refused without an overrun; a full FIFO loses samples: an overrun */
    TEST_ASSERT_FALSE(imu_host_push(NULL));
    TEST_ASSERT_EQUAL(IMU_REASON_NONE, read_period(N_PERIOD, 0, false).reason);
    push(IMU_HOST_FIFO_DEPTH, 0, 0, 0);
    const hal_imu_sample_t s = {{0, 0, 0}};
    TEST_ASSERT_FALSE(imu_host_push(&s));
    now += PERIOD_MS;
    TEST_ASSERT_EQUAL(IMU_REASON_OVERRUN, imu_read(&imu, now, false).reason);
}

static void test_a_port_fault_is_invalid_and_flushes(void)
{
    learn(0);
    push(N_PERIOD, 0, 0, 0);
    imu_host_set_error(true);
    now += PERIOD_MS;
    TEST_ASSERT_EQUAL(IMU_REASON_HAL, imu_read(&imu, now, false).reason);
    TEST_ASSERT_EQUAL_UINT32(1u, imu_host_flush_count());
    imu_host_set_error(false);
    TEST_ASSERT_EQUAL_UINT32(0u, imu_host_level()); /* the old samples are gone */
    TEST_ASSERT_EQUAL(IMU_REASON_NONE, read_period(N_PERIOD, 0, false).reason);
}

static void test_a_bad_port_configuration_is_invalid(void)
{
    learn(0);
    imu_host_set_config(IMU_HOST_DEFAULT_RPS_PER_LSB, 400u);
    TEST_ASSERT_EQUAL(IMU_REASON_CONFIG, read_period(N_PERIOD, 0, false).reason);
    imu_host_set_config(0.0f, IMU_HOST_DEFAULT_ODR_HZ);
    TEST_ASSERT_EQUAL(IMU_REASON_CONFIG, read_period(N_PERIOD, 0, false).reason);
}

static void test_a_second_call_in_the_same_ms_leaves_the_fifo_alone(void)
{
    learn(0);
    push(5u, 0, 0, 0);
    const imu_sample_t s = imu_read(&imu, now, false); /* same ms as the last read */
    TEST_ASSERT_EQUAL(IMU_REASON_NO_NEW, s.reason);
    TEST_ASSERT_EQUAL_UINT32(5u, imu_host_level());
    push(N_PERIOD - 5u, 0, 0, 0);
    now += PERIOD_MS;
    const imu_sample_t t = imu_read(&imu, now, false);
    TEST_ASSERT_EQUAL(IMU_REASON_NONE, t.reason);
    TEST_ASSERT_EQUAL_UINT16(N_PERIOD, t.count);
}

static void test_null_reads_arg(void)
{
    imu_init(NULL);
    push(N_PERIOD, 0, 0, 0);
    TEST_ASSERT_EQUAL(IMU_REASON_ARG, imu_read(NULL, 20u, false).reason);
    TEST_ASSERT_EQUAL_UINT32(N_PERIOD, imu_host_level());
}

/* Two threads: the feeder pushes one period's samples on request (the IMU ISR's place),
 * the reader waits for them and reads the period (the EKF task). */
#define STRESS_PERIODS (400u)
static int feeder_stop;
static int feeder_request;

static void* feeder(void* arg)
{
    (void)arg;
    const hal_imu_sample_t s[2] = {{{6, -4, 1}}, {{8, -2, 3}}}; /* noise: never frozen */
    while (__atomic_load_n(&feeder_stop, __ATOMIC_ACQUIRE) == 0) {
        if (__atomic_load_n(&feeder_request, __ATOMIC_ACQUIRE) != 0) {
            for (uint32_t i = 0u; i < N_PERIOD; i++) {
                (void)imu_host_push(&s[i % 2u]);
            }
            __atomic_store_n(&feeder_request, 0, __ATOMIC_RELEASE);
        } else {
            (void)sched_yield();
        }
    }
    return NULL;
}

static void test_a_feed_thread_and_the_reader_thread_share_the_fifo(void)
{
    pthread_t t;
    __atomic_store_n(&feeder_stop, 0, __ATOMIC_RELEASE);
    TEST_ASSERT_EQUAL_INT(0, pthread_create(&t, NULL, feeder, NULL));
    uint32_t valid = 0u;
    for (uint32_t p = 0u; p < STRESS_PERIODS; p++) {
        __atomic_store_n(&feeder_request, 1, __ATOMIC_RELEASE);
        while (__atomic_load_n(&feeder_request, __ATOMIC_ACQUIRE) != 0) {
            (void)sched_yield();
        }
        now += PERIOD_MS;
        const imu_sample_t s = imu_read(&imu, now, p < 120u);
        TEST_ASSERT_EQUAL_UINT16(N_PERIOD, s.count);
        if (s.state == IMU_SAMPLE_VALID) {
            TEST_ASSERT_EQUAL_FLOAT(7.0f * IMU_HOST_DEFAULT_RPS_PER_LSB, s.gyro_x_rps);
            TEST_ASSERT_FLOAT_WITHIN(1.0e-6f, 0.0f, s.gyro_z_rps);
            valid++;
        }
    }
    __atomic_store_n(&feeder_stop, 1, __ATOMIC_RELEASE);
    TEST_ASSERT_EQUAL_INT(0, pthread_join(t, NULL));
    TEST_ASSERT_EQUAL_UINT32(STRESS_PERIODS - 100u, valid); /* learnt after START + 2 s */
}

static void test_REQ_FD03_the_offset_health_follows_the_learning(void)
{
    imu_offset_health_t h = imu_offset_health(&imu);
    TEST_ASSERT_FALSE(h.offset_learned);
    TEST_ASSERT_FALSE(h.offset_rejected);
    TEST_ASSERT_EQUAL_UINT32(0u, h.reject_count);
    learn(30);
    h = imu_offset_health(&imu);
    TEST_ASSERT_TRUE(h.offset_learned);
    TEST_ASSERT_FALSE(h.offset_rejected);
    /* a window beyond IMU_OFFSET_MAX_RPS (2 deg/s = 33 LSB): rejected, counted, dropped */
    for (uint32_t i = 0u; i < 110u; i++) {
        (void)read_period(N_PERIOD, 100, true);
    }
    h = imu_offset_health(&imu);
    TEST_ASSERT_FALSE(h.offset_learned);
    TEST_ASSERT_TRUE(h.offset_rejected);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(1u, h.reject_count);
    /* a good window learns again and clears the rejection, the count stays */
    const uint32_t rejects = h.reject_count;
    for (uint32_t i = 0u; i < 110u; i++) {
        (void)read_period(N_PERIOD, 30, true);
    }
    h = imu_offset_health(&imu);
    TEST_ASSERT_TRUE(h.offset_learned);
    TEST_ASSERT_FALSE(h.offset_rejected);
    TEST_ASSERT_EQUAL_UINT32(rejects, h.reject_count);
}

static void test_REQ_FD03_a_null_imu_reads_no_offset_health(void)
{
    const imu_offset_health_t h = imu_offset_health(NULL);
    TEST_ASSERT_FALSE(h.offset_learned);
    TEST_ASSERT_FALSE(h.offset_rejected);
    TEST_ASSERT_EQUAL_UINT32(0u, h.reject_count);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_a_drained_period_is_valid_after_the_offset_was_learnt);
    RUN_TEST(test_an_empty_fifo_is_stale);
    RUN_TEST(test_the_drain_is_bounded_and_a_backlog_is_flushed);
    RUN_TEST(test_an_overrun_is_invalid_and_the_rest_is_still_drained);
    RUN_TEST(test_a_port_fault_is_invalid_and_flushes);
    RUN_TEST(test_a_bad_port_configuration_is_invalid);
    RUN_TEST(test_a_second_call_in_the_same_ms_leaves_the_fifo_alone);
    RUN_TEST(test_null_reads_arg);
    RUN_TEST(test_REQ_FD03_the_offset_health_follows_the_learning);
    RUN_TEST(test_REQ_FD03_a_null_imu_reads_no_offset_health);
    RUN_TEST(test_a_feed_thread_and_the_reader_thread_share_the_fifo);
    return UNITY_END();
}
