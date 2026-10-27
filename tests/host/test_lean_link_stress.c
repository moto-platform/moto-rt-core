/*
 * Two-thread stress test of the lean link (D-065 item 2, E-17 (b)): the real EKF-task
 * step (lean_ekf_step on the host IMU fake) in one thread and the real comms-task side
 * (lean_tx_publish_speed + lean_tx_step on an in-process platform bus) in another, sharing
 * only lean_link_t, as on the H7. Run under ASan + UBSan (host-tests) and ThreadSanitizer
 * (host-tsan preset). test_snapshot_stress covers the bare exchange.
 *
 * Time is a shared 1 ms virtual clock. The EKF thread owns it, steps every EKF cycle and
 * may lead the comms thread by at most LEAD_MAX_MS, like the higher-priority EKF task
 * preempting a comms pass (within LEAN_TX_FUTURE_MAX_MS). The comms thread makes one pass
 * per ms, in order, never skipping one. The handshake orders everything outside that
 * window for TSan: EKF step S can only race the comms passes S-4 .. S. So each scenario
 * puts the 0x020 send grid at one phase in that window, S - k for k = 0..4 (the grid
 * anchors at the first lean_tx_step() call, services/com_core), and every send pass of
 * the production reader (lean_tx_step -> lean_tx_msg -> com_send) can race an EKF step
 * (safety-reviewer MAJOR-1 on the first version). The bound also keeps the timeline
 * realistic, which makes the outcome exact: once the lean is ESTIMATED, every 0x020 frame
 * stays non-INVALID.
 *
 * What it checks besides TSan:
 * - every speed belongs to its stamp (the speed alternates with the stamp's parity), and
 *   speed stamps never go backwards or ahead of the EKF;
 * - on every pass from the first send, the comms task reads the alive count and then the
 *   newest output record: the record is at least as new as the count says (the EKF bumps alive only
 *   after the publish), the count never decreases, the record's fields agree with its
 *   state, and its stamp never goes backwards;
 * - the DID 0xFD03 fields (E-17 (a), D-067) travel in the same record and are checked with
 *   it: the offset flags agree with the state (ESTIMATED needs a learnt offset, a learnt
 *   offset reads 0 unlearned seconds, an unlearned run reads the seconds since the first
 *   step), nothing is rejected, and a torn record would break one of these. The comms
 *   task also builds the DID itself every pass from lean_tx_step()'s comms-local copy
 *   (production path; that copy never reads the snapshot) and checks its layout rules;
 * - one 0x020 frame per cycle, all E2E-clean, and no INVALID one after the first valid.
 *
 * Limits (README "Lean glue" tests): the IMU fake is fed by the EKF thread (test_imu covers
 * a feeder thread); the EKF never overruns its period, and WCET and preemption depth are
 * not modelled (HIL, Q-019); the H7 hal_atomic port is not exercised; the comms thread
 * runs the lean side only, not the whole comms_pass(). The record read is a second read by
 * the same consumer task, after lean_tx_step() is done with its index; reading every ms, it
 * leaves the "publish replaces an unread record" path to test_snapshot_stress.
 *
 * Unity asserts only in the main thread; the workers count what they saw, and the main
 * thread checks it after pthread_join().
 */
#include "features/cornering/lean.h"
#include "features/cornering/lean_tx.h"
#include "hal/host/can_port_host.h"
#include "hal/host/imu_host.h"
#include "hal/host/vbus.h"
#include "vehicle_e2e.h"
#include "platform.h"
#include "platform_e2e.h"
#include "services/can_if.h"
#include "services/lean_health.h"
#include "services/vehicle_signals.h"
#include "vehicle_cl250.h"

#include <math.h>
#include <pthread.h>
#include <sched.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <unity.h>

#define CYCLE_MS PLATFORM_EKF_LEAN_CYCLE_TIME_MS
#define START_MS 1000u
#define RIDE_MS (START_MS + 3000u) /* standstill until here: the z offset is learnt */
#define END_MS (START_MS + 60000u) /* 1 min of virtual time, 3000 EKF cycles per phase */
#define LEAD_MAX_MS 5u             /* the EKF's lead over the comms thread */
#define N_CYCLE (16u)              /* IMU samples per cycle at the fake's 800 Hz */
#define Z_OFFSET (8)               /* raw z gyro offset, about 0.49 deg/s */
#define RIDE_KMH 50u               /* ridden at 50 or 51 km/h, by the sample's stamp */
#define STRAIGHT_ROLL_MAX_DEG (2.0f)

_Static_assert(LEAD_MAX_MS <= LEAN_TX_FUTURE_MAX_MS, "the lead must stay a tolerated preemption");
_Static_assert((START_MS % CYCLE_MS) == 0u, "EKF steps on the cycle grid");
_Static_assert(LEAD_MAX_MS < CYCLE_MS, "the send phases must lie within one cycle");
_Static_assert(LEAD_MAX_MS == 5u, "one RUN_TEST per send phase 0..LEAD_MAX_MS-1 below");

static vbus_t bus_p;
static uint8_t node_platform;
static uint8_t node_peer;
static lean_link_t link;
static lean_ekf_t ekf;
static lean_tx_t tx;

static uint32_t sim_ms;   /* EKF thread writes: the virtual clock */
static uint32_t comms_ms; /* comms thread writes: the last pass it completed */
static uint32_t phase_k;  /* set before the threads start: 0x020 sends at S - phase_k */

typedef struct {
    uint32_t steps;
    uint32_t speeds_valid;    /* VALID speed records the EKF read */
    uint32_t speed_torn;      /* a speed that does not belong to its stamp */
    uint32_t speed_backwards; /* a speed stamp older than one read before */
    uint32_t speed_future;    /* a speed stamp after the EKF's own time */
    uint32_t imu_push_failed;
} ekf_result_t;

typedef struct {
    uint32_t passes;
    uint32_t write_failed;     /* vehicle_signals or the gen/ decoder refused a sample */
    uint32_t first_send_ms;    /* the pass that anchored the 0x020 grid */
    uint32_t records;          /* output records checked (every pass from the first send) */
    uint32_t record_backwards; /* an output stamp older than one read before */
    uint32_t record_off_grid;  /* an output stamp off the EKF cycle grid */
    uint32_t record_fault;     /* an EKF-side snapshot fault in a record */
    uint32_t record_bad;       /* fields that disagree with the record's state */
    uint32_t record_unlearned_after_learnt; /* the offset flag went back to false */
    uint32_t health_reads;     /* DID 0xFD03 builds in the comms task */
    uint32_t health_fresh;     /* of them with RECORD_FRESH set */
    uint32_t health_bad;       /* a DID that breaks its layout rules */
    uint32_t record_behind;    /* a record older than the alive count read before it */
    uint32_t alive_backwards;  /* an alive count lower than one read before */
    uint32_t estimated;
    uint32_t frames;
    uint32_t frames_off_phase; /* a frame from a pass off the scenario's send phase */
    uint32_t frames_invalid;
    uint32_t pack_failed;
    uint32_t e2e_bad;
    uint32_t first_valid_ms;      /* 0: none yet */
    uint32_t invalid_after_valid; /* INVALID frames after the first valid one */
} comms_result_t;

static ekf_result_t eres;
static comms_result_t cres;

void setUp(void)
{
    memset(&eres, 0, sizeof eres);
    memset(&cres, 0, sizeof cres);
    vbus_init(&bus_p);
    TEST_ASSERT_TRUE(vbus_attach(&bus_p, &node_platform));
    TEST_ASSERT_TRUE(vbus_attach(&bus_p, &node_peer));
    TEST_ASSERT_TRUE(can_port_host_bind_vbus(CAN_PORT_PLATFORM, &bus_p, node_platform));
    can_if_init();
    vehicle_signals_init();
    imu_host_reset();
    lean_link_init(&link);
    lean_ekf_init(&ekf, &link);
    TEST_ASSERT_TRUE(lean_tx_open(&tx, &link, lean_link_alive(&link)));
    TEST_ASSERT_EQUAL(CAN_PORT_OK, can_if_apply_filters(CAN_PORT_PLATFORM));
    __atomic_store_n(&sim_ms, START_MS, __ATOMIC_SEQ_CST);
    __atomic_store_n(&comms_ms, START_MS, __ATOMIC_SEQ_CST);
}

void tearDown(void)
{
    can_port_host_unbind_all();
}

/* The speed the comms thread hands over at stamp_ms, in km/h (0xF40D raw = km/h): 0 while
 * standing, then RIDE_KMH or one more by the stamp's parity, so a speed glued to another
 * publish's stamp shows in every other record. */
static uint8_t speed_kmh_at(uint32_t stamp_ms)
{
    return (stamp_ms < RIDE_MS) ? 0u : (uint8_t)(RIDE_KMH + (stamp_ms & 1u));
}

static void check_speed(const lean_speed_t* s, uint32_t now, uint32_t* last_stamp)
{
    if (s->state != LEAN_SPEED_VALID) {
        return; /* the boot record, before the first sample */
    }
    eres.speeds_valid++;
    const float kmh = s->speed_mps * LEAN_KMH_PER_MPS;
    eres.speed_torn += (fabsf(kmh - (float)speed_kmh_at(s->stamp_ms)) < 0.01f) ? 0u : 1u;
    eres.speed_backwards += (s->stamp_ms < *last_stamp) ? 1u : 0u;
    eres.speed_future += (s->stamp_ms > now) ? 1u : 0u;
    *last_stamp = s->stamp_ms;
}

/* EKF task: owns the clock, leads the comms thread by at most LEAD_MAX_MS. */
static void* ekf_thread(void* arg)
{
    (void)arg;
    uint32_t last_speed_stamp = 0u;
    for (uint32_t now = START_MS + 1u; now <= END_MS; now++) {
        while ((__atomic_load_n(&comms_ms, __ATOMIC_ACQUIRE) + LEAD_MAX_MS) < now) {
            (void)sched_yield();
        }
        __atomic_store_n(&sim_ms, now, __ATOMIC_RELEASE);
        if ((now % CYCLE_MS) != 0u) {
            continue;
        }
        for (uint32_t i = 0u; i < N_CYCLE; i++) {
            const int16_t d = ((i % 2u) == 0u) ? -1 : 1; /* straight, -1/+1 LSB of noise */
            const hal_imu_sample_t s = {{d, d, (int16_t)(Z_OFFSET + d)}};
            eres.imu_push_failed += imu_host_push(&s) ? 0u : 1u;
        }
        lean_ekf_step(&ekf, now);
        eres.steps++;
        check_speed(&ekf.speed, now, &last_speed_stamp);
    }
    return NULL;
}

/* True if data is the gen/ INVALID 0x020 frame, packed with the frame's own E2E CRC and
 * counter (gen/ offsets): rt_core's gen/ has no 0x020 unpack (as in test_comms_lean). */
static bool frame_is_invalid(const uint8_t* data)
{
    struct platform_ekf_lean_t m;
    (void)platform_ekf_lean_init(&m);
    m.lean_angle_state = PLATFORM_EKF_LEAN_LEAN_ANGLE_STATE_INVALID_CHOICE;
    m.lean_angle = 0;
    m.lean_angle_quality = 0u;
    m.e2_e_crc = data[VEHICLE_E2E_CRC_OFFSET];
    m.e2_e_counter = (uint8_t)(data[VEHICLE_E2E_COUNTER_OFFSET] & VEHICLE_E2E_COUNTER_MASK);
    uint8_t ref[PLATFORM_EKF_LEAN_LENGTH];
    if (platform_ekf_lean_pack(ref, &m, sizeof ref) != (int)PLATFORM_EKF_LEAN_LENGTH) {
        cres.pack_failed++;
        return true;
    }
    return memcmp(ref, data, sizeof ref) == 0;
}

/* The first EKF step's time: the unlearned run starts there (lean_health_unlearned_s). */
#define FIRST_STEP_MS (START_MS + CYCLE_MS)

/* The EKF-side 0xFD03 fields agree with the state and the stamp (E-17 (a)). The scenario
 * rides straight on a clean IMU: nothing is ever rejected. */
static bool health_consistent(const lean_record_t* r)
{
    const lean_health_ekf_t* h = &r->health;
    if (h->offset_rejected || (h->reject_count != 0u)) {
        return false;
    }
    if (h->offset_learned) {
        return h->unlearned_s == 0u;
    }
    return h->unlearned_s == (uint16_t)((r->stamp_ms - FIRST_STEP_MS) / 1000u);
}

/* The fields agree with the state (a torn record would mix two outputs). The boot record
 * (stamp 0) is the zero record. */
static bool record_consistent(const lean_record_t* r)
{
    if (r->stamp_ms == 0u) {
        const lean_health_ekf_t zero = {false, false, 0u, 0u};
        return (r->out.state == LEAN_OUT_INVALID) && (memcmp(&r->health, &zero, sizeof zero) == 0);
    }
    if (!health_consistent(r)) {
        return false;
    }
    if (r->out.state == LEAN_OUT_ESTIMATED) {
        return r->health.offset_learned && isfinite(r->out.roll_deg) &&
               (fabsf(r->out.roll_deg) < STRAIGHT_ROLL_MAX_DEG) && (r->out.quality > 0u) &&
               (r->out.quality <= 100u) && (r->out.reason == LEAN_REASON_NONE);
    }
    return (r->out.state == LEAN_OUT_INVALID) && (r->out.roll_deg == 0.0f) && (r->out.quality == 0u) &&
           (r->out.reason != LEAN_REASON_NONE);
}

/* Comms task, every pass from the first send: the alive count, then the newest output
 * record (a second read by the same consumer, after lean_tx_step() is done with its index). */
static void check_record(uint32_t* last_stamp, uint32_t* last_alive, bool* seen_learnt)
{
    const uint32_t alive = alive_count(&link.alive);
    uint8_t i = 0u;
    (void)snapshot_read_index(&link.out_snap, &i);
    const lean_record_t* r = &link.out_slots[i];
    cres.records++;
    cres.alive_backwards += (alive < *last_alive) ? 1u : 0u;
    /* alive = n means steps 1..n published first, the newest at START_MS + n * CYCLE_MS */
    cres.record_behind += ((alive > 0u) && (r->stamp_ms < (START_MS + (alive * CYCLE_MS)))) ? 1u : 0u;
    cres.record_backwards += (r->stamp_ms < *last_stamp) ? 1u : 0u;
    cres.record_off_grid += ((r->stamp_ms % CYCLE_MS) == 0u) ? 0u : 1u;
    cres.record_fault += r->fault ? 1u : 0u;
    cres.record_bad += record_consistent(r) ? 0u : 1u;
    cres.record_unlearned_after_learnt += (*seen_learnt && !r->health.offset_learned) ? 1u : 0u;
    *seen_learnt = *seen_learnt || r->health.offset_learned;
    cres.estimated += (r->out.state == LEAN_OUT_ESTIMATED) ? 1u : 0u;
    *last_stamp = r->stamp_ms;
    *last_alive = alive;
}

/* DID 0xFD03 as the UDS server builds it, from lean_tx_step()'s comms-local copy and the
 * comms flags app/comms sets (production path). The copy lags a record that check_record()
 * took first, so only the layout rules are checked, not freshness against the clock. */
static void check_health_did(uint32_t now)
{
    const lean_health_comms_t comms = {lean_tx_speed_producer_fault(&tx), lean_tx_output_consumer_fault(&tx), true};
    lean_health_set_comms(&comms);
    uint8_t d[PLATFORM_UDS_DID_LEAN_HEALTH_LENGTH];
    cres.health_reads++;
    if (!lean_health_read(now, d, sizeof d)) {
        cres.health_bad++;
        return;
    }
    const uint8_t f = d[PLATFORM_UDS_LEAN_HEALTH_RECORD_FRESH_BYTE];
    const uint16_t rejects = (uint16_t)(((uint16_t)d[PLATFORM_UDS_LEAN_HEALTH_OFFSET_REJECTS_BYTE] << 8u) |
                                        d[PLATFORM_UDS_LEAN_HEALTH_OFFSET_REJECTS_BYTE + 1u]);
    const uint16_t unl = (uint16_t)(((uint16_t)d[PLATFORM_UDS_LEAN_HEALTH_UNLEARNED_S_BYTE] << 8u) |
                                    d[PLATFORM_UDS_LEAN_HEALTH_UNLEARNED_S_BYTE + 1u]);
    const bool fresh = (f & PLATFORM_UDS_LEAN_HEALTH_RECORD_FRESH_MASK) != 0u;
    const bool learned = (f & PLATFORM_UDS_LEAN_HEALTH_OFFSET_LEARNED_MASK) != 0u;
    bool ok = (f & PLATFORM_UDS_LEAN_HEALTH_RESERVED_MASK) == 0u;
    ok = ok && ((f & (PLATFORM_UDS_LEAN_HEALTH_OFFSET_REJECTED_MASK | PLATFORM_UDS_LEAN_HEALTH_SPEED_PRODUCER_FAULT_MASK |
                      PLATFORM_UDS_LEAN_HEALTH_OUTPUT_CONSUMER_FAULT_MASK | PLATFORM_UDS_LEAN_HEALTH_SPEED_CONSUMER_FAULT_MASK)) == 0u);
    ok = ok && ((f & PLATFORM_UDS_LEAN_HEALTH_EKF_ALIVE_OK_MASK) != 0u) && (rejects == 0u);
    ok = ok && (!learned || fresh);                          /* a learnt offset is reported fresh only */
    ok = ok && (fresh || (unl == PLATFORM_UDS_LEAN_HEALTH_UNLEARNED_S_UNKNOWN));
    ok = ok && (!(fresh && learned) || (unl == 0u));
    cres.health_bad += ok ? 0u : 1u;
    cres.health_fresh += fresh ? 1u : 0u;
}

static void drain(uint32_t now, vehicle_e2e_rx_state_t* rx)
{
    can_frame_t got;
    while (vbus_recv(&bus_p, node_peer, &got) == CAN_PORT_OK) {
        if (got.id != PLATFORM_EKF_LEAN_FRAME_ID) {
            continue;
        }
        cres.frames++;
        cres.frames_off_phase += (((now + phase_k) % CYCLE_MS) == 0u) ? 0u : 1u;
        const vehicle_e2e_status_t st =
            vehicle_e2e_check(PLATFORM_EKF_LEAN_E2E_DATA_ID, PLATFORM_EKF_LEAN_E2E_MAX_DELTA_COUNTER,
                           PLATFORM_EKF_LEAN_E2E_TIMEOUT_MS, got.data, got.dlc, rx, now);
        cres.e2e_bad += ((st == VEHICLE_E2E_OK) || (st == VEHICLE_E2E_INITIAL)) ? 0u : 1u;
        if (frame_is_invalid(got.data)) {
            cres.frames_invalid++;
            cres.invalid_after_valid += (cres.first_valid_ms != 0u) ? 1u : 0u;
        } else if (cres.first_valid_ms == 0u) {
            cres.first_valid_ms = now;
        } else {
            /* valid again */
        }
    }
}

/* Comms task: one pass per ms, in order, as the comms task does with the UDS client's
 * speed sample; 0x020 starts at the scenario's phase. Then it reports the pass done. */
static void* comms_thread(void* arg)
{
    (void)arg;
    vehicle_e2e_rx_state_t rx;
    vehicle_e2e_rx_init(&rx);
    uint32_t last_record_stamp = 0u;
    uint32_t last_alive = 0u;
    bool seen_learnt = false;
    bool sending = false;
    uint32_t done = START_MS;
    while (done < END_MS) {
        if (__atomic_load_n(&sim_ms, __ATOMIC_ACQUIRE) == done) {
            (void)sched_yield();
            continue;
        }
        const uint32_t now = done + 1u;
        const uint8_t raw = speed_kmh_at(now);
        float phys = 0.0f;
        const bool decoded =
            vehicle_cl250_decode(&vehicle_cl250_dids[VEHICLE_CL250_IDX_VEHICLE_SPEED], &raw, 1u, &phys);
        const bool written = decoded && vehicle_signals_write(VEHICLE_CL250_IDX_VEHICLE_SPEED, raw, phys, now);
        cres.write_failed += written ? 0u : 1u;
        lean_tx_publish_speed(&tx, now);
        const bool on_phase = ((now + phase_k) % CYCLE_MS) == 0u;
        if (!sending && on_phase) {
            sending = true; /* the first call anchors the 0x020 grid here */
            cres.first_send_ms = now;
        }
        if (sending) {
            lean_tx_step(&tx, now);
            check_health_did(now);
        }
        drain(now, &rx);
        if (sending) {
            check_record(&last_record_stamp, &last_alive, &seen_learnt);
        }
        cres.passes++;
        done = now;
        __atomic_store_n(&comms_ms, done, __ATOMIC_RELEASE);
    }
    return NULL;
}

/* One scenario with 0x020 sent k ms before each EKF step (k = 0: in the same ms). */
static void run_phase(uint32_t k)
{
    phase_k = k;
    pthread_t e;
    pthread_t c;
    TEST_ASSERT_EQUAL_INT(0, pthread_create(&c, NULL, comms_thread, NULL));
    TEST_ASSERT_EQUAL_INT(0, pthread_create(&e, NULL, ekf_thread, NULL));
    TEST_ASSERT_EQUAL_INT(0, pthread_join(e, NULL));
    TEST_ASSERT_EQUAL_INT(0, pthread_join(c, NULL));

    /* EKF side: every cycle complete, the speeds whole, newest-first, never ahead */
    TEST_ASSERT_EQUAL_UINT32((END_MS - START_MS) / CYCLE_MS, eres.steps);
    TEST_ASSERT_EQUAL_UINT32(eres.steps, alive_count(&link.alive));
    TEST_ASSERT_EQUAL_UINT32(0u, eres.imu_push_failed);
    TEST_ASSERT_GREATER_THAN_UINT32(eres.steps / 2u, eres.speeds_valid);
    TEST_ASSERT_EQUAL_UINT32(0u, eres.speed_torn);
    TEST_ASSERT_EQUAL_UINT32(0u, eres.speed_backwards);
    TEST_ASSERT_EQUAL_UINT32(0u, eres.speed_future);
    TEST_ASSERT_TRUE(ekf.imu.core.offset_learned);

    /* comms side: one pass per ms, the records whole, in order and never behind alive */
    TEST_ASSERT_EQUAL_UINT32(END_MS - START_MS, cres.passes);
    TEST_ASSERT_EQUAL_UINT32(0u, cres.write_failed);
    TEST_ASSERT_EQUAL_UINT32(CYCLE_MS - k, cres.first_send_ms - START_MS);
    const uint32_t sends = ((END_MS - cres.first_send_ms) / CYCLE_MS) + 1u;
    TEST_ASSERT_EQUAL_UINT32((END_MS - cres.first_send_ms) + 1u, cres.records);
    TEST_ASSERT_EQUAL_UINT32(0u, cres.record_backwards);
    TEST_ASSERT_EQUAL_UINT32(0u, cres.record_off_grid);
    TEST_ASSERT_EQUAL_UINT32(0u, cres.record_fault);
    TEST_ASSERT_EQUAL_UINT32(0u, cres.record_bad);
    TEST_ASSERT_EQUAL_UINT32(0u, cres.record_behind);
    TEST_ASSERT_EQUAL_UINT32(0u, cres.record_unlearned_after_learnt);
    TEST_ASSERT_EQUAL_UINT32(0u, cres.health_bad);
    TEST_ASSERT_EQUAL_UINT32((END_MS - cres.first_send_ms) + 1u, cres.health_reads);
    TEST_ASSERT_GREATER_THAN_UINT32(cres.health_reads / 2u, cres.health_fresh);
    TEST_ASSERT_EQUAL_UINT32(0u, cres.alive_backwards);
    TEST_ASSERT_GREATER_THAN_UINT32(0u, cres.estimated);

    /* frames: one per cycle at the scenario's phase, E2E-clean */
    TEST_ASSERT_EQUAL_UINT32(sends, cres.frames);
    TEST_ASSERT_EQUAL_UINT32(0u, cres.frames_off_phase);
    TEST_ASSERT_EQUAL_UINT32(0u, cres.pack_failed);
    TEST_ASSERT_EQUAL_UINT32(0u, cres.e2e_bad);

    /* the lean turns valid soon after riding off and no race ever makes it INVALID again */
    TEST_ASSERT_NOT_EQUAL_UINT32(0u, cres.first_valid_ms);
    TEST_ASSERT_LESS_THAN_UINT32(RIDE_MS + 3000u, cres.first_valid_ms);
    TEST_ASSERT_EQUAL_UINT32(0u, cres.invalid_after_valid);

    /* no snapshot fault on either side */
    TEST_ASSERT_FALSE(snapshot_producer_fault(&link.speed_snap));
    TEST_ASSERT_FALSE(snapshot_consumer_fault(&link.speed_snap));
    TEST_ASSERT_FALSE(snapshot_producer_fault(&link.out_snap));
    TEST_ASSERT_FALSE(snapshot_consumer_fault(&link.out_snap));
}

static void test_0x020_sent_in_the_ms_of_the_ekf_step(void)
{
    run_phase(0u);
}

static void test_0x020_sent_1_ms_before_the_ekf_step(void)
{
    run_phase(1u);
}

static void test_0x020_sent_2_ms_before_the_ekf_step(void)
{
    run_phase(2u);
}

static void test_0x020_sent_3_ms_before_the_ekf_step(void)
{
    run_phase(3u);
}

static void test_0x020_sent_4_ms_before_the_ekf_step(void)
{
    run_phase(4u);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_0x020_sent_in_the_ms_of_the_ekf_step);
    RUN_TEST(test_0x020_sent_1_ms_before_the_ekf_step);
    RUN_TEST(test_0x020_sent_2_ms_before_the_ekf_step);
    RUN_TEST(test_0x020_sent_3_ms_before_the_ekf_step);
    RUN_TEST(test_0x020_sent_4_ms_before_the_ekf_step);
    return UNITY_END();
}
