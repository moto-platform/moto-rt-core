/*
 * SIL test (D-070, safety-reviewer MINOR-2): the DID hand-over to the derived task must not
 * disturb the comms pass. The real comms_pass() with the UDS client and the simulated CL250
 * ECU, the heartbeat (0x081) and the lean sender (0x020), with services/derived_inputs
 *   A  never initialised (a missing boot call: counted, nothing published),
 *   B  healthy, nobody consuming (the publish only overwrites),
 *   C  faulted (the snapshot's exchange hands back a bad word in the middle of the run).
 * The frame counts and the longest gap of 0x081 and 0x020 must be identical in all three.
 * The test defines hal_atomic_u32_* itself (a double that can corrupt the derived word),
 * so the host port's object is not linked; A must run before B because the module stays
 * initialised once derived_inputs_init() ran.
 */
#include "app/comms.h"
#include "app/host/sim_ecu.h"
#include "features/cornering/lean.h"
#include "features/cornering/lean_tx.h"
#include "hal/hal_atomic.h"
#include "hal/host/can_port_host.h"
#include "hal/host/hal_time_host.h"
#include "hal/host/imu_host.h"
#include "platform.h"
#include "services/can_if.h"
#include "services/derived_inputs.h"
#include "services/diag.h"
#include "services/timebase.h"
#include "services/vehicle_signals.h"

#include <string.h>
#include <unity.h>

#define START_MS 1000u
#define RUN_MS 1500u

/* ------------------------------------------------------- hal_atomic test double */

static const hal_atomic_u32_t* derived_word;
static bool capture;
static bool corrupt_exchange;

uint32_t hal_atomic_u32_load(const hal_atomic_u32_t* a)
{
    return a->value;
}

void hal_atomic_u32_store(hal_atomic_u32_t* a, uint32_t v)
{
    if (capture) {
        derived_word = a;
    }
    a->value = v;
}

uint32_t hal_atomic_u32_exchange(hal_atomic_u32_t* a, uint32_t v)
{
    const uint32_t old = a->value;
    a->value = v;
    if (corrupt_exchange && (a == derived_word)) {
        corrupt_exchange = false;
        return 3u | 0x4u;
    }
    return old;
}

/* ------------------------------------------------------------------------- rig */

static vbus_t bus_v;
static vbus_t bus_p;
static uint8_t node_vehicle;
static uint8_t node_platform;
static uint8_t node_ppeer;
static uds_client_t client;
static uds_server_t server;
static vehicle_republish_t republisher;
static heartbeat_t heartbeat;
static comms_ekf_t ekf;
static lean_link_t link;
static lean_ekf_t lean;
static lean_tx_t tx;
static sim_ecu_t ecu;

typedef struct {
    uint32_t hb_frames;
    uint32_t hb_gap_max;
    uint32_t hb_last;
    uint32_t lean_frames;
    uint32_t lean_gap_max;
    uint32_t lean_last;
} counts_t;

void setUp(void) {}
void tearDown(void) {}

static void rig_up(void)
{
    hal_time_host_use_manual(START_MS);
    vbus_init(&bus_v);
    vbus_init(&bus_p);
    TEST_ASSERT_TRUE(vbus_attach(&bus_v, &node_vehicle));
    TEST_ASSERT_TRUE(vbus_attach(&bus_p, &node_platform));
    TEST_ASSERT_TRUE(vbus_attach(&bus_p, &node_ppeer));
    TEST_ASSERT_TRUE(can_port_host_bind_vbus(CAN_PORT_VEHICLE, &bus_v, node_vehicle));
    TEST_ASSERT_TRUE(can_port_host_bind_vbus(CAN_PORT_PLATFORM, &bus_p, node_platform));
    TEST_ASSERT_TRUE(sim_ecu_init(&ecu, &bus_v));
    ecu.require_session = true;
    can_if_init();
    diag_init(timebase_now_ms());
    vehicle_signals_init();
    imu_host_reset();
    (void)memset(&client, 0, sizeof client);
    (void)memset(&server, 0, sizeof server);
    TEST_ASSERT_EQUAL(ISOTP_OK, uds_client_open(&client));
    TEST_ASSERT_EQUAL(ISOTP_OK, uds_server_open(&server));
    TEST_ASSERT_TRUE(vehicle_republish_open(&republisher));
    TEST_ASSERT_TRUE(heartbeat_open(&heartbeat));
    comms_ekf_init(&ekf);
    lean_link_init(&link);
    lean_ekf_init(&lean, &link);
    TEST_ASSERT_TRUE(comms_ekf_register(&ekf, lean_link_alive(&link)));
    TEST_ASSERT_TRUE(comms_open_lean(&ekf, &tx, &link));
    TEST_ASSERT_TRUE(comms_apply_filters());
}

/* derived_inputs_init() with the double learning the service's middle word. */
static void derived_boot(void)
{
    capture = true;
    derived_inputs_init();
    capture = false;
}

static void note(uint32_t now, uint32_t* frames, uint32_t* last, uint32_t* gap_max)
{
    if ((*frames > 0u) && ((now - *last) > *gap_max)) {
        *gap_max = now - *last;
    }
    *last = now;
    (*frames)++;
}

/* RUN_MS passes of 1 ms; corrupt_at_ms (0: never) arms the derived exchange fault. */
static counts_t run(uint32_t corrupt_at_ms)
{
    counts_t c;
    (void)memset(&c, 0, sizeof c);
    for (uint32_t i = 0u; i < RUN_MS; i++) {
        const uint32_t now = timebase_now_ms();
        if ((corrupt_at_ms != 0u) && (i == corrupt_at_ms)) {
            corrupt_exchange = true;
        }
        if ((now % PLATFORM_EKF_LEAN_CYCLE_TIME_MS) == 0u) {
            lean_ekf_step(&lean, now);
        }
        sim_ecu_step(&ecu, now);
        comms_pass(&client, &server, &republisher, &heartbeat, &ekf, &tx);
        can_frame_t got;
        while (vbus_recv(&bus_p, node_ppeer, &got) == CAN_PORT_OK) {
            if (got.id == PLATFORM_HEARTBEAT_RT_CORE_FRAME_ID) {
                note(now, &c.hb_frames, &c.hb_last, &c.hb_gap_max);
            } else if (got.id == PLATFORM_EKF_LEAN_FRAME_ID) {
                note(now, &c.lean_frames, &c.lean_last, &c.lean_gap_max);
            } else {
                /* 0x021, 0x110: not counted here */
            }
        }
        hal_time_host_advance(1u);
    }
    can_port_host_unbind_all();
    hal_time_host_use_monotonic();
    return c;
}

static void test_0x081_and_0x020_keep_their_cycle_whatever_derived_inputs_does(void)
{
    /* A: derived_inputs_init() never ran */
    rig_up();
    const counts_t a = run(0u);
    TEST_ASSERT_EQUAL_UINT32(RUN_MS, derived_inputs_skipped_count());
    TEST_ASSERT_TRUE(a.hb_frames > 10u);
    TEST_ASSERT_TRUE(a.lean_frames > 40u);

    /* B: healthy, nobody consuming */
    rig_up();
    derived_boot();
    const counts_t b = run(0u);
    TEST_ASSERT_EQUAL_UINT32(0u, derived_inputs_skipped_count());
    TEST_ASSERT_FALSE(derived_inputs_producer_fault());

    /* C: the exchange fails 300 ms into the run */
    rig_up();
    derived_boot();
    const counts_t c = run(300u);
    TEST_ASSERT_TRUE(derived_inputs_producer_fault());

    TEST_ASSERT_EQUAL_UINT32(a.hb_frames, b.hb_frames);
    TEST_ASSERT_EQUAL_UINT32(a.hb_frames, c.hb_frames);
    TEST_ASSERT_EQUAL_UINT32(a.lean_frames, b.lean_frames);
    TEST_ASSERT_EQUAL_UINT32(a.lean_frames, c.lean_frames);
    TEST_ASSERT_EQUAL_UINT32(a.hb_gap_max, b.hb_gap_max);
    TEST_ASSERT_EQUAL_UINT32(a.hb_gap_max, c.hb_gap_max);
    TEST_ASSERT_EQUAL_UINT32(a.lean_gap_max, b.lean_gap_max);
    TEST_ASSERT_EQUAL_UINT32(a.lean_gap_max, c.lean_gap_max);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_0x081_and_0x020_keep_their_cycle_whatever_derived_inputs_does);
    return UNITY_END();
}
