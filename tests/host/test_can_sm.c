/*
 * L0 mock-bus tests for the CAN state manager glue (services/can_sm, Ç1) together with
 * services/can_if on the host platform layer (D-034): in-process buses, manual ms clock.
 *
 * Two buses, each with the device under test's port and one peer node:
 *   vehicle bus  node_vehicle (CAN_PORT_VEHICLE) + node_vpeer (and the simulated ECU)
 *   platform bus node_platform (CAN_PORT_PLATFORM) + node_ppeer
 * Faults are injected on the DUT's node (vbus_set_bus_off, vbus_set_tx_stalled), the
 * state manager is stepped by hand at 1 ms resolution so every delay is exact.
 * The pure state machine (backoff, latch, N_As timer) is tested in test_can_sm_core.c;
 * here the glue's actions are checked on the bus: what goes out, and what never does.
 * Vehicle/platform IDs come from gen/ only; the generic frames use test-only IDs.
 */
#include "app/comms.h"
#include "app/host/sim_ecu.h"
#include "app/host/sim_listener.h"
#include "features/uds/uds_client.h"
#include "features/uds/uds_server.h"
#include "hal/host/can_port_host.h"
#include "hal/host/hal_time_host.h"
#include "platform.h"
#include "platform_uds.h"
#include "services/can_if.h"
#include "services/can_sm.h"
#include "services/diag.h"
#include "services/timebase.h"
#include "services/vehicle_signals.h"
#include "uds_iso14229.h"
#include "vehicle_cl250.h"

#include <string.h>
#include <unity.h>

/* Test-only identifiers (not platform or vehicle IDs). */
#define ID_A 0x100u
#define ID_B 0x101u
#define ID_UNREGISTERED 0x555u

static vbus_t bus_v, bus_p;
static uint8_t node_vehicle, node_vpeer, node_platform, node_ppeer;

typedef struct {
    uint32_t calls;
    can_frame_t last;
} sink_t;
static sink_t sink_a;
static sink_t sink_b;

static void on_frame(void* ctx, const can_frame_t* f)
{
    sink_t* s = (sink_t*)ctx;
    s->calls++;
    s->last = *f;
}

/* Before can_sm_init(): nothing may have run yet, so this is captured in main(). */
static uint8_t before_init_tec_max = 0xFFu;
static bool before_init_known = true;

static void capture_before_init(void)
{
    static vbus_t b;
    uint8_t n;
    vbus_init(&b);
    (void)vbus_attach(&b, &n);
    (void)can_port_host_bind_vbus(CAN_PORT_VEHICLE, &b, n);
    vbus_set_error_counters(&b, n, 200u, 200u);
    can_sm_step(CAN_PORT_VEHICLE); /* must be a no-op: the manager is not initialised */
    before_init_known = can_sm_state_known(CAN_PORT_VEHICLE) ||
                        can_sm_state_known(CAN_PORT_PLATFORM);
    const can_sm_stats_t* st = can_sm_stats(CAN_PORT_VEHICLE);
    if (st != NULL) {
        before_init_tec_max = st->tec_max;
    }
    can_port_host_unbind_all();
}

void setUp(void)
{
    hal_time_host_use_manual(1000u);
    memset(&sink_a, 0, sizeof sink_a);
    memset(&sink_b, 0, sizeof sink_b);
    vbus_init(&bus_v);
    vbus_init(&bus_p);
    TEST_ASSERT_TRUE(vbus_attach(&bus_v, &node_vehicle));
    TEST_ASSERT_TRUE(vbus_attach(&bus_v, &node_vpeer));
    TEST_ASSERT_TRUE(vbus_attach(&bus_p, &node_platform));
    TEST_ASSERT_TRUE(vbus_attach(&bus_p, &node_ppeer));
    TEST_ASSERT_TRUE(can_port_host_bind_vbus(CAN_PORT_VEHICLE, &bus_v, node_vehicle));
    TEST_ASSERT_TRUE(can_port_host_bind_vbus(CAN_PORT_PLATFORM, &bus_p, node_platform));
    can_if_init(); /* also resets services/can_sm */
    can_sm_step(CAN_PORT_VEHICLE); /* the first step only takes the counters' baseline */
    can_sm_step(CAN_PORT_PLATFORM);
}

void tearDown(void)
{
    can_port_host_unbind_all();
    hal_time_host_use_monotonic();
}

/* ------------------------------------------------------------------ helpers */

/* One ms of manual time, then one state-manager step of the port. */
static void tick(can_port_id_t port)
{
    hal_time_host_advance(1u);
    can_sm_step(port);
}

static void tick_n(can_port_id_t port, uint32_t ms)
{
    for (uint32_t i = 0u; i < ms; i++) {
        tick(port);
    }
}

/* The padded CL250 tester-present Single Frame (0x3E, suppress positive response). */
static can_frame_t vehicle_tester_present(void)
{
    can_frame_t f;
    memset(f.data, VEHICLE_CL250_PADDING_BYTE, sizeof f.data);
    f.id = VEHICLE_CL250_REQUEST_ID;
    f.extended = true;
    f.dlc = VEHICLE_CL250_FRAME_DLC;
    f.data[0] = 2u;
    f.data[1] = VEHICLE_CL250_TESTER_PRESENT_SID;
    f.data[2] = VEHICLE_CL250_TESTER_PRESENT_SUBFUNCTION;
    return f;
}

static can_frame_t test_frame(uint32_t id, bool ext, uint8_t first)
{
    can_frame_t f;
    memset(&f, 0, sizeof f);
    f.id = id;
    f.extended = ext;
    f.dlc = 1u;
    f.data[0] = first;
    return f;
}

static void peer_send(vbus_t* bus, uint8_t node, uint32_t id, bool ext, uint8_t first)
{
    const can_frame_t f = test_frame(id, ext, first);
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_send(bus, node, &f));
}

static uint32_t pending(const vbus_t* bus, uint8_t node)
{
    can_port_state_t st;
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_state(bus, node, &st));
    return st.tx_pending;
}

/* min(initial * 2^k, max): the wait before the k-th recovery attempt (0-based). */
static uint32_t expected_backoff(uint32_t initial, uint32_t max, uint32_t k)
{
    uint32_t b = initial;
    for (uint32_t i = 0u; i < k; i++) {
        b = (b * 2u > max) ? max : (b * 2u);
    }
    return b;
}

/* One bus-off of the DUT's node: the manager sees it, then steps once per ms until it
 * asks the port to recover. Returns the ms from the bus-off to that attempt (0: none
 * within limit_ms). The port is bus-on again afterwards. */
static uint32_t bus_off_cycle(can_port_id_t port, vbus_t* bus, uint8_t node, uint32_t limit_ms)
{
    can_sm_step(port); /* see the bus-on state left by the previous recovery */
    const uint32_t before = vbus_recover_count(bus, node);
    vbus_set_bus_off(bus, node, true);
    can_sm_step(port); /* the backoff runs from here */
    for (uint32_t i = 1u; i <= limit_ms; i++) {
        tick(port);
        if (vbus_recover_count(bus, node) != before) {
            return i;
        }
    }
    return 0u;
}

/* ------------------------------------------------------------------ before init */

static void test_step_before_can_sm_init_does_nothing(void)
{
    TEST_ASSERT_EQUAL_UINT8(0u, before_init_tec_max); /* the TEC of 200 was never read */
}

/* ------------------------------------------------------------------ bus-off */

static void test_vehicle_bus_off_blocks_writes_without_counting_a_guard_refusal(void)
{
    const can_frame_t tp = vehicle_tester_present();
    TEST_ASSERT_TRUE(can_if_tx_allowed(CAN_PORT_VEHICLE, &tp)); /* the D-020 guard passes it */
    TEST_ASSERT_TRUE(can_if_tx_free(CAN_PORT_VEHICLE));

    vbus_set_bus_off(&bus_v, node_vehicle, true);
    can_sm_step(CAN_PORT_VEHICLE);
    TEST_ASSERT_EQUAL(CAN_SM_BUS_OFF, can_sm_state(CAN_PORT_VEHICLE));
    TEST_ASSERT_FALSE(can_sm_tx_allowed(CAN_PORT_VEHICLE));
    TEST_ASSERT_FALSE(can_if_tx_free(CAN_PORT_VEHICLE));

    const uint32_t refused = can_if_tx_refused_count(CAN_PORT_VEHICLE);
    const uint32_t blocked = can_if_tx_blocked_count(CAN_PORT_VEHICLE);
    TEST_ASSERT_EQUAL(CAN_PORT_ERR_IO, can_if_write(CAN_PORT_VEHICLE, &tp));
    TEST_ASSERT_EQUAL_UINT32(blocked + 1u, can_if_tx_blocked_count(CAN_PORT_VEHICLE));
    TEST_ASSERT_EQUAL_UINT32(refused, can_if_tx_refused_count(CAN_PORT_VEHICLE));
    TEST_ASSERT_EQUAL_UINT32(0u, vbus_frame_count(&bus_v)); /* nothing reached the bus */
    can_frame_t got;
    TEST_ASSERT_EQUAL(CAN_PORT_EMPTY, vbus_recv(&bus_v, node_vpeer, &got));
    /* the platform port is not affected */
    TEST_ASSERT_TRUE(can_if_tx_free(CAN_PORT_PLATFORM));
    TEST_ASSERT_EQUAL(CAN_SM_ERROR_ACTIVE, can_sm_state(CAN_PORT_PLATFORM));
    TEST_ASSERT_EQUAL_UINT32(0u, can_if_tx_blocked_count(CAN_PORT_PLATFORM));
}

static void test_a_forbidden_frame_on_a_bus_off_port_is_still_a_guard_refusal(void)
{
    vbus_set_bus_off(&bus_v, node_vehicle, true);
    can_sm_step(CAN_PORT_VEHICLE);
    can_frame_t bad = vehicle_tester_present();
    bad.data[1] = 0x11u; /* ECUReset: refused by the guard whatever the port state */
    TEST_ASSERT_EQUAL(CAN_PORT_ERR_REFUSED, can_if_write(CAN_PORT_VEHICLE, &bad));
    TEST_ASSERT_EQUAL_UINT32(1u, can_if_tx_refused_count(CAN_PORT_VEHICLE));
    TEST_ASSERT_EQUAL_UINT32(0u, can_if_tx_blocked_count(CAN_PORT_VEHICLE));
}

static void test_vehicle_recovery_waits_for_the_initial_backoff_then_returns_to_error_active(void)
{
    vbus_set_bus_off(&bus_v, node_vehicle, true);
    can_sm_step(CAN_PORT_VEHICLE);
    tick_n(CAN_PORT_VEHICLE, VEHICLE_CL250_BUS_OFF_BACKOFF_INITIAL_MS - 1u);
    TEST_ASSERT_EQUAL_UINT32(0u, vbus_recover_count(&bus_v, node_vehicle));
    TEST_ASSERT_EQUAL(CAN_SM_BUS_OFF, can_sm_state(CAN_PORT_VEHICLE));

    tick(CAN_PORT_VEHICLE);
    TEST_ASSERT_EQUAL_UINT32(1u, vbus_recover_count(&bus_v, node_vehicle));
    TEST_ASSERT_EQUAL_UINT32(1u, can_sm_stats(CAN_PORT_VEHICLE)->recover_attempts);
    /* the port left bus-off; the manager sees it on the next step */
    can_port_state_t st;
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_state(&bus_v, node_vehicle, &st));
    TEST_ASSERT_FALSE(st.bus_off);
    TEST_ASSERT_EQUAL(CAN_SM_BUS_OFF, can_sm_state(CAN_PORT_VEHICLE));
    tick(CAN_PORT_VEHICLE);
    TEST_ASSERT_EQUAL(CAN_SM_ERROR_ACTIVE, can_sm_state(CAN_PORT_VEHICLE));
    TEST_ASSERT_TRUE(can_if_tx_free(CAN_PORT_VEHICLE));

    const can_frame_t tp = vehicle_tester_present();
    TEST_ASSERT_EQUAL(CAN_PORT_OK, can_if_write(CAN_PORT_VEHICLE, &tp));
    can_frame_t got;
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_recv(&bus_v, node_vpeer, &got));
    TEST_ASSERT_EQUAL_HEX32(VEHICLE_CL250_REQUEST_ID, got.id);
    TEST_ASSERT_EQUAL_UINT32(1u, can_sm_stats(CAN_PORT_VEHICLE)->bus_off_events);
}

static void test_vehicle_backoff_doubles_and_the_fifth_bus_off_latches(void)
{
    for (uint32_t k = 0u; k + 1u < CAN_SM_VEHICLE_BUS_OFF_LATCH; k++) {
        const uint32_t wait =
            bus_off_cycle(CAN_PORT_VEHICLE, &bus_v, node_vehicle,
                          2u * VEHICLE_CL250_BUS_OFF_BACKOFF_MAX_MS);
        TEST_ASSERT_EQUAL_UINT32(expected_backoff(VEHICLE_CL250_BUS_OFF_BACKOFF_INITIAL_MS,
                                                  VEHICLE_CL250_BUS_OFF_BACKOFF_MAX_MS, k),
                                 wait);
    }
    /* the schedule is 1000, 2000, 4000, 8000 ms for the CL250 values */
    TEST_ASSERT_EQUAL_UINT32(CAN_SM_VEHICLE_BUS_OFF_LATCH - 1u,
                             vbus_recover_count(&bus_v, node_vehicle));

    can_sm_step(CAN_PORT_VEHICLE);
    TEST_ASSERT_EQUAL(CAN_SM_ERROR_ACTIVE, can_sm_state(CAN_PORT_VEHICLE));
    vbus_set_bus_off(&bus_v, node_vehicle, true); /* the 5th bus-off */
    can_sm_step(CAN_PORT_VEHICLE);
    TEST_ASSERT_EQUAL(CAN_SM_LATCHED, can_sm_state(CAN_PORT_VEHICLE));
    TEST_ASSERT_EQUAL_UINT32(CAN_SM_VEHICLE_BUS_OFF_LATCH,
                             can_sm_stats(CAN_PORT_VEHICLE)->bus_off_events);

    tick_n(CAN_PORT_VEHICLE, 2u * VEHICLE_CL250_BUS_OFF_BACKOFF_MAX_MS);
    TEST_ASSERT_EQUAL_UINT32(CAN_SM_VEHICLE_BUS_OFF_LATCH - 1u,
                             vbus_recover_count(&bus_v, node_vehicle)); /* no further attempt */
    TEST_ASSERT_EQUAL(CAN_SM_LATCHED, can_sm_state(CAN_PORT_VEHICLE));
    TEST_ASSERT_FALSE(can_if_tx_free(CAN_PORT_VEHICLE));
    const can_frame_t tp = vehicle_tester_present();
    for (uint32_t i = 0u; i < 3u; i++) {
        TEST_ASSERT_EQUAL(CAN_PORT_ERR_IO, can_if_write(CAN_PORT_VEHICLE, &tp));
    }
    TEST_ASSERT_EQUAL_UINT32(3u, can_if_tx_blocked_count(CAN_PORT_VEHICLE));
    TEST_ASSERT_EQUAL_UINT32(0u, can_if_tx_refused_count(CAN_PORT_VEHICLE));
    TEST_ASSERT_EQUAL_UINT32(0u, vbus_frame_count(&bus_v));
}

static void test_a_latched_vehicle_port_stays_off_even_when_the_bus_is_clear(void)
{
    for (uint32_t k = 0u; k < CAN_SM_VEHICLE_BUS_OFF_LATCH - 1u; k++) {
        TEST_ASSERT_NOT_EQUAL_UINT32(0u, bus_off_cycle(CAN_PORT_VEHICLE, &bus_v, node_vehicle,
                                                       2u * VEHICLE_CL250_BUS_OFF_BACKOFF_MAX_MS));
    }
    can_sm_step(CAN_PORT_VEHICLE);
    vbus_set_bus_off(&bus_v, node_vehicle, true);
    can_sm_step(CAN_PORT_VEHICLE);
    TEST_ASSERT_EQUAL(CAN_SM_LATCHED, can_sm_state(CAN_PORT_VEHICLE));

    vbus_set_bus_off(&bus_v, node_vehicle, false); /* the controller itself is fine again */
    tick_n(CAN_PORT_VEHICLE, 3u * VEHICLE_CL250_BUS_OFF_BACKOFF_MAX_MS);
    TEST_ASSERT_EQUAL(CAN_SM_LATCHED, can_sm_state(CAN_PORT_VEHICLE));
    TEST_ASSERT_FALSE(can_if_tx_free(CAN_PORT_VEHICLE));
    const can_frame_t tp = vehicle_tester_present();
    TEST_ASSERT_EQUAL(CAN_PORT_ERR_IO, can_if_write(CAN_PORT_VEHICLE, &tp));
    TEST_ASSERT_EQUAL_UINT32(0u, vbus_frame_count(&bus_v));
}

static void test_a_latched_port_aborts_frames_that_are_still_pending(void)
{
    for (uint32_t k = 0u; k < CAN_SM_VEHICLE_BUS_OFF_LATCH - 1u; k++) {
        TEST_ASSERT_NOT_EQUAL_UINT32(0u, bus_off_cycle(CAN_PORT_VEHICLE, &bus_v, node_vehicle,
                                                       2u * VEHICLE_CL250_BUS_OFF_BACKOFF_MAX_MS));
    }
    can_sm_step(CAN_PORT_VEHICLE);
    vbus_set_tx_stalled(&bus_v, node_vehicle, true);
    const can_frame_t tp = vehicle_tester_present();
    TEST_ASSERT_EQUAL(CAN_PORT_OK, can_if_write(CAN_PORT_VEHICLE, &tp));
    TEST_ASSERT_EQUAL_UINT32(1u, pending(&bus_v, node_vehicle));
    vbus_set_bus_off(&bus_v, node_vehicle, true);
    can_sm_step(CAN_PORT_VEHICLE);
    TEST_ASSERT_EQUAL(CAN_SM_LATCHED, can_sm_state(CAN_PORT_VEHICLE));
    TEST_ASSERT_EQUAL_UINT32(0u, pending(&bus_v, node_vehicle));
    vbus_set_tx_stalled(&bus_v, node_vehicle, false);
    vbus_set_bus_off(&bus_v, node_vehicle, false);
    tick_n(CAN_PORT_VEHICLE, 10u);
    TEST_ASSERT_EQUAL_UINT32(0u, vbus_frame_count(&bus_v));
}

static void test_platform_port_never_latches_and_caps_its_backoff(void)
{
    const uint32_t cycles = 10u;
    for (uint32_t k = 0u; k < cycles; k++) {
        const uint32_t wait = bus_off_cycle(CAN_PORT_PLATFORM, &bus_p, node_platform,
                                            2u * CAN_SM_PLATFORM_BACKOFF_MAX_MS);
        TEST_ASSERT_EQUAL_UINT32(expected_backoff(CAN_SM_PLATFORM_BACKOFF_INITIAL_MS,
                                                  CAN_SM_PLATFORM_BACKOFF_MAX_MS, k),
                                 wait);
        TEST_ASSERT_NOT_EQUAL(CAN_SM_LATCHED, can_sm_state(CAN_PORT_PLATFORM));
    }
    TEST_ASSERT_EQUAL_UINT32(cycles, vbus_recover_count(&bus_p, node_platform));
    TEST_ASSERT_EQUAL_UINT32(cycles, can_sm_stats(CAN_PORT_PLATFORM)->bus_off_events);
    can_sm_step(CAN_PORT_PLATFORM);
    TEST_ASSERT_EQUAL(CAN_SM_ERROR_ACTIVE, can_sm_state(CAN_PORT_PLATFORM));
    TEST_ASSERT_TRUE(can_if_tx_free(CAN_PORT_PLATFORM));
    const can_frame_t f = test_frame(ID_A, false, 1u);
    TEST_ASSERT_EQUAL(CAN_PORT_OK, can_if_write(CAN_PORT_PLATFORM, &f));
    /* the vehicle port saw none of it */
    TEST_ASSERT_EQUAL(CAN_SM_ERROR_ACTIVE, can_sm_state(CAN_PORT_VEHICLE));
    TEST_ASSERT_EQUAL_UINT32(0u, vbus_recover_count(&bus_v, node_vehicle));
}

static void test_backoff_resets_after_the_port_stayed_bus_on_for_the_max_backoff(void)
{
    TEST_ASSERT_EQUAL_UINT32(VEHICLE_CL250_BUS_OFF_BACKOFF_INITIAL_MS,
                             bus_off_cycle(CAN_PORT_VEHICLE, &bus_v, node_vehicle, 100000u));
    can_sm_step(CAN_PORT_VEHICLE); /* bus-on: the stable period starts */
    tick_n(CAN_PORT_VEHICLE, VEHICLE_CL250_BUS_OFF_BACKOFF_MAX_MS);
    /* stable for BACKOFF_MAX_MS: the next bus-off starts over at the initial value */
    TEST_ASSERT_EQUAL_UINT32(VEHICLE_CL250_BUS_OFF_BACKOFF_INITIAL_MS,
                             bus_off_cycle(CAN_PORT_VEHICLE, &bus_v, node_vehicle, 100000u));
}

static void test_backoff_does_not_reset_before_the_max_backoff(void)
{
    TEST_ASSERT_EQUAL_UINT32(VEHICLE_CL250_BUS_OFF_BACKOFF_INITIAL_MS,
                             bus_off_cycle(CAN_PORT_VEHICLE, &bus_v, node_vehicle, 100000u));
    can_sm_step(CAN_PORT_VEHICLE);
    tick_n(CAN_PORT_VEHICLE, VEHICLE_CL250_BUS_OFF_BACKOFF_MAX_MS - 1u);
    TEST_ASSERT_EQUAL_UINT32(expected_backoff(VEHICLE_CL250_BUS_OFF_BACKOFF_INITIAL_MS,
                                              VEHICLE_CL250_BUS_OFF_BACKOFF_MAX_MS, 1u),
                             bus_off_cycle(CAN_PORT_VEHICLE, &bus_v, node_vehicle, 100000u));
}

static void test_bus_off_right_after_a_recovery_without_a_step_keeps_the_doubled_wait(void)
{
    TEST_ASSERT_EQUAL_UINT32(VEHICLE_CL250_BUS_OFF_BACKOFF_INITIAL_MS,
                             bus_off_cycle(CAN_PORT_VEHICLE, &bus_v, node_vehicle, 100000u));
    /* the controller is bus-off again before the manager saw the bus on */
    vbus_set_bus_off(&bus_v, node_vehicle, true);
    const uint32_t before = vbus_recover_count(&bus_v, node_vehicle);
    can_sm_step(CAN_PORT_VEHICLE);
    TEST_ASSERT_EQUAL(CAN_SM_BUS_OFF, can_sm_state(CAN_PORT_VEHICLE));
    uint32_t waited = 0u;
    while ((vbus_recover_count(&bus_v, node_vehicle) == before) && (waited < 100000u)) {
        tick(CAN_PORT_VEHICLE);
        waited++;
    }
    TEST_ASSERT_EQUAL_UINT32(2u * VEHICLE_CL250_BUS_OFF_BACKOFF_INITIAL_MS, waited);
    TEST_ASSERT_EQUAL_UINT32(2u, can_sm_stats(CAN_PORT_VEHICLE)->bus_off_events);
}

static void test_error_passive_port_still_sends(void)
{
    vbus_set_error_counters(&bus_p, node_platform, 128u, 0u);
    can_sm_step(CAN_PORT_PLATFORM);
    TEST_ASSERT_EQUAL(CAN_SM_ERROR_PASSIVE, can_sm_state(CAN_PORT_PLATFORM));
    TEST_ASSERT_TRUE(can_sm_tx_allowed(CAN_PORT_PLATFORM));
    TEST_ASSERT_TRUE(can_if_tx_free(CAN_PORT_PLATFORM));
    const can_frame_t f = test_frame(ID_A, false, 7u);
    TEST_ASSERT_EQUAL(CAN_PORT_OK, can_if_write(CAN_PORT_PLATFORM, &f));
    can_frame_t got;
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_recv(&bus_p, node_ppeer, &got));
    TEST_ASSERT_EQUAL_UINT8(128u, can_sm_stats(CAN_PORT_PLATFORM)->tec_max);

    vbus_set_error_counters(&bus_p, node_platform, 0u, 0u);
    can_sm_step(CAN_PORT_PLATFORM);
    TEST_ASSERT_EQUAL(CAN_SM_ERROR_ACTIVE, can_sm_state(CAN_PORT_PLATFORM));
    vbus_set_error_counters(&bus_p, node_platform, 0u, 127u); /* 127 is still active */
    can_sm_step(CAN_PORT_PLATFORM);
    TEST_ASSERT_EQUAL(CAN_SM_ERROR_ACTIVE, can_sm_state(CAN_PORT_PLATFORM));
}

/* ------------------------------------------------------------------ N_As */

static void test_n_as_abort_fires_at_the_timeout_and_the_stale_frame_never_goes_out(void)
{
    vbus_set_tx_stalled(&bus_p, node_platform, true);
    const can_frame_t f = test_frame(ID_A, false, 0x11u);
    const uint32_t aborts = can_if_tx_abort_count(CAN_PORT_PLATFORM);
    TEST_ASSERT_EQUAL(CAN_PORT_OK, can_if_write(CAN_PORT_PLATFORM, &f));
    TEST_ASSERT_EQUAL_UINT32(1u, pending(&bus_p, node_platform));
    can_sm_step(CAN_PORT_PLATFORM); /* the N_As timer starts here */

    tick_n(CAN_PORT_PLATFORM, CAN_SM_TX_TIMEOUT_MS - 1u);
    TEST_ASSERT_EQUAL_UINT32(aborts, can_if_tx_abort_count(CAN_PORT_PLATFORM));
    TEST_ASSERT_EQUAL_UINT32(1u, pending(&bus_p, node_platform));
    TEST_ASSERT_EQUAL_UINT32(0u, can_sm_stats(CAN_PORT_PLATFORM)->tx_timeouts);

    tick(CAN_PORT_PLATFORM);
    TEST_ASSERT_NOT_EQUAL_UINT32(aborts, can_if_tx_abort_count(CAN_PORT_PLATFORM));
    TEST_ASSERT_EQUAL_UINT32(1u, can_sm_stats(CAN_PORT_PLATFORM)->tx_timeouts);
    TEST_ASSERT_EQUAL_UINT32(0u, pending(&bus_p, node_platform));
    TEST_ASSERT_EQUAL(CAN_SM_ERROR_ACTIVE, can_sm_state(CAN_PORT_PLATFORM)); /* not a bus-off */

    vbus_set_tx_stalled(&bus_p, node_platform, false);
    can_frame_t got;
    TEST_ASSERT_EQUAL(CAN_PORT_EMPTY, vbus_recv(&bus_p, node_ppeer, &got)); /* never goes out */
    TEST_ASSERT_EQUAL_UINT32(0u, vbus_frame_count(&bus_p));
    tick_n(CAN_PORT_PLATFORM, 10u);
    TEST_ASSERT_EQUAL(CAN_PORT_EMPTY, vbus_recv(&bus_p, node_ppeer, &got));
}

static void test_without_the_state_manager_an_unstalled_port_sends_the_stale_frame_late(void)
{
    /* The hazard N_As removes: no can_sm_step(), so nothing aborts the stuck frame. */
    vbus_set_tx_stalled(&bus_p, node_platform, true);
    const can_frame_t f = test_frame(ID_A, false, 0x22u);
    TEST_ASSERT_EQUAL(CAN_PORT_OK, can_if_write(CAN_PORT_PLATFORM, &f));
    hal_time_host_advance(5u * CAN_SM_TX_TIMEOUT_MS);
    vbus_set_tx_stalled(&bus_p, node_platform, false);
    can_frame_t got;
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_recv(&bus_p, node_ppeer, &got));
    TEST_ASSERT_EQUAL_UINT32(ID_A, got.id);
    TEST_ASSERT_EQUAL_UINT8(0x22u, got.data[0]);
}

static void test_a_frame_that_is_sent_within_n_as_is_not_aborted(void)
{
    vbus_set_tx_stalled(&bus_p, node_platform, true);
    const can_frame_t f = test_frame(ID_A, false, 0x33u);
    TEST_ASSERT_EQUAL(CAN_PORT_OK, can_if_write(CAN_PORT_PLATFORM, &f));
    can_sm_step(CAN_PORT_PLATFORM);
    tick_n(CAN_PORT_PLATFORM, CAN_SM_TX_TIMEOUT_MS / 2u);
    vbus_set_tx_stalled(&bus_p, node_platform, false); /* the ACK finally comes */
    can_frame_t got;
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_recv(&bus_p, node_ppeer, &got));
    tick_n(CAN_PORT_PLATFORM, 2u * CAN_SM_TX_TIMEOUT_MS);
    TEST_ASSERT_EQUAL_UINT32(0u, can_sm_stats(CAN_PORT_PLATFORM)->tx_timeouts);
    TEST_ASSERT_EQUAL_UINT32(0u, can_if_tx_abort_count(CAN_PORT_PLATFORM));
}

static void test_bus_off_aborts_the_pending_tx_and_recovery_does_not_deliver_it(void)
{
    vbus_set_tx_stalled(&bus_p, node_platform, true);
    const can_frame_t stale = test_frame(ID_A, false, 0x44u);
    TEST_ASSERT_EQUAL(CAN_PORT_OK, can_if_write(CAN_PORT_PLATFORM, &stale));
    TEST_ASSERT_EQUAL_UINT32(1u, pending(&bus_p, node_platform));
    can_sm_step(CAN_PORT_PLATFORM);
    const uint32_t aborts = can_if_tx_abort_count(CAN_PORT_PLATFORM);

    vbus_set_bus_off(&bus_p, node_platform, true);
    can_sm_step(CAN_PORT_PLATFORM);
    TEST_ASSERT_NOT_EQUAL_UINT32(aborts, can_if_tx_abort_count(CAN_PORT_PLATFORM));
    TEST_ASSERT_EQUAL_UINT32(0u, pending(&bus_p, node_platform));
    TEST_ASSERT_EQUAL_UINT32(0u, can_sm_stats(CAN_PORT_PLATFORM)->tx_timeouts); /* not N_As */

    vbus_set_tx_stalled(&bus_p, node_platform, false); /* the stall ends during the bus-off */
    tick_n(CAN_PORT_PLATFORM, CAN_SM_PLATFORM_BACKOFF_INITIAL_MS);
    TEST_ASSERT_EQUAL_UINT32(1u, vbus_recover_count(&bus_p, node_platform));
    tick(CAN_PORT_PLATFORM);
    TEST_ASSERT_EQUAL(CAN_SM_ERROR_ACTIVE, can_sm_state(CAN_PORT_PLATFORM));
    can_frame_t got;
    TEST_ASSERT_EQUAL(CAN_PORT_EMPTY, vbus_recv(&bus_p, node_ppeer, &got)); /* the old one is gone */

    const can_frame_t fresh = test_frame(ID_B, false, 0x55u);
    TEST_ASSERT_EQUAL(CAN_PORT_OK, can_if_write(CAN_PORT_PLATFORM, &fresh));
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_recv(&bus_p, node_ppeer, &got));
    TEST_ASSERT_EQUAL_UINT32(ID_B, got.id);
    TEST_ASSERT_EQUAL(CAN_PORT_EMPTY, vbus_recv(&bus_p, node_ppeer, &got));
}

/* ------------------------------------------------------- state unknown (safety m1) */

static void test_a_port_whose_state_cannot_be_read_refuses_tx_and_drops_its_pending_frames(void)
{
    vbus_set_tx_stalled(&bus_p, node_platform, true);
    const can_frame_t f = test_frame(ID_A, false, 1u);
    TEST_ASSERT_EQUAL(CAN_PORT_OK, can_if_write(CAN_PORT_PLATFORM, &f));
    can_sm_step(CAN_PORT_PLATFORM);
    const uint32_t seq = can_if_tx_abort_count(CAN_PORT_PLATFORM);

    can_port_host_unbind(CAN_PORT_PLATFORM); /* get_state now fails */
    can_sm_step(CAN_PORT_PLATFORM);
    TEST_ASSERT_FALSE(can_sm_tx_allowed(CAN_PORT_PLATFORM));
    TEST_ASSERT_NOT_EQUAL(seq, can_if_tx_abort_count(CAN_PORT_PLATFORM));
    const uint32_t seq2 = can_if_tx_abort_count(CAN_PORT_PLATFORM);
    can_sm_step(CAN_PORT_PLATFORM); /* aborted once, not every step */
    TEST_ASSERT_EQUAL_UINT32(seq2, can_if_tx_abort_count(CAN_PORT_PLATFORM));

    TEST_ASSERT_TRUE(can_port_host_bind_vbus(CAN_PORT_PLATFORM, &bus_p, node_platform));
    can_sm_step(CAN_PORT_PLATFORM); /* a good snapshot again */
    TEST_ASSERT_TRUE(can_sm_tx_allowed(CAN_PORT_PLATFORM));
}

static void test_the_platform_filters_take_a_full_receiver_table(void)
{
    can_if_init();
    for (uint32_t i = 0u; i < CAN_IF_MAX_RECEIVERS; i++) {
        TEST_ASSERT_EQUAL(CAN_IF_OK, can_if_register_rx(CAN_PORT_PLATFORM, ID_A + i, false,
                                                        on_frame, &sink_a));
    }
    TEST_ASSERT_EQUAL(CAN_PORT_OK, can_if_apply_filters(CAN_PORT_PLATFORM));
    TEST_ASSERT_EQUAL_UINT32(CAN_IF_MAX_RECEIVERS, bus_p.nodes[node_platform].filter_count);
}

/* ------------------------------------------------------------------ unbound, unknown */

static void test_unbound_ports_and_unknown_port_ids_are_safe(void)
{
    can_port_host_unbind_all();
    can_sm_step(CAN_PORT_PLATFORM); /* no state to read: TX refused (fail-closed) */
    can_sm_step(CAN_PORT_VEHICLE);
    TEST_ASSERT_EQUAL(CAN_SM_ERROR_ACTIVE, can_sm_state(CAN_PORT_PLATFORM));
    TEST_ASSERT_FALSE(can_sm_tx_allowed(CAN_PORT_PLATFORM));
    TEST_ASSERT_FALSE(can_if_tx_free(CAN_PORT_PLATFORM)); /* nothing to send on */
    const can_frame_t f = test_frame(ID_A, false, 1u);
    TEST_ASSERT_EQUAL(CAN_PORT_ERR_IO, can_if_write(CAN_PORT_PLATFORM, &f));

    can_sm_step(CAN_PORT_COUNT); /* unknown port: no-op */
    TEST_ASSERT_EQUAL(CAN_SM_LATCHED, can_sm_state(CAN_PORT_COUNT));
    TEST_ASSERT_FALSE(can_sm_tx_allowed(CAN_PORT_COUNT));
    TEST_ASSERT_EQUAL_UINT32(0u, can_sm_abort_seq(CAN_PORT_COUNT));
    TEST_ASSERT_NULL(can_sm_stats(CAN_PORT_COUNT));
    TEST_ASSERT_FALSE(can_if_tx_free(CAN_PORT_COUNT));
    TEST_ASSERT_EQUAL_UINT32(0u, can_if_tx_abort_count(CAN_PORT_COUNT));
    TEST_ASSERT_EQUAL_UINT32(0u, can_if_tx_blocked_count(CAN_PORT_COUNT));
    TEST_ASSERT_EQUAL(CAN_PORT_ERR_ARG, can_if_write(CAN_PORT_COUNT, &f));
}

/* ------------------------------------------------------------------ filters */

static const can_port_filter_t* find_filter(const vbus_t* bus, uint8_t node, uint32_t id,
                                            bool ext)
{
    const vbus_node_t* n = &bus->nodes[node];
    for (uint32_t i = 0u; i < n->filter_count; i++) {
        if ((n->filters[i].id == id) && (n->filters[i].extended == ext)) {
            return &n->filters[i];
        }
    }
    return NULL;
}

static void test_applied_filters_pass_only_registered_ids_and_unmatched_ones_are_not_unrouted(void)
{
    TEST_ASSERT_EQUAL(CAN_IF_OK,
                      can_if_register_rx(CAN_PORT_PLATFORM, ID_A, false, on_frame, &sink_a));
    /* before the filters: an unregistered ID reaches can_if and is counted as unrouted */
    peer_send(&bus_p, node_ppeer, ID_UNREGISTERED, false, 1u);
    TEST_ASSERT_EQUAL_UINT32(1u, can_if_dispatch(CAN_PORT_PLATFORM, 10u));
    TEST_ASSERT_EQUAL_UINT32(1u, can_if_unrouted_count(CAN_PORT_PLATFORM));

    TEST_ASSERT_EQUAL(CAN_PORT_OK, can_if_apply_filters(CAN_PORT_PLATFORM));
    peer_send(&bus_p, node_ppeer, ID_A, false, 2u);
    peer_send(&bus_p, node_ppeer, ID_UNREGISTERED, false, 3u); /* dropped by the node filter */
    peer_send(&bus_p, node_ppeer, ID_A, true, 4u);             /* same number, other format */
    TEST_ASSERT_EQUAL_UINT32(1u, can_if_dispatch(CAN_PORT_PLATFORM, 10u));
    TEST_ASSERT_EQUAL_UINT32(1u, sink_a.calls);
    TEST_ASSERT_EQUAL_UINT8(2u, sink_a.last.data[0]);
    TEST_ASSERT_EQUAL_UINT32(1u, can_if_unrouted_count(CAN_PORT_PLATFORM)); /* unchanged */
    TEST_ASSERT_EQUAL_UINT32(0u, vbus_overruns(&bus_p, node_platform));
}

static void test_filters_of_one_port_do_not_touch_the_other_port(void)
{
    TEST_ASSERT_EQUAL(CAN_IF_OK,
                      can_if_register_rx(CAN_PORT_PLATFORM, ID_A, false, on_frame, &sink_a));
    TEST_ASSERT_EQUAL(CAN_PORT_OK, can_if_apply_filters(CAN_PORT_PLATFORM));
    TEST_ASSERT_TRUE(bus_p.nodes[node_platform].filtered);
    TEST_ASSERT_FALSE(bus_v.nodes[node_vehicle].filtered);
    /* the vehicle port is not sealed: it still takes a receiver, and then its own filters */
    TEST_ASSERT_EQUAL(CAN_IF_OK,
                      can_if_register_rx(CAN_PORT_VEHICLE, ID_B, false, on_frame, &sink_b));
    TEST_ASSERT_EQUAL(CAN_IF_ERR_SEALED,
                      can_if_register_rx(CAN_PORT_PLATFORM, ID_B, false, on_frame, &sink_b));
}

static void test_filter_fifo_is_1_for_the_platform_diagnostic_ids_and_0_for_everything_else(void)
{
    /* platform port */
    TEST_ASSERT_EQUAL(CAN_IF_OK, can_if_register_rx(CAN_PORT_PLATFORM, PLATFORM_UDS_PHYS_REQUEST_ID,
                                                    false, on_frame, &sink_a));
    TEST_ASSERT_EQUAL(CAN_IF_OK,
                      can_if_register_rx(CAN_PORT_PLATFORM, PLATFORM_UDS_FUNCTIONAL_REQUEST_ID,
                                         false, on_frame, &sink_a));
    TEST_ASSERT_EQUAL(CAN_IF_OK, can_if_register_rx(CAN_PORT_PLATFORM, PLATFORM_UDS_PHYS_RESPONSE_ID,
                                                    false, on_frame, &sink_a));
    TEST_ASSERT_EQUAL(CAN_IF_OK,
                      can_if_register_rx(CAN_PORT_PLATFORM, ID_A, false, on_frame, &sink_b));
    /* the 29-bit frame with the same number is not a diagnostic ID */
    TEST_ASSERT_EQUAL(CAN_IF_OK, can_if_register_rx(CAN_PORT_PLATFORM, PLATFORM_UDS_PHYS_REQUEST_ID,
                                                    true, on_frame, &sink_b));
    /* vehicle port: even the same numbers stay in FIFO0 */
    TEST_ASSERT_EQUAL(CAN_IF_OK, can_if_register_rx(CAN_PORT_VEHICLE, VEHICLE_CL250_RESPONSE_ID,
                                                    true, on_frame, &sink_a));
    TEST_ASSERT_EQUAL(CAN_IF_OK, can_if_register_rx(CAN_PORT_VEHICLE,
                                                    VEHICLE_CL250_FALLBACK_REQUEST_ID, false,
                                                    on_frame, &sink_a));
    TEST_ASSERT_EQUAL(CAN_IF_OK, can_if_register_rx(CAN_PORT_VEHICLE, PLATFORM_UDS_PHYS_REQUEST_ID,
                                                    false, on_frame, &sink_b));
    TEST_ASSERT_EQUAL(CAN_PORT_OK, can_if_apply_filters(CAN_PORT_PLATFORM));
    TEST_ASSERT_EQUAL(CAN_PORT_OK, can_if_apply_filters(CAN_PORT_VEHICLE));

    TEST_ASSERT_EQUAL_UINT32(5u, bus_p.nodes[node_platform].filter_count);
    const can_port_filter_t* f = find_filter(&bus_p, node_platform, PLATFORM_UDS_PHYS_REQUEST_ID, false);
    TEST_ASSERT_NOT_NULL(f);
    TEST_ASSERT_EQUAL(CAN_PORT_FIFO1, f->fifo);
    f = find_filter(&bus_p, node_platform, PLATFORM_UDS_FUNCTIONAL_REQUEST_ID, false);
    TEST_ASSERT_NOT_NULL(f);
    TEST_ASSERT_EQUAL(CAN_PORT_FIFO1, f->fifo);
    f = find_filter(&bus_p, node_platform, PLATFORM_UDS_PHYS_RESPONSE_ID, false);
    TEST_ASSERT_NOT_NULL(f);
    TEST_ASSERT_EQUAL(CAN_PORT_FIFO1, f->fifo);
    f = find_filter(&bus_p, node_platform, ID_A, false);
    TEST_ASSERT_NOT_NULL(f);
    TEST_ASSERT_EQUAL(CAN_PORT_FIFO0, f->fifo);
    f = find_filter(&bus_p, node_platform, PLATFORM_UDS_PHYS_REQUEST_ID, true);
    TEST_ASSERT_NOT_NULL(f);
    TEST_ASSERT_EQUAL(CAN_PORT_FIFO0, f->fifo);

    TEST_ASSERT_EQUAL_UINT32(3u, bus_v.nodes[node_vehicle].filter_count);
    for (uint32_t i = 0u; i < bus_v.nodes[node_vehicle].filter_count; i++) {
        TEST_ASSERT_EQUAL(CAN_PORT_FIFO0, bus_v.nodes[node_vehicle].filters[i].fifo);
    }
    TEST_ASSERT_NOT_NULL(find_filter(&bus_v, node_vehicle, VEHICLE_CL250_RESPONSE_ID, true));
}

static void test_registering_after_the_filters_is_sealed_and_init_clears_the_seal(void)
{
    TEST_ASSERT_EQUAL(CAN_IF_OK,
                      can_if_register_rx(CAN_PORT_PLATFORM, ID_A, false, on_frame, &sink_a));
    TEST_ASSERT_EQUAL(CAN_PORT_OK, can_if_apply_filters(CAN_PORT_PLATFORM));
    TEST_ASSERT_EQUAL(CAN_IF_ERR_SEALED,
                      can_if_register_rx(CAN_PORT_PLATFORM, ID_B, false, on_frame, &sink_b));
    /* a rejected registration changes nothing: ID_B still does not reach the dispatch */
    peer_send(&bus_p, node_ppeer, ID_B, false, 9u);
    TEST_ASSERT_EQUAL_UINT32(0u, can_if_dispatch(CAN_PORT_PLATFORM, 10u));
    TEST_ASSERT_EQUAL_UINT32(0u, sink_b.calls);

    can_if_init();
    TEST_ASSERT_EQUAL(CAN_IF_OK,
                      can_if_register_rx(CAN_PORT_PLATFORM, ID_B, false, on_frame, &sink_b));
}

static void test_apply_filters_on_an_unbound_or_unknown_port_reports_it_and_does_not_seal(void)
{
    TEST_ASSERT_EQUAL(CAN_IF_OK,
                      can_if_register_rx(CAN_PORT_PLATFORM, ID_A, false, on_frame, &sink_a));
    TEST_ASSERT_EQUAL(CAN_PORT_ERR_ARG, can_if_apply_filters(CAN_PORT_COUNT));
    can_port_host_unbind(CAN_PORT_PLATFORM);
    TEST_ASSERT_EQUAL(CAN_PORT_ERR_CLOSED, can_if_apply_filters(CAN_PORT_PLATFORM));
    TEST_ASSERT_EQUAL(CAN_IF_OK,
                      can_if_register_rx(CAN_PORT_PLATFORM, ID_B, false, on_frame, &sink_b));
    TEST_ASSERT_TRUE(can_port_host_bind_vbus(CAN_PORT_PLATFORM, &bus_p, node_platform));
    TEST_ASSERT_EQUAL(CAN_PORT_OK, can_if_apply_filters(CAN_PORT_PLATFORM));
    TEST_ASSERT_EQUAL_UINT32(2u, bus_p.nodes[node_platform].filter_count);
}

/* ------------------------------------------------------------------ comms */

static uds_client_t client;
static uds_server_t server;
static vehicle_republish_t republisher;
static heartbeat_t heartbeat;
static comms_ekf_t ekf; /* zeroed: no EKF registered */
static sim_ecu_t ecu;

static void test_comms_applies_the_filters_and_the_sil_exchange_still_works(void)
{
    diag_init(timebase_now_ms());
    vehicle_signals_init();
    TEST_ASSERT_TRUE(sim_ecu_init(&ecu, &bus_v));
    ecu.require_session = true;
    memset(&client, 0, sizeof client);
    memset(&server, 0, sizeof server);
    TEST_ASSERT_EQUAL(ISOTP_OK, uds_client_open(&client));
    TEST_ASSERT_EQUAL(ISOTP_OK, uds_server_open(&server));
    TEST_ASSERT_TRUE(vehicle_republish_open(&republisher));
    TEST_ASSERT_TRUE(heartbeat_open(&heartbeat));
    TEST_ASSERT_TRUE(comms_apply_filters());
    TEST_ASSERT_TRUE(bus_v.nodes[node_vehicle].filtered);
    TEST_ASSERT_TRUE(bus_p.nodes[node_platform].filtered);
    /* both ports are sealed */
    TEST_ASSERT_EQUAL(CAN_IF_ERR_SEALED,
                      can_if_register_rx(CAN_PORT_VEHICLE, ID_A, false, on_frame, &sink_a));
    TEST_ASSERT_EQUAL(CAN_IF_ERR_SEALED,
                      can_if_register_rx(CAN_PORT_PLATFORM, ID_A, false, on_frame, &sink_a));
    /* the server's IDs are in FIFO1 of the platform node */
    const can_port_filter_t* f =
        find_filter(&bus_p, node_platform, PLATFORM_UDS_PHYS_REQUEST_ID, false);
    TEST_ASSERT_NOT_NULL(f);
    TEST_ASSERT_EQUAL(CAN_PORT_FIFO1, f->fifo);

    /* a platform tester asks for a tester present while the client polls the ECU */
    can_frame_t req;
    memset(req.data, PLATFORM_UDS_PADDING_BYTE, sizeof req.data);
    req.id = PLATFORM_UDS_PHYS_REQUEST_ID;
    req.extended = false;
    req.dlc = PLATFORM_UDS_FRAME_DLC;
    req.data[0] = 2u;
    req.data[1] = UDS_SID_TESTER_PRESENT;
    req.data[2] = UDS_TESTER_PRESENT_ZERO_SUBFUNCTION;
    peer_send(&bus_p, node_ppeer, ID_UNREGISTERED, false, 1u); /* dropped by the filter */
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_send(&bus_p, node_ppeer, &req));

    bool answered = false;
    uint32_t speed_frames = 0u;
    uint32_t engine_frames = 0u;
    uint32_t heartbeat_frames = 0u;
    for (uint32_t i = 0u; i < 300u; i++) {
        sim_ecu_step(&ecu, timebase_now_ms());
        comms_pass(&client, &server, &republisher, &heartbeat, &ekf);
        can_frame_t got;
        while (vbus_recv(&bus_p, node_ppeer, &got) == CAN_PORT_OK) {
            speed_frames += (got.id == PLATFORM_VEHICLE_SPEED_FRAME_ID) ? 1u : 0u;
            engine_frames += (got.id == PLATFORM_VEHICLE_ENGINE_FRAME_ID) ? 1u : 0u;
            heartbeat_frames += (got.id == PLATFORM_HEARTBEAT_RT_CORE_FRAME_ID) ? 1u : 0u;
            if ((got.id == PLATFORM_UDS_PHYS_RESPONSE_ID) &&
                (got.data[1] == (uint8_t)(UDS_SID_TESTER_PRESENT + UDS_POSITIVE_RESPONSE_OFFSET))) {
                answered = true;
            }
        }
        hal_time_host_advance(1u);
    }
    TEST_ASSERT_TRUE(answered);
    /* the republisher's frames share the pass: 300 ms at the gen/ cycle times */
    TEST_ASSERT_EQUAL_UINT32(300u / PLATFORM_VEHICLE_SPEED_CYCLE_TIME_MS, speed_frames);
    TEST_ASSERT_EQUAL_UINT32(300u / PLATFORM_VEHICLE_ENGINE_CYCLE_TIME_MS, engine_frames);
    TEST_ASSERT_EQUAL_UINT32(300u / PLATFORM_HEARTBEAT_RT_CORE_CYCLE_TIME_MS, heartbeat_frames);
    TEST_ASSERT_GREATER_THAN_UINT32(0u, uds_client_stats(&client)->reads_ok);
    TEST_ASSERT_TRUE(uds_client_session_up(&client));
    TEST_ASSERT_FALSE(uds_client_failed(&client));
    TEST_ASSERT_EQUAL_UINT32(0u, can_if_unrouted_count(CAN_PORT_VEHICLE));
    TEST_ASSERT_EQUAL_UINT32(0u, can_if_unrouted_count(CAN_PORT_PLATFORM));
    TEST_ASSERT_EQUAL_UINT32(0u, can_if_tx_refused_count(CAN_PORT_VEHICLE));
    TEST_ASSERT_EQUAL(CAN_SM_ERROR_ACTIVE, can_sm_state(CAN_PORT_VEHICLE));
    TEST_ASSERT_EQUAL(CAN_SM_ERROR_ACTIVE, can_sm_state(CAN_PORT_PLATFORM));
}

/* D-064 boot window (safety-reviewer MINOR-1): with the ECU silent from boot, U0100-00 arms
 * only VEHICLE_CL250_ECU_ABSENT_TIMEOUT_MS after the client opened, so the frames between
 * the first (INIT) and that point read NORMAL. Pinned here as the decided behaviour. */
static void test_comms_heartbeat_reads_normal_until_a_silent_ecu_arms_u0100(void)
{
    diag_init(timebase_now_ms());
    vehicle_signals_init();
    memset(&client, 0, sizeof client);
    memset(&server, 0, sizeof server);
    TEST_ASSERT_EQUAL(ISOTP_OK, uds_client_open(&client));
    TEST_ASSERT_EQUAL(ISOTP_OK, uds_server_open(&server));
    TEST_ASSERT_TRUE(vehicle_republish_open(&republisher));
    TEST_ASSERT_TRUE(heartbeat_open(&heartbeat));
    TEST_ASSERT_TRUE(comms_apply_filters());
    const uint32_t open_ms = timebase_now_ms();

    uint32_t frames = 0u;
    uint32_t init = 0u;
    uint32_t normal = 0u;
    bool degraded = false;
    uint32_t degraded_at_ms = 0u;
    uint8_t error_count = 0u;
    const uint32_t run_ms =
        VEHICLE_CL250_ECU_ABSENT_TIMEOUT_MS + (3u * PLATFORM_HEARTBEAT_RT_CORE_CYCLE_TIME_MS);
    for (uint32_t i = 0u; i < run_ms; i++) {
        comms_pass(&client, &server, &republisher, &heartbeat, &ekf);
        can_frame_t got;
        while (vbus_recv(&bus_p, node_ppeer, &got) == CAN_PORT_OK) {
            if (got.id != PLATFORM_HEARTBEAT_RT_CORE_FRAME_ID) {
                continue;
            }
            struct platform_heartbeat_rt_core_t m;
            sim_listener_heartbeat_decode(got.data, &m);
            frames++;
            if (m.node_mode == PLATFORM_HEARTBEAT_RT_CORE_NODE_MODE_INIT_CHOICE) {
                TEST_ASSERT_EQUAL_UINT32_MESSAGE(1u, frames, "INIT only on the first frame");
                init++;
            } else if (!degraded) {
                if (m.node_mode == PLATFORM_HEARTBEAT_RT_CORE_NODE_MODE_DEGRADED_CHOICE) {
                    degraded = true;
                    degraded_at_ms = timebase_now_ms() - open_ms;
                    error_count = m.error_count;
                } else {
                    TEST_ASSERT_EQUAL_UINT8(PLATFORM_HEARTBEAT_RT_CORE_NODE_MODE_NORMAL_CHOICE,
                                            m.node_mode);
                    TEST_ASSERT_EQUAL_UINT8(0u, m.error_count);
                    normal++;
                }
            } else {
                TEST_ASSERT_EQUAL_UINT8(PLATFORM_HEARTBEAT_RT_CORE_NODE_MODE_DEGRADED_CHOICE,
                                        m.node_mode);
                TEST_ASSERT_EQUAL_UINT8(error_count, m.error_count); /* one onset, no flapping */
            }
        }
        hal_time_host_advance(1u);
    }
    TEST_ASSERT_EQUAL_UINT32(1u, init);
    TEST_ASSERT_GREATER_THAN_UINT32(0u, normal);
    TEST_ASSERT_TRUE(degraded);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(VEHICLE_CL250_ECU_ABSENT_TIMEOUT_MS, degraded_at_ms);
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(
        VEHICLE_CL250_ECU_ABSENT_TIMEOUT_MS + PLATFORM_HEARTBEAT_RT_CORE_CYCLE_TIME_MS, degraded_at_ms);
    TEST_ASSERT_EQUAL_UINT8(1u, error_count);
}

/* comms_pass() without uds_client_step(): the client task stalled, the rest runs on. */
static void comms_pass_without_client(void)
{
    can_sm_step(CAN_PORT_VEHICLE);
    (void)can_if_dispatch(CAN_PORT_VEHICLE, COMMS_RX_PER_PASS);
    can_sm_step(CAN_PORT_PLATFORM);
    (void)can_if_dispatch(CAN_PORT_PLATFORM, COMMS_RX_PER_PASS);
    vehicle_republish_step(&republisher);
    heartbeat_step(&heartbeat, false);
    uds_server_step(&server);
}

/* Runs ms passes and returns the last 0x081 seen; counts its NODE_MODEs. */
static struct platform_heartbeat_rt_core_t run_comms(uint32_t ms, bool client_runs,
                                                     uint32_t* normal, uint32_t* degraded)
{
    struct platform_heartbeat_rt_core_t last;
    memset(&last, 0, sizeof last);
    for (uint32_t i = 0u; i < ms; i++) {
        sim_ecu_step(&ecu, timebase_now_ms());
        if (client_runs) {
            comms_pass(&client, &server, &republisher, &heartbeat, &ekf);
        } else {
            comms_pass_without_client();
        }
        can_frame_t got;
        while (vbus_recv(&bus_p, node_ppeer, &got) == CAN_PORT_OK) {
            if (got.id == PLATFORM_HEARTBEAT_RT_CORE_FRAME_ID) {
                sim_listener_heartbeat_decode(got.data, &last);
                *normal += (last.node_mode == PLATFORM_HEARTBEAT_RT_CORE_NODE_MODE_NORMAL_CHOICE) ? 1u : 0u;
                *degraded +=
                    (last.node_mode == PLATFORM_HEARTBEAT_RT_CORE_NODE_MODE_DEGRADED_CHOICE) ? 1u : 0u;
            }
        }
        hal_time_host_advance(1u);
    }
    return last;
}

/* U3000-00 has two writers: the client glue (status every step) and the server's
 * supervision (stale status). Each client stall past the max age is exactly one onset;
 * the writers never make ERROR_COUNT oscillate (safety-reviewer, D-064 item 2). */
static void test_comms_each_client_stall_is_one_onset_and_the_writers_never_flap(void)
{
    diag_init(timebase_now_ms());
    vehicle_signals_init();
    TEST_ASSERT_TRUE(sim_ecu_init(&ecu, &bus_v));
    ecu.require_session = true;
    memset(&client, 0, sizeof client);
    memset(&server, 0, sizeof server);
    TEST_ASSERT_EQUAL(ISOTP_OK, uds_client_open(&client));
    TEST_ASSERT_EQUAL(ISOTP_OK, uds_server_open(&server));
    TEST_ASSERT_TRUE(vehicle_republish_open(&republisher));
    TEST_ASSERT_TRUE(heartbeat_open(&heartbeat));
    TEST_ASSERT_TRUE(comms_apply_filters());

    uint32_t normal = 0u;
    uint32_t degraded = 0u;
    struct platform_heartbeat_rt_core_t m = run_comms(1000u, true, &normal, &degraded);
    TEST_ASSERT_EQUAL_UINT8(PLATFORM_HEARTBEAT_RT_CORE_NODE_MODE_NORMAL_CHOICE, m.node_mode);
    TEST_ASSERT_EQUAL_UINT8(0u, m.error_count);
    TEST_ASSERT_EQUAL_UINT32(0u, degraded);

    const uint32_t stalls = 3u;
    for (uint32_t k = 1u; k <= stalls; k++) {
        normal = 0u;
        degraded = 0u;
        m = run_comms(PLATFORM_UDS_VEHICLE_TESTER_STATUS_MAX_AGE_MS + 100u, false, &normal,
                      &degraded);
        TEST_ASSERT_EQUAL_UINT8(PLATFORM_HEARTBEAT_RT_CORE_NODE_MODE_DEGRADED_CHOICE, m.node_mode);
        TEST_ASSERT_EQUAL_UINT8((uint8_t)k, m.error_count);
        normal = 0u;
        degraded = 0u;
        m = run_comms(1000u, true, &normal, &degraded);
        TEST_ASSERT_EQUAL_UINT8(PLATFORM_HEARTBEAT_RT_CORE_NODE_MODE_NORMAL_CHOICE, m.node_mode);
        TEST_ASSERT_EQUAL_UINT8((uint8_t)k, m.error_count); /* no onset from the resume */
        TEST_ASSERT_LESS_OR_EQUAL_UINT32(1u, degraded);       /* at most the frame before it */
    }
    TEST_ASSERT_FALSE(uds_client_failed(&client));
}

/* ------------------------------------------------ state known (D-055, health DID) */

static void test_no_state_is_known_before_init_or_before_the_first_snapshot(void)
{
    TEST_ASSERT_FALSE(before_init_known); /* can_sm_state() says LATCHED there */
    can_if_init();                        /* reset, no step yet */
    TEST_ASSERT_FALSE(can_sm_state_known(CAN_PORT_VEHICLE));
    TEST_ASSERT_FALSE(can_sm_state_known(CAN_PORT_PLATFORM));
    can_sm_step(CAN_PORT_VEHICLE);
    TEST_ASSERT_TRUE(can_sm_state_known(CAN_PORT_VEHICLE));
    TEST_ASSERT_FALSE(can_sm_state_known(CAN_PORT_PLATFORM)); /* per port */
    TEST_ASSERT_FALSE(can_sm_state_known(CAN_PORT_COUNT));
}

static void test_an_unreadable_controller_is_unknown_until_a_good_snapshot(void)
{
    TEST_ASSERT_TRUE(can_sm_state_known(CAN_PORT_PLATFORM));
    can_port_host_unbind(CAN_PORT_PLATFORM);
    can_sm_step(CAN_PORT_PLATFORM);
    TEST_ASSERT_EQUAL(CAN_SM_ERROR_ACTIVE, can_sm_state(CAN_PORT_PLATFORM)); /* the last one */
    TEST_ASSERT_FALSE(can_sm_state_known(CAN_PORT_PLATFORM));
    TEST_ASSERT_TRUE(can_port_host_bind_vbus(CAN_PORT_PLATFORM, &bus_p, node_platform));
    can_sm_step(CAN_PORT_PLATFORM);
    TEST_ASSERT_TRUE(can_sm_state_known(CAN_PORT_PLATFORM));
}

static void test_a_latched_port_stays_known_when_its_controller_cannot_be_read(void)
{
    for (uint32_t k = 0u; k < CAN_SM_VEHICLE_BUS_OFF_LATCH - 1u; k++) {
        TEST_ASSERT_NOT_EQUAL_UINT32(0u, bus_off_cycle(CAN_PORT_VEHICLE, &bus_v, node_vehicle,
                                                       2u * VEHICLE_CL250_BUS_OFF_BACKOFF_MAX_MS));
    }
    can_sm_step(CAN_PORT_VEHICLE);
    vbus_set_bus_off(&bus_v, node_vehicle, true);
    can_sm_step(CAN_PORT_VEHICLE);
    TEST_ASSERT_EQUAL(CAN_SM_LATCHED, can_sm_state(CAN_PORT_VEHICLE));
    can_port_host_unbind(CAN_PORT_VEHICLE);
    can_sm_step(CAN_PORT_VEHICLE);
    TEST_ASSERT_TRUE(can_sm_state_known(CAN_PORT_VEHICLE)); /* the latch is our own state */
    TEST_ASSERT_EQUAL(CAN_SM_LATCHED, can_sm_state(CAN_PORT_VEHICLE));
}

/* ------------------------------------------- dedicated TX buffers (D-056, PR-A review) */

/* ID_A gets a dedicated replace-on-new buffer on the platform port. */
static void use_dedicated_a(void)
{
    can_if_init();
    TEST_ASSERT_EQUAL(CAN_IF_OK, can_if_register_tx_dedicated(CAN_PORT_PLATFORM, ID_A, false));
    TEST_ASSERT_EQUAL(CAN_PORT_OK, can_if_apply_filters(CAN_PORT_PLATFORM));
    can_sm_step(CAN_PORT_PLATFORM);
}

static void test_a_dedicated_frame_nobody_replaces_is_aborted_by_n_as(void)
{
    use_dedicated_a();
    vbus_set_tx_stalled(&bus_p, node_platform, true);
    const can_frame_t d = test_frame(ID_A, false, 0x61u);
    const can_frame_t q = test_frame(ID_B, false, 0x62u);
    TEST_ASSERT_EQUAL(CAN_PORT_OK, can_if_write(CAN_PORT_PLATFORM, &d));
    TEST_ASSERT_EQUAL(CAN_PORT_OK, can_if_write(CAN_PORT_PLATFORM, &q)); /* and one FIFO frame */
    TEST_ASSERT_EQUAL_UINT32(2u, pending(&bus_p, node_platform));
    const uint32_t aborts = can_if_tx_abort_count(CAN_PORT_PLATFORM);
    can_sm_step(CAN_PORT_PLATFORM);
    tick_n(CAN_PORT_PLATFORM, CAN_SM_TX_TIMEOUT_MS);
    /* one N_As abort cancels the dedicated buffer and the FIFO together */
    TEST_ASSERT_EQUAL_UINT32(aborts + 1u, can_if_tx_abort_count(CAN_PORT_PLATFORM));
    TEST_ASSERT_EQUAL_UINT32(1u, can_sm_stats(CAN_PORT_PLATFORM)->tx_timeouts);
    TEST_ASSERT_EQUAL_UINT32(0u, pending(&bus_p, node_platform));
    vbus_set_tx_stalled(&bus_p, node_platform, false);
    tick_n(CAN_PORT_PLATFORM, 10u);
    TEST_ASSERT_EQUAL_UINT32(0u, vbus_frame_count(&bus_p)); /* neither went out late */
}

static void test_a_dedicated_frame_written_every_cycle_is_replaced_without_aborts(void)
{
    use_dedicated_a();
    vbus_set_tx_stalled(&bus_p, node_platform, true);
    const uint32_t aborts = can_if_tx_abort_count(CAN_PORT_PLATFORM);
    can_port_state_t before;
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_state(&bus_p, node_platform, &before));
    uint8_t seq = 0u;
    uint32_t refused = 0u;
    bool retry = false;
    for (uint32_t ms = 0u; ms < 2u * CAN_SM_TX_TIMEOUT_MS; ms++) {
        tick(CAN_PORT_PLATFORM);
        if ((ms % 50u) == 0u) { /* a writer with a 50 ms cycle, retrying in the next pass */
            seq++;
            retry = true;
        }
        if (retry) {
            const can_frame_t d = test_frame(ID_A, false, seq);
            const can_port_status_t st = can_if_write(CAN_PORT_PLATFORM, &d);
            refused += (st == CAN_PORT_TX_FULL) ? 1u : 0u;
            retry = (st == CAN_PORT_TX_FULL);
        }
    }
    TEST_ASSERT_FALSE(retry);
    TEST_ASSERT_EQUAL_UINT32(2u * CAN_SM_TX_TIMEOUT_MS / 50u - 1u, refused); /* all but the first */
    /* the pending count drops to 0 at every replace, so N_As never fires: each frame is
     * at most one write old, never an abort of the ISO-TP links */
    TEST_ASSERT_EQUAL_UINT32(aborts, can_if_tx_abort_count(CAN_PORT_PLATFORM));
    TEST_ASSERT_EQUAL_UINT32(0u, can_sm_stats(CAN_PORT_PLATFORM)->tx_timeouts);
    TEST_ASSERT_EQUAL_UINT32(refused, vbus_tx_replaced(&bus_p, node_platform));
    can_port_state_t after;
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_state(&bus_p, node_platform, &after));
    TEST_ASSERT_EQUAL_UINT32(before.tx_done, after.tx_done); /* a replace is not a TX */
    vbus_set_tx_stalled(&bus_p, node_platform, false);
    can_frame_t got;
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_recv(&bus_p, node_ppeer, &got));
    TEST_ASSERT_EQUAL_UINT8(seq, got.data[0]); /* the newest only */
    TEST_ASSERT_EQUAL(CAN_PORT_EMPTY, vbus_recv(&bus_p, node_ppeer, &got));
}

static void test_a_bus_off_drops_a_pending_dedicated_frame_for_good(void)
{
    use_dedicated_a();
    vbus_set_tx_stalled(&bus_p, node_platform, true);
    const can_frame_t d = test_frame(ID_A, false, 0x63u);
    TEST_ASSERT_EQUAL(CAN_PORT_OK, can_if_write(CAN_PORT_PLATFORM, &d));
    vbus_set_bus_off(&bus_p, node_platform, true);
    can_sm_step(CAN_PORT_PLATFORM);
    TEST_ASSERT_EQUAL_UINT32(0u, pending(&bus_p, node_platform));
    TEST_ASSERT_EQUAL(CAN_PORT_ERR_IO, can_if_write(CAN_PORT_PLATFORM, &d));
    vbus_set_tx_stalled(&bus_p, node_platform, false);
    tick_n(CAN_PORT_PLATFORM, CAN_SM_PLATFORM_BACKOFF_INITIAL_MS + 1u);
    TEST_ASSERT_EQUAL(CAN_SM_ERROR_ACTIVE, can_sm_state(CAN_PORT_PLATFORM));
    TEST_ASSERT_EQUAL_UINT32(0u, vbus_frame_count(&bus_p)); /* no stale frame after recovery */
    TEST_ASSERT_EQUAL(CAN_PORT_OK, can_if_write(CAN_PORT_PLATFORM, &d));
    TEST_ASSERT_EQUAL_UINT32(1u, vbus_frame_count(&bus_p));
}

static void test_a_replace_that_comes_too_late_lets_the_old_frame_out_once(void)
{
    use_dedicated_a();
    vbus_set_tx_stalled(&bus_p, node_platform, true);
    const can_frame_t d1 = test_frame(ID_A, false, 1u);
    const can_frame_t d2 = test_frame(ID_A, false, 2u);
    TEST_ASSERT_EQUAL(CAN_PORT_OK, can_if_write(CAN_PORT_PLATFORM, &d1));
    vbus_set_tx_cancel_late(&bus_p, node_platform, true); /* d1 already in arbitration */
    TEST_ASSERT_EQUAL(CAN_PORT_TX_FULL, can_if_write(CAN_PORT_PLATFORM, &d2));
    TEST_ASSERT_EQUAL(CAN_PORT_OK, can_if_write(CAN_PORT_PLATFORM, &d2));
    TEST_ASSERT_EQUAL_UINT32(0u, vbus_tx_replaced(&bus_p, node_platform));
    vbus_set_tx_stalled(&bus_p, node_platform, false);
    can_frame_t got;
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_recv(&bus_p, node_ppeer, &got));
    TEST_ASSERT_EQUAL_UINT8(1u, got.data[0]);
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_recv(&bus_p, node_ppeer, &got));
    TEST_ASSERT_EQUAL_UINT8(2u, got.data[0]);
    TEST_ASSERT_EQUAL_UINT32(0u, can_if_tx_abort_count(CAN_PORT_PLATFORM));
}

int main(void)
{
    capture_before_init();
    UNITY_BEGIN();
    RUN_TEST(test_step_before_can_sm_init_does_nothing);
    RUN_TEST(test_vehicle_bus_off_blocks_writes_without_counting_a_guard_refusal);
    RUN_TEST(test_a_forbidden_frame_on_a_bus_off_port_is_still_a_guard_refusal);
    RUN_TEST(test_vehicle_recovery_waits_for_the_initial_backoff_then_returns_to_error_active);
    RUN_TEST(test_vehicle_backoff_doubles_and_the_fifth_bus_off_latches);
    RUN_TEST(test_a_latched_vehicle_port_stays_off_even_when_the_bus_is_clear);
    RUN_TEST(test_a_latched_port_aborts_frames_that_are_still_pending);
    RUN_TEST(test_platform_port_never_latches_and_caps_its_backoff);
    RUN_TEST(test_backoff_resets_after_the_port_stayed_bus_on_for_the_max_backoff);
    RUN_TEST(test_backoff_does_not_reset_before_the_max_backoff);
    RUN_TEST(test_bus_off_right_after_a_recovery_without_a_step_keeps_the_doubled_wait);
    RUN_TEST(test_error_passive_port_still_sends);
    RUN_TEST(test_n_as_abort_fires_at_the_timeout_and_the_stale_frame_never_goes_out);
    RUN_TEST(test_without_the_state_manager_an_unstalled_port_sends_the_stale_frame_late);
    RUN_TEST(test_a_frame_that_is_sent_within_n_as_is_not_aborted);
    RUN_TEST(test_bus_off_aborts_the_pending_tx_and_recovery_does_not_deliver_it);
    RUN_TEST(test_a_dedicated_frame_nobody_replaces_is_aborted_by_n_as);
    RUN_TEST(test_a_dedicated_frame_written_every_cycle_is_replaced_without_aborts);
    RUN_TEST(test_a_bus_off_drops_a_pending_dedicated_frame_for_good);
    RUN_TEST(test_a_replace_that_comes_too_late_lets_the_old_frame_out_once);
    RUN_TEST(test_a_port_whose_state_cannot_be_read_refuses_tx_and_drops_its_pending_frames);
    RUN_TEST(test_the_platform_filters_take_a_full_receiver_table);
    RUN_TEST(test_unbound_ports_and_unknown_port_ids_are_safe);
    RUN_TEST(test_no_state_is_known_before_init_or_before_the_first_snapshot);
    RUN_TEST(test_an_unreadable_controller_is_unknown_until_a_good_snapshot);
    RUN_TEST(test_a_latched_port_stays_known_when_its_controller_cannot_be_read);
    RUN_TEST(test_applied_filters_pass_only_registered_ids_and_unmatched_ones_are_not_unrouted);
    RUN_TEST(test_filters_of_one_port_do_not_touch_the_other_port);
    RUN_TEST(test_filter_fifo_is_1_for_the_platform_diagnostic_ids_and_0_for_everything_else);
    RUN_TEST(test_registering_after_the_filters_is_sealed_and_init_clears_the_seal);
    RUN_TEST(test_apply_filters_on_an_unbound_or_unknown_port_reports_it_and_does_not_seal);
    RUN_TEST(test_comms_applies_the_filters_and_the_sil_exchange_still_works);
    RUN_TEST(test_comms_heartbeat_reads_normal_until_a_silent_ecu_arms_u0100);
    RUN_TEST(test_comms_each_client_stall_is_one_onset_and_the_writers_never_flap);
    return UNITY_END();
}
