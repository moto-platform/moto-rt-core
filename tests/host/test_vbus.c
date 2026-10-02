/*
 * L0 tests for the in-process virtual CAN bus of the host platform layer (D-034).
 */
#include "hal/host/vbus.h"

#include <string.h>
#include <unity.h>

static vbus_t bus;

void setUp(void)
{
    vbus_init(&bus);
}

void tearDown(void) {}

static can_frame_t frame(uint32_t id, bool ext, uint8_t dlc, uint8_t seed)
{
    can_frame_t f;
    memset(&f, 0, sizeof f);
    f.id = id;
    f.extended = ext;
    f.dlc = dlc;
    for (uint8_t i = 0u; i < dlc; i++) {
        f.data[i] = (uint8_t)(seed + i);
    }
    return f;
}

static void test_attach_is_limited_to_max_nodes(void)
{
    uint8_t n = 0xFFu;
    for (uint8_t i = 0u; i < VBUS_MAX_NODES; i++) {
        TEST_ASSERT_TRUE(vbus_attach(&bus, &n));
        TEST_ASSERT_EQUAL_UINT8(i, n);
    }
    TEST_ASSERT_FALSE(vbus_attach(&bus, &n));
    TEST_ASSERT_FALSE(vbus_attach(NULL, &n));
    TEST_ASSERT_FALSE(vbus_attach(&bus, NULL));
}

static void test_frame_reaches_every_other_node_but_not_the_sender(void)
{
    uint8_t a, b, c;
    TEST_ASSERT_TRUE(vbus_attach(&bus, &a));
    TEST_ASSERT_TRUE(vbus_attach(&bus, &b));
    TEST_ASSERT_TRUE(vbus_attach(&bus, &c));
    const can_frame_t f = frame(0x1ABCDE01u, true, 8u, 0x10u); /* any 29-bit ID: the bus is protocol-agnostic */
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_send(&bus, a, &f));

    can_frame_t got;
    TEST_ASSERT_EQUAL(CAN_PORT_EMPTY, vbus_recv(&bus, a, &got));
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_recv(&bus, b, &got));
    TEST_ASSERT_EQUAL_MEMORY(&f, &got, sizeof f);
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_recv(&bus, c, &got));
    TEST_ASSERT_EQUAL_MEMORY(&f, &got, sizeof f);
    TEST_ASSERT_EQUAL(CAN_PORT_EMPTY, vbus_recv(&bus, b, &got));
    TEST_ASSERT_EQUAL_UINT32(1u, vbus_frame_count(&bus));
}

static void test_frames_keep_their_order(void)
{
    uint8_t a, b;
    TEST_ASSERT_TRUE(vbus_attach(&bus, &a));
    TEST_ASSERT_TRUE(vbus_attach(&bus, &b));
    for (uint8_t i = 0u; i < 10u; i++) {
        const can_frame_t f = frame(0x100u + i, false, 1u, i);
        TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_send(&bus, a, &f));
    }
    for (uint8_t i = 0u; i < 10u; i++) {
        can_frame_t got;
        TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_recv(&bus, b, &got));
        TEST_ASSERT_EQUAL_UINT32(0x100u + i, got.id);
    }
}

static void test_full_rx_queue_drops_and_counts_overruns(void)
{
    uint8_t a, b;
    TEST_ASSERT_TRUE(vbus_attach(&bus, &a));
    TEST_ASSERT_TRUE(vbus_attach(&bus, &b));
    for (uint32_t i = 0u; i < VBUS_RX_DEPTH + 3u; i++) {
        const can_frame_t f = frame(i & 0x7FFu, false, 0u, 0u);
        TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_send(&bus, a, &f));
    }
    TEST_ASSERT_EQUAL_UINT32(3u, vbus_overruns(&bus, b));
    can_frame_t got;
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_recv(&bus, b, &got));
    TEST_ASSERT_EQUAL_UINT32(0u, got.id); /* the oldest frames are kept */
}

static void test_blocked_tx_reports_full_and_sends_nothing(void)
{
    uint8_t a, b;
    TEST_ASSERT_TRUE(vbus_attach(&bus, &a));
    TEST_ASSERT_TRUE(vbus_attach(&bus, &b));
    vbus_set_tx_blocked(&bus, a, true);
    TEST_ASSERT_FALSE(vbus_tx_free(&bus, a));
    const can_frame_t f = frame(0x123u, false, 2u, 0u);
    TEST_ASSERT_EQUAL(CAN_PORT_TX_FULL, vbus_send(&bus, a, &f));
    can_frame_t got;
    TEST_ASSERT_EQUAL(CAN_PORT_EMPTY, vbus_recv(&bus, b, &got));
    vbus_set_tx_blocked(&bus, a, false);
    TEST_ASSERT_TRUE(vbus_tx_free(&bus, a));
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_send(&bus, a, &f));
}

static void test_invalid_frames_and_nodes_are_rejected(void)
{
    uint8_t a;
    TEST_ASSERT_TRUE(vbus_attach(&bus, &a));
    can_frame_t f = frame(0x800u, false, 1u, 0u); /* 12-bit ID in standard format */
    TEST_ASSERT_EQUAL(CAN_PORT_ERR_ARG, vbus_send(&bus, a, &f));
    f = frame(0x20000000u, true, 1u, 0u);          /* 30-bit ID */
    TEST_ASSERT_EQUAL(CAN_PORT_ERR_ARG, vbus_send(&bus, a, &f));
    f = frame(0x100u, false, 1u, 0u);
    f.dlc = 9u;
    TEST_ASSERT_EQUAL(CAN_PORT_ERR_ARG, vbus_send(&bus, a, &f));
    f.dlc = 1u;
    TEST_ASSERT_EQUAL(CAN_PORT_ERR_ARG, vbus_send(&bus, 3u, &f)); /* not attached */
    TEST_ASSERT_EQUAL(CAN_PORT_ERR_ARG, vbus_send(&bus, VBUS_MAX_NODES, &f));
    TEST_ASSERT_EQUAL(CAN_PORT_ERR_ARG, vbus_recv(&bus, a, NULL));
    TEST_ASSERT_EQUAL_UINT32(0u, vbus_frame_count(&bus));
}

/* ------------------------------------------------------------------------- */
/* Controller faults (Ç1): TX stall, abort, bus-off, error counters, filters  */
/* ------------------------------------------------------------------------- */

static can_port_state_t state_of(uint8_t node)
{
    can_port_state_t st;
    memset(&st, 0xFF, sizeof st);
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_state(&bus, node, &st));
    return st;
}

static void test_stalled_tx_holds_frames_and_flushes_them_in_order(void)
{
    uint8_t a, b;
    TEST_ASSERT_TRUE(vbus_attach(&bus, &a));
    TEST_ASSERT_TRUE(vbus_attach(&bus, &b));
    vbus_set_tx_stalled(&bus, a, true);
    for (uint8_t i = 0u; i < 3u; i++) {
        const can_frame_t f = frame(0x100u + i, false, 1u, i);
        TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_send(&bus, a, &f));
    }
    can_frame_t got;
    TEST_ASSERT_EQUAL(CAN_PORT_EMPTY, vbus_recv(&bus, b, &got)); /* nothing went out */
    TEST_ASSERT_EQUAL_UINT32(0u, vbus_frame_count(&bus));
    TEST_ASSERT_EQUAL_UINT32(3u, state_of(a).tx_pending);
    TEST_ASSERT_EQUAL_UINT32(0u, state_of(a).tx_done);

    vbus_set_tx_stalled(&bus, a, false); /* the late frames, oldest first */
    for (uint8_t i = 0u; i < 3u; i++) {
        TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_recv(&bus, b, &got));
        TEST_ASSERT_EQUAL_UINT32(0x100u + i, got.id);
        TEST_ASSERT_EQUAL_UINT8(i, got.data[0]);
    }
    TEST_ASSERT_EQUAL(CAN_PORT_EMPTY, vbus_recv(&bus, b, &got));
    TEST_ASSERT_EQUAL_UINT32(0u, state_of(a).tx_pending);
    TEST_ASSERT_EQUAL_UINT32(3u, state_of(a).tx_done);
    TEST_ASSERT_EQUAL_UINT32(3u, vbus_frame_count(&bus));
}

static void test_unstalling_flushes_the_old_frames_before_a_new_frame_goes_out(void)
{
    uint8_t a, b;
    TEST_ASSERT_TRUE(vbus_attach(&bus, &a));
    TEST_ASSERT_TRUE(vbus_attach(&bus, &b));
    vbus_set_tx_stalled(&bus, a, true);
    const can_frame_t first = frame(0x100u, false, 1u, 0u);
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_send(&bus, a, &first));
    vbus_set_tx_stalled(&bus, a, false);
    const can_frame_t second = frame(0x101u, false, 1u, 0u);
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_send(&bus, a, &second)); /* not stalled: sent at once */
    can_frame_t got;
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_recv(&bus, b, &got));
    TEST_ASSERT_EQUAL_UINT32(0x100u, got.id); /* the flush came first, order kept */
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_recv(&bus, b, &got));
    TEST_ASSERT_EQUAL_UINT32(0x101u, got.id);
}

static void test_tx_abort_drops_the_pending_frames(void)
{
    uint8_t a, b;
    TEST_ASSERT_TRUE(vbus_attach(&bus, &a));
    TEST_ASSERT_TRUE(vbus_attach(&bus, &b));
    vbus_set_tx_stalled(&bus, a, true);
    for (uint8_t i = 0u; i < 2u; i++) {
        const can_frame_t f = frame(0x100u + i, false, 1u, i);
        TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_send(&bus, a, &f));
    }
    TEST_ASSERT_EQUAL_UINT32(2u, state_of(a).tx_pending);
    vbus_tx_abort(&bus, a);
    TEST_ASSERT_EQUAL_UINT32(0u, state_of(a).tx_pending);
    vbus_set_tx_stalled(&bus, a, false);
    can_frame_t got;
    TEST_ASSERT_EQUAL(CAN_PORT_EMPTY, vbus_recv(&bus, b, &got));
    TEST_ASSERT_EQUAL_UINT32(0u, state_of(a).tx_done);
    TEST_ASSERT_EQUAL_UINT32(0u, vbus_frame_count(&bus));
    /* the node sends normally afterwards; an abort on an idle or unknown node is harmless */
    const can_frame_t f = frame(0x120u, false, 1u, 0u);
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_send(&bus, a, &f));
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_recv(&bus, b, &got));
    vbus_tx_abort(&bus, a);
    vbus_tx_abort(&bus, 5u);
    vbus_tx_abort(NULL, a);
}

static void test_stalled_tx_is_full_at_the_buffer_depth(void)
{
    uint8_t a, b;
    TEST_ASSERT_TRUE(vbus_attach(&bus, &a));
    TEST_ASSERT_TRUE(vbus_attach(&bus, &b));
    vbus_set_tx_stalled(&bus, a, true);
    for (uint32_t i = 0u; i < VBUS_TX_DEPTH; i++) {
        TEST_ASSERT_TRUE(vbus_tx_free(&bus, a));
        const can_frame_t f = frame(0x100u + i, false, 1u, (uint8_t)i);
        TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_send(&bus, a, &f));
    }
    TEST_ASSERT_FALSE(vbus_tx_free(&bus, a));
    const can_frame_t extra = frame(0x1FFu, false, 1u, 0u);
    TEST_ASSERT_EQUAL(CAN_PORT_TX_FULL, vbus_send(&bus, a, &extra));
    TEST_ASSERT_EQUAL_UINT32(VBUS_TX_DEPTH, state_of(a).tx_pending);
    vbus_set_tx_stalled(&bus, a, false);
    uint32_t n = 0u;
    can_frame_t got;
    while (vbus_recv(&bus, b, &got) == CAN_PORT_OK) {
        TEST_ASSERT_EQUAL_UINT32(0x100u + n, got.id); /* the refused frame is not among them */
        n++;
    }
    TEST_ASSERT_EQUAL_UINT32(VBUS_TX_DEPTH, n);
    TEST_ASSERT_TRUE(vbus_tx_free(&bus, a));
}

static void test_bus_off_counts_once_per_entry_and_blocks_send_and_receive(void)
{
    uint8_t a, b;
    TEST_ASSERT_TRUE(vbus_attach(&bus, &a));
    TEST_ASSERT_TRUE(vbus_attach(&bus, &b));
    vbus_set_bus_off(&bus, a, true);
    vbus_set_bus_off(&bus, a, true); /* already bus-off: not a new entry */
    TEST_ASSERT_TRUE(state_of(a).bus_off);
    TEST_ASSERT_EQUAL_UINT32(1u, state_of(a).bus_off_events);
    TEST_ASSERT_FALSE(vbus_tx_free(&bus, a));

    const can_frame_t f = frame(0x123u, false, 2u, 0u);
    TEST_ASSERT_EQUAL(CAN_PORT_ERR_IO, vbus_send(&bus, a, &f));
    TEST_ASSERT_EQUAL_UINT32(0u, vbus_frame_count(&bus));
    can_frame_t got;
    TEST_ASSERT_EQUAL(CAN_PORT_EMPTY, vbus_recv(&bus, b, &got));

    /* the bus-off node receives nothing, and does not count an overrun either */
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_send(&bus, b, &f));
    TEST_ASSERT_EQUAL(CAN_PORT_EMPTY, vbus_recv(&bus, a, &got));
    TEST_ASSERT_EQUAL_UINT32(0u, vbus_overruns(&bus, a));
    vbus_set_bus_off(&bus, a, false);
    TEST_ASSERT_EQUAL(CAN_PORT_EMPTY, vbus_recv(&bus, a, &got)); /* the frame is gone for good */
    TEST_ASSERT_FALSE(state_of(a).bus_off);
    TEST_ASSERT_EQUAL_UINT32(1u, state_of(a).bus_off_events); /* leaving does not count */

    vbus_set_bus_off(&bus, a, true); /* a second entry */
    TEST_ASSERT_EQUAL_UINT32(2u, state_of(a).bus_off_events);
    TEST_ASSERT_EQUAL_UINT32(0u, state_of(b).bus_off_events); /* the peer is unaffected */
}

static void test_recover_leaves_bus_off_clears_the_counters_and_counts_each_attempt(void)
{
    uint8_t a, b;
    TEST_ASSERT_TRUE(vbus_attach(&bus, &a));
    TEST_ASSERT_TRUE(vbus_attach(&bus, &b));
    TEST_ASSERT_EQUAL_UINT32(0u, vbus_recover_count(&bus, a));

    /* not bus-off: the attempt is counted but changes nothing */
    vbus_set_error_counters(&bus, a, 50u, 60u);
    vbus_recover(&bus, a);
    TEST_ASSERT_EQUAL_UINT32(1u, vbus_recover_count(&bus, a));
    TEST_ASSERT_EQUAL_UINT8(50u, state_of(a).tec);
    TEST_ASSERT_EQUAL_UINT8(60u, state_of(a).rec);

    vbus_set_error_counters(&bus, a, 255u, 130u);
    vbus_set_bus_off(&bus, a, true);
    vbus_recover(&bus, a);
    TEST_ASSERT_EQUAL_UINT32(2u, vbus_recover_count(&bus, a));
    TEST_ASSERT_FALSE(state_of(a).bus_off);
    TEST_ASSERT_EQUAL_UINT8(0u, state_of(a).tec); /* the controller restarts clean */
    TEST_ASSERT_EQUAL_UINT8(0u, state_of(a).rec);
    TEST_ASSERT_EQUAL_UINT32(1u, state_of(a).bus_off_events);
    TEST_ASSERT_EQUAL_UINT32(0u, vbus_recover_count(&bus, b));

    const can_frame_t f = frame(0x321u, false, 1u, 0u);
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_send(&bus, a, &f)); /* sends again */
    can_frame_t got;
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_recv(&bus, b, &got));
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_send(&bus, b, &f)); /* and receives again */
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_recv(&bus, a, &got));

    vbus_recover(&bus, 5u); /* unknown node: no effect */
    TEST_ASSERT_EQUAL_UINT32(0u, vbus_recover_count(&bus, 5u));
    TEST_ASSERT_EQUAL_UINT32(0u, vbus_recover_count(NULL, a));
}

static void test_a_frame_pending_across_a_bus_off_goes_out_late_at_the_recovery(void)
{
    /* The hazard can_sm_step() prevents by aborting: accepted, bus-off, recovered, sent. */
    uint8_t a, b;
    TEST_ASSERT_TRUE(vbus_attach(&bus, &a));
    TEST_ASSERT_TRUE(vbus_attach(&bus, &b));
    vbus_set_tx_stalled(&bus, a, true);
    const can_frame_t f = frame(0x222u, false, 1u, 0u);
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_send(&bus, a, &f));
    vbus_set_bus_off(&bus, a, true);
    vbus_set_tx_stalled(&bus, a, false); /* a bus-off node sends nothing */
    can_frame_t got;
    TEST_ASSERT_EQUAL(CAN_PORT_EMPTY, vbus_recv(&bus, b, &got));
    TEST_ASSERT_EQUAL_UINT32(1u, state_of(a).tx_pending);
    vbus_recover(&bus, a);
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_recv(&bus, b, &got));
    TEST_ASSERT_EQUAL_UINT32(0x222u, got.id);

    /* with an abort in between the old frame is gone */
    vbus_set_tx_stalled(&bus, a, true);
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_send(&bus, a, &f));
    vbus_set_bus_off(&bus, a, true);
    vbus_tx_abort(&bus, a);
    vbus_set_tx_stalled(&bus, a, false);
    vbus_recover(&bus, a);
    TEST_ASSERT_EQUAL(CAN_PORT_EMPTY, vbus_recv(&bus, b, &got));
}

static void test_error_passive_starts_above_127_for_tec_and_rec(void)
{
    uint8_t a;
    TEST_ASSERT_TRUE(vbus_attach(&bus, &a));
    TEST_ASSERT_FALSE(state_of(a).error_passive);

    vbus_set_error_counters(&bus, a, 127u, 0u);
    TEST_ASSERT_FALSE(state_of(a).error_passive);
    vbus_set_error_counters(&bus, a, 128u, 0u);
    TEST_ASSERT_TRUE(state_of(a).error_passive);
    vbus_set_error_counters(&bus, a, 0u, 127u);
    TEST_ASSERT_FALSE(state_of(a).error_passive);
    vbus_set_error_counters(&bus, a, 0u, 128u);
    TEST_ASSERT_TRUE(state_of(a).error_passive);
    vbus_set_error_counters(&bus, a, 127u, 127u);
    TEST_ASSERT_FALSE(state_of(a).error_passive);
    vbus_set_error_counters(&bus, a, 255u, 255u);
    TEST_ASSERT_TRUE(state_of(a).error_passive);
    TEST_ASSERT_EQUAL_UINT8(255u, state_of(a).tec);
    TEST_ASSERT_EQUAL_UINT8(255u, state_of(a).rec);
    vbus_set_error_counters(&bus, a, 0u, 0u);
    TEST_ASSERT_FALSE(state_of(a).error_passive);
    vbus_set_error_counters(&bus, 5u, 200u, 200u); /* unknown node: no effect, no crash */
    vbus_set_error_counters(NULL, a, 200u, 200u);
}

static void test_set_filters_checks_its_arguments_and_keeps_the_old_filters_on_error(void)
{
    uint8_t a;
    TEST_ASSERT_TRUE(vbus_attach(&bus, &a));
    can_port_filter_t list[CAN_PORT_MAX_FILTERS + 1u];
    memset(list, 0, sizeof list);
    TEST_ASSERT_EQUAL(CAN_PORT_ERR_ARG, vbus_set_filters(&bus, a, list, CAN_PORT_MAX_FILTERS + 1u));
    TEST_ASSERT_EQUAL(CAN_PORT_ERR_ARG, vbus_set_filters(&bus, a, NULL, 1u));
    TEST_ASSERT_EQUAL(CAN_PORT_ERR_ARG, vbus_set_filters(&bus, 3u, list, 1u));     /* not attached */
    TEST_ASSERT_EQUAL(CAN_PORT_ERR_ARG, vbus_set_filters(NULL, a, list, 1u));
    TEST_ASSERT_FALSE(bus.nodes[a].filtered); /* every refused call left the node unfiltered */

    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_set_filters(&bus, a, list, CAN_PORT_MAX_FILTERS));
    TEST_ASSERT_TRUE(bus.nodes[a].filtered);
    TEST_ASSERT_EQUAL_UINT32(CAN_PORT_MAX_FILTERS, bus.nodes[a].filter_count);
    TEST_ASSERT_EQUAL(CAN_PORT_ERR_ARG, vbus_set_filters(&bus, a, list, CAN_PORT_MAX_FILTERS + 1u));
    TEST_ASSERT_EQUAL_UINT32(CAN_PORT_MAX_FILTERS, bus.nodes[a].filter_count); /* kept */
}

static void test_a_filtered_node_receives_only_the_matching_id_and_format(void)
{
    uint8_t a, b;
    TEST_ASSERT_TRUE(vbus_attach(&bus, &a));
    TEST_ASSERT_TRUE(vbus_attach(&bus, &b));
    const can_port_filter_t list[2] = {{0x100u, false, CAN_PORT_FIFO0},
                                       {0x200u, true, CAN_PORT_FIFO1}};
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_set_filters(&bus, a, list, 2u));
    const struct {
        uint32_t id;
        bool ext;
        bool accepted;
    } sent[] = {{0x100u, false, true},  {0x100u, true, false}, {0x200u, false, false},
                {0x200u, true, true},   {0x300u, false, false}};
    for (uint32_t i = 0u; i < sizeof sent / sizeof sent[0]; i++) {
        const can_frame_t f = frame(sent[i].id, sent[i].ext, 1u, (uint8_t)i);
        TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_send(&bus, b, &f));
    }
    can_frame_t got;
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_recv(&bus, a, &got)); /* the FIFO is ignored */
    TEST_ASSERT_EQUAL_UINT32(0x100u, got.id);
    TEST_ASSERT_FALSE(got.extended);
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_recv(&bus, a, &got));
    TEST_ASSERT_EQUAL_UINT32(0x200u, got.id);
    TEST_ASSERT_TRUE(got.extended);
    TEST_ASSERT_EQUAL(CAN_PORT_EMPTY, vbus_recv(&bus, a, &got));
    TEST_ASSERT_EQUAL_UINT32(0u, vbus_overruns(&bus, a)); /* a filter drop is not an overrun */
    TEST_ASSERT_EQUAL_UINT32(5u, vbus_frame_count(&bus)); /* the bus carried all five */

    /* new filters replace the old ones; an empty list receives nothing */
    const can_port_filter_t one = {0x300u, false, CAN_PORT_FIFO0};
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_set_filters(&bus, a, &one, 1u));
    const can_frame_t f1 = frame(0x100u, false, 1u, 0u);
    const can_frame_t f3 = frame(0x300u, false, 1u, 0u);
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_send(&bus, b, &f1));
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_send(&bus, b, &f3));
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_recv(&bus, a, &got));
    TEST_ASSERT_EQUAL_UINT32(0x300u, got.id);
    TEST_ASSERT_EQUAL(CAN_PORT_EMPTY, vbus_recv(&bus, a, &got));
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_set_filters(&bus, a, NULL, 0u));
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_send(&bus, b, &f3));
    TEST_ASSERT_EQUAL(CAN_PORT_EMPTY, vbus_recv(&bus, a, &got));
    /* the sender's own filters never touch what it transmits */
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_send(&bus, a, &f1));
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_recv(&bus, b, &got));
}

static void test_vbus_state_reports_every_field(void)
{
    uint8_t a, b;
    TEST_ASSERT_TRUE(vbus_attach(&bus, &a));
    TEST_ASSERT_TRUE(vbus_attach(&bus, &b));
    can_port_state_t st = state_of(a); /* a fresh node: all zero */
    TEST_ASSERT_EQUAL_UINT8(0u, st.tec);
    TEST_ASSERT_EQUAL_UINT8(0u, st.rec);
    TEST_ASSERT_FALSE(st.error_passive);
    TEST_ASSERT_FALSE(st.bus_off);
    TEST_ASSERT_EQUAL_UINT32(0u, st.bus_off_events);
    TEST_ASSERT_EQUAL_UINT32(0u, st.tx_pending);
    TEST_ASSERT_EQUAL_UINT32(0u, st.tx_done);

    const can_frame_t f = frame(0x150u, false, 1u, 0u);
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_send(&bus, a, &f));
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_send(&bus, a, &f));
    TEST_ASSERT_EQUAL_UINT32(2u, state_of(a).tx_done); /* counted per sent frame, ACKed */
    TEST_ASSERT_EQUAL_UINT32(0u, state_of(b).tx_done); /* the receiver did not send */

    vbus_set_tx_stalled(&bus, a, true);
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_send(&bus, a, &f));
    vbus_set_error_counters(&bus, a, 130u, 7u);
    vbus_set_bus_off(&bus, a, true);
    st = state_of(a);
    TEST_ASSERT_EQUAL_UINT8(130u, st.tec);
    TEST_ASSERT_EQUAL_UINT8(7u, st.rec);
    TEST_ASSERT_TRUE(st.error_passive);
    TEST_ASSERT_TRUE(st.bus_off);
    TEST_ASSERT_EQUAL_UINT32(1u, st.bus_off_events);
    TEST_ASSERT_EQUAL_UINT32(1u, st.tx_pending);
    TEST_ASSERT_EQUAL_UINT32(2u, st.tx_done);

    TEST_ASSERT_EQUAL(CAN_PORT_ERR_ARG, vbus_state(&bus, a, NULL));
    TEST_ASSERT_EQUAL(CAN_PORT_ERR_ARG, vbus_state(&bus, 4u, &st)); /* not attached */
    TEST_ASSERT_EQUAL(CAN_PORT_ERR_ARG, vbus_state(&bus, VBUS_MAX_NODES, &st));
    TEST_ASSERT_EQUAL(CAN_PORT_ERR_ARG, vbus_state(NULL, a, &st));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_attach_is_limited_to_max_nodes);
    RUN_TEST(test_frame_reaches_every_other_node_but_not_the_sender);
    RUN_TEST(test_frames_keep_their_order);
    RUN_TEST(test_full_rx_queue_drops_and_counts_overruns);
    RUN_TEST(test_blocked_tx_reports_full_and_sends_nothing);
    RUN_TEST(test_invalid_frames_and_nodes_are_rejected);
    RUN_TEST(test_stalled_tx_holds_frames_and_flushes_them_in_order);
    RUN_TEST(test_unstalling_flushes_the_old_frames_before_a_new_frame_goes_out);
    RUN_TEST(test_tx_abort_drops_the_pending_frames);
    RUN_TEST(test_stalled_tx_is_full_at_the_buffer_depth);
    RUN_TEST(test_bus_off_counts_once_per_entry_and_blocks_send_and_receive);
    RUN_TEST(test_recover_leaves_bus_off_clears_the_counters_and_counts_each_attempt);
    RUN_TEST(test_a_frame_pending_across_a_bus_off_goes_out_late_at_the_recovery);
    RUN_TEST(test_error_passive_starts_above_127_for_tec_and_rec);
    RUN_TEST(test_set_filters_checks_its_arguments_and_keeps_the_old_filters_on_error);
    RUN_TEST(test_a_filtered_node_receives_only_the_matching_id_and_format);
    RUN_TEST(test_vbus_state_reports_every_field);
    return UNITY_END();
}
