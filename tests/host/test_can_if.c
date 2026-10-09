/*
 * L0 tests for services/can_if (RX routing by port/ID/format, TX pass-through) on the
 * host CAN port bound to the in-process bus.
 */
#include "hal/can_port.h"
#include "hal/host/can_port_host.h"
#include "services/can_if.h"
#include "vehicle_cl250.h"

#include <string.h>
#include <unity.h>

/* Test-only identifiers (not platform or vehicle IDs). */
#define ID_A 0x100u
#define ID_B 0x101u

static vbus_t bus;
static uint8_t node_dut;
static uint8_t node_peer;
static uint8_t node_vehicle;

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

void setUp(void)
{
    memset(&sink_a, 0, sizeof sink_a);
    memset(&sink_b, 0, sizeof sink_b);
    vbus_init(&bus);
    TEST_ASSERT_TRUE(vbus_attach(&bus, &node_dut));
    TEST_ASSERT_TRUE(vbus_attach(&bus, &node_peer));
    TEST_ASSERT_TRUE(vbus_attach(&bus, &node_vehicle));
    TEST_ASSERT_TRUE(can_port_host_bind_vbus(CAN_PORT_PLATFORM, &bus, node_dut));
    can_if_init();
}

void tearDown(void)
{
    can_port_host_unbind_all();
}

static void peer_send(uint32_t id, bool ext, uint8_t first)
{
    can_frame_t f;
    memset(&f, 0, sizeof f);
    f.id = id;
    f.extended = ext;
    f.dlc = 1u;
    f.data[0] = first;
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_send(&bus, node_peer, &f));
}

static void test_register_rejects_bad_arguments_and_duplicates(void)
{
    TEST_ASSERT_EQUAL(CAN_IF_ERR_ARG, can_if_register_rx(CAN_PORT_COUNT, ID_A, false, on_frame, &sink_a));
    TEST_ASSERT_EQUAL(CAN_IF_ERR_ARG, can_if_register_rx(CAN_PORT_PLATFORM, ID_A, false, NULL, &sink_a));
    TEST_ASSERT_EQUAL(CAN_IF_ERR_ARG,
                      can_if_register_rx(CAN_PORT_PLATFORM, 0x800u, false, on_frame, &sink_a));
    TEST_ASSERT_EQUAL(CAN_IF_OK, can_if_register_rx(CAN_PORT_PLATFORM, ID_A, false, on_frame, &sink_a));
    TEST_ASSERT_EQUAL(CAN_IF_ERR_DUP, can_if_register_rx(CAN_PORT_PLATFORM, ID_A, false, on_frame, &sink_b));
    /* the same number in the other format or on the other port is a different receiver */
    TEST_ASSERT_EQUAL(CAN_IF_OK, can_if_register_rx(CAN_PORT_PLATFORM, ID_A, true, on_frame, &sink_b));
    TEST_ASSERT_EQUAL(CAN_IF_OK, can_if_register_rx(CAN_PORT_VEHICLE, ID_A, false, on_frame, &sink_b));
}

static void test_table_full(void)
{
    for (uint32_t i = 0u; i < CAN_IF_MAX_RECEIVERS; i++) {
        TEST_ASSERT_EQUAL(CAN_IF_OK,
                          can_if_register_rx(CAN_PORT_PLATFORM, 0x200u + i, false, on_frame, &sink_a));
    }
    TEST_ASSERT_EQUAL(CAN_IF_ERR_FULL,
                      can_if_register_rx(CAN_PORT_PLATFORM, 0x300u, false, on_frame, &sink_a));
}

static void test_dispatch_routes_by_id_and_format(void)
{
    TEST_ASSERT_EQUAL(CAN_IF_OK, can_if_register_rx(CAN_PORT_PLATFORM, ID_A, false, on_frame, &sink_a));
    TEST_ASSERT_EQUAL(CAN_IF_OK, can_if_register_rx(CAN_PORT_PLATFORM, ID_B, false, on_frame, &sink_b));
    peer_send(ID_A, false, 1u);
    peer_send(ID_B, false, 2u);
    peer_send(ID_A, true, 3u);  /* extended ID 0x100: nobody registered */
    peer_send(0x555u, false, 4u);
    TEST_ASSERT_EQUAL_UINT32(4u, can_if_dispatch(CAN_PORT_PLATFORM, 10u));
    TEST_ASSERT_EQUAL_UINT32(1u, sink_a.calls);
    TEST_ASSERT_EQUAL_UINT8(1u, sink_a.last.data[0]);
    TEST_ASSERT_EQUAL_UINT32(1u, sink_b.calls);
    TEST_ASSERT_EQUAL_UINT8(2u, sink_b.last.data[0]);
    TEST_ASSERT_EQUAL_UINT32(2u, can_if_unrouted_count(CAN_PORT_PLATFORM));
    TEST_ASSERT_EQUAL_UINT32(0u, can_if_unrouted_count(CAN_PORT_VEHICLE));
}

static void test_dispatch_reads_at_most_max_frames(void)
{
    TEST_ASSERT_EQUAL(CAN_IF_OK, can_if_register_rx(CAN_PORT_PLATFORM, ID_A, false, on_frame, &sink_a));
    for (uint8_t i = 0u; i < 5u; i++) {
        peer_send(ID_A, false, i);
    }
    TEST_ASSERT_EQUAL_UINT32(3u, can_if_dispatch(CAN_PORT_PLATFORM, 3u));
    TEST_ASSERT_EQUAL_UINT32(3u, sink_a.calls);
    TEST_ASSERT_EQUAL_UINT32(2u, can_if_dispatch(CAN_PORT_PLATFORM, 3u));
    TEST_ASSERT_EQUAL_UINT32(0u, can_if_dispatch(CAN_PORT_PLATFORM, 3u));
    TEST_ASSERT_EQUAL_UINT8(4u, sink_a.last.data[0]);
}

/* A padded CL250 Single Frame request on the request ID, as the vehicle link builds it. */
static can_frame_t vehicle_sf(uint8_t sid, uint8_t b1, uint8_t b2, uint8_t len)
{
    can_frame_t f;
    memset(f.data, VEHICLE_CL250_PADDING_BYTE, sizeof f.data);
    f.id = VEHICLE_CL250_REQUEST_ID;
    f.extended = true;
    f.dlc = VEHICLE_CL250_FRAME_DLC;
    f.data[0] = len;
    f.data[1] = sid;
    f.data[2] = b1;
    f.data[3] = b2;
    return f;
}

static void test_vehicle_guard_passes_only_allowed_single_frames_on_the_request_id(void)
{
    TEST_ASSERT_TRUE(can_port_host_bind_vbus(CAN_PORT_VEHICLE, &bus, node_vehicle));
    const can_frame_t ok = vehicle_sf(0x22u, 0xF4u, 0x0Cu, 3u);
    TEST_ASSERT_EQUAL(CAN_PORT_OK, can_if_write(CAN_PORT_VEHICLE, &ok));

    can_frame_t bad[9];
    bad[0] = vehicle_sf(0x2Eu, 0xF1u, 0x90u, 3u); /* WriteDataByIdentifier (D-020) */
    bad[1] = vehicle_sf(0x11u, 0x01u, 0x00u, 2u); /* ECUReset */
    bad[2] = vehicle_sf(0x10u, 0x02u, 0x00u, 2u); /* programming session */
    bad[3] = ok;
    bad[3].id = VEHICLE_CL250_REQUEST_ID + 1u;    /* wrong ID */
    bad[4] = ok;
    bad[4].id = VEHICLE_CL250_REQUEST_ID & CAN_PORT_STD_ID_MAX;
    bad[4].extended = false;                      /* 11-bit */
    bad[5] = ok;
    bad[5].dlc = 4u;                              /* unpadded */
    bad[6] = vehicle_sf(0x00u, 0x00u, 0x00u, 0x30u); /* FC.CTS */
    bad[7] = vehicle_sf(0x22u, 0xF4u, 0x0Cu, 0x10u); /* First Frame */
    bad[8] = ok;
    bad[8].data[7] = 0x00u;                       /* not padding after the payload */
    const uint32_t n = sizeof bad / sizeof bad[0];
    for (uint32_t i = 0u; i < n; i++) {
        TEST_ASSERT_FALSE(can_if_tx_allowed(CAN_PORT_VEHICLE, &bad[i]));
        TEST_ASSERT_EQUAL(CAN_PORT_ERR_REFUSED, can_if_write(CAN_PORT_VEHICLE, &bad[i]));
    }
    TEST_ASSERT_EQUAL_UINT32(n, can_if_tx_refused_count(CAN_PORT_VEHICLE));
    TEST_ASSERT_EQUAL_UINT32(1u, vbus_frame_count(&bus)); /* only the allowed one */

    /* D-059: exactly the gen/ FC.CTS passes (stateless; when is the link's check) */
    can_frame_t fc = ok;
    for (uint8_t i = 0u; i < VEHICLE_CL250_FRAME_DLC; i++) {
        fc.data[i] = vehicle_cl250_fc_cts[i];
    }
    TEST_ASSERT_TRUE(can_if_tx_allowed(CAN_PORT_VEHICLE, &fc));
    TEST_ASSERT_EQUAL(CAN_PORT_OK, can_if_write(CAN_PORT_VEHICLE, &fc));
    TEST_ASSERT_EQUAL_UINT32(2u, vbus_frame_count(&bus));
    can_frame_t fc_bad[9];
    for (uint32_t i = 0u; i < 9u; i++) {
        fc_bad[i] = fc;
    }
    fc_bad[0].data[0] = 0x31u; /* FC.WAIT */
    fc_bad[1].data[0] = 0x32u; /* FC.OVFLW */
    fc_bad[2].data[1] = 0x01u; /* another block size */
    fc_bad[3].data[2] = 0x01u; /* another STmin */
    fc_bad[4].data[7] = 0x00u; /* another padding byte */
    fc_bad[5].dlc = 3u;        /* short FC */
    fc_bad[6].id = VEHICLE_CL250_FALLBACK_REQUEST_ID; /* another ID */
    fc_bad[7].id = VEHICLE_CL250_REQUEST_ID & CAN_PORT_STD_ID_MAX;
    fc_bad[7].extended = false; /* 11-bit */
    fc_bad[8].data[0] = 0x21u;  /* a Consecutive Frame */
    for (uint32_t i = 0u; i < 9u; i++) {
        TEST_ASSERT_FALSE_MESSAGE(can_if_tx_allowed(CAN_PORT_VEHICLE, &fc_bad[i]), "FC variant");
        TEST_ASSERT_EQUAL(CAN_PORT_ERR_REFUSED, can_if_write(CAN_PORT_VEHICLE, &fc_bad[i]));
    }
    TEST_ASSERT_EQUAL_UINT32(n + 9u, can_if_tx_refused_count(CAN_PORT_VEHICLE));
    TEST_ASSERT_EQUAL_UINT32(2u, vbus_frame_count(&bus));

    /* the platform port has no vehicle guard */
    TEST_ASSERT_TRUE(can_if_tx_allowed(CAN_PORT_PLATFORM, &bad[0]));
    TEST_ASSERT_EQUAL_UINT32(0u, can_if_tx_refused_count(CAN_PORT_PLATFORM));
    TEST_ASSERT_FALSE(can_if_tx_allowed(CAN_PORT_VEHICLE, NULL));
}

static void test_write_goes_to_the_bound_bus_and_unbound_ports_are_closed(void)
{
    can_frame_t f;
    memset(&f, 0, sizeof f);
    f.id = ID_B;
    f.dlc = 2u;
    TEST_ASSERT_TRUE(can_if_tx_free(CAN_PORT_PLATFORM));
    TEST_ASSERT_EQUAL(CAN_PORT_OK, can_if_write(CAN_PORT_PLATFORM, &f));
    can_frame_t got;
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_recv(&bus, node_peer, &got));
    TEST_ASSERT_EQUAL_UINT32(ID_B, got.id);

    TEST_ASSERT_FALSE(can_if_tx_free(CAN_PORT_VEHICLE));
    const can_frame_t v = vehicle_sf(0x22u, 0xF4u, 0x0Du, 3u);
    TEST_ASSERT_EQUAL(CAN_PORT_ERR_CLOSED, can_if_write(CAN_PORT_VEHICLE, &v));
    TEST_ASSERT_EQUAL_UINT32(0u, can_if_dispatch(CAN_PORT_VEHICLE, 10u));
    TEST_ASSERT_EQUAL(CAN_PORT_ERR_ARG, can_if_write(CAN_PORT_COUNT, &f));
}

/* Dedicated TX buffers (D-056). */
static void test_register_tx_dedicated_only_on_the_platform_port_until_sealed(void)
{
    TEST_ASSERT_EQUAL(CAN_IF_ERR_ARG, can_if_register_tx_dedicated(CAN_PORT_VEHICLE, ID_A, false));
    TEST_ASSERT_EQUAL(CAN_IF_ERR_ARG, can_if_register_tx_dedicated(CAN_PORT_COUNT, ID_A, false));
    TEST_ASSERT_EQUAL(CAN_IF_ERR_ARG, can_if_register_tx_dedicated(CAN_PORT_PLATFORM, 0x800u, false));
    TEST_ASSERT_EQUAL(CAN_IF_OK, can_if_register_tx_dedicated(CAN_PORT_PLATFORM, ID_A, false));
    TEST_ASSERT_EQUAL(CAN_IF_ERR_DUP, can_if_register_tx_dedicated(CAN_PORT_PLATFORM, ID_A, false));
    TEST_ASSERT_EQUAL(CAN_IF_OK, can_if_register_tx_dedicated(CAN_PORT_PLATFORM, ID_A, true));
    for (uint32_t i = 2u; i < CAN_IF_MAX_TX_DEDICATED; i++) {
        TEST_ASSERT_EQUAL(CAN_IF_OK, can_if_register_tx_dedicated(CAN_PORT_PLATFORM, ID_B + i, false));
    }
    TEST_ASSERT_EQUAL(CAN_IF_ERR_FULL, can_if_register_tx_dedicated(CAN_PORT_PLATFORM, ID_B, false));
    TEST_ASSERT_EQUAL(CAN_PORT_OK, can_if_apply_filters(CAN_PORT_PLATFORM));
    TEST_ASSERT_EQUAL(CAN_IF_ERR_SEALED, can_if_register_tx_dedicated(CAN_PORT_PLATFORM, ID_B, false));
    can_if_init(); /* boot only: clears the list and the seal */
    TEST_ASSERT_EQUAL(CAN_IF_OK, can_if_register_tx_dedicated(CAN_PORT_PLATFORM, ID_B, false));
}

static void test_a_stalled_dedicated_frame_is_replaced_and_no_stale_frame_goes_out_late(void)
{
    TEST_ASSERT_EQUAL(CAN_IF_OK, can_if_register_tx_dedicated(CAN_PORT_PLATFORM, ID_A, false));
    TEST_ASSERT_EQUAL(CAN_PORT_OK, can_if_apply_filters(CAN_PORT_PLATFORM));
    const uint32_t aborts = can_if_tx_abort_count(CAN_PORT_PLATFORM);
    can_frame_t f;
    memset(&f, 0, sizeof f);
    f.id = ID_A;
    f.dlc = 8u;
    vbus_set_tx_stalled(&bus, node_dut, true);
    for (uint8_t cycle = 1u; cycle <= 5u; cycle++) {
        f.data[0] = cycle;
        const can_port_status_t st = can_if_write(CAN_PORT_PLATFORM, &f);
        /* odd cycles fill the buffer, even ones cancel the unsent frame and are refused */
        TEST_ASSERT_EQUAL(((cycle % 2u) != 0u) ? CAN_PORT_OK : CAN_PORT_TX_FULL, st);
    }
    TEST_ASSERT_EQUAL_UINT32(2u, vbus_tx_replaced(&bus, node_dut));
    TEST_ASSERT_TRUE(can_if_tx_free(CAN_PORT_PLATFORM)); /* the Tx FIFO is untouched */
    TEST_ASSERT_EQUAL_UINT32(aborts, can_if_tx_abort_count(CAN_PORT_PLATFORM)); /* not N_As */
    vbus_set_tx_stalled(&bus, node_dut, false);
    can_frame_t got;
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_recv(&bus, node_peer, &got));
    TEST_ASSERT_EQUAL_UINT8(5u, got.data[0]); /* only the newest */
    TEST_ASSERT_EQUAL(CAN_PORT_EMPTY, vbus_recv(&bus, node_peer, &got));

    /* an ID without a dedicated buffer uses the Tx FIFO, one element pending at a time
     * (M_CAN erratum "Tx FIFO message sequence inversion", hal/README.md): never replaced,
     * the next one waits until it went out */
    f.id = ID_B;
    vbus_set_tx_stalled(&bus, node_dut, true);
    f.data[0] = 1u;
    TEST_ASSERT_EQUAL(CAN_PORT_OK, can_if_write(CAN_PORT_PLATFORM, &f));
    TEST_ASSERT_FALSE(can_if_tx_free(CAN_PORT_PLATFORM));
    f.data[0] = 2u;
    TEST_ASSERT_EQUAL(CAN_PORT_TX_FULL, can_if_write(CAN_PORT_PLATFORM, &f));
    f.id = ID_A; /* the dedicated buffer does not wait for the FIFO */
    TEST_ASSERT_EQUAL(CAN_PORT_OK, can_if_write(CAN_PORT_PLATFORM, &f));
    vbus_set_tx_stalled(&bus, node_dut, false); /* arbitration: the lower ID first */
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_recv(&bus, node_peer, &got));
    TEST_ASSERT_EQUAL_UINT32(ID_A, got.id);
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_recv(&bus, node_peer, &got));
    TEST_ASSERT_EQUAL_UINT32(ID_B, got.id);
    TEST_ASSERT_EQUAL_UINT8(1u, got.data[0]);
    TEST_ASSERT_TRUE(can_if_tx_free(CAN_PORT_PLATFORM));
}

static void test_dedicated_buffers_leave_the_vehicle_port_and_its_guard_unchanged(void)
{
    TEST_ASSERT_TRUE(can_port_host_bind_vbus(CAN_PORT_VEHICLE, &bus, node_vehicle));
    TEST_ASSERT_EQUAL(CAN_IF_OK, can_if_register_tx_dedicated(CAN_PORT_PLATFORM, ID_A, false));
    TEST_ASSERT_EQUAL(CAN_PORT_OK, can_if_apply_filters(CAN_PORT_PLATFORM));
    TEST_ASSERT_EQUAL(CAN_PORT_OK, can_if_apply_filters(CAN_PORT_VEHICLE)); /* empty list */
    /* the port refuses a dedicated list on the vehicle port: one TX buffer (D-021) */
    const can_port_tx_id_t req = {VEHICLE_CL250_REQUEST_ID, true};
    TEST_ASSERT_EQUAL(CAN_PORT_ERR_ARG, can_port_set_tx_dedicated(CAN_PORT_VEHICLE, &req, 1u));
    TEST_ASSERT_EQUAL(CAN_PORT_OK, can_port_set_tx_dedicated(CAN_PORT_VEHICLE, NULL, 0u));

    const can_frame_t ok = vehicle_sf(0x22u, 0xF4u, 0x0Cu, 3u);
    const can_frame_t bad = vehicle_sf(0x2Eu, 0xF1u, 0x90u, 3u); /* WriteDataByIdentifier */
    TEST_ASSERT_EQUAL(CAN_PORT_OK, can_if_write(CAN_PORT_VEHICLE, &ok));
    TEST_ASSERT_EQUAL(CAN_PORT_ERR_REFUSED, can_if_write(CAN_PORT_VEHICLE, &bad));
    TEST_ASSERT_EQUAL_UINT32(1u, can_if_tx_refused_count(CAN_PORT_VEHICLE));
    TEST_ASSERT_EQUAL_UINT32(0u, can_if_tx_refused_count(CAN_PORT_PLATFORM));
    TEST_ASSERT_EQUAL_UINT32(0u, vbus_tx_replaced(&bus, node_vehicle));
    /* the vehicle request is held in its single buffer, not replaced, while it is pending */
    vbus_set_tx_stalled(&bus, node_vehicle, true);
    TEST_ASSERT_EQUAL(CAN_PORT_OK, can_if_write(CAN_PORT_VEHICLE, &ok));
    TEST_ASSERT_FALSE(can_if_tx_free(CAN_PORT_VEHICLE));
    TEST_ASSERT_EQUAL_UINT32(0u, vbus_tx_replaced(&bus, node_vehicle));
}

static void test_a_refused_dedicated_list_leaves_the_port_unsealed(void)
{
    can_port_host_unbind(CAN_PORT_PLATFORM);
    TEST_ASSERT_EQUAL(CAN_IF_OK, can_if_register_tx_dedicated(CAN_PORT_PLATFORM, ID_A, false));
    TEST_ASSERT_EQUAL(CAN_PORT_ERR_CLOSED, can_if_apply_filters(CAN_PORT_PLATFORM));
    TEST_ASSERT_EQUAL(CAN_IF_OK, can_if_register_tx_dedicated(CAN_PORT_PLATFORM, ID_B, false));
    TEST_ASSERT_EQUAL(CAN_IF_OK, can_if_register_rx(CAN_PORT_PLATFORM, ID_B, false, on_frame, &sink_b));
    TEST_ASSERT_EQUAL(CAN_PORT_ERR_ARG, can_if_apply_filters(CAN_PORT_COUNT));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_register_rejects_bad_arguments_and_duplicates);
    RUN_TEST(test_table_full);
    RUN_TEST(test_dispatch_routes_by_id_and_format);
    RUN_TEST(test_dispatch_reads_at_most_max_frames);
    RUN_TEST(test_vehicle_guard_passes_only_allowed_single_frames_on_the_request_id);
    RUN_TEST(test_write_goes_to_the_bound_bus_and_unbound_ports_are_closed);
    RUN_TEST(test_register_tx_dedicated_only_on_the_platform_port_until_sealed);
    RUN_TEST(test_a_stalled_dedicated_frame_is_replaced_and_no_stale_frame_goes_out_late);
    RUN_TEST(test_dedicated_buffers_leave_the_vehicle_port_and_its_guard_unchanged);
    RUN_TEST(test_a_refused_dedicated_list_leaves_the_port_unsealed);
    return UNITY_END();
}
