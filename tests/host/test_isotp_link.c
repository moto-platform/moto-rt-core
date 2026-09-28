/*
 * L0 mock-bus tests for the ISO-TP link glue (features/uds/isotp_link, Ç2) on the host
 * platform layer (D-034): in-process bus, manual ms clock, services/can_if routing.
 *
 * Bus nodes: the vehicle port and the platform port of the device under test, a raw
 * ISO-TP peer (isotp_core straight on the bus, also used to inject frames), a sniffer
 * that records every frame with its time, and optionally the simulated CL250 ECU.
 * CL250 tests use the vehicle port; generic (ungated) tests use the platform port,
 * since the vehicle port only takes the CL250 link (D-020).
 * Section numbers refer to ISO 15765-2:2016. No requirement IDs yet (Q-006).
 */
#include "app/host/sim_ecu.h"
#include "features/uds/isotp_link.h"
#include "hal/host/can_port_host.h"
#include "hal/host/hal_time_host.h"
#include "services/timebase.h"
#include "vehicle_cl250.h"

#include <string.h>
#include <unity.h>

/* Test-only identifiers for generic links (not platform or vehicle IDs). */
#define TEST_ID_A 0x100u /* DUT -> peer */
#define TEST_ID_B 0x101u /* peer -> DUT */
#define BUF 4095u
#define SNIFF_MAX 1024u

static vbus_t bus;
static uint8_t node_vehicle, node_platform, node_peer, node_sniff;
static sim_ecu_t ecu;
static bool ecu_on;

static isotp_can_link_t dut;
static uint8_t dut_rx[BUF], dut_tx[BUF];

/* Raw peer: isotp_core on its own node, TX TEST_ID_B, RX TEST_ID_A. */
static isotp_link_t peer;
static bool peer_on;
static uint8_t peer_rx[BUF], peer_tx[BUF];

typedef struct {
    can_frame_t f;
    uint32_t t;
    bool from_dut;
} sniffed_t;
static sniffed_t sniffed[SNIFF_MAX];
static uint32_t sniff_count;

void setUp(void)
{
    hal_time_host_use_manual(1000u);
    vbus_init(&bus);
    TEST_ASSERT_TRUE(vbus_attach(&bus, &node_vehicle));
    TEST_ASSERT_TRUE(vbus_attach(&bus, &node_platform));
    TEST_ASSERT_TRUE(vbus_attach(&bus, &node_peer));
    TEST_ASSERT_TRUE(vbus_attach(&bus, &node_sniff));
    TEST_ASSERT_TRUE(can_port_host_bind_vbus(CAN_PORT_VEHICLE, &bus, node_vehicle));
    TEST_ASSERT_TRUE(can_port_host_bind_vbus(CAN_PORT_PLATFORM, &bus, node_platform));
    can_if_init();
    memset(&dut, 0, sizeof dut);
    sniff_count = 0u;
    ecu_on = false;
    peer_on = false;
}

void tearDown(void)
{
    can_port_host_unbind_all();
    hal_time_host_use_monotonic();
}

static void open_peer(const isotp_config_t* cfg)
{
    TEST_ASSERT_EQUAL(ISOTP_OK, isotp_init(&peer, cfg, peer_rx, BUF, peer_tx, BUF));
    peer_on = true;
}

static void peer_step(uint32_t now)
{
    can_frame_t f;
    while (vbus_recv(&bus, node_peer, &f) == CAN_PORT_OK) {
        if (peer_on && (f.id == TEST_ID_A) && !f.extended) {
            isotp_frame_t in;
            in.dlc = f.dlc;
            memcpy(in.data, f.data, sizeof in.data);
            isotp_on_frame(&peer, &in, now);
        }
    }
    isotp_frame_t out;
    while (peer_on && vbus_tx_free(&bus, node_peer) && isotp_poll(&peer, now, &out)) {
        can_frame_t cf;
        memset(&cf, 0, sizeof cf);
        cf.id = TEST_ID_B;
        cf.dlc = out.dlc;
        memcpy(cf.data, out.data, sizeof cf.data);
        TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_send(&bus, node_peer, &cf));
    }
}

static void sniff(void)
{
    can_frame_t f;
    while (vbus_recv(&bus, node_sniff, &f) == CAN_PORT_OK) {
        if (sniff_count < SNIFF_MAX) {
            sniffed[sniff_count].f = f;
            sniffed[sniff_count].t = timebase_now_ms();
            sniffed[sniff_count].from_dut = dut.open && (f.id == dut.addr.tx_id);
            sniff_count++;
        }
    }
}

/* One main-loop pass per ms, like app/host/main.c. */
static void run_ms(uint32_t ms)
{
    for (uint32_t i = 0u; i < ms; i++) {
        const uint32_t now = timebase_now_ms();
        (void)can_if_dispatch(CAN_PORT_VEHICLE, 64u);
        (void)can_if_dispatch(CAN_PORT_PLATFORM, 64u);
        if (ecu_on) {
            sim_ecu_step(&ecu, now);
            (void)can_if_dispatch(CAN_PORT_VEHICLE, 64u);
        }
        peer_step(now);
        isotp_link_step(&dut);
        sniff();
        hal_time_host_advance(1u);
    }
}

static void open_cl250(void)
{
    TEST_ASSERT_EQUAL(ISOTP_OK, isotp_link_open_vehicle_cl250(&dut, dut_rx, BUF, dut_tx, BUF));
}

/* DUT on the platform port (no D-020 checks) against the raw peer. */
static void open_generic_pair(const isotp_config_t* dut_cfg, const isotp_config_t* peer_cfg)
{
    const isotp_link_addr_t a = {CAN_PORT_PLATFORM, TEST_ID_A, TEST_ID_B, false};
    TEST_ASSERT_EQUAL(ISOTP_OK, isotp_link_open(&dut, &a, dut_cfg, dut_rx, BUF, dut_tx, BUF));
    open_peer(peer_cfg);
}

static void inject(uint32_t id, bool ext, const uint8_t* data, uint8_t dlc)
{
    can_frame_t f;
    memset(&f, 0, sizeof f);
    f.id = id;
    f.extended = ext;
    f.dlc = dlc;
    memcpy(f.data, data, dlc);
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_send(&bus, node_peer, &f));
}

/* ------------------------------------------------------------------------- */
/* CL250 vehicle link: configuration from gen/, wire format, simulated ECU    */
/* ------------------------------------------------------------------------- */

static void test_cl250_link_takes_ids_and_padding_from_gen(void)
{
    open_cl250();
    TEST_ASSERT_EQUAL(CAN_PORT_VEHICLE, dut.addr.port);
    TEST_ASSERT_EQUAL_HEX32(VEHICLE_CL250_REQUEST_ID, dut.addr.tx_id);
    TEST_ASSERT_EQUAL_HEX32(VEHICLE_CL250_RESPONSE_ID, dut.addr.rx_id);
    TEST_ASSERT_TRUE(dut.addr.extended); /* 29-bit IDs (D-019) */
    TEST_ASSERT_TRUE(dut.vehicle);
    TEST_ASSERT_TRUE(dut.iso.cfg.padding_enabled);
    TEST_ASSERT_EQUAL_HEX8(VEHICLE_CL250_PADDING_BYTE, dut.iso.cfg.padding_byte);
    TEST_ASSERT_EQUAL_UINT16(ISOTP_DEFAULT_N_BS_MS, dut.iso.cfg.n_bs_ms);
    TEST_ASSERT_EQUAL_UINT16(ISOTP_DEFAULT_N_CR_MS, dut.iso.cfg.n_cr_ms);
}

static void test_cl250_request_is_a_padded_single_frame_on_the_request_id(void)
{
    open_cl250();
    const uint8_t req[3] = {0x22u, (uint8_t)(VEHICLE_CL250_DID_ENGINE_SPEED >> 8u),
                            (uint8_t)(VEHICLE_CL250_DID_ENGINE_SPEED & 0xFFu)};
    TEST_ASSERT_EQUAL(ISOTP_OK, isotp_link_send(&dut, req, 3u));
    run_ms(1u);
    TEST_ASSERT_EQUAL_UINT32(1u, sniff_count);
    const can_frame_t* f = &sniffed[0].f;
    TEST_ASSERT_EQUAL_HEX32(VEHICLE_CL250_REQUEST_ID, f->id);
    TEST_ASSERT_TRUE(f->extended);
    TEST_ASSERT_EQUAL_UINT8(VEHICLE_CL250_FRAME_DLC, f->dlc);
    const uint8_t p = VEHICLE_CL250_PADDING_BYTE;
    const uint8_t expect[8] = {0x03u, req[0], req[1], req[2], p, p, p, p};
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expect, f->data, 8u);

    isotp_n_result_t res;
    TEST_ASSERT_TRUE(isotp_link_take_tx_confirm(&dut, &res));
    TEST_ASSERT_EQUAL(ISOTP_N_OK, res);
    TEST_ASSERT_EQUAL_UINT32(0u, can_if_tx_refused_count(CAN_PORT_VEHICLE));
}

static void test_cl250_every_gen_did_round_trips_through_the_simulated_ecu(void)
{
    open_cl250();
    TEST_ASSERT_TRUE(sim_ecu_init(&ecu, &bus));
    ecu_on = true;
    for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
        const vehicle_cl250_did_t* e = &vehicle_cl250_dids[i];
        const uint8_t req[3] = {0x22u, (uint8_t)(e->did >> 8u), (uint8_t)(e->did & 0xFFu)};
        TEST_ASSERT_EQUAL(ISOTP_OK, isotp_link_send(&dut, req, 3u));
        run_ms(5u);
        isotp_n_result_t res;
        uint16_t len = 0u;
        TEST_ASSERT_TRUE(isotp_link_take_rx(&dut, &res, &len));
        TEST_ASSERT_EQUAL(ISOTP_N_OK, res);
        TEST_ASSERT_EQUAL_UINT16(3u + e->length, len);
        const uint8_t* d = isotp_link_rx_data(&dut);
        TEST_ASSERT_EQUAL_HEX8(0x62u, d[0]);
        TEST_ASSERT_EQUAL_HEX8(req[1], d[1]);
        TEST_ASSERT_EQUAL_HEX8(req[2], d[2]);
        float v = 0.0f;
        TEST_ASSERT_TRUE(vehicle_cl250_decode(e, &d[3], (size_t)len - 3u, &v));
        isotp_link_rx_release(&dut);
    }
    TEST_ASSERT_EQUAL_UINT32(VEHICLE_CL250_DID_COUNT, ecu.requests);
    TEST_ASSERT_EQUAL_UINT32(0u, isotp_link_tx_refused_count(&dut));
    TEST_ASSERT_EQUAL_UINT32(0u, isotp_link_tx_error_count(&dut));
    TEST_ASSERT_EQUAL_UINT32(0u, can_if_tx_refused_count(CAN_PORT_VEHICLE));
}

/* ------------------------------------------------------------------------- */
/* D-020 on the vehicle link                                                  */
/* ------------------------------------------------------------------------- */

static void test_cl250_refuses_forbidden_services(void)
{
    open_cl250();
    static const uint8_t forbidden[][3] = {
        {0x10u, 0x02u, 0x00u}, /* programming session */
        {0x11u, 0x01u, 0x00u}, /* ECU reset */
        {0x14u, 0xFFu, 0xFFu}, /* clear DTCs */
        {0x27u, 0x01u, 0x00u}, /* security access */
        {0x2Eu, 0xF1u, 0x90u}, /* write DID */
        {0x31u, 0x01u, 0xFFu}, /* routine control */
    };
    const uint32_t n = sizeof forbidden / sizeof forbidden[0];
    for (uint32_t i = 0u; i < n; i++) {
        TEST_ASSERT_EQUAL(ISOTP_ERR_ARG, isotp_link_send(&dut, forbidden[i], 3u));
    }
    run_ms(10u);
    TEST_ASSERT_EQUAL_UINT32(0u, sniff_count);
    TEST_ASSERT_EQUAL_UINT32(n, isotp_link_tx_refused_count(&dut));
    TEST_ASSERT_EQUAL_UINT32(0u, vbus_frame_count(&bus));
}

static void test_cl250_refuses_a_request_that_needs_a_first_frame(void)
{
    open_cl250();
    /* 0x22 is allowed, but 9 bytes would need a First Frame (§9.6.3), which D-020
     * does not allow: refused at once, the link stays free. */
    const uint8_t req[9] = {0x22u, 0xF4u, 0x0Cu, 0xF4u, 0x0Du, 0xF4u, 0x11u, 0xF4u, 0x05u};
    TEST_ASSERT_EQUAL(ISOTP_ERR_LENGTH, isotp_link_send(&dut, req, 9u));
    TEST_ASSERT_EQUAL_UINT32(1u, isotp_link_tx_refused_count(&dut));
    TEST_ASSERT_EQUAL(ISOTP_OK, isotp_link_send(&dut, req, 3u));
    run_ms(2u);
    TEST_ASSERT_EQUAL_UINT32(1u, sniff_count);
}

/* Documents the open question: the D-020 frame gate refuses FC frames, so the vehicle
 * link cannot receive a segmented response (e.g. a long 0x19 or 0x09 answer). */
static void test_cl250_segmented_response_gets_no_flow_control(void)
{
    open_cl250();
    const uint8_t ff[8] = {0x10u, 0x0Au, 0x62u, 0xF4u, 0x0Cu, 0x01u, 0x02u, 0x03u};
    inject(VEHICLE_CL250_RESPONSE_ID, true, ff, 8u);
    run_ms(2u);
    TEST_ASSERT_EQUAL_UINT32(1u, sniff_count); /* only the injected FF, no FC */
    TEST_ASSERT_FALSE(sniffed[0].from_dut);
    TEST_ASSERT_EQUAL_UINT32(1u, isotp_link_tx_refused_count(&dut));
    run_ms(ISOTP_DEFAULT_N_CR_MS);
    isotp_n_result_t res;
    uint16_t len;
    TEST_ASSERT_TRUE(isotp_link_take_rx(&dut, &res, &len));
    TEST_ASSERT_EQUAL(ISOTP_N_TIMEOUT_CR, res);
}

static void test_cl250_link_ignores_other_ids_and_formats(void)
{
    open_cl250();
    const uint8_t sf[8] = {0x03u, 0x62u, 0xF4u, 0x0Du, 0xAAu, 0xAAu, 0xAAu, 0xAAu};
    inject(VEHICLE_CL250_REQUEST_ID, true, sf, 8u);                        /* our own request ID */
    inject(VEHICLE_CL250_RESPONSE_ID & CAN_PORT_STD_ID_MAX, false, sf, 8u); /* 11-bit */
    inject(VEHICLE_CL250_RESPONSE_ID + 1u, true, sf, 8u);
    run_ms(2u);
    isotp_n_result_t res;
    uint16_t len;
    TEST_ASSERT_FALSE(isotp_link_take_rx(&dut, &res, &len));
    TEST_ASSERT_EQUAL_UINT32(3u, can_if_unrouted_count(CAN_PORT_VEHICLE));
    inject(VEHICLE_CL250_RESPONSE_ID, true, sf, 8u);
    run_ms(1u);
    TEST_ASSERT_TRUE(isotp_link_take_rx(&dut, &res, &len));
    TEST_ASSERT_EQUAL(ISOTP_N_OK, res);
    TEST_ASSERT_EQUAL_UINT16(3u, len);
}

static void test_generic_link_cannot_open_on_the_vehicle_port(void)
{
    isotp_config_t cfg;
    isotp_default_config(&cfg);
    const isotp_link_addr_t v = {CAN_PORT_VEHICLE, TEST_ID_A, TEST_ID_B, false};
    TEST_ASSERT_EQUAL(ISOTP_ERR_ARG, isotp_link_open(&dut, &v, &cfg, dut_rx, BUF, dut_tx, BUF));
    TEST_ASSERT_FALSE(dut.open);
    /* even an ungated frame straight to can_if is refused on the vehicle port */
    can_frame_t f;
    memset(&f, 0, sizeof f);
    f.id = TEST_ID_A;
    f.dlc = 8u;
    f.data[0] = 0x10u; /* a First Frame */
    TEST_ASSERT_EQUAL(CAN_PORT_ERR_REFUSED, can_if_write(CAN_PORT_VEHICLE, &f));
    TEST_ASSERT_EQUAL_UINT32(0u, vbus_frame_count(&bus));
}

/* ------------------------------------------------------------------------- */
/* Generic link over the bus: segmentation, flow control, timing, faults      */
/* ------------------------------------------------------------------------- */

static void test_segmented_message_with_block_size_and_st_min_over_the_bus(void)
{
    isotp_config_t dut_cfg, peer_cfg;
    isotp_default_config(&dut_cfg);
    isotp_default_config(&peer_cfg);
    peer_cfg.block_size = 2u; /* §9.6.5.3 */
    peer_cfg.st_min = 5u;     /* 5 ms, §9.6.5.4 */
    open_generic_pair(&dut_cfg, &peer_cfg);

    uint8_t msg[100];
    for (uint16_t i = 0u; i < sizeof msg; i++) {
        msg[i] = (uint8_t)(i * 3u + 1u);
    }
    TEST_ASSERT_EQUAL(ISOTP_OK, isotp_link_send(&dut, msg, (uint16_t)sizeof msg));
    run_ms(200u);

    isotp_n_result_t res;
    uint16_t len = 0u;
    TEST_ASSERT_TRUE(isotp_take_rx_indication(&peer, &res, &len));
    TEST_ASSERT_EQUAL(ISOTP_N_OK, res);
    TEST_ASSERT_EQUAL_UINT16(sizeof msg, len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(msg, peer_rx, sizeof msg);
    TEST_ASSERT_TRUE(isotp_link_take_tx_confirm(&dut, &res));
    TEST_ASSERT_EQUAL(ISOTP_N_OK, res);

    /* FF + 14 CF from the DUT, one FC per block of 2 CF from the peer: 7 FC.
     * STmin separates the CFs inside a block. */
    uint32_t cf = 0u, fcs = 0u;
    uint32_t last_cf_t = 0u;
    bool cf_in_block = false;
    for (uint32_t i = 0u; i < sniff_count; i++) {
        const uint8_t pci = (uint8_t)(sniffed[i].f.data[0] >> 4u);
        if (!sniffed[i].from_dut) {
            TEST_ASSERT_EQUAL_UINT8(3u, pci);
            fcs++;
            cf_in_block = false;
        } else if (pci == 2u) {
            if (cf_in_block) {
                TEST_ASSERT_GREATER_OR_EQUAL_UINT32(5u, sniffed[i].t - last_cf_t);
            }
            last_cf_t = sniffed[i].t;
            cf_in_block = true;
            cf++;
        } else {
            TEST_ASSERT_EQUAL_UINT8(1u, pci);
        }
    }
    TEST_ASSERT_EQUAL_UINT32(14u, cf);
    TEST_ASSERT_EQUAL_UINT32(7u, fcs);
    TEST_ASSERT_EQUAL_UINT32(0u, isotp_link_tx_error_count(&dut));
}

static void test_segmented_message_from_the_peer_is_reassembled(void)
{
    isotp_config_t cfg;
    isotp_default_config(&cfg);
    cfg.block_size = 4u; /* the DUT's FC asks for blocks of 4 */
    isotp_config_t peer_cfg;
    isotp_default_config(&peer_cfg);
    open_generic_pair(&cfg, &peer_cfg);
    uint8_t msg[60];
    for (uint16_t i = 0u; i < sizeof msg; i++) {
        msg[i] = (uint8_t)(0xC0u ^ i);
    }
    TEST_ASSERT_EQUAL(ISOTP_OK, isotp_send(&peer, msg, (uint16_t)sizeof msg));
    run_ms(50u);
    isotp_n_result_t res;
    uint16_t len = 0u;
    TEST_ASSERT_TRUE(isotp_link_take_rx(&dut, &res, &len));
    TEST_ASSERT_EQUAL(ISOTP_N_OK, res);
    TEST_ASSERT_EQUAL_UINT16(sizeof msg, len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(msg, isotp_link_rx_data(&dut), sizeof msg);
    isotp_link_rx_release(&dut);
}

static void test_full_tx_mailbox_holds_frames_until_it_frees(void)
{
    isotp_config_t cfg;
    isotp_default_config(&cfg);
    open_generic_pair(&cfg, &cfg);
    vbus_set_tx_blocked(&bus, node_platform, true);
    const uint8_t msg[5] = {1u, 2u, 3u, 4u, 5u};
    TEST_ASSERT_EQUAL(ISOTP_OK, isotp_link_send(&dut, msg, 5u));
    run_ms(20u);
    TEST_ASSERT_EQUAL_UINT32(0u, sniff_count);
    TEST_ASSERT_EQUAL_UINT32(0u, isotp_link_tx_error_count(&dut));
    vbus_set_tx_blocked(&bus, node_platform, false);
    run_ms(2u);
    isotp_n_result_t res;
    uint16_t len;
    TEST_ASSERT_TRUE(isotp_take_rx_indication(&peer, &res, &len));
    TEST_ASSERT_EQUAL(ISOTP_N_OK, res);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(msg, peer_rx, 5u);
}

static void test_sender_going_silent_mid_message_gives_n_cr_on_the_receiver(void)
{
    isotp_config_t cfg;
    isotp_default_config(&cfg);
    open_generic_pair(&cfg, &cfg);
    uint8_t msg[40];
    memset(msg, 0x5A, sizeof msg);
    TEST_ASSERT_EQUAL(ISOTP_OK, isotp_send(&peer, msg, (uint16_t)sizeof msg));
    run_ms(2u); /* FF from the peer, FC.CTS from the DUT */
    vbus_set_tx_blocked(&bus, node_peer, true);
    run_ms(ISOTP_DEFAULT_N_CR_MS + 2u);
    isotp_n_result_t res;
    uint16_t len;
    TEST_ASSERT_TRUE(isotp_link_take_rx(&dut, &res, &len));
    TEST_ASSERT_EQUAL(ISOTP_N_TIMEOUT_CR, res);
    TEST_ASSERT_EQUAL_UINT16(1u, isotp_link_rx_error_count(&dut));
}

static void test_one_step_writes_at_most_the_per_step_limit(void)
{
    isotp_config_t cfg;
    isotp_default_config(&cfg); /* BS 0, STmin 0: the whole message may flow */
    open_generic_pair(&cfg, &cfg);
    static uint8_t big[BUF];
    memset(big, 0x11, sizeof big);
    TEST_ASSERT_EQUAL(ISOTP_OK, isotp_link_send(&dut, big, (uint16_t)sizeof big));
    run_ms(2u); /* FF, then FC.CTS from the peer */
    const uint32_t before = sniff_count;
    run_ms(1u);
    TEST_ASSERT_EQUAL_UINT32(ISOTP_LINK_MAX_TX_PER_STEP, sniff_count - before);
    run_ms(300u);
    isotp_n_result_t res;
    uint16_t len;
    TEST_ASSERT_TRUE(isotp_take_rx_indication(&peer, &res, &len));
    TEST_ASSERT_EQUAL(ISOTP_N_OK, res);
    TEST_ASSERT_EQUAL_UINT16(BUF, len);
    TEST_ASSERT_EQUAL_UINT32(0u, vbus_overruns(&bus, node_peer));
}

static void test_open_rejects_bad_arguments_and_double_open(void)
{
    isotp_config_t cfg;
    isotp_default_config(&cfg);
    const isotp_link_addr_t same = {CAN_PORT_PLATFORM, TEST_ID_A, TEST_ID_A, false};
    const isotp_link_addr_t tx_big = {CAN_PORT_PLATFORM, 0x800u, TEST_ID_B, false};
    const isotp_link_addr_t rx_big = {CAN_PORT_PLATFORM, TEST_ID_A, 0x800u, false};
    const isotp_link_addr_t ok = {CAN_PORT_PLATFORM, TEST_ID_A, TEST_ID_B, false};
    TEST_ASSERT_EQUAL(ISOTP_ERR_ARG, isotp_link_open(NULL, &ok, &cfg, dut_rx, BUF, dut_tx, BUF));
    TEST_ASSERT_EQUAL(ISOTP_ERR_ARG, isotp_link_open(&dut, NULL, &cfg, dut_rx, BUF, dut_tx, BUF));
    TEST_ASSERT_EQUAL(ISOTP_ERR_ARG, isotp_link_open(&dut, &same, &cfg, dut_rx, BUF, dut_tx, BUF));
    TEST_ASSERT_EQUAL(ISOTP_ERR_ARG, isotp_link_open(&dut, &tx_big, &cfg, dut_rx, BUF, dut_tx, BUF));
    TEST_ASSERT_EQUAL(ISOTP_ERR_ARG, isotp_link_open(&dut, &rx_big, &cfg, dut_rx, BUF, dut_tx, BUF));
    TEST_ASSERT_EQUAL(ISOTP_ERR_ARG, isotp_link_open(&dut, &ok, &cfg, NULL, BUF, dut_tx, BUF));
    TEST_ASSERT_EQUAL(ISOTP_ERR_ARG, isotp_link_open_vehicle_cl250(NULL, dut_rx, BUF, dut_tx, BUF));
    TEST_ASSERT_EQUAL(ISOTP_OK, isotp_link_open(&dut, &ok, &cfg, dut_rx, BUF, dut_tx, BUF));
    TEST_ASSERT_EQUAL(ISOTP_ERR_BUSY, isotp_link_open(&dut, &ok, &cfg, dut_rx, BUF, dut_tx, BUF));
    TEST_ASSERT_TRUE(dut.open); /* a second open leaves the link working */

    /* a second link on the same RX ID cannot register with can_if */
    static isotp_can_link_t other;
    memset(&other, 0, sizeof other);
    static uint8_t o_rx[8], o_tx[8];
    TEST_ASSERT_EQUAL(ISOTP_ERR_ARG, isotp_link_open(&other, &ok, &cfg, o_rx, 8u, o_tx, 8u));
    TEST_ASSERT_EQUAL(ISOTP_ERR_ARG, isotp_link_send(&other, o_tx, 1u));
    isotp_link_step(&other); /* closed link: no effect */
    TEST_ASSERT_NULL(isotp_link_rx_data(&other));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_cl250_link_takes_ids_and_padding_from_gen);
    RUN_TEST(test_cl250_request_is_a_padded_single_frame_on_the_request_id);
    RUN_TEST(test_cl250_every_gen_did_round_trips_through_the_simulated_ecu);
    RUN_TEST(test_cl250_refuses_forbidden_services);
    RUN_TEST(test_cl250_refuses_a_request_that_needs_a_first_frame);
    RUN_TEST(test_cl250_segmented_response_gets_no_flow_control);
    RUN_TEST(test_cl250_link_ignores_other_ids_and_formats);
    RUN_TEST(test_generic_link_cannot_open_on_the_vehicle_port);
    RUN_TEST(test_segmented_message_with_block_size_and_st_min_over_the_bus);
    RUN_TEST(test_segmented_message_from_the_peer_is_reassembled);
    RUN_TEST(test_full_tx_mailbox_holds_frames_until_it_frees);
    RUN_TEST(test_sender_going_silent_mid_message_gives_n_cr_on_the_receiver);
    RUN_TEST(test_one_step_writes_at_most_the_per_step_limit);
    RUN_TEST(test_open_rejects_bad_arguments_and_double_open);
    return UNITY_END();
}
