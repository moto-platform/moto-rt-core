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
#include "services/can_if.h"
#include "services/can_sm.h"
#include "services/timebase.h"
#include "vehicle_cl250.h"

#include <string.h>
#include <unity.h>

/* Test-only identifiers for generic links (not platform or vehicle IDs). */
#define TEST_ID_A 0x100u /* DUT -> peer */
#define TEST_ID_B 0x101u /* peer -> DUT */
#define TEST_ID_RAW 0x1F0u /* a raw frame the tests write through can_if (never delivered) */
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
/* ---------------------------------------------------- D-059 Flow Control (vehicle) */

static const uint8_t ff10[8] = {0x10u, 0x0Au, 0x62u, 0xF4u, 0x0Cu, 0x01u, 0x02u, 0x03u};

/* An allowed read goes out and the client arms the link for its answer. */
static void request_and_expect(void)
{
    open_cl250();
    const uint8_t req[3] = {0x22u, (uint8_t)(VEHICLE_CL250_DID_ENGINE_SPEED >> 8u),
                            (uint8_t)(VEHICLE_CL250_DID_ENGINE_SPEED & 0xFFu)};
    TEST_ASSERT_EQUAL(ISOTP_OK, isotp_link_send(&dut, req, 3u));
    isotp_link_expect_response(&dut, 0x62u); /* ReadDataByIdentifier positive response */
    run_ms(1u);
    sniff_count = 0u;
}

static uint32_t dut_frames(void)
{
    uint32_t n = 0u;
    for (uint32_t i = 0u; i < sniff_count; i++) {
        n += sniffed[i].from_dut ? 1u : 0u;
    }
    return n;
}

static void test_cl250_first_frame_of_the_expected_answer_gets_the_one_gen_fc(void)
{
    request_and_expect();
    inject(VEHICLE_CL250_RESPONSE_ID, true, ff10, 8u);
    run_ms(2u);
    TEST_ASSERT_EQUAL_UINT32(2u, sniff_count); /* the FF, then our FC */
    TEST_ASSERT_TRUE(sniffed[1].from_dut);
    TEST_ASSERT_EQUAL_HEX32(VEHICLE_CL250_REQUEST_ID, sniffed[1].f.id); /* physical, never fallback */
    TEST_ASSERT_TRUE(sniffed[1].f.extended);
    TEST_ASSERT_EQUAL_UINT8(VEHICLE_CL250_FRAME_DLC, sniffed[1].f.dlc);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(vehicle_cl250_fc_cts, sniffed[1].f.data, VEHICLE_CL250_FRAME_DLC);
    TEST_ASSERT_FALSE(dut.response_open); /* one FC per request */
    const uint8_t cf[8] = {0x21u, 0x04u, 0x05u, 0x06u, 0x07u, 0xAAu, 0xAAu, 0xAAu};
    inject(VEHICLE_CL250_RESPONSE_ID, true, cf, 8u);
    run_ms(1u);
    isotp_n_result_t res;
    uint16_t len;
    TEST_ASSERT_TRUE(isotp_link_take_rx(&dut, &res, &len));
    TEST_ASSERT_EQUAL(ISOTP_N_OK, res);
    TEST_ASSERT_EQUAL_UINT16(10u, len);
    TEST_ASSERT_EQUAL_UINT32(0u, isotp_link_fc_withheld_count(&dut));
    TEST_ASSERT_EQUAL_UINT32(0u, isotp_link_tx_refused_count(&dut));
    TEST_ASSERT_EQUAL_UINT32(0u, can_if_tx_refused_count(CAN_PORT_VEHICLE));
}

static void test_cl250_a_second_first_frame_in_one_request_gets_no_second_fc(void)
{
    request_and_expect();
    inject(VEHICLE_CL250_RESPONSE_ID, true, ff10, 8u);
    run_ms(2u);
    inject(VEHICLE_CL250_RESPONSE_ID, true, ff10, 8u); /* restarts the reception */
    run_ms(2u);
    TEST_ASSERT_EQUAL_UINT32(1u, dut_frames());
    TEST_ASSERT_EQUAL_UINT32(1u, isotp_link_fc_withheld_count(&dut));
    TEST_ASSERT_TRUE(isotp_link_take_fc_withheld(&dut));
    isotp_n_result_t res;
    uint16_t len;
    TEST_ASSERT_TRUE(isotp_link_take_rx(&dut, &res, &len));
    TEST_ASSERT_EQUAL(ISOTP_N_UNEXP_PDU, res); /* the first reception, ended by the second FF */
    TEST_ASSERT_FALSE(isotp_link_rx_busy(&dut));
}

static void test_cl250_first_frame_above_the_cap_gets_nothing_on_the_bus(void)
{
    request_and_expect();
    const uint16_t ff_dl = (uint16_t)(VEHICLE_CL250_MAX_FF_DL + 1u);
    uint8_t ff[8];
    memcpy(ff, ff10, sizeof ff);
    ff[0] = (uint8_t)(0x10u | (ff_dl >> 8u));
    ff[1] = (uint8_t)(ff_dl & 0xFFu);
    inject(VEHICLE_CL250_RESPONSE_ID, true, ff, 8u);
    run_ms(2u);
    TEST_ASSERT_EQUAL_UINT32(0u, dut_frames()); /* no FC.OVFLW either */
    TEST_ASSERT_EQUAL_UINT32(1u, isotp_link_fc_withheld_count(&dut));
    TEST_ASSERT_EQUAL_UINT32(0u, isotp_link_tx_refused_count(&dut));
    TEST_ASSERT_FALSE(isotp_link_rx_busy(&dut));
}

static void test_cl250_missing_cf_after_the_fc_ends_in_n_cr_base(void)
{
    request_and_expect();
    inject(VEHICLE_CL250_RESPONSE_ID, true, ff10, 8u);
    run_ms(2u);
    TEST_ASSERT_EQUAL_UINT32(1u, dut_frames());
    run_ms(VEHICLE_CL250_RESPONSE_TIMEOUT_BASE_MS - 3u);
    isotp_n_result_t res;
    uint16_t len;
    TEST_ASSERT_FALSE(isotp_link_take_rx(&dut, &res, &len));
    run_ms(3u);
    TEST_ASSERT_TRUE(isotp_link_take_rx(&dut, &res, &len));
    TEST_ASSERT_EQUAL(ISOTP_N_TIMEOUT_CR, res);
}

static void test_cl250_wrong_sequence_number_after_the_fc_aborts(void)
{
    request_and_expect();
    inject(VEHICLE_CL250_RESPONSE_ID, true, ff10, 8u);
    run_ms(2u);
    const uint8_t cf[8] = {0x22u, 0x04u, 0x05u, 0x06u, 0x07u, 0xAAu, 0xAAu, 0xAAu};
    inject(VEHICLE_CL250_RESPONSE_ID, true, cf, 8u);
    run_ms(1u);
    isotp_n_result_t res;
    uint16_t len;
    TEST_ASSERT_TRUE(isotp_link_take_rx(&dut, &res, &len));
    TEST_ASSERT_EQUAL(ISOTP_N_WRONG_SN, res);
}

/* MINOR-2: an FF that starts with another response SID (a late answer to an earlier
 * request of another service) gets no FC. */
static void test_cl250_first_frame_with_another_response_sid_gets_no_fc(void)
{
    request_and_expect();
    uint8_t ff[8];
    memcpy(ff, ff10, sizeof ff);
    ff[2] = 0x50u; /* a session answer, not the expected 0x62 */
    inject(VEHICLE_CL250_RESPONSE_ID, true, ff, 8u);
    run_ms(2u);
    TEST_ASSERT_EQUAL_UINT32(0u, dut_frames());
    TEST_ASSERT_EQUAL_UINT32(1u, isotp_link_fc_withheld_count(&dut));
    TEST_ASSERT_TRUE(dut.response_open); /* the expected answer may still come */
}

/* The clamp's upper boundary: FF_DL = VEHICLE_CL250_MAX_FF_DL is received. */
static void test_cl250_first_frame_at_the_cap_gets_the_fc(void)
{
    request_and_expect();
    uint8_t ff[8];
    memcpy(ff, ff10, sizeof ff);
    ff[0] = (uint8_t)(0x10u | ((uint16_t)VEHICLE_CL250_MAX_FF_DL >> 8u));
    ff[1] = (uint8_t)((uint16_t)VEHICLE_CL250_MAX_FF_DL & 0xFFu);
    inject(VEHICLE_CL250_RESPONSE_ID, true, ff, 8u);
    run_ms(2u);
    TEST_ASSERT_EQUAL_UINT32(1u, dut_frames());
    TEST_ASSERT_EQUAL_HEX8_ARRAY(vehicle_cl250_fc_cts, sniffed[1].f.data, VEHICLE_CL250_FRAME_DLC);
    TEST_ASSERT_TRUE(isotp_link_rx_busy(&dut));
}

static void test_cl250_closed_or_unarmed_response_gets_no_fc(void)
{
    request_and_expect();
    isotp_link_close_response(&dut); /* answered, timed out or latched */
    inject(VEHICLE_CL250_RESPONSE_ID, true, ff10, 8u);
    run_ms(2u);
    TEST_ASSERT_EQUAL_UINT32(0u, dut_frames());
    TEST_ASSERT_EQUAL_UINT32(1u, isotp_link_fc_withheld_count(&dut));

    /* a request sent without arming (tester present expects no answer) */
    const uint8_t tp[2] = {VEHICLE_CL250_TESTER_PRESENT_SID, VEHICLE_CL250_TESTER_PRESENT_SUBFUNCTION};
    TEST_ASSERT_EQUAL(ISOTP_OK, isotp_link_send(&dut, tp, 2u));
    run_ms(1u);
    sniff_count = 0u;
    inject(VEHICLE_CL250_RESPONSE_ID, true, ff10, 8u);
    run_ms(2u);
    TEST_ASSERT_EQUAL_UINT32(0u, dut_frames());
    TEST_ASSERT_EQUAL_UINT32(2u, isotp_link_fc_withheld_count(&dut));
}

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
    /* D-059: N_Cr = response_timeout_base_ms, FC parameters from gen/ */
    TEST_ASSERT_EQUAL_UINT16(VEHICLE_CL250_RESPONSE_TIMEOUT_BASE_MS, dut.iso.cfg.n_cr_ms);
    TEST_ASSERT_EQUAL_UINT8(VEHICLE_CL250_FC_BLOCK_SIZE, dut.iso.cfg.block_size);
    TEST_ASSERT_EQUAL_UINT8(VEHICLE_CL250_FC_ST_MIN_MS, dut.iso.cfg.st_min);
    TEST_ASSERT_FALSE(dut.response_open); /* fail-closed until the client arms it */
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
static void test_cl250_unsolicited_first_frame_gets_no_flow_control(void)
{
    open_cl250();
    const uint8_t ff[8] = {0x10u, 0x0Au, 0x62u, 0xF4u, 0x0Cu, 0x01u, 0x02u, 0x03u};
    inject(VEHICLE_CL250_RESPONSE_ID, true, ff, 8u);
    run_ms(2u);
    TEST_ASSERT_EQUAL_UINT32(1u, sniff_count); /* only the injected FF, no FC */
    TEST_ASSERT_FALSE(sniffed[0].from_dut);
    /* withheld, not refused: the reception ends at once, without an indication */
    TEST_ASSERT_EQUAL_UINT32(0u, isotp_link_tx_refused_count(&dut));
    TEST_ASSERT_EQUAL_UINT32(1u, isotp_link_fc_withheld_count(&dut));
    TEST_ASSERT_FALSE(isotp_link_rx_busy(&dut));
    TEST_ASSERT_TRUE(isotp_link_take_fc_withheld(&dut));
    TEST_ASSERT_FALSE(isotp_link_take_fc_withheld(&dut)); /* once */
    TEST_ASSERT_EQUAL_UINT16(1u, isotp_link_rx_error_count(&dut));
    run_ms(ISOTP_DEFAULT_N_CR_MS);
    isotp_n_result_t res;
    uint16_t len;
    TEST_ASSERT_FALSE(isotp_link_take_rx(&dut, &res, &len)); /* no N_Cr timeout follows */
    TEST_ASSERT_EQUAL_UINT32(1u, sniff_count);
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

/* ------------------------------------------------------------------------- */
/* N_As (Ç1): services/can_sm aborts stuck frames, the link ends the message  */
/* ------------------------------------------------------------------------- */

/* run_ms() with the platform port's state manager in front, like app/comms.c. */
static void run_ms_sm(uint32_t ms)
{
    for (uint32_t i = 0u; i < ms; i++) {
        can_sm_step(CAN_PORT_PLATFORM);
        run_ms(1u);
    }
}

/* A raw frame through can_if on the stalled platform node: it sits in the TX buffers. */
static void write_raw_frame_into_the_stalled_node(void)
{
    can_frame_t raw;
    memset(&raw, 0, sizeof raw);
    raw.id = TEST_ID_RAW;
    raw.dlc = 1u;
    TEST_ASSERT_EQUAL(CAN_PORT_OK, can_if_write(CAN_PORT_PLATFORM, &raw));
}

static uint32_t consecutive_frames_from_dut(void)
{
    uint32_t n = 0u;
    for (uint32_t i = 0u; i < sniff_count; i++) {
        n += (sniffed[i].from_dut && ((sniffed[i].f.data[0] >> 4u) == 2u)) ? 1u : 0u;
    }
    return n;
}

static void test_n_as_abort_ends_a_stalled_multi_frame_message_with_n_timeout_a(void)
{
    isotp_config_t cfg;
    isotp_default_config(&cfg);
    open_generic_pair(&cfg, &cfg);
    can_sm_step(CAN_PORT_PLATFORM); /* baseline of the controller counters */
    uint8_t msg[100];
    memset(msg, 0x6B, sizeof msg);
    TEST_ASSERT_EQUAL(ISOTP_OK, isotp_link_send(&dut, msg, (uint16_t)sizeof msg));
    run_ms_sm(2u); /* the First Frame goes out, the peer answers with FC.CTS */
    TEST_ASSERT_GREATER_THAN_UINT32(0u, sniff_count); /* the First Frame */
    vbus_set_tx_stalled(&bus, node_platform, true); /* the Consecutive Frames get stuck */

    run_ms_sm(CAN_SM_TX_TIMEOUT_MS - 10u);
    isotp_n_result_t res;
    TEST_ASSERT_FALSE(isotp_link_take_tx_confirm(&dut, &res)); /* still within N_As */
    TEST_ASSERT_EQUAL_UINT32(0u, isotp_link_tx_timeout_count(&dut));

    run_ms_sm(20u); /* N_As expires, can_sm aborts, the link ends the message */
    TEST_ASSERT_TRUE(isotp_link_take_tx_confirm(&dut, &res));
    TEST_ASSERT_EQUAL(ISOTP_N_TIMEOUT_A, res);
    TEST_ASSERT_EQUAL_UINT32(1u, isotp_link_tx_timeout_count(&dut));
    TEST_ASSERT_EQUAL_UINT32(1u, can_sm_stats(CAN_PORT_PLATFORM)->tx_timeouts);
    TEST_ASSERT_FALSE(isotp_link_take_tx_confirm(&dut, &res)); /* once only */

    /* the stale Consecutive Frames never go out when the stall ends */
    vbus_set_tx_stalled(&bus, node_platform, false);
    run_ms_sm(50u);
    TEST_ASSERT_EQUAL_UINT32(0u, consecutive_frames_from_dut());
    TEST_ASSERT_EQUAL_UINT32(1u, isotp_link_tx_timeout_count(&dut));

    /* the peer's reception of the cut message dies with N_Cr; the link is usable again */
    uint16_t len;
    TEST_ASSERT_TRUE(isotp_take_rx_indication(&peer, &res, &len));
    TEST_ASSERT_EQUAL(ISOTP_N_TIMEOUT_CR, res);
    TEST_ASSERT_TRUE(isotp_link_tx_ready(&dut));
    uint8_t again[20];
    memset(again, 0x2C, sizeof again);
    TEST_ASSERT_EQUAL(ISOTP_OK, isotp_link_send(&dut, again, (uint16_t)sizeof again));
    run_ms_sm(100u);
    TEST_ASSERT_TRUE(isotp_link_take_tx_confirm(&dut, &res));
    TEST_ASSERT_EQUAL(ISOTP_N_OK, res);
    TEST_ASSERT_TRUE(isotp_take_rx_indication(&peer, &res, &len));
    TEST_ASSERT_EQUAL(ISOTP_N_OK, res);
    TEST_ASSERT_EQUAL_UINT16(sizeof again, len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(again, peer_rx, sizeof again);
}

static void test_an_abort_before_isotp_link_send_does_not_end_the_new_message(void)
{
    isotp_config_t cfg;
    isotp_default_config(&cfg);
    open_generic_pair(&cfg, &cfg);
    can_sm_step(CAN_PORT_PLATFORM);
    /* An N_As abort of a frame that is not the link's, with the link not stepping meanwhile. */
    vbus_set_tx_stalled(&bus, node_platform, true);
    write_raw_frame_into_the_stalled_node();
    can_sm_step(CAN_PORT_PLATFORM);
    hal_time_host_advance(CAN_SM_TX_TIMEOUT_MS);
    can_sm_step(CAN_PORT_PLATFORM);
    TEST_ASSERT_EQUAL_UINT32(1u, can_sm_stats(CAN_PORT_PLATFORM)->tx_timeouts);
    TEST_ASSERT_NOT_EQUAL_UINT32(dut.abort_seen, can_if_tx_abort_count(CAN_PORT_PLATFORM));
    vbus_set_tx_stalled(&bus, node_platform, false);

    /* The new message is multi-frame, so it is under way for several steps. */
    uint8_t msg[30];
    for (uint16_t i = 0u; i < sizeof msg; i++) {
        msg[i] = (uint8_t)(0x40u + i);
    }
    TEST_ASSERT_EQUAL(ISOTP_OK, isotp_link_send(&dut, msg, (uint16_t)sizeof msg));
    run_ms_sm(100u);
    isotp_n_result_t res;
    TEST_ASSERT_TRUE(isotp_link_take_tx_confirm(&dut, &res));
    TEST_ASSERT_EQUAL(ISOTP_N_OK, res); /* not N_TIMEOUT_A */
    TEST_ASSERT_EQUAL_UINT32(0u, isotp_link_tx_timeout_count(&dut));
    uint16_t len;
    TEST_ASSERT_TRUE(isotp_take_rx_indication(&peer, &res, &len));
    TEST_ASSERT_EQUAL(ISOTP_N_OK, res);
    TEST_ASSERT_EQUAL_UINT16(sizeof msg, len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(msg, peer_rx, sizeof msg);
}

static void test_an_abort_while_the_link_is_idle_is_not_counted_and_confirms_nothing(void)
{
    isotp_config_t cfg;
    isotp_default_config(&cfg);
    open_generic_pair(&cfg, &cfg);
    can_sm_step(CAN_PORT_PLATFORM);
    vbus_set_tx_stalled(&bus, node_platform, true);
    write_raw_frame_into_the_stalled_node();
    run_ms_sm(CAN_SM_TX_TIMEOUT_MS + 10u); /* the link steps all the time and sees the abort */
    TEST_ASSERT_EQUAL_UINT32(1u, can_sm_stats(CAN_PORT_PLATFORM)->tx_timeouts);
    TEST_ASSERT_EQUAL_UINT32(can_if_tx_abort_count(CAN_PORT_PLATFORM), dut.abort_seen);
    isotp_n_result_t res;
    TEST_ASSERT_FALSE(isotp_link_take_tx_confirm(&dut, &res));
    TEST_ASSERT_EQUAL_UINT32(0u, isotp_link_tx_timeout_count(&dut));
}

static void test_isotp_abort_tx_on_an_idle_link_is_a_no_op(void)
{
    isotp_config_t cfg;
    isotp_default_config(&cfg);
    open_generic_pair(&cfg, &cfg);
    isotp_n_result_t res;
    isotp_abort_tx(&dut.iso, ISOTP_N_TIMEOUT_A); /* never sent anything */
    TEST_ASSERT_FALSE(isotp_link_take_tx_confirm(&dut, &res));

    const uint8_t msg[5] = {1u, 2u, 3u, 4u, 5u};
    TEST_ASSERT_EQUAL(ISOTP_OK, isotp_link_send(&dut, msg, 5u));
    run_ms_sm(5u);
    TEST_ASSERT_TRUE(isotp_link_take_tx_confirm(&dut, &res));
    TEST_ASSERT_EQUAL(ISOTP_N_OK, res);
    isotp_abort_tx(&dut.iso, ISOTP_N_TIMEOUT_A); /* finished: idle again, nothing to end */
    TEST_ASSERT_FALSE(isotp_link_take_tx_confirm(&dut, &res));
    TEST_ASSERT_EQUAL_UINT32(0u, isotp_link_tx_timeout_count(&dut));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_cl250_link_takes_ids_and_padding_from_gen);
    RUN_TEST(test_cl250_request_is_a_padded_single_frame_on_the_request_id);
    RUN_TEST(test_cl250_every_gen_did_round_trips_through_the_simulated_ecu);
    RUN_TEST(test_cl250_refuses_forbidden_services);
    RUN_TEST(test_cl250_refuses_a_request_that_needs_a_first_frame);
    RUN_TEST(test_cl250_unsolicited_first_frame_gets_no_flow_control);
    RUN_TEST(test_cl250_first_frame_of_the_expected_answer_gets_the_one_gen_fc);
    RUN_TEST(test_cl250_a_second_first_frame_in_one_request_gets_no_second_fc);
    RUN_TEST(test_cl250_first_frame_above_the_cap_gets_nothing_on_the_bus);
    RUN_TEST(test_cl250_missing_cf_after_the_fc_ends_in_n_cr_base);
    RUN_TEST(test_cl250_wrong_sequence_number_after_the_fc_aborts);
    RUN_TEST(test_cl250_closed_or_unarmed_response_gets_no_fc);
    RUN_TEST(test_cl250_first_frame_with_another_response_sid_gets_no_fc);
    RUN_TEST(test_cl250_first_frame_at_the_cap_gets_the_fc);
    RUN_TEST(test_cl250_link_ignores_other_ids_and_formats);
    RUN_TEST(test_generic_link_cannot_open_on_the_vehicle_port);
    RUN_TEST(test_segmented_message_with_block_size_and_st_min_over_the_bus);
    RUN_TEST(test_segmented_message_from_the_peer_is_reassembled);
    RUN_TEST(test_full_tx_mailbox_holds_frames_until_it_frees);
    RUN_TEST(test_sender_going_silent_mid_message_gives_n_cr_on_the_receiver);
    RUN_TEST(test_one_step_writes_at_most_the_per_step_limit);
    RUN_TEST(test_open_rejects_bad_arguments_and_double_open);
    RUN_TEST(test_n_as_abort_ends_a_stalled_multi_frame_message_with_n_timeout_a);
    RUN_TEST(test_an_abort_before_isotp_link_send_does_not_end_the_new_message);
    RUN_TEST(test_an_abort_while_the_link_is_idle_is_not_counted_and_confirms_nothing);
    RUN_TEST(test_isotp_abort_tx_on_an_idle_link_is_a_no_op);
    return UNITY_END();
}
