/*
 * SIL tests for the UDS server (features/uds/uds_server, Ç3) end to end on the host
 * platform layer (D-034): the real server, ISO-TP link, can_if, services/diag and
 * services/vehicle_signals on an in-process platform bus, with a manual ms clock.
 *
 * Bus nodes: the server's platform port, a test-side ISO-TP tester (isotp_core straight
 * on the bus: tx PLATFORM_UDS_PHYS_REQUEST_ID, rx PLATFORM_UDS_PHYS_RESPONSE_ID) that
 * also injects raw frames, and a sniffer that records every frame with its time. The
 * vehicle port is bound to a separate bus that must never see a frame (D-020, D-037).
 * tearDown checks both for every test. No requirement IDs yet (Q-006).
 */
#include "app/host/sim_ecu.h"
#include "features/uds/isotp_core.h"
#include "features/uds/uds_client.h"
#include "features/uds/uds_server.h"
#include "hal/host/can_port_host.h"
#include "hal/host/hal_time_host.h"
#include "platform_uds.h"
#include "services/can_if.h"
#include "services/can_sm.h"
#include "services/diag.h"
#include "services/timebase.h"
#include "services/vehicle_signals.h"
#include "uds_iso14229.h"
#include "vehicle_cl250.h"

#include <ctype.h>
#include <string.h>
#include <unity.h>

#define HI(v) ((uint8_t)(((v) >> 8u) & 0xFFu))
#define LO(v) ((uint8_t)((v) & 0xFFu))
#define DID_BYTES(d) HI(d), LO(d)
#define AVAIL ((uint8_t)PLATFORM_UDS_DTC_STATUS_AVAILABILITY_MASK)
#define POS(sid) ((uint8_t)((sid) + UDS_POSITIVE_RESPONSE_OFFSET))
/* ISO 15765-2 PCI types (protocol constants, not platform values). */
#define PCI_MASK 0xF0u
#define PCI_SF 0x00u
#define PCI_FF 0x10u
#define PCI_CF 0x20u
#define PCI_FC 0x30u
#define BOUND_MS 300u
#define SNIFF_MAX 2048u
#define MSG_MAX 128u

static vbus_t bus_platform, bus_vehicle;
static uint8_t node_dut, node_tester, node_sniff, node_vehicle;
static uds_server_t server;
static uint8_t node_vinject;
static sim_ecu_t ecu;
static uds_client_t client;
static bool client_on;          /* a real UDS client runs on the vehicle bus */
static bool feed_status;        /* stands in for a healthy client's diag status every pass */
static bool allow_extended;     /* the test injects a 29-bit frame on purpose */
static bool vehicle_bus_used;   /* the client legitimately talks on the vehicle bus */

static isotp_link_t tester;
static uint8_t tester_rx[MSG_MAX], tester_tx[MSG_MAX];
static uint8_t got[MSG_MAX];
static uint16_t got_len;
static bool have_got;
static uint32_t extra_messages;

typedef struct {
    can_frame_t f;
    uint32_t t;
} sniffed_t;
static sniffed_t sniffed[SNIFF_MAX];
static uint32_t sniff_count;

static void tester_init(void)
{
    isotp_config_t cfg;
    isotp_default_config(&cfg);
    cfg.padding_enabled = true;
    cfg.padding_byte = PLATFORM_UDS_PADDING_BYTE;
    cfg.block_size = PLATFORM_UDS_BLOCK_SIZE;
    cfg.st_min = PLATFORM_UDS_ST_MIN_MS;
    TEST_ASSERT_EQUAL(ISOTP_OK, isotp_init(&tester, &cfg, tester_rx, sizeof tester_rx, tester_tx,
                                           sizeof tester_tx));
}

void setUp(void)
{
    hal_time_host_use_manual(1000u);
    vbus_init(&bus_platform);
    vbus_init(&bus_vehicle);
    TEST_ASSERT_TRUE(vbus_attach(&bus_platform, &node_dut));
    TEST_ASSERT_TRUE(vbus_attach(&bus_platform, &node_tester));
    TEST_ASSERT_TRUE(vbus_attach(&bus_platform, &node_sniff));
    TEST_ASSERT_TRUE(vbus_attach(&bus_vehicle, &node_vehicle));
    TEST_ASSERT_TRUE(vbus_attach(&bus_vehicle, &node_vinject));
    TEST_ASSERT_TRUE(can_port_host_bind_vbus(CAN_PORT_PLATFORM, &bus_platform, node_dut));
    TEST_ASSERT_TRUE(can_port_host_bind_vbus(CAN_PORT_VEHICLE, &bus_vehicle, node_vehicle));
    can_if_init();
    diag_init(timebase_now_ms());
    vehicle_signals_init();
    memset(&server, 0, sizeof server);
    TEST_ASSERT_EQUAL(ISOTP_OK, uds_server_open(&server));
    tester_init();
    sniff_count = 0u;
    have_got = false;
    got_len = 0u;
    extra_messages = 0u;
    client_on = false;
    feed_status = true;
    allow_extended = false;
    vehicle_bus_used = false;
}

/* Universal checks: the platform bus carries only the three gen/ IDs, in 11-bit format,
 * with DLC 8; every server frame is on the response ID, padded; the vehicle bus is
 * silent. */
static void assert_bus_rules(void)
{
    for (uint32_t i = 0u; i < sniff_count; i++) {
        const can_frame_t* f = &sniffed[i].f;
        const bool known = (f->id == PLATFORM_UDS_PHYS_REQUEST_ID) ||
                           (f->id == PLATFORM_UDS_PHYS_RESPONSE_ID) ||
                           (f->id == PLATFORM_UDS_FUNCTIONAL_REQUEST_ID);
        TEST_ASSERT_TRUE_MESSAGE(known, "a frame on an ID that is not a gen/ UDS ID");
        TEST_ASSERT_TRUE(allow_extended || !f->extended);
        if (f->id != PLATFORM_UDS_PHYS_RESPONSE_ID) {
            continue;
        }
        TEST_ASSERT_EQUAL_UINT8(PLATFORM_UDS_FRAME_DLC, f->dlc);
        const uint8_t type = (uint8_t)(f->data[0] & PCI_MASK);
        if (type == PCI_SF) {
            for (uint8_t b = (uint8_t)(f->data[0] + 1u); b < f->dlc; b++) {
                TEST_ASSERT_EQUAL_HEX8(PLATFORM_UDS_PADDING_BYTE, f->data[b]);
            }
        } else if (type == PCI_FC) {
            for (uint8_t b = 3u; b < f->dlc; b++) {
                TEST_ASSERT_EQUAL_HEX8(PLATFORM_UDS_PADDING_BYTE, f->data[b]);
            }
        } else {
            /* First and Consecutive Frames carry no padding except the last one */
        }
    }
    TEST_ASSERT_TRUE(vehicle_bus_used || (vbus_frame_count(&bus_vehicle) == 0u));
    TEST_ASSERT_EQUAL_UINT32(0u, can_if_tx_refused_count(CAN_PORT_VEHICLE));
}

void tearDown(void)
{
    assert_bus_rules();
    can_port_host_unbind_all();
    hal_time_host_use_monotonic();
}

/* ------------------------------------------------------------------ harness */

static void sniff(void)
{
    can_frame_t f;
    while (vbus_recv(&bus_platform, node_sniff, &f) == CAN_PORT_OK) {
        TEST_ASSERT_LESS_THAN_UINT32(SNIFF_MAX, sniff_count);
        sniffed[sniff_count].f = f;
        sniffed[sniff_count].t = timebase_now_ms();
        sniff_count++;
    }
}

static void tester_step(uint32_t t)
{
    can_frame_t f;
    while (vbus_recv(&bus_platform, node_tester, &f) == CAN_PORT_OK) {
        if ((f.id == PLATFORM_UDS_PHYS_RESPONSE_ID) && !f.extended) {
            isotp_frame_t fr;
            fr.dlc = f.dlc;
            memcpy(fr.data, f.data, sizeof fr.data);
            isotp_on_frame(&tester, &fr, t);
        }
    }
    isotp_frame_t out;
    while (isotp_poll(&tester, t, &out)) {
        can_frame_t cf;
        memset(&cf, 0, sizeof cf);
        cf.id = PLATFORM_UDS_PHYS_REQUEST_ID;
        cf.extended = false;
        cf.dlc = out.dlc;
        memcpy(cf.data, out.data, sizeof cf.data);
        TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_send(&bus_platform, node_tester, &cf));
    }
    isotp_n_result_t res = ISOTP_N_OK;
    uint16_t len = 0u;
    if (isotp_take_rx_indication(&tester, &res, &len)) {
        TEST_ASSERT_EQUAL(ISOTP_N_OK, res);
        TEST_ASSERT_LESS_OR_EQUAL_UINT16(MSG_MAX, len);
        if (have_got) {
            extra_messages++;
        } else {
            memcpy(got, tester_rx, len);
            got_len = len;
            have_got = true;
        }
        isotp_rx_release(&tester);
    }
    isotp_n_result_t conf = ISOTP_N_OK;
    (void)isotp_take_tx_confirm(&tester, &conf);
}

/* One main-loop pass per ms, like app/host/main.c. */
static void tick(void)
{
    const uint32_t t = timebase_now_ms();
    tester_step(t);
    (void)can_if_dispatch(CAN_PORT_PLATFORM, 64u);
    if (client_on) {
        (void)can_if_dispatch(CAN_PORT_VEHICLE, 64u);
        sim_ecu_step(&ecu, t);
        (void)can_if_dispatch(CAN_PORT_VEHICLE, 64u);
        uds_client_step(&client);
    }
    if (feed_status) {
        const diag_vehicle_tester_t healthy = {true, true, false,
                                               PLATFORM_UDS_VEHICLE_TESTER_STATUS_FAULT_NONE};
        diag_set_vehicle_tester(&healthy, t);
    }
    uds_server_step(&server);
    sniff();
    hal_time_host_advance(1u);
}

static void pump(uint32_t ms)
{
    for (uint32_t i = 0u; i < ms; i++) {
        tick();
    }
}

static bool wait_response(uint32_t bound_ms)
{
    for (uint32_t i = 0u; (i < bound_ms) && !have_got; i++) {
        tick();
    }
    return have_got;
}

/* ISO 14229-2 P2server: the first frame of the answer (not a Flow Control) arrives
 * within PLATFORM_UDS_P2_SERVER_MAX_MS of the request's last frame. */
static uint32_t assert_answer_timing(uint32_t from)
{
    uint32_t first_answer = sniff_count;
    for (uint32_t i = from; i < sniff_count; i++) {
        if ((sniffed[i].f.id == PLATFORM_UDS_PHYS_RESPONSE_ID) &&
            ((sniffed[i].f.data[0] & PCI_MASK) != PCI_FC)) {
            first_answer = i;
            break;
        }
    }
    TEST_ASSERT_LESS_THAN_UINT32_MESSAGE(sniff_count, first_answer, "no answer frame");
    uint32_t last_request = sniff_count;
    for (uint32_t i = from; i < first_answer; i++) {
        if ((sniffed[i].f.id == PLATFORM_UDS_PHYS_REQUEST_ID) ||
            (sniffed[i].f.id == PLATFORM_UDS_FUNCTIONAL_REQUEST_ID)) {
            last_request = i;
        }
    }
    TEST_ASSERT_LESS_THAN_UINT32_MESSAGE(sniff_count, last_request, "no request frame");
    const uint32_t latency = sniffed[first_answer].t - sniffed[last_request].t;
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(PLATFORM_UDS_P2_SERVER_MAX_MS, latency);
    return latency;
}

static uint32_t last_latency_ms;

/* Sends a physical request through the tester's ISO-TP link; true with the answer in
 * got[] / got_len. */
static bool request(const uint8_t* req, uint16_t len)
{
    have_got = false;
    const uint32_t mark = sniff_count;
    TEST_ASSERT_EQUAL(ISOTP_OK, isotp_send(&tester, req, len));
    if (!wait_response(BOUND_MS)) {
        return false;
    }
    last_latency_ms = assert_answer_timing(mark);
    return true;
}

#define REQUEST(arr) request((arr), (uint16_t)sizeof(arr))

static void send_frame(uint32_t id, bool extended, const uint8_t* data, uint8_t dlc)
{
    can_frame_t f;
    memset(&f, 0, sizeof f);
    f.id = id;
    f.extended = extended;
    f.dlc = dlc;
    memcpy(f.data, data, dlc);
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_send(&bus_platform, node_tester, &f));
}

static void send_raw(uint32_t id, const uint8_t* data, uint8_t dlc)
{
    send_frame(id, false, data, dlc);
}

/* A functional Single Frame, padded like the tester pads. */
static void send_functional_sf(const uint8_t* req, uint8_t len)
{
    uint8_t d[PLATFORM_UDS_FRAME_DLC];
    memset(d, PLATFORM_UDS_PADDING_BYTE, sizeof d);
    d[0] = len;
    memcpy(&d[1], req, len);
    send_raw(PLATFORM_UDS_FUNCTIONAL_REQUEST_ID, d, PLATFORM_UDS_FRAME_DLC);
}

static uint32_t response_frames_since(uint32_t mark)
{
    uint32_t n = 0u;
    for (uint32_t i = mark; i < sniff_count; i++) {
        if (sniffed[i].f.id == PLATFORM_UDS_PHYS_RESPONSE_ID) {
            n++;
        }
    }
    return n;
}

static void assert_nrc(uint8_t sid, uint8_t nrc)
{
    TEST_ASSERT_EQUAL_UINT16(UDS_NEGATIVE_RESPONSE_LEN, got_len);
    TEST_ASSERT_EQUAL_HEX8(UDS_SID_NEGATIVE_RESPONSE, got[0]);
    TEST_ASSERT_EQUAL_HEX8(sid, got[1]);
    TEST_ASSERT_EQUAL_HEX8(nrc, got[2]);
}

static void enter_extended(void)
{
    const uint8_t req[] = {UDS_SID_DIAGNOSTIC_SESSION_CONTROL, UDS_SESSION_EXTENDED};
    TEST_ASSERT_TRUE(REQUEST(req));
    TEST_ASSERT_EQUAL_HEX8(POS(UDS_SID_DIAGNOSTIC_SESSION_CONTROL), got[0]);
}

/* Reads one DID and checks the echo; the data record starts at got[3]. */
static bool read_did(uint16_t did)
{
    const uint8_t req[] = {UDS_SID_READ_DATA_BY_IDENTIFIER, DID_BYTES(did)};
    if (!REQUEST(req)) {
        return false;
    }
    TEST_ASSERT_EQUAL_HEX8(POS(UDS_SID_READ_DATA_BY_IDENTIFIER), got[0]);
    TEST_ASSERT_EQUAL_HEX8(HI(did), got[1]);
    TEST_ASSERT_EQUAL_HEX8(LO(did), got[2]);
    return true;
}

/* ------------------------------------------------------------------ open, getters */

static void test_open_twice_is_busy_and_null_is_an_argument_error(void)
{
    TEST_ASSERT_EQUAL(ISOTP_ERR_BUSY, uds_server_open(&server));
    TEST_ASSERT_EQUAL(ISOTP_ERR_ARG, uds_server_open(NULL));
    const uint8_t req[] = {UDS_SID_TESTER_PRESENT, UDS_TESTER_PRESENT_ZERO_SUBFUNCTION};
    TEST_ASSERT_TRUE(REQUEST(req)); /* the first open still works */
}

static void test_getters_and_step_are_safe_for_null_and_unopened_servers(void)
{
    static uds_server_t closed;
    memset(&closed, 0, sizeof closed);
    uds_server_step(NULL);
    uds_server_step(&closed);
    TEST_ASSERT_EQUAL_UINT8(UDS_SESSION_DEFAULT, uds_server_session(NULL));
    TEST_ASSERT_EQUAL_UINT8(UDS_SESSION_DEFAULT, uds_server_session(&closed));
    TEST_ASSERT_NULL(uds_server_stats(NULL));
    TEST_ASSERT_NULL(uds_server_glue_stats(NULL));
    TEST_ASSERT_NOT_NULL(uds_server_stats(&server));
    TEST_ASSERT_NOT_NULL(uds_server_glue_stats(&server));
    TEST_ASSERT_EQUAL_UINT8(UDS_SESSION_DEFAULT, uds_server_session(&server));
}

static void dummy_rx(void* ctx, const can_frame_t* frame)
{
    (void)ctx;
    (void)frame;
}

static void test_a_full_receiver_table_fails_open_and_leaves_the_server_closed(void)
{
    static uds_server_t second;
    server.open = false; /* retire the setUp server: its receivers go with can_if_init() */
    can_if_init();
    /* leave room for the link's receiver but not for the functional one */
    for (uint32_t i = 0u; (i + 1u) < CAN_IF_MAX_RECEIVERS; i++) {
        TEST_ASSERT_EQUAL(CAN_IF_OK,
                          can_if_register_rx(CAN_PORT_VEHICLE, 0x100u + i, false, dummy_rx, NULL));
    }
    memset(&second, 0, sizeof second);
    TEST_ASSERT_EQUAL(ISOTP_ERR_ARG, uds_server_open(&second));
    uds_server_step(&second);
    TEST_ASSERT_EQUAL_UINT8(UDS_SESSION_DEFAULT, uds_server_session(&second));
    pump(20u);
    TEST_ASSERT_EQUAL_UINT32(0u, sniff_count);
}

/* ------------------------------------------------------------------ bus rules */

static void test_server_frames_use_only_the_gen_response_id_dlc_and_padding(void)
{
    const uint8_t tp[] = {UDS_SID_TESTER_PRESENT, UDS_TESTER_PRESENT_ZERO_SUBFUNCTION};
    const uint8_t rd[] = {UDS_SID_READ_DATA_BY_IDENTIFIER, DID_BYTES(PLATFORM_UDS_DID_SW_VERSION)};
    const uint8_t bad[] = {UDS_SID_CLEAR_DIAGNOSTIC_INFORMATION};
    TEST_ASSERT_TRUE(REQUEST(tp));
    TEST_ASSERT_TRUE(REQUEST(rd));
    TEST_ASSERT_TRUE(REQUEST(bad));
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(3u, response_frames_since(0u));
    /* the universal rules of assert_bus_rules() ran on real frames: none is vacuous */
    bool saw_sf = false;
    for (uint32_t i = 0u; i < sniff_count; i++) {
        if ((sniffed[i].f.id == PLATFORM_UDS_PHYS_RESPONSE_ID) &&
            ((sniffed[i].f.data[0] & PCI_MASK) == PCI_SF)) {
            saw_sf = true;
        }
    }
    TEST_ASSERT_TRUE(saw_sf);
}

static void test_the_vehicle_bus_never_sees_a_frame(void)
{
    const uint8_t rd[] = {UDS_SID_READ_DATA_BY_IDENTIFIER, DID_BYTES(PLATFORM_UDS_DID_UPTIME)};
    TEST_ASSERT_TRUE(REQUEST(rd));
    enter_extended();
    pump(100u);
    TEST_ASSERT_EQUAL_UINT32(0u, vbus_frame_count(&bus_vehicle));
    TEST_ASSERT_GREATER_THAN_UINT32(0u, vbus_frame_count(&bus_platform));
    TEST_ASSERT_EQUAL_UINT32(0u, can_if_unrouted_count(CAN_PORT_VEHICLE));
}

/* ------------------------------------------------------------------ DIDs */

static void test_read_of_the_software_version_is_a_segmented_nul_padded_record(void)
{
    const uint32_t mark = sniff_count;
    TEST_ASSERT_TRUE(read_did(PLATFORM_UDS_DID_SW_VERSION));
    const uint16_t dlen = platform_uds_dids[PLATFORM_UDS_IDX_SW_VERSION].length;
    TEST_ASSERT_EQUAL_UINT16(3u + dlen, got_len);
    TEST_ASSERT_GREATER_THAN_UINT16(ISOTP_SF_MAX_LEN, got_len);
    const uint8_t* v = &got[3];
    TEST_ASSERT_TRUE(isdigit(v[0]));
    TEST_ASSERT_NOT_NULL(memchr(v, '.', dlen));
    uint16_t n = 0u;
    while ((n < dlen) && (v[n] != 0u)) {
        n++;
    }
    TEST_ASSERT_GREATER_THAN_UINT16(0u, n);
    for (uint16_t i = n; i < dlen; i++) {
        TEST_ASSERT_EQUAL_HEX8(0u, v[i]); /* NUL-padded to the record length */
    }
    /* it went out as First Frame + Consecutive Frames, the last one padded */
    bool ff = false;
    const can_frame_t* last = NULL;
    for (uint32_t i = mark; i < sniff_count; i++) {
        if (sniffed[i].f.id == PLATFORM_UDS_PHYS_RESPONSE_ID) {
            ff = ff || ((sniffed[i].f.data[0] & PCI_MASK) == PCI_FF);
            last = &sniffed[i].f;
        }
    }
    TEST_ASSERT_TRUE(ff);
    TEST_ASSERT_NOT_NULL(last);
    TEST_ASSERT_EQUAL_HEX8(PCI_CF, (uint8_t)(last->data[0] & PCI_MASK));
    const uint16_t in_ff = 6u;
    const uint16_t in_last = (uint16_t)((got_len - in_ff) % 7u);
    if (in_last != 0u) {
        for (uint16_t b = (uint16_t)(1u + in_last); b < PLATFORM_UDS_FRAME_DLC; b++) {
            TEST_ASSERT_EQUAL_HEX8(PLATFORM_UDS_PADDING_BYTE, last->data[b]);
        }
    }
    TEST_ASSERT_EQUAL_UINT32(0u, extra_messages);
}

static void test_multi_did_read_is_segmented_both_ways_and_in_request_order(void)
{
    const uint16_t dids[] = {PLATFORM_UDS_DID_ACTIVE_DIAGNOSTIC_SESSION, PLATFORM_UDS_DID_SW_VERSION,
                             PLATFORM_UDS_DID_VEHICLE_TESTER_STATUS, PLATFORM_UDS_DID_UPTIME};
    _Static_assert(sizeof(dids) / sizeof(dids[0]) == PLATFORM_UDS_MAX_READ_DIDS,
                   "the request carries the maximum number of DIDs");
    uint8_t req[1u + (2u * PLATFORM_UDS_MAX_READ_DIDS)];
    req[0] = UDS_SID_READ_DATA_BY_IDENTIFIER;
    for (uint32_t i = 0u; i < PLATFORM_UDS_MAX_READ_DIDS; i++) {
        req[1u + (2u * i)] = HI(dids[i]);
        req[2u + (2u * i)] = LO(dids[i]);
    }
    TEST_ASSERT_GREATER_THAN_UINT16(ISOTP_SF_MAX_LEN, sizeof req); /* the request is segmented too */
    TEST_ASSERT_TRUE(request(req, (uint16_t)sizeof req));
    TEST_ASSERT_EQUAL_HEX8(POS(UDS_SID_READ_DATA_BY_IDENTIFIER), got[0]);
    uint16_t pos = 1u;
    for (uint32_t i = 0u; i < PLATFORM_UDS_MAX_READ_DIDS; i++) {
        const platform_uds_did_t* d = platform_uds_find_did(dids[i]);
        TEST_ASSERT_NOT_NULL(d);
        TEST_ASSERT_EQUAL_HEX8(HI(dids[i]), got[pos]);
        TEST_ASSERT_EQUAL_HEX8(LO(dids[i]), got[pos + 1u]);
        pos = (uint16_t)(pos + 2u + d->length);
    }
    TEST_ASSERT_EQUAL_UINT16(pos, got_len);
    TEST_ASSERT_EQUAL_HEX8(UDS_SESSION_DEFAULT, got[3]); /* F186 record */
    TEST_ASSERT_EQUAL_UINT32(0u, extra_messages);
}

static void test_tester_status_did_reflects_diag_bits_and_fault(void)
{
    const uint8_t at = PLATFORM_UDS_VEHICLE_TESTER_STATUS_ECU_PRESENT_BYTE;
    const uint8_t fb = PLATFORM_UDS_VEHICLE_TESTER_STATUS_FAULT_BYTE;
    feed_status = false;
    TEST_ASSERT_TRUE(read_did(PLATFORM_UDS_DID_VEHICLE_TESTER_STATUS));
    TEST_ASSERT_EQUAL_UINT16(3u + PLATFORM_UDS_DID_VEHICLE_TESTER_STATUS_LENGTH, got_len);
    TEST_ASSERT_EQUAL_HEX8(0u, got[3u + at]); /* no status yet: not running */
    TEST_ASSERT_EQUAL_HEX8(PLATFORM_UDS_VEHICLE_TESTER_STATUS_FAULT_NOT_RUNNING, got[3u + fb]);

    const diag_vehicle_tester_t t = {true, false, true,
                                     PLATFORM_UDS_VEHICLE_TESTER_STATUS_FAULT_FOREIGN_TESTER};
    diag_set_vehicle_tester(&t, timebase_now_ms());
    TEST_ASSERT_TRUE(read_did(PLATFORM_UDS_DID_VEHICLE_TESTER_STATUS));
    TEST_ASSERT_EQUAL_HEX8(PLATFORM_UDS_VEHICLE_TESTER_STATUS_ECU_PRESENT_MASK |
                               PLATFORM_UDS_VEHICLE_TESTER_STATUS_LATCHED_MASK,
                           got[3u + at]);
    TEST_ASSERT_EQUAL_HEX8(PLATFORM_UDS_VEHICLE_TESTER_STATUS_FAULT_FOREIGN_TESTER, got[3u + fb]);

    const diag_vehicle_tester_t u = {false, true, false, PLATFORM_UDS_VEHICLE_TESTER_STATUS_FAULT_GATE};
    diag_set_vehicle_tester(&u, timebase_now_ms());
    TEST_ASSERT_TRUE(read_did(PLATFORM_UDS_DID_VEHICLE_TESTER_STATUS));
    TEST_ASSERT_EQUAL_HEX8(PLATFORM_UDS_VEHICLE_TESTER_STATUS_SESSION_UP_MASK, got[3u + at]);
    TEST_ASSERT_EQUAL_HEX8(PLATFORM_UDS_VEHICLE_TESTER_STATUS_FAULT_GATE, got[3u + fb]);
}

static uint32_t be32(const uint8_t* p, uint16_t len)
{
    uint32_t v = 0u;
    for (uint16_t i = 0u; i < len; i++) {
        v = (v << 8u) | p[i];
    }
    return v;
}

static void test_uptime_did_counts_seconds_since_open(void)
{
    const uint16_t len = platform_uds_dids[PLATFORM_UDS_IDX_UPTIME].length;
    TEST_ASSERT_TRUE(read_did(PLATFORM_UDS_DID_UPTIME));
    TEST_ASSERT_EQUAL_UINT16(3u + len, got_len);
    TEST_ASSERT_EQUAL_UINT32(0u, be32(&got[3], len));
    pump(2000u);
    TEST_ASSERT_TRUE(read_did(PLATFORM_UDS_DID_UPTIME));
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(2u, be32(&got[3], len));
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(3u, be32(&got[3], len));
}

static void test_vehicle_sample_dids_go_none_valid_stale_with_big_endian_raw(void)
{
    uint32_t checked = 0u;
    for (uint32_t idx = 0u; idx < PLATFORM_UDS_DID_COUNT; idx++) {
        const platform_uds_did_t* d = &platform_uds_dids[idx];
        if (d->encoding != (uint8_t)PLATFORM_UDS_ENC_VEHICLE_SAMPLE) {
            continue;
        }
        checked++;
        vehicle_signals_init();
        const uint16_t rlen = (uint16_t)(d->length - PLATFORM_UDS_VEHICLE_SAMPLE_HEADER_LEN);
        TEST_ASSERT_TRUE((rlen >= 1u) && (rlen <= 4u));
        const uint32_t raw = 0x0A0B0C0Du >> (8u * (4u - rlen));

        /* no sample yet */
        TEST_ASSERT_TRUE(read_did(d->did));
        TEST_ASSERT_EQUAL_UINT16(3u + d->length, got_len);
        TEST_ASSERT_EQUAL_HEX8(PLATFORM_UDS_VEHICLE_SAMPLE_STATE_NONE, got[3]);
        TEST_ASSERT_EQUAL_HEX16(PLATFORM_UDS_VEHICLE_SAMPLE_AGE_MAX_MS,
                                (uint16_t)((got[4] << 8u) | got[5]));

        /* a fresh sample */
        TEST_ASSERT_TRUE(vehicle_signals_write(d->vehicle_idx, raw, 0.0f, timebase_now_ms()));
        pump(10u);
        TEST_ASSERT_TRUE(read_did(d->did));
        TEST_ASSERT_EQUAL_HEX8(PLATFORM_UDS_VEHICLE_SAMPLE_STATE_VALID, got[3]);
        const uint16_t age = (uint16_t)((got[4] << 8u) | got[5]);
        TEST_ASSERT_UINT16_WITHIN(5u, 10u, age);
        TEST_ASSERT_EQUAL_UINT32(raw, be32(&got[3u + PLATFORM_UDS_VEHICLE_SAMPLE_HEADER_LEN], rlen));

        /* older than stale_after_ms */
        pump((uint32_t)vehicle_cl250_dids[d->vehicle_idx].stale_after_ms + 5u);
        TEST_ASSERT_TRUE(read_did(d->did));
        TEST_ASSERT_EQUAL_HEX8(PLATFORM_UDS_VEHICLE_SAMPLE_STATE_STALE, got[3]);
        const uint16_t stale_age = (uint16_t)((got[4] << 8u) | got[5]);
        TEST_ASSERT_GREATER_THAN_UINT16(vehicle_cl250_dids[d->vehicle_idx].stale_after_ms, stale_age);
        TEST_ASSERT_EQUAL_UINT32(raw, be32(&got[3u + PLATFORM_UDS_VEHICLE_SAMPLE_HEADER_LEN], rlen));
    }
    TEST_ASSERT_EQUAL_UINT32(VEHICLE_CL250_DID_COUNT, checked);
}

/* ------------------------------------------------------------------ DTCs, 0x14 */

static uint8_t report_dtc_by_mask(uint8_t mask)
{
    const uint8_t req[] = {UDS_SID_READ_DTC_INFORMATION, UDS_READ_DTC_REPORT_DTC_BY_STATUS_MASK, mask};
    TEST_ASSERT_TRUE(REQUEST(req));
    TEST_ASSERT_EQUAL_HEX8(POS(UDS_SID_READ_DTC_INFORMATION), got[0]);
    TEST_ASSERT_EQUAL_HEX8(AVAIL, got[2]);
    return (uint8_t)((got_len - 3u) / 4u);
}

static void test_reported_dtc_is_listed_by_read_dtc_information(void)
{
    TEST_ASSERT_EQUAL_UINT8(0u, report_dtc_by_mask(UDS_DTC_STATUS_TEST_FAILED));
    diag_dtc_report(PLATFORM_UDS_DTC_IDX_VEHICLE_ECU_COMM_LOST, true);
    TEST_ASSERT_EQUAL_UINT8(1u, report_dtc_by_mask(UDS_DTC_STATUS_TEST_FAILED));
    const uint32_t dtc = PLATFORM_UDS_DTC_VEHICLE_ECU_COMM_LOST;
    TEST_ASSERT_EQUAL_HEX8((dtc >> 16u) & 0xFFu, got[3]);
    TEST_ASSERT_EQUAL_HEX8((dtc >> 8u) & 0xFFu, got[4]);
    TEST_ASSERT_EQUAL_HEX8(dtc & 0xFFu, got[5]);
    TEST_ASSERT_EQUAL_HEX8((UDS_DTC_STATUS_TEST_FAILED | UDS_DTC_STATUS_CONFIRMED_DTC) & AVAIL, got[6]);
}

static void test_dtc_count_and_supported_list_through_the_link(void)
{
    diag_dtc_report(PLATFORM_UDS_DTC_IDX_VEHICLE_TESTER_LATCHED, true);
    const uint8_t count[] = {UDS_SID_READ_DTC_INFORMATION,
                             UDS_READ_DTC_REPORT_NUMBER_OF_DTC_BY_STATUS_MASK,
                             UDS_DTC_STATUS_CONFIRMED_DTC};
    TEST_ASSERT_TRUE(REQUEST(count));
    const uint8_t expect[] = {POS(UDS_SID_READ_DTC_INFORMATION),
                              UDS_READ_DTC_REPORT_NUMBER_OF_DTC_BY_STATUS_MASK, AVAIL,
                              UDS_DTC_FORMAT_ISO_14229_1, 0x00u, 0x01u};
    TEST_ASSERT_EQUAL_UINT16(sizeof expect, got_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expect, got, sizeof expect);
    const uint8_t all[] = {UDS_SID_READ_DTC_INFORMATION, UDS_READ_DTC_REPORT_SUPPORTED_DTC};
    TEST_ASSERT_TRUE(REQUEST(all));
    TEST_ASSERT_EQUAL_UINT16(3u + (4u * PLATFORM_UDS_DTC_COUNT), got_len);
}

static void test_clear_in_the_default_session_is_refused_and_keeps_the_dtc(void)
{
    diag_dtc_report(PLATFORM_UDS_DTC_IDX_VEHICLE_ECU_COMM_LOST, true);
    const uint8_t req[] = {UDS_SID_CLEAR_DIAGNOSTIC_INFORMATION, (UDS_GROUP_OF_DTC_ALL >> 16u) & 0xFFu,
                           (UDS_GROUP_OF_DTC_ALL >> 8u) & 0xFFu, UDS_GROUP_OF_DTC_ALL & 0xFFu};
    TEST_ASSERT_TRUE(REQUEST(req));
    assert_nrc(UDS_SID_CLEAR_DIAGNOSTIC_INFORMATION, UDS_NRC_SERVICE_NOT_SUPPORTED_IN_ACTIVE_SESSION);
    TEST_ASSERT_NOT_EQUAL(0u, diag_dtc_status(PLATFORM_UDS_DTC_IDX_VEHICLE_ECU_COMM_LOST));
}

static void test_clear_in_the_extended_session_clears_and_a_still_failing_monitor_sets_it_again(void)
{
    diag_dtc_report(PLATFORM_UDS_DTC_IDX_VEHICLE_ECU_COMM_LOST, true);
    diag_dtc_report(PLATFORM_UDS_DTC_IDX_VEHICLE_TESTER_LATCHED, true);
    enter_extended();
    const uint8_t req[] = {UDS_SID_CLEAR_DIAGNOSTIC_INFORMATION, (UDS_GROUP_OF_DTC_ALL >> 16u) & 0xFFu,
                           (UDS_GROUP_OF_DTC_ALL >> 8u) & 0xFFu, UDS_GROUP_OF_DTC_ALL & 0xFFu};
    TEST_ASSERT_TRUE(REQUEST(req));
    TEST_ASSERT_EQUAL_UINT16(1u, got_len);
    TEST_ASSERT_EQUAL_HEX8(POS(UDS_SID_CLEAR_DIAGNOSTIC_INFORMATION), got[0]);
    TEST_ASSERT_EQUAL_UINT8(0u, report_dtc_by_mask(UDS_DTC_STATUS_TEST_FAILED));
    TEST_ASSERT_EQUAL_HEX8(0u, diag_dtc_status(PLATFORM_UDS_DTC_IDX_VEHICLE_ECU_COMM_LOST));

    /* level-triggered: the monitor reports its still-active condition on its next pass */
    diag_dtc_report(PLATFORM_UDS_DTC_IDX_VEHICLE_TESTER_LATCHED, true);
    TEST_ASSERT_EQUAL_UINT8(1u, report_dtc_by_mask(UDS_DTC_STATUS_TEST_FAILED));
    TEST_ASSERT_EQUAL_HEX8((PLATFORM_UDS_DTC_VEHICLE_TESTER_LATCHED >> 16u) & 0xFFu, got[3]);
}

/* ------------------------------------------------------------------ sessions, S3 */

static void test_session_control_through_the_glue_changes_the_session_getter(void)
{
    enter_extended();
    TEST_ASSERT_EQUAL_UINT8(UDS_SESSION_EXTENDED, uds_server_session(&server));
    TEST_ASSERT_TRUE(read_did(PLATFORM_UDS_DID_ACTIVE_DIAGNOSTIC_SESSION));
    TEST_ASSERT_EQUAL_HEX8(UDS_SESSION_EXTENDED, got[3]);
    const uint8_t back[] = {UDS_SID_DIAGNOSTIC_SESSION_CONTROL, UDS_SESSION_DEFAULT};
    TEST_ASSERT_TRUE(REQUEST(back));
    TEST_ASSERT_EQUAL_UINT8(UDS_SESSION_DEFAULT, uds_server_session(&server));
}

static void test_the_session_survives_until_just_before_s3(void)
{
    const uint32_t t0 = timebase_now_ms();
    enter_extended();
    pump((t0 + PLATFORM_UDS_S3_SERVER_MS - 100u) - timebase_now_ms());
    TEST_ASSERT_TRUE(read_did(PLATFORM_UDS_DID_ACTIVE_DIAGNOSTIC_SESSION));
    TEST_ASSERT_EQUAL_HEX8(UDS_SESSION_EXTENDED, got[3]);
    TEST_ASSERT_EQUAL_UINT32(0u, uds_server_stats(&server)->s3_timeouts);
}

static void test_the_session_falls_back_to_default_after_s3(void)
{
    const uint32_t t0 = timebase_now_ms();
    enter_extended();
    pump((t0 + PLATFORM_UDS_S3_SERVER_MS + 100u) - timebase_now_ms());
    TEST_ASSERT_TRUE(read_did(PLATFORM_UDS_DID_ACTIVE_DIAGNOSTIC_SESSION));
    TEST_ASSERT_EQUAL_HEX8(UDS_SESSION_DEFAULT, got[3]);
    TEST_ASSERT_EQUAL_UINT32(1u, uds_server_stats(&server)->s3_timeouts);
    TEST_ASSERT_EQUAL_UINT8(UDS_SESSION_DEFAULT, uds_server_session(&server));
}

static void test_suppressed_tester_present_keeps_the_session_and_sends_no_frame(void)
{
    enter_extended();
    const uint8_t tp[] = {UDS_SID_TESTER_PRESENT,
                          (uint8_t)(UDS_TESTER_PRESENT_ZERO_SUBFUNCTION | UDS_SUPPRESS_POS_RSP_BIT)};
    const uint32_t mark = sniff_count;
    for (uint32_t i = 0u; i < 3u; i++) {
        pump(PLATFORM_UDS_S3_SERVER_MS - 500u);
        TEST_ASSERT_EQUAL(ISOTP_OK, isotp_send(&tester, tp, sizeof tp));
        pump(5u);
    }
    TEST_ASSERT_EQUAL_UINT8(UDS_SESSION_EXTENDED, uds_server_session(&server));
    TEST_ASSERT_EQUAL_UINT32(0u, response_frames_since(mark));
    TEST_ASSERT_EQUAL_UINT32(3u, uds_server_stats(&server)->suppressed);
}

/* ------------------------------------------------------------------ functional */

static void test_functional_single_frame_read_is_answered_on_the_physical_response_id(void)
{
    const uint8_t req[] = {UDS_SID_READ_DATA_BY_IDENTIFIER,
                           DID_BYTES(PLATFORM_UDS_DID_ACTIVE_DIAGNOSTIC_SESSION)};
    const uint32_t mark = sniff_count;
    have_got = false;
    send_functional_sf(req, sizeof req);
    TEST_ASSERT_TRUE(wait_response(BOUND_MS));
    (void)assert_answer_timing(mark);
    const uint8_t expect[] = {POS(UDS_SID_READ_DATA_BY_IDENTIFIER),
                              DID_BYTES(PLATFORM_UDS_DID_ACTIVE_DIAGNOSTIC_SESSION),
                              UDS_SESSION_DEFAULT};
    TEST_ASSERT_EQUAL_UINT16(sizeof expect, got_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expect, got, sizeof expect);
    TEST_ASSERT_EQUAL_UINT32(1u, uds_server_glue_stats(&server)->functional_taken);
    TEST_ASSERT_EQUAL_UINT32(0u, uds_server_glue_stats(&server)->functional_dropped);
}

static void test_functional_unsupported_service_gets_no_answer(void)
{
    uint8_t sid = 1u;
    while (platform_uds_service_supported(sid)) {
        sid++;
    }
    const uint8_t req[] = {sid, 0x00u};
    const uint32_t mark = sniff_count;
    send_functional_sf(req, sizeof req);
    pump(100u);
    TEST_ASSERT_FALSE(have_got);
    TEST_ASSERT_EQUAL_UINT32(0u, response_frames_since(mark));
    TEST_ASSERT_EQUAL_UINT32(1u, uds_server_stats(&server)->suppressed);
    TEST_ASSERT_EQUAL_UINT32(1u, uds_server_glue_stats(&server)->functional_taken);
}

static void test_functional_first_frame_is_dropped_without_flow_control(void)
{
    const uint8_t ff[PLATFORM_UDS_FRAME_DLC] = {PCI_FF, 0x09u, UDS_SID_READ_DATA_BY_IDENTIFIER,
                                                HI(PLATFORM_UDS_DID_SW_VERSION),
                                                LO(PLATFORM_UDS_DID_SW_VERSION),
                                                HI(PLATFORM_UDS_DID_UPTIME),
                                                LO(PLATFORM_UDS_DID_UPTIME), 0x00u};
    const uint32_t mark = sniff_count;
    send_raw(PLATFORM_UDS_FUNCTIONAL_REQUEST_ID, ff, PLATFORM_UDS_FRAME_DLC);
    pump(100u);
    TEST_ASSERT_EQUAL_UINT32(1u, uds_server_glue_stats(&server)->functional_dropped);
    TEST_ASSERT_EQUAL_UINT32(0u, uds_server_glue_stats(&server)->functional_taken);
    TEST_ASSERT_EQUAL_UINT32(0u, response_frames_since(mark)); /* no FC, no answer */
    TEST_ASSERT_FALSE(have_got);
}

static void test_malformed_functional_single_frames_are_dropped(void)
{
    uint8_t zero_len[PLATFORM_UDS_FRAME_DLC];
    memset(zero_len, PLATFORM_UDS_PADDING_BYTE, sizeof zero_len);
    zero_len[0] = 0u; /* SF_DL 0 */
    send_raw(PLATFORM_UDS_FUNCTIONAL_REQUEST_ID, zero_len, PLATFORM_UDS_FRAME_DLC);
    pump(5u);
    const uint8_t too_long[3] = {5u, UDS_SID_TESTER_PRESENT, UDS_TESTER_PRESENT_ZERO_SUBFUNCTION};
    send_raw(PLATFORM_UDS_FUNCTIONAL_REQUEST_ID, too_long, sizeof too_long); /* SF_DL > DLC - 1 */
    pump(50u);
    TEST_ASSERT_EQUAL_UINT32(2u, uds_server_glue_stats(&server)->functional_dropped);
    TEST_ASSERT_EQUAL_UINT32(0u, uds_server_glue_stats(&server)->functional_taken);
    TEST_ASSERT_EQUAL_UINT32(0u, response_frames_since(0u));
}

static void test_functional_request_during_a_physical_reception_is_dropped_and_the_reception_completes(void)
{
    const uint16_t dids[] = {PLATFORM_UDS_DID_ACTIVE_DIAGNOSTIC_SESSION, PLATFORM_UDS_DID_SW_VERSION,
                             PLATFORM_UDS_DID_UPTIME, PLATFORM_UDS_DID_VEHICLE_TESTER_STATUS};
    uint8_t req[1u + (2u * (sizeof dids / sizeof dids[0]))];
    req[0] = UDS_SID_READ_DATA_BY_IDENTIFIER;
    for (uint32_t i = 0u; i < (sizeof dids / sizeof dids[0]); i++) {
        req[1u + (2u * i)] = HI(dids[i]);
        req[2u + (2u * i)] = LO(dids[i]);
    }
    TEST_ASSERT_EQUAL_UINT16(9u, sizeof req);
    /* Physical First Frame (6 bytes) by hand: the server answers Flow Control and waits. */
    uint8_t ff[PLATFORM_UDS_FRAME_DLC];
    ff[0] = (uint8_t)(PCI_FF | ((sizeof req >> 8u) & 0x0Fu));
    ff[1] = (uint8_t)(sizeof req & 0xFFu);
    memcpy(&ff[2], req, 6u);
    send_raw(PLATFORM_UDS_PHYS_REQUEST_ID, ff, PLATFORM_UDS_FRAME_DLC);
    pump(3u);
    TEST_ASSERT_GREATER_THAN_UINT32(0u, response_frames_since(0u)); /* the FC */

    const uint8_t func[] = {UDS_SID_READ_DATA_BY_IDENTIFIER,
                            DID_BYTES(PLATFORM_UDS_DID_ACTIVE_DIAGNOSTIC_SESSION)};
    send_functional_sf(func, sizeof func);
    pump(3u);
    TEST_ASSERT_EQUAL_UINT32(1u, uds_server_glue_stats(&server)->functional_dropped);
    TEST_ASSERT_EQUAL_UINT32(0u, uds_server_glue_stats(&server)->functional_taken);
    TEST_ASSERT_FALSE(have_got);

    /* Consecutive Frame with the last 3 bytes: the physical request is answered whole. */
    uint8_t cf[PLATFORM_UDS_FRAME_DLC];
    memset(cf, PLATFORM_UDS_PADDING_BYTE, sizeof cf);
    cf[0] = (uint8_t)(PCI_CF | 1u);
    memcpy(&cf[1], &req[6], 3u);
    send_raw(PLATFORM_UDS_PHYS_REQUEST_ID, cf, PLATFORM_UDS_FRAME_DLC);
    TEST_ASSERT_TRUE(wait_response(BOUND_MS));
    TEST_ASSERT_EQUAL_HEX8(POS(UDS_SID_READ_DATA_BY_IDENTIFIER), got[0]);
    TEST_ASSERT_EQUAL_HEX8(HI(dids[0]), got[1]);
    TEST_ASSERT_EQUAL_HEX8(LO(dids[0]), got[2]);
    TEST_ASSERT_EQUAL_HEX8(UDS_SESSION_DEFAULT, got[3]);
    TEST_ASSERT_EQUAL_UINT32(0u, extra_messages);
    TEST_ASSERT_EQUAL_UINT32(0u, uds_server_glue_stats(&server)->rx_errors);
}

static void test_functional_request_after_the_answer_is_taken_again(void)
{
    const uint8_t req[] = {UDS_SID_READ_DATA_BY_IDENTIFIER,
                           DID_BYTES(PLATFORM_UDS_DID_ACTIVE_DIAGNOSTIC_SESSION)};
    const uint8_t tp[] = {UDS_SID_TESTER_PRESENT, UDS_TESTER_PRESENT_ZERO_SUBFUNCTION};
    send_functional_sf(req, sizeof req);
    TEST_ASSERT_TRUE(wait_response(BOUND_MS));
    have_got = false;
    TEST_ASSERT_TRUE(REQUEST(tp)); /* physical still works */
    have_got = false;
    send_functional_sf(req, sizeof req);
    TEST_ASSERT_TRUE(wait_response(BOUND_MS));
    TEST_ASSERT_EQUAL_UINT32(2u, uds_server_glue_stats(&server)->functional_taken);
    TEST_ASSERT_EQUAL_UINT32(0u, uds_server_glue_stats(&server)->functional_dropped);
}

/* ------------------------------------------------------------------ timing */

static void test_every_answer_starts_within_p2_server_max_of_the_request(void)
{
    const uint8_t tp[] = {UDS_SID_TESTER_PRESENT, UDS_TESTER_PRESENT_ZERO_SUBFUNCTION};
    const uint8_t sw[] = {UDS_SID_READ_DATA_BY_IDENTIFIER, DID_BYTES(PLATFORM_UDS_DID_SW_VERSION)};
    const uint8_t dtc[] = {UDS_SID_READ_DTC_INFORMATION, UDS_READ_DTC_REPORT_SUPPORTED_DTC};
    uint16_t missing = 1u; /* a DID the server does not offer */
    while (platform_uds_find_did(missing) != NULL) {
        missing++;
    }
    const uint8_t unknown[] = {UDS_SID_READ_DATA_BY_IDENTIFIER, DID_BYTES(missing)};
    uint32_t worst = 0u;
    TEST_ASSERT_TRUE(REQUEST(tp));
    worst = (last_latency_ms > worst) ? last_latency_ms : worst;
    TEST_ASSERT_TRUE(REQUEST(sw));
    worst = (last_latency_ms > worst) ? last_latency_ms : worst;
    TEST_ASSERT_TRUE(REQUEST(dtc));
    worst = (last_latency_ms > worst) ? last_latency_ms : worst;
    TEST_ASSERT_TRUE(REQUEST(unknown));
    worst = (last_latency_ms > worst) ? last_latency_ms : worst;
    enter_extended();
    worst = (last_latency_ms > worst) ? last_latency_ms : worst;
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(PLATFORM_UDS_P2_SERVER_MAX_MS, worst);
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(5u, worst); /* same pass in the host loop, well inside P2 */
}

/* ------------------------------------------------------------------ safety-review cases */

static void start_client(void)
{
    feed_status = false;
    vehicle_bus_used = true;
    TEST_ASSERT_TRUE(sim_ecu_init(&ecu, &bus_vehicle));
    ecu.require_session = true;
    memset(&client, 0, sizeof client);
    TEST_ASSERT_EQUAL(ISOTP_OK, uds_client_open(&client));
    client_on = true;
}

static void test_without_a_client_the_tester_status_reads_not_running_and_the_dtc_fails(void)
{
    feed_status = false;
    const uint8_t at = PLATFORM_UDS_VEHICLE_TESTER_STATUS_ECU_PRESENT_BYTE;
    const uint8_t fb = PLATFORM_UDS_VEHICLE_TESTER_STATUS_FAULT_BYTE;
    TEST_ASSERT_TRUE(read_did(PLATFORM_UDS_DID_VEHICLE_TESTER_STATUS));
    TEST_ASSERT_EQUAL_HEX8(0u, got[3u + at]);
    TEST_ASSERT_EQUAL_HEX8(PLATFORM_UDS_VEHICLE_TESTER_STATUS_FAULT_NOT_RUNNING, got[3u + fb]);
    TEST_ASSERT_EQUAL_UINT8(0u, report_dtc_by_mask(UDS_DTC_STATUS_TEST_FAILED)); /* too early */
    pump(PLATFORM_UDS_VEHICLE_TESTER_STATUS_MAX_AGE_MS);
    TEST_ASSERT_EQUAL_UINT8(1u, report_dtc_by_mask(UDS_DTC_STATUS_TEST_FAILED));
    const uint32_t dtc = PLATFORM_UDS_DTC_VEHICLE_TESTER_LATCHED;
    TEST_ASSERT_EQUAL_HEX8((dtc >> 16u) & 0xFFu, got[3]);
    TEST_ASSERT_EQUAL_HEX8((dtc >> 8u) & 0xFFu, got[4]);
    TEST_ASSERT_EQUAL_HEX8(dtc & 0xFFu, got[5]);
    TEST_ASSERT_EQUAL_HEX8(UDS_DTC_STATUS_TEST_FAILED, (uint8_t)(got[6] & UDS_DTC_STATUS_TEST_FAILED));
    TEST_ASSERT_TRUE(read_did(PLATFORM_UDS_DID_VEHICLE_TESTER_STATUS));
    TEST_ASSERT_EQUAL_HEX8(PLATFORM_UDS_VEHICLE_TESTER_STATUS_FAULT_NOT_RUNNING, got[3u + fb]);
}

static void test_a_status_that_stops_being_written_goes_stale_in_the_did_and_the_dtc(void)
{
    pump(300u); /* the healthy status is fed every pass */
    TEST_ASSERT_EQUAL_UINT8(0u, report_dtc_by_mask(UDS_DTC_STATUS_TEST_FAILED));
    feed_status = false; /* the client stops */
    pump(PLATFORM_UDS_VEHICLE_TESTER_STATUS_MAX_AGE_MS + 5u);
    TEST_ASSERT_TRUE(read_did(PLATFORM_UDS_DID_VEHICLE_TESTER_STATUS));
    TEST_ASSERT_EQUAL_HEX8(PLATFORM_UDS_VEHICLE_TESTER_STATUS_FAULT_NOT_RUNNING,
                           got[3u + PLATFORM_UDS_VEHICLE_TESTER_STATUS_FAULT_BYTE]);
    TEST_ASSERT_EQUAL_UINT8(1u, report_dtc_by_mask(UDS_DTC_STATUS_TEST_FAILED));
}

static void test_a_blocked_tx_queue_drops_the_answer_after_p2_star_but_s3_still_runs(void)
{
    enter_extended();
    vbus_set_tx_blocked(&bus_platform, node_dut, true);
    const uint8_t a[] = {UDS_SID_READ_DATA_BY_IDENTIFIER,
                         DID_BYTES(PLATFORM_UDS_DID_ACTIVE_DIAGNOSTIC_SESSION)};
    const uint8_t b[] = {UDS_SID_TESTER_PRESENT, UDS_TESTER_PRESENT_ZERO_SUBFUNCTION};
    have_got = false;
    TEST_ASSERT_EQUAL(ISOTP_OK, isotp_send(&tester, a, sizeof a));
    pump(5u); /* its answer sits in the link, which cannot transmit */
    TEST_ASSERT_EQUAL(ISOTP_OK, isotp_send(&tester, b, sizeof b));
    pump(5u);
    TEST_ASSERT_GREATER_THAN_UINT32(0u, uds_server_glue_stats(&server)->tx_busy);
    TEST_ASSERT_EQUAL_UINT32(0u, uds_server_glue_stats(&server)->tx_expired);
    pump(PLATFORM_UDS_P2_STAR_SERVER_MAX_MS + 100u);
    TEST_ASSERT_EQUAL_UINT32(1u, uds_server_glue_stats(&server)->tx_expired);
    /* S3 kept running although the answer was stuck */
    TEST_ASSERT_EQUAL_UINT8(UDS_SESSION_DEFAULT, uds_server_session(&server));
    TEST_ASSERT_EQUAL_UINT32(1u, uds_server_stats(&server)->s3_timeouts);

    vbus_set_tx_blocked(&bus_platform, node_dut, false);
    pump(200u);
    /* the dropped answer to B (7E 00) never appears; only A's own answer may */
    TEST_ASSERT_EQUAL_UINT32(1u, uds_server_glue_stats(&server)->tx_expired);
    if (have_got) {
        TEST_ASSERT_NOT_EQUAL(POS(UDS_SID_TESTER_PRESENT), got[0]);
    }
    TEST_ASSERT_EQUAL_UINT32(0u, extra_messages);
    /* and the server serves again */
    have_got = false;
    TEST_ASSERT_TRUE(read_did(PLATFORM_UDS_DID_ACTIVE_DIAGNOSTIC_SESSION));
    TEST_ASSERT_EQUAL_HEX8(UDS_SESSION_DEFAULT, got[3]);
}

/* Ç1: a multi-frame answer whose Consecutive Frames get stuck on the platform bus is
 * aborted by services/can_sm after N_As; the link ends the message (tx_failed) and the
 * stale frames never go out when the bus recovers. */
static void test_a_stalled_multi_frame_answer_is_aborted_after_n_as_and_never_goes_out_late(void)
{
    can_sm_step(CAN_PORT_PLATFORM); /* baseline of the controller counters */
    /* A 35-byte answer: a First Frame and five Consecutive Frames, more than the three TX
     * buffers hold, so the message is still under way when the bus stalls. (A message whose
     * last frames all sit in the buffers already counts as sent: the tester's N_Cr covers it.) */
    const uint8_t req[] = {UDS_SID_READ_DATA_BY_IDENTIFIER, DID_BYTES(PLATFORM_UDS_DID_SW_VERSION),
                           DID_BYTES(PLATFORM_UDS_DID_UPTIME),
                           DID_BYTES(PLATFORM_UDS_DID_VEHICLE_ENGINE_SPEED),
                           DID_BYTES(PLATFORM_UDS_DID_VEHICLE_BATTERY_VOLTAGE)};
    const uint32_t mark = sniff_count;
    have_got = false;
    TEST_ASSERT_EQUAL(ISOTP_OK, isotp_send(&tester, req, sizeof req));
    bool first_frame = false;
    for (uint32_t i = 0u; (i < 100u) && !first_frame; i++) {
        can_sm_step(CAN_PORT_PLATFORM);
        tick();
        for (uint32_t k = mark; k < sniff_count; k++) {
            first_frame = first_frame || ((sniffed[k].f.id == PLATFORM_UDS_PHYS_RESPONSE_ID) &&
                                          ((sniffed[k].f.data[0] & PCI_MASK) == PCI_FF));
        }
    }
    TEST_ASSERT_TRUE(first_frame);
    /* the First Frame and the tester's Flow Control got through; now the bus stalls, so
     * the Consecutive Frames the server writes next stay in its TX buffers */
    vbus_set_tx_stalled(&bus_platform, node_dut, true);
    const uint32_t stall_mark = sniff_count;
    const uint32_t failed = uds_server_glue_stats(&server)->tx_failed;
    for (uint32_t i = 0u; i < 3u; i++) {
        can_sm_step(CAN_PORT_PLATFORM);
        tick();
    }
    TEST_ASSERT_EQUAL_UINT32(VBUS_TX_DEPTH, bus_platform.nodes[node_dut].tx_pending_count);
    /* the cut answer would end the tester's reception with N_Cr, which this harness treats
     * as a failure; the tester's side is not under test, so it starts over */
    tester_init();

    for (uint32_t i = 0u; i < CAN_SM_TX_TIMEOUT_MS + 100u; i++) {
        can_sm_step(CAN_PORT_PLATFORM);
        tick();
    }
    TEST_ASSERT_EQUAL_UINT32(0u, bus_platform.nodes[node_dut].tx_pending_count);
    TEST_ASSERT_EQUAL_UINT32(1u, can_sm_stats(CAN_PORT_PLATFORM)->tx_timeouts);
    TEST_ASSERT_EQUAL_UINT32(failed + 1u, uds_server_glue_stats(&server)->tx_failed);
    TEST_ASSERT_EQUAL_UINT32(1u, isotp_link_tx_timeout_count(&server.link));

    vbus_set_tx_stalled(&bus_platform, node_dut, false);
    for (uint32_t i = 0u; i < 200u; i++) {
        can_sm_step(CAN_PORT_PLATFORM);
        tick();
    }
    TEST_ASSERT_EQUAL_UINT32(0u, response_frames_since(stall_mark)); /* nothing went out late */
    TEST_ASSERT_FALSE(have_got);
    TEST_ASSERT_EQUAL_UINT32(0u, extra_messages);
    TEST_ASSERT_EQUAL_UINT32(failed + 1u, uds_server_glue_stats(&server)->tx_failed);

    /* and the server serves again */
    TEST_ASSERT_TRUE(read_did(PLATFORM_UDS_DID_ACTIVE_DIAGNOSTIC_SESSION));
    TEST_ASSERT_EQUAL_HEX8(UDS_SESSION_DEFAULT, got[3]);
}

/* Safety re-review MINOR-A: a functional request that arrives while an answer waits for
 * the link is dropped and counted, never run seconds later. */
static void test_a_functional_request_while_an_answer_is_queued_is_dropped_not_run_late(void)
{
    vbus_set_tx_blocked(&bus_platform, node_dut, true);
    const uint8_t a[] = {UDS_SID_READ_DATA_BY_IDENTIFIER,
                         DID_BYTES(PLATFORM_UDS_DID_ACTIVE_DIAGNOSTIC_SESSION)};
    const uint8_t b[] = {UDS_SID_TESTER_PRESENT, UDS_TESTER_PRESENT_ZERO_SUBFUNCTION};
    const uint8_t f[] = {UDS_SID_DIAGNOSTIC_SESSION_CONTROL, UDS_SESSION_EXTENDED};
    have_got = false;
    TEST_ASSERT_EQUAL(ISOTP_OK, isotp_send(&tester, a, sizeof a));
    pump(5u); /* A's answer sits in the link */
    TEST_ASSERT_EQUAL(ISOTP_OK, isotp_send(&tester, b, sizeof b));
    pump(5u); /* B's answer waits in the glue queue */
    const uint32_t dropped = uds_server_glue_stats(&server)->functional_dropped;
    const uint32_t taken = uds_server_glue_stats(&server)->functional_taken;
    send_functional_sf(f, (uint8_t)sizeof f);
    pump(3000u);
    TEST_ASSERT_EQUAL_UINT32(dropped + 1u, uds_server_glue_stats(&server)->functional_dropped);
    vbus_set_tx_blocked(&bus_platform, node_dut, false);
    pump(PLATFORM_UDS_P2_STAR_SERVER_MAX_MS);
    TEST_ASSERT_EQUAL_UINT32(taken, uds_server_glue_stats(&server)->functional_taken);
    TEST_ASSERT_EQUAL_UINT8(UDS_SESSION_DEFAULT, uds_server_session(&server));
    extra_messages = 0u; /* A's and B's late answers are not under test here */
}

static void test_functional_single_frame_with_a_short_dlc_is_accepted(void)
{
    const uint8_t sf[4] = {3u, UDS_SID_READ_DATA_BY_IDENTIFIER,
                           HI(PLATFORM_UDS_DID_ACTIVE_DIAGNOSTIC_SESSION),
                           LO(PLATFORM_UDS_DID_ACTIVE_DIAGNOSTIC_SESSION)};
    have_got = false;
    send_raw(PLATFORM_UDS_FUNCTIONAL_REQUEST_ID, sf, sizeof sf); /* DLC 4, SF_DL = DLC - 1 */
    TEST_ASSERT_TRUE(wait_response(BOUND_MS));
    TEST_ASSERT_EQUAL_UINT16(4u, got_len);
    TEST_ASSERT_EQUAL_HEX8(UDS_SESSION_DEFAULT, got[3]);
    TEST_ASSERT_EQUAL_UINT32(1u, uds_server_glue_stats(&server)->functional_taken);
}

static void test_functional_frame_with_dlc_zero_is_dropped(void)
{
    const uint8_t none[1] = {0u};
    const uint32_t mark = sniff_count;
    send_raw(PLATFORM_UDS_FUNCTIONAL_REQUEST_ID, none, 0u);
    pump(50u);
    TEST_ASSERT_EQUAL_UINT32(1u, uds_server_glue_stats(&server)->functional_dropped);
    TEST_ASSERT_EQUAL_UINT32(0u, uds_server_glue_stats(&server)->functional_taken);
    TEST_ASSERT_EQUAL_UINT32(0u, response_frames_since(mark));
}

static void test_an_extended_format_frame_on_the_functional_id_is_not_routed_or_answered(void)
{
    allow_extended = true;
    const uint8_t sf[PLATFORM_UDS_FRAME_DLC] = {3u, UDS_SID_READ_DATA_BY_IDENTIFIER,
                                                HI(PLATFORM_UDS_DID_ACTIVE_DIAGNOSTIC_SESSION),
                                                LO(PLATFORM_UDS_DID_ACTIVE_DIAGNOSTIC_SESSION),
                                                PLATFORM_UDS_PADDING_BYTE, PLATFORM_UDS_PADDING_BYTE,
                                                PLATFORM_UDS_PADDING_BYTE, PLATFORM_UDS_PADDING_BYTE};
    const uint32_t unrouted = can_if_unrouted_count(CAN_PORT_PLATFORM);
    const uint32_t mark = sniff_count;
    send_frame(PLATFORM_UDS_FUNCTIONAL_REQUEST_ID, true, sf, PLATFORM_UDS_FRAME_DLC);
    pump(50u);
    TEST_ASSERT_EQUAL_UINT32(unrouted + 1u, can_if_unrouted_count(CAN_PORT_PLATFORM));
    TEST_ASSERT_EQUAL_UINT32(0u, uds_server_glue_stats(&server)->functional_taken);
    TEST_ASSERT_EQUAL_UINT32(0u, response_frames_since(mark));
    TEST_ASSERT_FALSE(have_got);
}

static void inject_vehicle_frame(uint32_t id, bool extended)
{
    can_frame_t f;
    memset(&f, 0, sizeof f);
    f.id = id;
    f.extended = extended;
    f.dlc = 8u;
    const uint8_t req[8] = {0x02u, UDS_SID_TESTER_PRESENT, UDS_TESTER_PRESENT_ZERO_SUBFUNCTION,
                            PLATFORM_UDS_PADDING_BYTE, PLATFORM_UDS_PADDING_BYTE,
                            PLATFORM_UDS_PADDING_BYTE, PLATFORM_UDS_PADDING_BYTE,
                            PLATFORM_UDS_PADDING_BYTE};
    memcpy(f.data, req, 8u);
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_send(&bus_vehicle, node_vinject, &f));
}

static void test_platform_request_ids_never_latch_the_vehicle_client(void)
{
    start_client();
    pump(600u);
    TEST_ASSERT_TRUE(uds_client_session_up(&client));
    TEST_ASSERT_FALSE(uds_client_failed(&client));
    const uint8_t func[] = {UDS_SID_READ_DATA_BY_IDENTIFIER,
                            DID_BYTES(PLATFORM_UDS_DID_ACTIVE_DIAGNOSTIC_SESSION)};
    have_got = false;
    send_functional_sf(func, sizeof func); /* platform 0x7DF, the ID the vehicle client watches */
    TEST_ASSERT_TRUE(wait_response(BOUND_MS));
    const uint8_t phys[] = {UDS_SID_TESTER_PRESENT, UDS_TESTER_PRESENT_ZERO_SUBFUNCTION};
    have_got = false;
    TEST_ASSERT_TRUE(REQUEST(phys)); /* platform 0x710 */
    pump(200u);
    TEST_ASSERT_FALSE(uds_client_failed(&client));
    TEST_ASSERT_EQUAL(UDS_CLIENT_FAULT_NONE, uds_client_fault(&client));
    TEST_ASSERT_TRUE(uds_client_session_up(&client));
}

static void test_vehicle_functional_watch_frames_latch_the_client_but_never_reach_the_server(void)
{
    for (uint32_t i = 0u; i < VEHICLE_CL250_FUNCTIONAL_WATCH_COUNT; i++) {
        if (i > 0u) {
            tearDown();
            setUp();
        }
        start_client();
        pump(600u);
        TEST_ASSERT_FALSE(uds_client_failed(&client));
        const uint32_t platform_frames = vbus_frame_count(&bus_platform);
        inject_vehicle_frame(vehicle_cl250_functional_watch[i].id,
                             vehicle_cl250_functional_watch[i].extended);
        pump(300u);
        TEST_ASSERT_TRUE(uds_client_failed(&client));
        TEST_ASSERT_EQUAL(UDS_CLIENT_FAULT_FOREIGN_TESTER, uds_client_fault(&client));
        TEST_ASSERT_EQUAL_UINT32(0u, uds_server_glue_stats(&server)->functional_taken);
        TEST_ASSERT_EQUAL_UINT32(0u, uds_server_glue_stats(&server)->functional_dropped);
        TEST_ASSERT_EQUAL_UINT32(0u, uds_server_stats(&server)->requests);
        TEST_ASSERT_EQUAL_UINT32(platform_frames, vbus_frame_count(&bus_platform));
    }
}

static void run_session_and_clear_sequence(void)
{
    enter_extended();
    const uint8_t clear[] = {UDS_SID_CLEAR_DIAGNOSTIC_INFORMATION,
                             (UDS_GROUP_OF_DTC_ALL >> 16u) & 0xFFu,
                             (UDS_GROUP_OF_DTC_ALL >> 8u) & 0xFFu, UDS_GROUP_OF_DTC_ALL & 0xFFu};
    TEST_ASSERT_TRUE(REQUEST(clear));
    TEST_ASSERT_EQUAL_HEX8(POS(UDS_SID_CLEAR_DIAGNOSTIC_INFORMATION), got[0]);
    const uint8_t back[] = {UDS_SID_DIAGNOSTIC_SESSION_CONTROL, UDS_SESSION_DEFAULT};
    TEST_ASSERT_TRUE(REQUEST(back));
    TEST_ASSERT_EQUAL_HEX8(POS(UDS_SID_DIAGNOSTIC_SESSION_CONTROL), got[0]);
    pump(100u);
}

static void test_platform_session_and_clear_requests_leave_the_vehicle_client_unchanged(void)
{
    start_client();
    pump(600u);
    TEST_ASSERT_TRUE(uds_client_session_up(&client));
    const uint32_t vehicle_frames = vbus_frame_count(&bus_vehicle);
    run_session_and_clear_sequence();
    TEST_ASSERT_TRUE(uds_client_session_up(&client));
    TEST_ASSERT_TRUE(uds_client_ecu_present(&client));
    TEST_ASSERT_FALSE(uds_client_failed(&client));
    TEST_ASSERT_EQUAL(UDS_CLIENT_FAULT_NONE, uds_client_fault(&client));
    TEST_ASSERT_GREATER_THAN_UINT32(vehicle_frames, vbus_frame_count(&bus_vehicle)); /* still polling */
}

static void test_platform_session_and_clear_requests_never_release_a_latched_client(void)
{
    start_client();
    pump(600u);
    inject_vehicle_frame(VEHICLE_CL250_REQUEST_ID, true); /* a second tester */
    pump(20u);
    TEST_ASSERT_TRUE(uds_client_failed(&client));
    const uds_client_fault_t fault = uds_client_fault(&client);
    TEST_ASSERT_EQUAL(UDS_CLIENT_FAULT_FOREIGN_TESTER, fault);
    run_session_and_clear_sequence();
    pump(600u);
    TEST_ASSERT_TRUE(uds_client_failed(&client));
    TEST_ASSERT_EQUAL(fault, uds_client_fault(&client));
    /* the level-triggered monitor reports the latch again after the 0x14 clear */
    TEST_ASSERT_EQUAL_UINT8(1u, report_dtc_by_mask(UDS_DTC_STATUS_TEST_FAILED));
    const uint32_t dtc = PLATFORM_UDS_DTC_VEHICLE_TESTER_LATCHED;
    TEST_ASSERT_EQUAL_HEX8((dtc >> 16u) & 0xFFu, got[3]);
    TEST_ASSERT_TRUE(read_did(PLATFORM_UDS_DID_VEHICLE_TESTER_STATUS));
    TEST_ASSERT_EQUAL_HEX8(PLATFORM_UDS_VEHICLE_TESTER_STATUS_FAULT_FOREIGN_TESTER,
                           got[3u + PLATFORM_UDS_VEHICLE_TESTER_STATUS_FAULT_BYTE]);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_open_twice_is_busy_and_null_is_an_argument_error);
    RUN_TEST(test_getters_and_step_are_safe_for_null_and_unopened_servers);
    RUN_TEST(test_a_full_receiver_table_fails_open_and_leaves_the_server_closed);
    RUN_TEST(test_server_frames_use_only_the_gen_response_id_dlc_and_padding);
    RUN_TEST(test_the_vehicle_bus_never_sees_a_frame);
    RUN_TEST(test_read_of_the_software_version_is_a_segmented_nul_padded_record);
    RUN_TEST(test_multi_did_read_is_segmented_both_ways_and_in_request_order);
    RUN_TEST(test_tester_status_did_reflects_diag_bits_and_fault);
    RUN_TEST(test_uptime_did_counts_seconds_since_open);
    RUN_TEST(test_vehicle_sample_dids_go_none_valid_stale_with_big_endian_raw);
    RUN_TEST(test_reported_dtc_is_listed_by_read_dtc_information);
    RUN_TEST(test_dtc_count_and_supported_list_through_the_link);
    RUN_TEST(test_clear_in_the_default_session_is_refused_and_keeps_the_dtc);
    RUN_TEST(test_clear_in_the_extended_session_clears_and_a_still_failing_monitor_sets_it_again);
    RUN_TEST(test_session_control_through_the_glue_changes_the_session_getter);
    RUN_TEST(test_the_session_survives_until_just_before_s3);
    RUN_TEST(test_the_session_falls_back_to_default_after_s3);
    RUN_TEST(test_suppressed_tester_present_keeps_the_session_and_sends_no_frame);
    RUN_TEST(test_functional_single_frame_read_is_answered_on_the_physical_response_id);
    RUN_TEST(test_functional_unsupported_service_gets_no_answer);
    RUN_TEST(test_functional_first_frame_is_dropped_without_flow_control);
    RUN_TEST(test_malformed_functional_single_frames_are_dropped);
    RUN_TEST(test_functional_request_during_a_physical_reception_is_dropped_and_the_reception_completes);
    RUN_TEST(test_functional_request_after_the_answer_is_taken_again);
    RUN_TEST(test_every_answer_starts_within_p2_server_max_of_the_request);
    RUN_TEST(test_without_a_client_the_tester_status_reads_not_running_and_the_dtc_fails);
    RUN_TEST(test_a_status_that_stops_being_written_goes_stale_in_the_did_and_the_dtc);
    RUN_TEST(test_a_blocked_tx_queue_drops_the_answer_after_p2_star_but_s3_still_runs);
    RUN_TEST(test_a_stalled_multi_frame_answer_is_aborted_after_n_as_and_never_goes_out_late);
    RUN_TEST(test_a_functional_request_while_an_answer_is_queued_is_dropped_not_run_late);
    RUN_TEST(test_functional_single_frame_with_a_short_dlc_is_accepted);
    RUN_TEST(test_functional_frame_with_dlc_zero_is_dropped);
    RUN_TEST(test_an_extended_format_frame_on_the_functional_id_is_not_routed_or_answered);
    RUN_TEST(test_platform_request_ids_never_latch_the_vehicle_client);
    RUN_TEST(test_vehicle_functional_watch_frames_latch_the_client_but_never_reach_the_server);
    RUN_TEST(test_platform_session_and_clear_requests_leave_the_vehicle_client_unchanged);
    RUN_TEST(test_platform_session_and_clear_requests_never_release_a_latched_client);
    return UNITY_END();
}
