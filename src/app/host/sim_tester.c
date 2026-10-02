#include "app/host/sim_tester.h"

#include "platform_uds.h"
#include "uds_iso14229.h"
#include "vehicle_cl250.h"

#include <stdio.h>
#include <string.h>

#define POS(sid) ((uint8_t)((sid) + UDS_POSITIVE_RESPONSE_OFFSET))
#define HI(v) ((uint8_t)(((uint32_t)(v) >> 8u) & 0xFFu))
#define LO(v) ((uint8_t)((uint32_t)(v) & 0xFFu))
#define B2(v) ((uint8_t)(((uint32_t)(v) >> 16u) & 0xFFu))
#define GROUP_ALL B2(UDS_GROUP_OF_DTC_ALL), HI(UDS_GROUP_OF_DTC_ALL), LO(UDS_GROUP_OF_DTC_ALL)
#define P2 PLATFORM_UDS_P2_SERVER_MAX_MS
#define P2S_UNITS (PLATFORM_UDS_P2_STAR_SERVER_MAX_MS / UDS_P2_STAR_RESOLUTION_MS)
/* How long a suppressed answer is waited for: twice the time a real one may take. */
#define SILENCE_MS (2u * P2)
/* Let the UDS client open the ECU session and read the engine speed first. */
#define START_DELAY_MS (VEHICLE_CL250_SESSION_RETRY_INTERVAL_MS / 4u)
#define MAX_REQ 12u
#define MAX_EXPECT 16u

typedef enum { CHECK_NONE = 0, CHECK_MULTI_READ, CHECK_HEALTH } check_t;

typedef struct {
    const char* name;
    uint32_t delay_ms;         /* wait before sending */
    bool functional;
    uint8_t req[MAX_REQ];
    uint8_t req_len;
    bool silent;               /* no answer may come within SILENCE_MS */
    uint8_t expect[MAX_EXPECT];
    uint8_t expect_len;        /* prefix compared byte for byte */
    uint16_t rsp_len;          /* exact answer length; 0 = expect_len */
    check_t check;
} step_t;

static const step_t script[] = {
    {"tester present", START_DELAY_MS, false,
     {UDS_SID_TESTER_PRESENT, UDS_TESTER_PRESENT_ZERO_SUBFUNCTION}, 2u, false,
     {POS(UDS_SID_TESTER_PRESENT), UDS_TESTER_PRESENT_ZERO_SUBFUNCTION}, 2u, 0u, CHECK_NONE},
    {"session is default", 0u, false,
     {UDS_SID_READ_DATA_BY_IDENTIFIER, HI(PLATFORM_UDS_DID_ACTIVE_DIAGNOSTIC_SESSION),
      LO(PLATFORM_UDS_DID_ACTIVE_DIAGNOSTIC_SESSION)}, 3u, false,
     {POS(UDS_SID_READ_DATA_BY_IDENTIFIER), HI(PLATFORM_UDS_DID_ACTIVE_DIAGNOSTIC_SESSION),
      LO(PLATFORM_UDS_DID_ACTIVE_DIAGNOSTIC_SESSION), UDS_SESSION_DEFAULT}, 4u, 0u, CHECK_NONE},
    {"0x14 refused in the default session", 0u, false,
     {UDS_SID_CLEAR_DIAGNOSTIC_INFORMATION, GROUP_ALL}, 4u, false,
     {UDS_SID_NEGATIVE_RESPONSE, UDS_SID_CLEAR_DIAGNOSTIC_INFORMATION,
      UDS_NRC_SERVICE_NOT_SUPPORTED_IN_ACTIVE_SESSION}, 3u, 0u, CHECK_NONE},
    {"extended session, P2 / P2*", 0u, false,
     {UDS_SID_DIAGNOSTIC_SESSION_CONTROL, UDS_SESSION_EXTENDED}, 2u, false,
     {POS(UDS_SID_DIAGNOSTIC_SESSION_CONTROL), UDS_SESSION_EXTENDED, HI(P2), LO(P2),
      HI(P2S_UNITS), LO(P2S_UNITS)}, 6u, 0u, CHECK_NONE},
    {"segmented multi-DID read", 0u, false,
     {UDS_SID_READ_DATA_BY_IDENTIFIER, HI(PLATFORM_UDS_DID_SW_VERSION),
      LO(PLATFORM_UDS_DID_SW_VERSION), HI(PLATFORM_UDS_DID_VEHICLE_TESTER_STATUS),
      LO(PLATFORM_UDS_DID_VEHICLE_TESTER_STATUS), HI(PLATFORM_UDS_DID_UPTIME),
      LO(PLATFORM_UDS_DID_UPTIME), HI(PLATFORM_UDS_DID_VEHICLE_ENGINE_SPEED),
      LO(PLATFORM_UDS_DID_VEHICLE_ENGINE_SPEED)}, 9u, false,
     {POS(UDS_SID_READ_DATA_BY_IDENTIFIER), HI(PLATFORM_UDS_DID_SW_VERSION),
      LO(PLATFORM_UDS_DID_SW_VERSION)}, 3u, 0u, CHECK_MULTI_READ},
    {"rt-core health (0xFD02)", 0u, false,
     {UDS_SID_READ_DATA_BY_IDENTIFIER, HI(PLATFORM_UDS_DID_RT_CORE_HEALTH),
      LO(PLATFORM_UDS_DID_RT_CORE_HEALTH)}, 3u, false,
     {POS(UDS_SID_READ_DATA_BY_IDENTIFIER), HI(PLATFORM_UDS_DID_RT_CORE_HEALTH),
      LO(PLATFORM_UDS_DID_RT_CORE_HEALTH)}, 3u,
     (uint16_t)(3u + PLATFORM_UDS_DID_RT_CORE_HEALTH_LENGTH), CHECK_HEALTH},
    {"DTC count (none failing)", 0u, false,
     {UDS_SID_READ_DTC_INFORMATION, UDS_READ_DTC_REPORT_NUMBER_OF_DTC_BY_STATUS_MASK,
      PLATFORM_UDS_DTC_STATUS_AVAILABILITY_MASK}, 3u,
     false,
     {POS(UDS_SID_READ_DTC_INFORMATION), UDS_READ_DTC_REPORT_NUMBER_OF_DTC_BY_STATUS_MASK,
      PLATFORM_UDS_DTC_STATUS_AVAILABILITY_MASK, UDS_DTC_FORMAT_ISO_14229_1, 0u, 0u}, 6u, 0u,
     CHECK_NONE},
    {"supported DTC list", 0u, false,
     {UDS_SID_READ_DTC_INFORMATION, UDS_READ_DTC_REPORT_SUPPORTED_DTC}, 2u, false,
     {POS(UDS_SID_READ_DTC_INFORMATION), UDS_READ_DTC_REPORT_SUPPORTED_DTC,
      PLATFORM_UDS_DTC_STATUS_AVAILABILITY_MASK}, 3u,
     (uint16_t)(3u + (4u * PLATFORM_UDS_DTC_COUNT)), CHECK_NONE},
    {"0x14 clear in the extended session", 0u, false,
     {UDS_SID_CLEAR_DIAGNOSTIC_INFORMATION, GROUP_ALL}, 4u, false,
     {POS(UDS_SID_CLEAR_DIAGNOSTIC_INFORMATION)}, 1u, 0u, CHECK_NONE},
    {"programming session refused", 0u, false,
     {UDS_SID_DIAGNOSTIC_SESSION_CONTROL, UDS_SESSION_PROGRAMMING}, 2u, false,
     {UDS_SID_NEGATIVE_RESPONSE, UDS_SID_DIAGNOSTIC_SESSION_CONTROL,
      UDS_NRC_SUBFUNCTION_NOT_SUPPORTED}, 3u, 0u, CHECK_NONE},
    {"functional tester present, suppressed", 0u, true,
     {UDS_SID_TESTER_PRESENT, (uint8_t)(UDS_TESTER_PRESENT_ZERO_SUBFUNCTION |
                                        UDS_SUPPRESS_POS_RSP_BIT)}, 2u, true,
     {0u}, 0u, 0u, CHECK_NONE},
    {"functional read", 0u, true,
     {UDS_SID_READ_DATA_BY_IDENTIFIER, HI(PLATFORM_UDS_DID_ACTIVE_DIAGNOSTIC_SESSION),
      LO(PLATFORM_UDS_DID_ACTIVE_DIAGNOSTIC_SESSION)}, 3u, false,
     {POS(UDS_SID_READ_DATA_BY_IDENTIFIER), HI(PLATFORM_UDS_DID_ACTIVE_DIAGNOSTIC_SESSION),
      LO(PLATFORM_UDS_DID_ACTIVE_DIAGNOSTIC_SESSION), UDS_SESSION_EXTENDED}, 4u, 0u, CHECK_NONE},
    {"functional NRC 0x12 suppressed", 0u, true,
     {UDS_SID_DIAGNOSTIC_SESSION_CONTROL, UDS_SESSION_PROGRAMMING}, 2u, true,
     {0u}, 0u, 0u, CHECK_NONE},
    {"S3 expiry back to default", PLATFORM_UDS_S3_SERVER_MS + P2, false,
     {UDS_SID_READ_DATA_BY_IDENTIFIER, HI(PLATFORM_UDS_DID_ACTIVE_DIAGNOSTIC_SESSION),
      LO(PLATFORM_UDS_DID_ACTIVE_DIAGNOSTIC_SESSION)}, 3u, false,
     {POS(UDS_SID_READ_DATA_BY_IDENTIFIER), HI(PLATFORM_UDS_DID_ACTIVE_DIAGNOSTIC_SESSION),
      LO(PLATFORM_UDS_DID_ACTIVE_DIAGNOSTIC_SESSION), UDS_SESSION_DEFAULT}, 4u, 0u, CHECK_NONE},
};

#define STEP_COUNT ((uint32_t)(sizeof script / sizeof script[0]))

static void fail(sim_tester_t* t, const char* what)
{
    (void)snprintf(t->reason, sizeof t->reason, "step %u (%s): %s", (unsigned)t->step,
                   script[t->step].name, what);
    t->state = SIM_TESTER_FAILED;
}

static void enter(sim_tester_t* t, sim_tester_state_t s, uint32_t now)
{
    t->state = s;
    t->state_since_ms = now;
}

static void next_step(sim_tester_t* t, uint32_t now)
{
    t->steps_passed++;
    t->step++;
    enter(t, (t->step < STEP_COUNT) ? SIM_TESTER_DELAY : SIM_TESTER_DONE, now);
}

bool sim_tester_init(sim_tester_t* t, vbus_t* bus, uint32_t now_ms)
{
    memset(t, 0, sizeof *t);
    t->bus = bus;
    if (!vbus_attach(bus, &t->node)) {
        return false;
    }
    isotp_config_t cfg;
    isotp_default_config(&cfg);
    cfg.padding_enabled = true;
    cfg.padding_byte = PLATFORM_UDS_PADDING_BYTE;
    cfg.n_bs_ms = PLATFORM_UDS_N_BS_MS;
    cfg.n_cr_ms = PLATFORM_UDS_N_CR_MS;
    if (isotp_init(&t->iso, &cfg, t->rx_buf, SIM_TESTER_BUF, t->tx_buf, SIM_TESTER_BUF) !=
        ISOTP_OK) {
        return false;
    }
    enter(t, SIM_TESTER_DELAY, now_ms);
    return true;
}

static uint16_t did_len(uint16_t did)
{
    const platform_uds_did_t* d = platform_uds_find_did(did);
    return (d == NULL) ? 0u : d->length;
}

/* Answer of 0x22 F189 FD00 FD01 FD10: [0x62] then (DID, record) in request order. */
static uint16_t multi_read_len(void)
{
    return (uint16_t)(1u + 8u + did_len(PLATFORM_UDS_DID_SW_VERSION) +
                      did_len(PLATFORM_UDS_DID_VEHICLE_TESTER_STATUS) +
                      did_len(PLATFORM_UDS_DID_UPTIME) +
                      did_len(PLATFORM_UDS_DID_VEHICLE_ENGINE_SPEED));
}

static bool check_multi_read(sim_tester_t* t, const uint8_t* rsp)
{
    const uint16_t off_status = (uint16_t)(1u + 2u + did_len(PLATFORM_UDS_DID_SW_VERSION) + 2u);
    const uint16_t off_sample = (uint16_t)(off_status + did_len(PLATFORM_UDS_DID_VEHICLE_TESTER_STATUS) +
                                           2u + did_len(PLATFORM_UDS_DID_UPTIME) + 2u);
    const uint8_t flags = rsp[off_status + PLATFORM_UDS_VEHICLE_TESTER_STATUS_ECU_PRESENT_BYTE];
    if (!((rsp[3] >= (uint8_t)'0') && (rsp[3] <= (uint8_t)'9'))) {
        fail(t, "0xF189 is not a version string");
        return false;
    }
    if ((flags & PLATFORM_UDS_VEHICLE_TESTER_STATUS_ECU_PRESENT_MASK) == 0u) {
        fail(t, "0xFD00 does not report the ECU present");
        return false;
    }
    if (rsp[off_sample] != PLATFORM_UDS_VEHICLE_SAMPLE_STATE_VALID) {
        fail(t, "0xFD10 engine speed sample is not VALID");
        return false;
    }
    return true;
}

/* 0xFD02 on a healthy bench (D-055): the client's step counters are fresh, both ports
 * are known and ERROR_ACTIVE, and the vehicle port is not latched. */
static bool check_health(sim_tester_t* t, const uint8_t* rsp)
{
    const uint8_t* rec = &rsp[3];
    const uint8_t flags = rec[PLATFORM_UDS_RT_CORE_HEALTH_STEP_STATS_FRESH_BYTE];
    if ((flags & PLATFORM_UDS_RT_CORE_HEALTH_STEP_STATS_FRESH_MASK) == 0u) {
        fail(t, "0xFD02 step counters are not fresh");
        return false;
    }
    if ((flags & PLATFORM_UDS_RT_CORE_HEALTH_VEHICLE_LATCHED_MASK) != 0u) {
        fail(t, "0xFD02 reports the vehicle port latched");
        return false;
    }
    if ((rec[PLATFORM_UDS_RT_CORE_HEALTH_VEHICLE_STATE_BYTE] !=
         PLATFORM_UDS_RT_CORE_HEALTH_VEHICLE_STATE_ERROR_ACTIVE) ||
        (rec[PLATFORM_UDS_RT_CORE_HEALTH_PLATFORM_STATE_BYTE] !=
         PLATFORM_UDS_RT_CORE_HEALTH_PLATFORM_STATE_ERROR_ACTIVE)) {
        fail(t, "0xFD02 port state is not ERROR_ACTIVE");
        return false;
    }
    return true;
}

static void on_answer(sim_tester_t* t, uint32_t now, const uint8_t* rsp, uint16_t len)
{
    const step_t* s = &script[t->step];
    const uint16_t want = (s->check == CHECK_MULTI_READ) ? multi_read_len()
                          : (s->rsp_len != 0u)             ? s->rsp_len
                                                           : s->expect_len;
    char what[SIM_TESTER_REASON_LEN];
    if (s->silent) {
        (void)snprintf(what, sizeof what, "answered (0x%02X, %u bytes) but must stay silent",
                       (unsigned)rsp[0], (unsigned)len);
        fail(t, what);
        return;
    }
    if ((len != want) || (memcmp(rsp, s->expect, s->expect_len) != 0)) {
        (void)snprintf(what, sizeof what, "answer %02X %02X %02X (%u bytes), want %02X %02X %02X (%u)",
                       (unsigned)rsp[0], (unsigned)((len > 1u) ? rsp[1] : 0u),
                       (unsigned)((len > 2u) ? rsp[2] : 0u), (unsigned)len,
                       (unsigned)s->expect[0], (unsigned)s->expect[1], (unsigned)s->expect[2],
                       (unsigned)want);
        fail(t, what);
        return;
    }
    if ((s->check == CHECK_MULTI_READ) && !check_multi_read(t, rsp)) {
        return;
    }
    if ((s->check == CHECK_HEALTH) && !check_health(t, rsp)) {
        return;
    }
    next_step(t, now);
}

static void send_functional(sim_tester_t* t, const step_t* s)
{
    can_frame_t f;
    memset(&f, 0, sizeof f);
    f.id = PLATFORM_UDS_FUNCTIONAL_REQUEST_ID;
    f.extended = false;
    f.dlc = PLATFORM_UDS_FRAME_DLC;
    memset(f.data, PLATFORM_UDS_PADDING_BYTE, sizeof f.data);
    f.data[0] = s->req_len; /* Single Frame PCI */
    memcpy(&f.data[1], s->req, s->req_len);
    (void)vbus_send(t->bus, t->node, &f);
}

void sim_tester_step(sim_tester_t* t, uint32_t now_ms)
{
    can_frame_t f;
    while (vbus_recv(t->bus, t->node, &f) == CAN_PORT_OK) {
        if (f.extended || (f.id != PLATFORM_UDS_PHYS_RESPONSE_ID)) {
            continue;
        }
        if ((t->state == SIM_TESTER_WAIT) && !t->first_frame_seen) {
            t->first_frame_seen = true;
            const uint32_t lat = now_ms - t->sent_ms;
            if (lat > t->max_latency_ms) {
                t->max_latency_ms = lat;
            }
            if (lat > P2) {
                fail(t, "first answer frame later than P2server_max");
            }
        }
        isotp_frame_t in;
        memcpy(in.data, f.data, sizeof in.data);
        in.dlc = f.dlc;
        isotp_on_frame(&t->iso, &in, now_ms);
    }
    isotp_n_result_t res = ISOTP_N_OK;
    uint16_t len = 0u;
    if (isotp_take_rx_indication(&t->iso, &res, &len)) {
        if ((t->state == SIM_TESTER_WAIT) && (res == ISOTP_N_OK)) {
            on_answer(t, now_ms, t->rx_buf, len);
        } else if (t->state == SIM_TESTER_WAIT) {
            fail(t, "ISO-TP reception failed");
        }
        if (res == ISOTP_N_OK) {
            isotp_rx_release(&t->iso);
        }
    }
    (void)isotp_take_tx_confirm(&t->iso, &res);

    if (t->state < SIM_TESTER_DONE) {
        const step_t* s = &script[t->step];
        if ((t->state == SIM_TESTER_DELAY) && ((now_ms - t->state_since_ms) >= s->delay_ms)) {
            if (s->functional) {
                send_functional(t, s);
            } else {
                (void)isotp_send(&t->iso, s->req, s->req_len);
            }
            t->first_frame_seen = false;
            t->sent_ms = now_ms;
            enter(t, SIM_TESTER_WAIT, now_ms);
        } else if ((t->state == SIM_TESTER_WAIT) && s->silent &&
                   ((now_ms - t->state_since_ms) >= SILENCE_MS)) {
            next_step(t, now_ms);
        } else if ((t->state == SIM_TESTER_WAIT) &&
                   ((now_ms - t->state_since_ms) >= PLATFORM_UDS_P2_STAR_SERVER_MAX_MS)) {
            fail(t, "no answer");
        } else {
            /* waiting */
        }
    }
    isotp_frame_t out;
    while (isotp_poll(&t->iso, now_ms, &out)) {
        can_frame_t tx;
        memset(&tx, 0, sizeof tx);
        tx.id = PLATFORM_UDS_PHYS_REQUEST_ID;
        tx.extended = false;
        tx.dlc = out.dlc;
        memcpy(tx.data, out.data, sizeof tx.data);
        (void)vbus_send(t->bus, t->node, &tx);
        if (t->state == SIM_TESTER_WAIT) {
            t->sent_ms = now_ms; /* P2 counts from the request's last frame */
        }
    }
}

bool sim_tester_finished(const sim_tester_t* t)
{
    return t->state >= SIM_TESTER_DONE;
}

bool sim_tester_passed(const sim_tester_t* t)
{
    return t->state == SIM_TESTER_DONE;
}

uint32_t sim_tester_step_count(void)
{
    return STEP_COUNT;
}
