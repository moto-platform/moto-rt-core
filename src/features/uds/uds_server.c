#include "features/uds/uds_server.h"

#include "platform_uds.h"
#include "services/can_if.h"
#include "services/can_sm.h"
#include "services/diag.h"
#include "services/timebase.h"
#include "services/vehicle_signals.h"
#include "uds_iso14229.h"
#include "vehicle_cl250.h"

#include <stddef.h>

#ifndef MOTO_RTCORE_VERSION
#error "MOTO_RTCORE_VERSION (CMake project VERSION) must be defined by the build"
#endif

_Static_assert(PLATFORM_UDS_FRAME_DLC == ISOTP_CAN_DL, "platform frames are 8 bytes");
_Static_assert(PLATFORM_UDS_PHYS_REQUEST_ID <= CAN_PORT_STD_ID_MAX, "11-bit platform IDs");
_Static_assert(PLATFORM_UDS_FUNCTIONAL_REQUEST_ID <= CAN_PORT_STD_ID_MAX, "11-bit platform IDs");
_Static_assert(sizeof(MOTO_RTCORE_VERSION) <= PLATFORM_UDS_DID_SW_VERSION_LENGTH,
               "version must fit DID 0xF189");
_Static_assert(PLATFORM_UDS_DID_VEHICLE_TESTER_STATUS_LENGTH == 2u, "0xFD00 layout");
/* 0xFD02 (D-055): one state mapping for both ports, flags in one byte, u16 step counters.
 * The per-port counter widths are asserted below with the layout table. */
_Static_assert(PLATFORM_UDS_DID_RT_CORE_HEALTH_LENGTH == 23u, "0xFD02 layout");
_Static_assert((PLATFORM_UDS_RT_CORE_HEALTH_STEP_STATS_FRESH_BYTE == 0u) &&
                   (PLATFORM_UDS_RT_CORE_HEALTH_VEHICLE_LATCHED_BYTE == 0u),
               "0xFD02 flags in byte 0");
_Static_assert((PLATFORM_UDS_RT_CORE_HEALTH_VEHICLE_STATE_ERROR_ACTIVE ==
                PLATFORM_UDS_RT_CORE_HEALTH_PLATFORM_STATE_ERROR_ACTIVE) &&
                   (PLATFORM_UDS_RT_CORE_HEALTH_VEHICLE_STATE_ERROR_PASSIVE ==
                    PLATFORM_UDS_RT_CORE_HEALTH_PLATFORM_STATE_ERROR_PASSIVE) &&
                   (PLATFORM_UDS_RT_CORE_HEALTH_VEHICLE_STATE_BUS_OFF ==
                    PLATFORM_UDS_RT_CORE_HEALTH_PLATFORM_STATE_BUS_OFF) &&
                   (PLATFORM_UDS_RT_CORE_HEALTH_VEHICLE_STATE_LATCHED ==
                    PLATFORM_UDS_RT_CORE_HEALTH_PLATFORM_STATE_LATCHED) &&
                   (PLATFORM_UDS_RT_CORE_HEALTH_VEHICLE_STATE_UNKNOWN ==
                    PLATFORM_UDS_RT_CORE_HEALTH_PLATFORM_STATE_UNKNOWN),
               "0xFD02 port state codes");
_Static_assert((PLATFORM_UDS_RT_CORE_HEALTH_VEHICLE_STATE_LENGTH == 1u) &&
                   (PLATFORM_UDS_RT_CORE_HEALTH_PLATFORM_STATE_LENGTH == 1u),
               "0xFD02 port state is one byte");

#define MS_PER_S 1000u

/* ------------------------------------------------------------------ provider */

static void put_be(uint8_t* out, uint16_t len, uint32_t value)
{
    uint32_t v = value;
    for (uint16_t i = len; i > 0u; i--) {
        out[i - 1u] = (uint8_t)(v & 0xFFu);
        v >>= 8u;
    }
}

/* Saturates instead of truncating (D-055: a reported counter never goes down). */
static uint32_t clamp(uint32_t value, uint32_t max)
{
    return (value > max) ? max : value;
}

/* Byte offsets and widths of one CAN port's block in 0xFD02. */
typedef struct {
    can_port_id_t port;
    uint8_t state_byte;
    uint8_t bus_offs_byte;
    uint8_t recoveries_byte;
    uint8_t deferred_byte;
    uint8_t nas_aborts_byte;
} health_port_t;

#define HEALTH_COUNTER_LEN 2u
#define HEALTH_COUNTER_MAX 0xFFFFu
_Static_assert((PLATFORM_UDS_RT_CORE_HEALTH_STEP_OVERRUNS_LENGTH == HEALTH_COUNTER_LEN) &&
                   (PLATFORM_UDS_RT_CORE_HEALTH_STEP_GAP_MAX_MS_LENGTH == HEALTH_COUNTER_LEN) &&
                   (PLATFORM_UDS_RT_CORE_HEALTH_VEHICLE_BUS_OFFS_LENGTH == HEALTH_COUNTER_LEN) &&
                   (PLATFORM_UDS_RT_CORE_HEALTH_VEHICLE_RECOVERIES_LENGTH == HEALTH_COUNTER_LEN) &&
                   (PLATFORM_UDS_RT_CORE_HEALTH_VEHICLE_RECOVER_DEFERRED_LENGTH ==
                    HEALTH_COUNTER_LEN) &&
                   (PLATFORM_UDS_RT_CORE_HEALTH_VEHICLE_NAS_ABORTS_LENGTH == HEALTH_COUNTER_LEN) &&
                   (PLATFORM_UDS_RT_CORE_HEALTH_PLATFORM_BUS_OFFS_LENGTH == HEALTH_COUNTER_LEN) &&
                   (PLATFORM_UDS_RT_CORE_HEALTH_PLATFORM_RECOVERIES_LENGTH ==
                    HEALTH_COUNTER_LEN) &&
                   (PLATFORM_UDS_RT_CORE_HEALTH_PLATFORM_RECOVER_DEFERRED_LENGTH ==
                    HEALTH_COUNTER_LEN) &&
                   (PLATFORM_UDS_RT_CORE_HEALTH_PLATFORM_NAS_ABORTS_LENGTH == HEALTH_COUNTER_LEN),
               "0xFD02 counters are u16");
_Static_assert((PLATFORM_UDS_RT_CORE_HEALTH_STEP_OVERRUNS_MAX == HEALTH_COUNTER_MAX) &&
                   (PLATFORM_UDS_RT_CORE_HEALTH_VEHICLE_BUS_OFFS_MAX == HEALTH_COUNTER_MAX),
               "0xFD02 counters saturate at 0xFFFF");

/* The 0xFD02 STATE code of a port: UNKNOWN unless can_sm knows it (D-055, never the
 * last state; 0 would read as healthy). */
static uint8_t health_state(can_port_id_t port)
{
    uint8_t code = (uint8_t)PLATFORM_UDS_RT_CORE_HEALTH_VEHICLE_STATE_UNKNOWN;
    if (can_sm_state_known(port)) {
        switch (can_sm_state(port)) {
        case CAN_SM_ERROR_ACTIVE:
            code = (uint8_t)PLATFORM_UDS_RT_CORE_HEALTH_VEHICLE_STATE_ERROR_ACTIVE;
            break;
        case CAN_SM_ERROR_PASSIVE:
            code = (uint8_t)PLATFORM_UDS_RT_CORE_HEALTH_VEHICLE_STATE_ERROR_PASSIVE;
            break;
        case CAN_SM_BUS_OFF:
            code = (uint8_t)PLATFORM_UDS_RT_CORE_HEALTH_VEHICLE_STATE_BUS_OFF;
            break;
        case CAN_SM_LATCHED:
            code = (uint8_t)PLATFORM_UDS_RT_CORE_HEALTH_VEHICLE_STATE_LATCHED;
            break;
        default:
            /* stays UNKNOWN */
            break;
        }
    }
    return code;
}

static uds_server_data_t read_health(uint8_t* out, uint16_t len, uint32_t now)
{
    static const health_port_t health_ports[CAN_PORT_COUNT] = {
        {CAN_PORT_VEHICLE, PLATFORM_UDS_RT_CORE_HEALTH_VEHICLE_STATE_BYTE,
         PLATFORM_UDS_RT_CORE_HEALTH_VEHICLE_BUS_OFFS_BYTE,
         PLATFORM_UDS_RT_CORE_HEALTH_VEHICLE_RECOVERIES_BYTE,
         PLATFORM_UDS_RT_CORE_HEALTH_VEHICLE_RECOVER_DEFERRED_BYTE,
         PLATFORM_UDS_RT_CORE_HEALTH_VEHICLE_NAS_ABORTS_BYTE},
        {CAN_PORT_PLATFORM, PLATFORM_UDS_RT_CORE_HEALTH_PLATFORM_STATE_BYTE,
         PLATFORM_UDS_RT_CORE_HEALTH_PLATFORM_BUS_OFFS_BYTE,
         PLATFORM_UDS_RT_CORE_HEALTH_PLATFORM_RECOVERIES_BYTE,
         PLATFORM_UDS_RT_CORE_HEALTH_PLATFORM_RECOVER_DEFERRED_BYTE,
         PLATFORM_UDS_RT_CORE_HEALTH_PLATFORM_NAS_ABORTS_BYTE},
    };

    if (len != PLATFORM_UDS_DID_RT_CORE_HEALTH_LENGTH) {
        return UDS_SERVER_DATA_FAIL; /* gen/ layout changed: refuse rather than guess */
    }
    const diag_client_steps_t steps = diag_client_steps(now); /* last values when stale */
    uint8_t flags = 0u;
    if (steps.fresh) {
        flags |= (uint8_t)PLATFORM_UDS_RT_CORE_HEALTH_STEP_STATS_FRESH_MASK;
    }
    put_be(&out[PLATFORM_UDS_RT_CORE_HEALTH_STEP_OVERRUNS_BYTE], HEALTH_COUNTER_LEN,
           clamp(steps.overruns, HEALTH_COUNTER_MAX));
    put_be(&out[PLATFORM_UDS_RT_CORE_HEALTH_STEP_GAP_MAX_MS_BYTE], HEALTH_COUNTER_LEN,
           clamp(steps.gap_max_ms, HEALTH_COUNTER_MAX));
    for (uint32_t i = 0u; i < (uint32_t)CAN_PORT_COUNT; i++) {
        const health_port_t* p = &health_ports[i];
        const uint8_t state = health_state(p->port); /* one read: STATE and the flag agree */
        const can_sm_stats_t* st = can_sm_stats(p->port);
        const can_sm_stats_t none = {0u, 0u, 0u, 0u, 0u, 0u};
        const can_sm_stats_t* s = (st != NULL) ? st : &none;
        out[p->state_byte] = state;
        if ((p->port == CAN_PORT_VEHICLE) &&
            (state == (uint8_t)PLATFORM_UDS_RT_CORE_HEALTH_VEHICLE_STATE_LATCHED)) {
            flags |= (uint8_t)PLATFORM_UDS_RT_CORE_HEALTH_VEHICLE_LATCHED_MASK;
        }
        put_be(&out[p->bus_offs_byte], HEALTH_COUNTER_LEN,
               clamp(s->bus_off_events, HEALTH_COUNTER_MAX));
        put_be(&out[p->recoveries_byte], HEALTH_COUNTER_LEN,
               clamp(s->recover_attempts, HEALTH_COUNTER_MAX));
        put_be(&out[p->deferred_byte], HEALTH_COUNTER_LEN,
               clamp(s->recover_deferred, HEALTH_COUNTER_MAX));
        put_be(&out[p->nas_aborts_byte], HEALTH_COUNTER_LEN,
               clamp(s->tx_timeouts, HEALTH_COUNTER_MAX));
    }
    out[PLATFORM_UDS_RT_CORE_HEALTH_STEP_STATS_FRESH_BYTE] = flags;
    return UDS_SERVER_DATA_OK;
}

static uds_server_data_t read_sw_version(uint8_t* out, uint16_t len)
{
    static const char version[] = MOTO_RTCORE_VERSION;
    const uint16_t chars = (uint16_t)(sizeof(version) - 1u); /* without the NUL */
    for (uint16_t i = 0u; i < len; i++) {
        out[i] = (i < chars) ? (uint8_t)version[i] : 0u;
    }
    return UDS_SERVER_DATA_OK;
}

static uds_server_data_t read_tester_status(uint8_t* out, uint16_t len, uint32_t now)
{
    if (len != PLATFORM_UDS_DID_VEHICLE_TESTER_STATUS_LENGTH) {
        return UDS_SERVER_DATA_FAIL; /* gen/ layout changed: refuse rather than guess */
    }
    const diag_vehicle_tester_t t = diag_vehicle_tester(now); /* NOT_RUNNING when stale */
    uint8_t flags = 0u;
    if (t.ecu_present) {
        flags |= (uint8_t)PLATFORM_UDS_VEHICLE_TESTER_STATUS_ECU_PRESENT_MASK;
    }
    if (t.session_up) {
        flags |= (uint8_t)PLATFORM_UDS_VEHICLE_TESTER_STATUS_SESSION_UP_MASK;
    }
    if (t.latched) {
        flags |= (uint8_t)PLATFORM_UDS_VEHICLE_TESTER_STATUS_LATCHED_MASK;
    }
    out[PLATFORM_UDS_VEHICLE_TESTER_STATUS_ECU_PRESENT_BYTE] = flags;
    out[PLATFORM_UDS_VEHICLE_TESTER_STATUS_FAULT_BYTE] =
        (uint8_t)(t.fault & PLATFORM_UDS_VEHICLE_TESTER_STATUS_FAULT_MASK);
    return UDS_SERVER_DATA_OK;
}

static uds_server_data_t read_vehicle_sample(uint32_t vidx, uint8_t* out, uint16_t len,
                                             uint32_t now)
{
    vehicle_signal_sample_t s;
    if ((vidx >= VEHICLE_CL250_DID_COUNT) ||
        (len != (PLATFORM_UDS_VEHICLE_SAMPLE_HEADER_LEN + vehicle_cl250_dids[vidx].length)) ||
        !vehicle_signals_get(vidx, now, &s)) {
        return UDS_SERVER_DATA_FAIL;
    }
    uint8_t state = (uint8_t)PLATFORM_UDS_VEHICLE_SAMPLE_STATE_NONE;
    uint32_t age = PLATFORM_UDS_VEHICLE_SAMPLE_AGE_MAX_MS;
    if (s.state == VEHICLE_SIGNAL_VALID) {
        state = (uint8_t)PLATFORM_UDS_VEHICLE_SAMPLE_STATE_VALID;
    } else if (s.state == VEHICLE_SIGNAL_STALE) {
        state = (uint8_t)PLATFORM_UDS_VEHICLE_SAMPLE_STATE_STALE;
    } else {
        /* NONE: no sample yet, age stays saturated */
    }
    if ((s.state != VEHICLE_SIGNAL_NONE) && (s.age_ms < PLATFORM_UDS_VEHICLE_SAMPLE_AGE_MAX_MS)) {
        age = s.age_ms;
    }
    out[0] = state;
    put_be(&out[1], 2u, age);
    put_be(&out[PLATFORM_UDS_VEHICLE_SAMPLE_HEADER_LEN],
           (uint16_t)(len - PLATFORM_UDS_VEHICLE_SAMPLE_HEADER_LEN), s.raw);
    return UDS_SERVER_DATA_OK;
}

static uds_server_data_t provider_read_did(const void* ctx, uint32_t idx, uint8_t* out,
                                           uint16_t len)
{
    // cppcheck-suppress misra-c2012-11.5 ; DEV-003: ctx is the server this file registered
    const uds_server_t* server = (const uds_server_t*)ctx;
    const uint32_t now = timebase_now_ms();
    if (idx >= PLATFORM_UDS_DID_COUNT) {
        return UDS_SERVER_DATA_FAIL;
    }
    const platform_uds_did_t* d = &platform_uds_dids[idx];
    if (d->encoding == (uint8_t)PLATFORM_UDS_ENC_VEHICLE_SAMPLE) {
        return read_vehicle_sample(d->vehicle_idx, out, len, now);
    }
    uds_server_data_t st;
    switch (idx) {
    case PLATFORM_UDS_IDX_SW_VERSION:
        st = read_sw_version(out, len);
        break;
    case PLATFORM_UDS_IDX_VEHICLE_TESTER_STATUS:
        st = read_tester_status(out, len, now);
        break;
    case PLATFORM_UDS_IDX_RT_CORE_HEALTH:
        st = read_health(out, len, now);
        break;
    case PLATFORM_UDS_IDX_UPTIME:
        put_be(out, len, timebase_elapsed_ms(now, server->open_ms) / MS_PER_S);
        st = UDS_SERVER_DATA_OK;
        break;
    default:
        st = UDS_SERVER_DATA_FAIL; /* a gen/ DID without a source here */
        break;
    }
    return st;
}

static uint8_t provider_dtc_status(const void* ctx, uint32_t idx)
{
    (void)ctx;
    return diag_dtc_status(idx);
}

static uds_server_data_t provider_clear_dtcs(void* ctx)
{
    (void)ctx;
    diag_dtc_clear_all(); /* RAM only (D-040); never touches the UDS client latch */
    return UDS_SERVER_DATA_OK;
}

/* ------------------------------------------------------------------ monitors */

/* VEHICLE_BUS_OFF_LATCHED (U0001-88, D-055), every pass: failed while the vehicle port is
 * latched, passed while can_sm knows it is not, and no report while its state is unknown
 * (never a pass on an unread state, never a latch nobody saw; the last status stays). */
static void report_bus_off_latch(void)
{
    if (can_sm_state_known(CAN_PORT_VEHICLE)) {
        diag_dtc_report(PLATFORM_UDS_DTC_IDX_VEHICLE_BUS_OFF_LATCHED,
                        can_sm_state(CAN_PORT_VEHICLE) == CAN_SM_LATCHED);
    }
}

/* ------------------------------------------------------------------ functional RX */

static void on_functional(void* ctx, const can_frame_t* frame)
{
    // cppcheck-suppress misra-c2012-11.5 ; DEV-003: ctx is the server this file registered
    uds_server_t* server = (uds_server_t*)ctx;
    uint8_t n = 0u;
    if (frame->extended || !isotp_single_frame(frame->data, frame->dlc, &n) ||
        (server->func_len != 0u)) {
        server->stats.functional_dropped++; /* not a Single Frame, or one is held */
        return;
    }
    for (uint8_t i = 0u; i < n; i++) {
        server->func_req[i] = frame->data[i + 1u];
    }
    server->func_len = n;
}

/* ------------------------------------------------------------------ API */

isotp_status_t uds_server_open(uds_server_t* server)
{
    if (server == NULL) {
        return ISOTP_ERR_ARG;
    }
    if (server->open) {
        return ISOTP_ERR_BUSY;
    }
    isotp_link_addr_t addr;
    addr.port = CAN_PORT_PLATFORM;
    addr.tx_id = PLATFORM_UDS_PHYS_RESPONSE_ID;
    addr.rx_id = PLATFORM_UDS_PHYS_REQUEST_ID;
    addr.extended = false;
    isotp_config_t cfg;
    isotp_default_config(&cfg);
    cfg.padding_enabled = true;
    cfg.padding_byte = PLATFORM_UDS_PADDING_BYTE;
    cfg.block_size = PLATFORM_UDS_BLOCK_SIZE;
    cfg.st_min = PLATFORM_UDS_ST_MIN_MS; /* 0..127 is milliseconds (ISO 15765-2 9.6.5.4) */
    cfg.n_bs_ms = PLATFORM_UDS_N_BS_MS;
    cfg.n_cr_ms = PLATFORM_UDS_N_CR_MS;
    const isotp_status_t st =
        isotp_link_open(&server->link, &addr, &cfg, server->rx_buf, PLATFORM_UDS_RX_BUFFER,
                        server->tx_buf, UDS_SERVER_RSP_MAX);
    if (st != ISOTP_OK) {
        return st;
    }
    if (can_if_register_rx(CAN_PORT_PLATFORM, PLATFORM_UDS_FUNCTIONAL_REQUEST_ID, false,
                           on_functional, server) != CAN_IF_OK) {
        return ISOTP_ERR_ARG;
    }
    server->provider.read_did = provider_read_did;
    server->provider.dtc_status = provider_dtc_status;
    server->provider.clear_dtcs = provider_clear_dtcs;
    server->provider.ctx = server;
    server->open_ms = timebase_now_ms();
    if (!uds_server_core_init(&server->core, &server->provider, server->open_ms)) {
        return ISOTP_ERR_ARG; /* unreachable: the provider is complete */
    }
    const uds_server_glue_stats_t zero = {0u, 0u, 0u, 0u, 0u, 0u};
    server->stats = zero;
    server->rsp_len = 0u;
    server->rsp_since_ms = server->open_ms;
    server->func_len = 0u;
    server->open = true;
    return ISOTP_OK;
}

static bool idle(const uds_server_t* server)
{
    return (server->rsp_len == 0u) && !uds_server_core_pending(&server->core) &&
           !isotp_link_rx_busy(&server->link) && isotp_link_tx_ready(&server->link);
}

static void take_request(uds_server_t* server, uint32_t now)
{
    isotp_n_result_t res = ISOTP_N_OK;
    uint16_t len = 0u;
    if (server->rsp_len != 0u) {
        /* The answer to the last request is still queued. A physical request waits in
         * the link; a functional one is dropped, never run late (arbitration rule). */
        if (server->func_len != 0u) {
            server->stats.functional_dropped++;
            server->func_len = 0u;
        }
        return;
    }
    if (isotp_link_take_rx(&server->link, &res, &len)) {
        if (res == ISOTP_N_OK) {
            server->rsp_len =
                uds_server_core_on_request(&server->core, now, isotp_link_rx_data(&server->link),
                                           len, false, server->rsp, UDS_SERVER_RSP_MAX);
            isotp_link_rx_release(&server->link);
        } else {
            server->stats.rx_errors++;
        }
        return;
    }
    if (server->func_len == 0u) {
        return;
    }
    if (idle(server)) {
        server->stats.functional_taken++;
        server->rsp_len = uds_server_core_on_request(&server->core, now, server->func_req,
                                                     server->func_len, true, server->rsp,
                                                     UDS_SERVER_RSP_MAX);
    } else {
        server->stats.functional_dropped++;
    }
    server->func_len = 0u;
}

void uds_server_step(uds_server_t* server)
{
    if ((server == NULL) || !server->open) {
        return;
    }
    const uint32_t now = timebase_now_ms();
    diag_supervise(now);
    report_bus_off_latch();
    const bool queued = (server->rsp_len != 0u);
    take_request(server, now);
    if (server->rsp_len == 0u) {
        server->rsp_len =
            uds_server_core_poll(&server->core, now, server->rsp, UDS_SERVER_RSP_MAX);
    } else {
        uds_server_core_tick(&server->core, now); /* S3 runs while an answer waits */
    }
    if ((server->rsp_len != 0u) && !queued) {
        server->rsp_since_ms = now;
    }
    if (server->rsp_len != 0u) {
        const isotp_status_t st = isotp_link_send(&server->link, server->rsp, server->rsp_len);
        if (st == ISOTP_OK) {
            server->rsp_len = 0u;
        } else if (st != ISOTP_ERR_BUSY) {
            server->rsp_len = 0u; /* not retried: only a busy link is transient */
            server->stats.tx_failed++;
        } else if (timebase_expired(now, server->rsp_since_ms,
                                    PLATFORM_UDS_P2_STAR_SERVER_MAX_MS)) {
            server->rsp_len = 0u; /* the tester gave up long ago (dead bus) */
            server->stats.tx_expired++;
        } else {
            server->stats.tx_busy++; /* retried next pass */
        }
    }
    isotp_link_step(&server->link);
    isotp_n_result_t conf = ISOTP_N_OK;
    if (isotp_link_take_tx_confirm(&server->link, &conf) && (conf != ISOTP_N_OK)) {
        server->stats.tx_failed++;
    }
}

uint8_t uds_server_session(const uds_server_t* server)
{
    return ((server == NULL) || !server->open) ? (uint8_t)UDS_SESSION_DEFAULT
                                               : uds_server_core_session(&server->core);
}

const uds_server_stats_t* uds_server_stats(const uds_server_t* server)
{
    return (server == NULL) ? NULL : uds_server_core_stats(&server->core);
}

const uds_server_glue_stats_t* uds_server_glue_stats(const uds_server_t* server)
{
    return (server == NULL) ? NULL : &server->stats;
}
