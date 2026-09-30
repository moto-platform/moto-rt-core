#include "features/uds/uds_server_core.h"

#include "uds_iso14229.h"

#include <stddef.h>

_Static_assert(PLATFORM_UDS_RX_BUFFER <= 0xFFFFu, "request length must fit uint16_t");
_Static_assert((PLATFORM_UDS_P2_STAR_SERVER_MAX_MS / UDS_P2_STAR_RESOLUTION_MS) <= 0xFFFFu,
               "P2* must fit the 0x50 response");
_Static_assert(PLATFORM_UDS_P2_SERVER_MAX_MS <= 0xFFFFu, "P2 must fit the 0x50 response");

#define POSITIVE(sid) ((uint8_t)((sid) + UDS_POSITIVE_RESPONSE_OFFSET))
#define SPR(sub) (((sub) & UDS_SUPPRESS_POS_RSP_BIT) != 0u)
#define SUB(sub) ((uint8_t)((sub) & (uint8_t)~UDS_SUPPRESS_POS_RSP_BIT))
#define RP_REPEAT_MS (PLATFORM_UDS_P2_STAR_SERVER_MAX_MS / 2u)

/* The answer of one handler before the suppression rules. */
typedef struct {
    uint16_t len;
    bool spr;     /* suppressPosRspMsgIndicationBit was set */
    bool pending; /* the provider is not ready: retry later */
} result_t;

static uint32_t elapsed(uint32_t now, uint32_t since)
{
    return now - since; /* modulo 2^32, correct across the counter wrap */
}

static result_t negative(uint8_t* rsp, uint8_t sid, uint8_t nrc)
{
    result_t r = {UDS_NEGATIVE_RESPONSE_LEN, false, false};
    rsp[0] = UDS_SID_NEGATIVE_RESPONSE;
    rsp[1] = sid;
    rsp[2] = nrc;
    return r;
}

static result_t positive(uint16_t len, bool spr)
{
    result_t r = {len, spr, false};
    return r;
}

static result_t pending(void)
{
    result_t r = {0u, false, true};
    return r;
}

/* ------------------------------------------------------------------ 0x10, 0x3E */

static result_t session_control(uds_server_core_t* core, const uint8_t* req, uint16_t len,
                                uint8_t* rsp)
{
    const uint8_t sid = UDS_SID_DIAGNOSTIC_SESSION_CONTROL;
    if (len < 2u) {
        return negative(rsp, sid, UDS_NRC_INCORRECT_MESSAGE_LENGTH_OR_INVALID_FORMAT);
    }
    const uint8_t sub = SUB(req[1]);
    if (!platform_uds_session_supported(sub)) {
        return negative(rsp, sid, UDS_NRC_SUBFUNCTION_NOT_SUPPORTED);
    }
    if (len != 2u) {
        return negative(rsp, sid, UDS_NRC_INCORRECT_MESSAGE_LENGTH_OR_INVALID_FORMAT);
    }
    const uint16_t p2 = (uint16_t)PLATFORM_UDS_P2_SERVER_MAX_MS;
    const uint16_t p2s =
        (uint16_t)(PLATFORM_UDS_P2_STAR_SERVER_MAX_MS / UDS_P2_STAR_RESOLUTION_MS);
    core->session = sub;
    rsp[0] = POSITIVE(sid);
    rsp[1] = sub;
    rsp[2] = (uint8_t)(p2 >> 8u);
    rsp[3] = (uint8_t)(p2 & 0xFFu);
    rsp[4] = (uint8_t)(p2s >> 8u);
    rsp[5] = (uint8_t)(p2s & 0xFFu);
    return positive(6u, SPR(req[1]));
}

static result_t tester_present(const uint8_t* req, uint16_t len, uint8_t* rsp)
{
    const uint8_t sid = UDS_SID_TESTER_PRESENT;
    if (len < 2u) {
        return negative(rsp, sid, UDS_NRC_INCORRECT_MESSAGE_LENGTH_OR_INVALID_FORMAT);
    }
    if (!platform_uds_subfunction_supported(sid, req[1])) {
        return negative(rsp, sid, UDS_NRC_SUBFUNCTION_NOT_SUPPORTED);
    }
    if (len != 2u) {
        return negative(rsp, sid, UDS_NRC_INCORRECT_MESSAGE_LENGTH_OR_INVALID_FORMAT);
    }
    rsp[0] = POSITIVE(sid);
    rsp[1] = SUB(req[1]);
    return positive(2u, SPR(req[1]));
}

/* ------------------------------------------------------------------ 0x22 */

static bool find_did(uint16_t did, uint32_t* idx)
{
    for (uint32_t i = 0u; i < PLATFORM_UDS_DID_COUNT; i++) {
        if (platform_uds_dids[i].did == did) {
            *idx = i;
            return true;
        }
    }
    return false;
}

static result_t read_did(const uds_server_core_t* core, const uint8_t* req, uint16_t len,
                         uint8_t* rsp, uint16_t cap)
{
    const uint8_t sid = UDS_SID_READ_DATA_BY_IDENTIFIER;
    if ((len < 3u) || (((uint16_t)(len - 1u) % 2u) != 0u) ||
        (((uint16_t)(len - 1u) / 2u) > PLATFORM_UDS_MAX_READ_DIDS)) {
        return negative(rsp, sid, UDS_NRC_INCORRECT_MESSAGE_LENGTH_OR_INVALID_FORMAT);
    }
    uint16_t pos = 1u;
    uint32_t found = 0u;
    rsp[0] = POSITIVE(sid);
    for (uint16_t i = 1u; (i + 1u) < len; i += 2u) {
        const uint16_t did = (uint16_t)(((uint16_t)req[i] << 8u) | req[i + 1u]);
        uint32_t idx = 0u;
        if (!find_did(did, &idx)) {
            continue; /* unsupported DIDs are left out; 0x31 only if none is supported */
        }
        const uint16_t dlen = platform_uds_dids[idx].length;
        if (((uint32_t)pos + 2u + dlen) > cap) {
            return negative(rsp, sid, UDS_NRC_RESPONSE_TOO_LONG);
        }
        rsp[pos] = req[i];
        rsp[pos + 1u] = req[i + 1u];
        if (idx == (uint32_t)PLATFORM_UDS_IDX_ACTIVE_DIAGNOSTIC_SESSION) {
            if (dlen != 1u) {
                return negative(rsp, sid, UDS_NRC_CONDITIONS_NOT_CORRECT); /* gen/ layout */
            }
            rsp[pos + 2u] = core->session;
        } else {
            const uds_server_data_t st =
                core->provider->read_did(core->provider->ctx, idx, &rsp[pos + 2u], dlen);
            if (st == UDS_SERVER_DATA_PENDING) {
                return pending();
            }
            if (st != UDS_SERVER_DATA_OK) {
                return negative(rsp, sid, UDS_NRC_CONDITIONS_NOT_CORRECT);
            }
        }
        pos = (uint16_t)(pos + 2u + dlen);
        found++;
    }
    if (found == 0u) {
        return negative(rsp, sid, UDS_NRC_REQUEST_OUT_OF_RANGE);
    }
    return positive(pos, false);
}

/* ------------------------------------------------------------------ 0x19, 0x14 */

static result_t read_dtc(const uds_server_core_t* core, const uint8_t* req, uint16_t len,
                         uint8_t* rsp, uint16_t cap)
{
    const uint8_t sid = UDS_SID_READ_DTC_INFORMATION;
    const uint8_t avail = (uint8_t)PLATFORM_UDS_DTC_STATUS_AVAILABILITY_MASK;
    if (len < 2u) {
        return negative(rsp, sid, UDS_NRC_INCORRECT_MESSAGE_LENGTH_OR_INVALID_FORMAT);
    }
    /* 0x19 has no suppressPosRsp bit: 0x81 and up are unknown sub-functions. */
    if (SPR(req[1]) || !platform_uds_subfunction_supported(sid, req[1])) {
        return negative(rsp, sid, UDS_NRC_SUBFUNCTION_NOT_SUPPORTED);
    }
    const uint8_t sub = req[1];
    const bool supported = (sub == UDS_READ_DTC_REPORT_SUPPORTED_DTC);
    if (len != (supported ? 2u : 3u)) {
        return negative(rsp, sid, UDS_NRC_INCORRECT_MESSAGE_LENGTH_OR_INVALID_FORMAT);
    }
    const uint8_t mask = supported ? avail : req[2];
    uint16_t count = 0u;
    uint16_t pos = 3u;
    rsp[0] = POSITIVE(sid);
    rsp[1] = sub;
    rsp[2] = avail;
    for (uint32_t i = 0u; i < PLATFORM_UDS_DTC_COUNT; i++) {
        const uint8_t st = (uint8_t)(core->provider->dtc_status(core->provider->ctx, i) & avail);
        if (!supported && ((st & mask) == 0u)) {
            continue;
        }
        count++;
        if (sub != UDS_READ_DTC_REPORT_NUMBER_OF_DTC_BY_STATUS_MASK) {
            if (((uint32_t)pos + 4u) > cap) {
                return negative(rsp, sid, UDS_NRC_RESPONSE_TOO_LONG);
            }
            const uint32_t dtc = platform_uds_dtcs[i];
            rsp[pos] = (uint8_t)((dtc >> 16u) & 0xFFu);
            rsp[pos + 1u] = (uint8_t)((dtc >> 8u) & 0xFFu);
            rsp[pos + 2u] = (uint8_t)(dtc & 0xFFu);
            rsp[pos + 3u] = st;
            pos = (uint16_t)(pos + 4u);
        }
    }
    if (sub == UDS_READ_DTC_REPORT_NUMBER_OF_DTC_BY_STATUS_MASK) {
        rsp[3] = UDS_DTC_FORMAT_ISO_14229_1;
        rsp[4] = (uint8_t)(count >> 8u);
        rsp[5] = (uint8_t)(count & 0xFFu);
        pos = 6u;
    }
    return positive(pos, false);
}

static result_t clear_dtc(const uds_server_core_t* core, const uint8_t* req, uint16_t len,
                          uint8_t* rsp)
{
    const uint8_t sid = UDS_SID_CLEAR_DIAGNOSTIC_INFORMATION;
    if (len != 4u) {
        return negative(rsp, sid, UDS_NRC_INCORRECT_MESSAGE_LENGTH_OR_INVALID_FORMAT);
    }
    const uint32_t group = ((uint32_t)req[1] << 16u) | ((uint32_t)req[2] << 8u) | req[3];
    if (group != UDS_GROUP_OF_DTC_ALL) {
        return negative(rsp, sid, UDS_NRC_REQUEST_OUT_OF_RANGE);
    }
    const uds_server_data_t st = core->provider->clear_dtcs(core->provider->ctx);
    if (st == UDS_SERVER_DATA_PENDING) {
        return pending();
    }
    if (st != UDS_SERVER_DATA_OK) {
        return negative(rsp, sid, UDS_NRC_CONDITIONS_NOT_CORRECT);
    }
    rsp[0] = POSITIVE(sid);
    return positive(1u, false);
}

/* ------------------------------------------------------------------ dispatch */

static result_t process(uds_server_core_t* core, const uint8_t* req, uint16_t len,
                        uint8_t* rsp, uint16_t cap)
{
    const uint8_t sid = req[0];
    if (!platform_uds_service_supported(sid)) {
        return negative(rsp, sid, UDS_NRC_SERVICE_NOT_SUPPORTED);
    }
    if (!platform_uds_service_allowed_in_session(sid, core->session)) {
        return negative(rsp, sid, UDS_NRC_SERVICE_NOT_SUPPORTED_IN_ACTIVE_SESSION);
    }
    result_t r;
    switch (sid) {
    case UDS_SID_DIAGNOSTIC_SESSION_CONTROL:
        r = session_control(core, req, len, rsp);
        break;
    case UDS_SID_TESTER_PRESENT:
        r = tester_present(req, len, rsp);
        break;
    case UDS_SID_READ_DATA_BY_IDENTIFIER:
        r = read_did(core, req, len, rsp, cap);
        break;
    case UDS_SID_READ_DTC_INFORMATION:
        r = read_dtc(core, req, len, rsp, cap);
        break;
    case UDS_SID_CLEAR_DIAGNOSTIC_INFORMATION:
        r = clear_dtc(core, req, len, rsp);
        break;
    default:
        /* offered by gen/ but not implemented here: never answer positively */
        r = negative(rsp, sid, UDS_NRC_SERVICE_NOT_SUPPORTED);
        break;
    }
    return r;
}

/* Applies suppressPosRsp and the functional NRC rule, except after an NRC 0x78 (the
 * tester then waits for the final answer); counts the outcome. */
static uint16_t finish(uds_server_core_t* core, result_t r, bool functional,
                       const uint8_t* rsp)
{
    const bool must_answer = core->rp_sent;
    core->rp_sent = false;
    if (r.len == 0u) {
        return 0u;
    }
    if (rsp[0] == UDS_SID_NEGATIVE_RESPONSE) {
        if (!must_answer && functional && platform_uds_nrc_suppressed_functional(rsp[2])) {
            core->stats.suppressed++;
            return 0u;
        }
        core->stats.negative++;
        return r.len;
    }
    if (r.spr && !must_answer) {
        core->stats.suppressed++;
        return 0u;
    }
    core->stats.positive++;
    return r.len;
}

static uint16_t response_pending(uds_server_core_t* core, uint32_t now, uint8_t* rsp)
{
    const result_t r = negative(rsp, core->req[0], UDS_NRC_RESPONSE_PENDING);
    core->last_rp_ms = now;
    core->rp_sent = true;
    core->stats.response_pending++;
    return r.len;
}

bool uds_server_core_init(uds_server_core_t* core, const uds_server_provider_t* provider,
                          uint32_t now_ms)
{
    if ((core == NULL) || (provider == NULL) || (provider->read_did == NULL) ||
        (provider->dtc_status == NULL) || (provider->clear_dtcs == NULL)) {
        return false;
    }
    const uds_server_stats_t zero = {0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u};
    core->provider = provider;
    core->session = UDS_SESSION_DEFAULT;
    core->s3_start_ms = now_ms;
    core->pending = false;
    core->pending_functional = false;
    core->rp_sent = false;
    core->pending_since_ms = now_ms;
    core->last_rp_ms = now_ms;
    core->req_len = 0u;
    core->stats = zero;
    return true;
}

static bool usable(const uds_server_core_t* core, const uint8_t* rsp, uint16_t cap)
{
    return (core != NULL) && (core->provider != NULL) && (rsp != NULL) &&
           (cap >= UDS_SERVER_RSP_MAX);
}

uint16_t uds_server_core_on_request(uds_server_core_t* core, uint32_t now_ms,
                                    const uint8_t* req, uint16_t len, bool functional,
                                    uint8_t* rsp, uint16_t cap)
{
    if (!usable(core, rsp, cap) || (req == NULL) || (len == 0u) ||
        (len > PLATFORM_UDS_RX_BUFFER)) {
        return 0u;
    }
    core->stats.requests++;
    core->s3_start_ms = now_ms;
    if (core->pending) {
        core->stats.busy++;
        if (functional || ((req[0] == UDS_SID_TESTER_PRESENT) && (len == 2u) && SPR(req[1]))) {
            return 0u; /* dropped; a suppressed tester present only keeps S3 alive */
        }
        return negative(rsp, req[0], UDS_NRC_BUSY_REPEAT_REQUEST).len;
    }
    const result_t r = process(core, req, len, rsp, cap);
    if (!r.pending) {
        return finish(core, r, functional, rsp);
    }
    for (uint16_t i = 0u; i < len; i++) {
        core->req[i] = req[i];
    }
    core->req_len = len;
    core->pending = true;
    core->pending_functional = functional;
    core->pending_since_ms = now_ms;
    return response_pending(core, now_ms, rsp);
}

uint16_t uds_server_core_poll(uds_server_core_t* core, uint32_t now_ms, uint8_t* rsp,
                              uint16_t cap)
{
    if (!usable(core, rsp, cap)) {
        return 0u;
    }
    if (!core->pending) {
        uds_server_core_tick(core, now_ms);
        return 0u;
    }
    core->s3_start_ms = now_ms; /* a pending request holds the session */
    const result_t r = process(core, core->req, core->req_len, rsp, cap);
    if (!r.pending) {
        core->pending = false;
        return finish(core, r, core->pending_functional, rsp);
    }
    if (elapsed(now_ms, core->pending_since_ms) >= PLATFORM_UDS_P2_STAR_SERVER_MAX_MS) {
        core->pending = false;
        core->stats.pending_expired++;
        const result_t gr = negative(rsp, core->req[0], UDS_NRC_GENERAL_REJECT);
        return finish(core, gr, core->pending_functional, rsp);
    }
    if (elapsed(now_ms, core->last_rp_ms) >= RP_REPEAT_MS) {
        return response_pending(core, now_ms, rsp);
    }
    return 0u;
}

void uds_server_core_tick(uds_server_core_t* core, uint32_t now_ms)
{
    if ((core == NULL) || core->pending) {
        return; /* a pending request holds the session */
    }
    if ((core->session != UDS_SESSION_DEFAULT) &&
        (elapsed(now_ms, core->s3_start_ms) >= PLATFORM_UDS_S3_SERVER_MS)) {
        core->session = UDS_SESSION_DEFAULT;
        core->stats.s3_timeouts++;
    }
}

uint8_t uds_server_core_session(const uds_server_core_t* core)
{
    return (core == NULL) ? (uint8_t)UDS_SESSION_DEFAULT : core->session;
}

bool uds_server_core_pending(const uds_server_core_t* core)
{
    return (core != NULL) && core->pending;
}

const uds_server_stats_t* uds_server_core_stats(const uds_server_core_t* core)
{
    return (core == NULL) ? NULL : &core->stats;
}
