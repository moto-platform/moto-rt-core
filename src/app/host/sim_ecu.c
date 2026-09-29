#include "app/host/sim_ecu.h"

#include "features/uds/uds_iso14229.h"
#include "vehicle_cl250.h"

#include <stddef.h>

/* ISO 14229-1 codes only the ECU side needs (host-only simulator). */
#define UDS_POSITIVE_OFFSET 0x40u
#define UDS_SUPPRESS_POS_RSP 0x80u
#define UDS_NRC_SERVICE_NOT_SUPPORTED 0x11u
#define UDS_NRC_SUBFUNCTION_NOT_SUPPORTED 0x12u
#define UDS_NRC_INCORRECT_LENGTH 0x13u
#define UDS_NRC_REQUEST_OUT_OF_RANGE 0x31u
#define UDS_SESSION_DEFAULT 0x01u
/* P2server 50 ms, P2*server 5000 ms (in 10 ms units) in the session response. */
#define SIM_P2_HI 0x00u
#define SIM_P2_LO 0x32u
#define SIM_P2STAR_HI 0x01u
#define SIM_P2STAR_LO 0xF4u
#define SWEEP_PERIOD_MS 10000u
#define RX_PER_STEP 32u
#define ECU_IDS_EXTENDED (VEHICLE_CL250_REQUEST_ID > CAN_PORT_STD_ID_MAX)

_Static_assert(ISOTP_CAN_DL == CAN_PORT_MAX_DLC, "ISO-TP and CAN frame sizes differ");
_Static_assert(SIM_ECU_SEGMENTED_LEN > ISOTP_SF_MAX_LEN, "segmented answer must need an FF");
_Static_assert(SIM_ECU_SEGMENTED_LEN <= SIM_ECU_BUF, "segmented answer too long");

bool sim_ecu_init(sim_ecu_t* ecu, vbus_t* bus)
{
    if (ecu == NULL) {
        return false;
    }
    *ecu = (sim_ecu_t){0};
    if (!vbus_attach(bus, &ecu->node)) {
        return false;
    }
    ecu->bus = bus;
    isotp_config_t cfg;
    isotp_default_config(&cfg);
    cfg.padding_enabled = (VEHICLE_CL250_FRAME_DLC == ISOTP_CAN_DL);
    cfg.padding_byte = VEHICLE_CL250_PADDING_BYTE;
    return isotp_init(&ecu->iso, &cfg, ecu->rx_buf, SIM_ECU_BUF, ecu->tx_buf, SIM_ECU_BUF) ==
           ISOTP_OK;
}

void sim_ecu_drop_session(sim_ecu_t* ecu)
{
    ecu->extended = false;
}

/* Raw value for a DID at the given time: a triangle sweep over [min, max]. */
static uint32_t synthetic_raw(const vehicle_cl250_did_t* e, uint32_t now_ms)
{
    uint32_t phase_ms = now_ms % SWEEP_PERIOD_MS;
    float t = (float)phase_ms / (float)SWEEP_PERIOD_MS;
    float tri = (t < 0.5f) ? (2.0f * t) : (2.0f - (2.0f * t));
    float phys = e->min + ((e->max - e->min) * tri);
    float raw = ((phys - (float)e->offset) * (float)e->factor_den) / (float)e->factor_num;
    uint32_t raw_max = (e->length >= 4u) ? UINT32_MAX : ((1u << (8u * e->length)) - 1u);
    if (raw <= 0.0f) {
        return 0u;
    }
    if (raw >= (float)raw_max) {
        return raw_max;
    }
    return (uint32_t)raw;
}

static uint16_t negative(uint8_t* out, uint8_t sid, uint8_t nrc)
{
    out[0] = UDS_SID_NEGATIVE_RESPONSE;
    out[1] = sid;
    out[2] = nrc;
    return 3u;
}

static uint16_t read_answer(const sim_ecu_t* ecu, const vehicle_cl250_did_t* e,
                            const uint8_t* req, uint32_t now_ms, uint8_t* out)
{
    uint32_t raw = synthetic_raw(e, now_ms);
    out[0] = (uint8_t)(UDS_SID_READ_DATA_BY_IDENTIFIER + UDS_POSITIVE_OFFSET);
    out[1] = req[1];
    out[2] = req[2];
    for (uint8_t i = 0u; i < e->length; i++) {
        uint8_t shift = (uint8_t)(8u * (e->length - 1u - i));
        out[3u + i] = (uint8_t)(raw >> shift);
    }
    uint16_t n = (uint16_t)(3u + e->length);
    if (ecu->segmented_enabled && (e->did == ecu->segmented_did)) {
        for (; n < SIM_ECU_SEGMENTED_LEN; n++) {
            out[n] = 0u; /* a longer record than the table says: needs a First Frame */
        }
    }
    return n;
}

static uint16_t answer_read(sim_ecu_t* ecu, const uint8_t* req, uint16_t len, uint32_t now_ms,
                            uint8_t* out)
{
    ecu->reads++;
    if (len != 3u) {
        return negative(out, req[0], UDS_NRC_INCORRECT_LENGTH);
    }
    if (ecu->require_session && !ecu->extended) {
        return negative(out, req[0], UDS_NRC_SERVICE_NOT_SUPPORTED_IN_ACTIVE_SESSION);
    }
    uint16_t did = (uint16_t)(((uint16_t)req[1] << 8u) | req[2]);
    const vehicle_cl250_did_t* e = vehicle_cl250_find(did);
    if (e == NULL) {
        return negative(out, req[0], UDS_NRC_REQUEST_OUT_OF_RANGE);
    }
    if (ecu->nrc_enabled && (did == ecu->nrc_did)) {
        return negative(out, req[0], ecu->nrc_code);
    }
    if (ecu->pending_count > 0u) {
        ecu->deferred = true;
        for (uint16_t i = 0u; i < len; i++) {
            ecu->deferred_req[i] = req[i];
        }
        ecu->deferred_len = len;
        ecu->deferred_left = (uint8_t)(ecu->pending_count - 1u);
        ecu->deferred_next_ms = now_ms + ecu->pending_interval_ms;
        return negative(out, req[0], UDS_NRC_RESPONSE_PENDING);
    }
    return read_answer(ecu, e, req, now_ms, out);
}

/* Returns the answer length; 0 = no answer (suppressed). */
static uint16_t answer(sim_ecu_t* ecu, const uint8_t* req, uint16_t len, uint32_t now_ms,
                       uint8_t* out)
{
    const uint8_t sid = req[0];
    const uint8_t sub = (len >= 2u) ? (uint8_t)(req[1] & 0x7Fu) : 0u;
    const bool suppress = (len >= 2u) && ((req[1] & UDS_SUPPRESS_POS_RSP) != 0u);
    if (sid == UDS_SID_READ_DATA_BY_IDENTIFIER) {
        return answer_read(ecu, req, len, now_ms, out);
    }
    if (sid == VEHICLE_CL250_SESSION_SID) {
        ecu->session_requests++;
        if (len != 2u) {
            return negative(out, sid, UDS_NRC_INCORRECT_LENGTH);
        }
        if ((sub != VEHICLE_CL250_SESSION_SUBFUNCTION) && (sub != UDS_SESSION_DEFAULT)) {
            return negative(out, sid, UDS_NRC_SUBFUNCTION_NOT_SUPPORTED);
        }
        ecu->extended = (sub == VEHICLE_CL250_SESSION_SUBFUNCTION);
        if (suppress) {
            return 0u;
        }
        out[0] = (uint8_t)(sid + UDS_POSITIVE_OFFSET);
        out[1] = sub;
        out[2] = SIM_P2_HI;
        out[3] = SIM_P2_LO;
        out[4] = SIM_P2STAR_HI;
        out[5] = SIM_P2STAR_LO;
        return 6u;
    }
    if (sid == VEHICLE_CL250_TESTER_PRESENT_SID) {
        ecu->tester_presents++;
        if (len != 2u) {
            return negative(out, sid, UDS_NRC_INCORRECT_LENGTH);
        }
        if (sub != 0u) {
            return negative(out, sid, UDS_NRC_SUBFUNCTION_NOT_SUPPORTED);
        }
        if (suppress) {
            return 0u;
        }
        out[0] = (uint8_t)(sid + UDS_POSITIVE_OFFSET);
        out[1] = sub;
        return 2u;
    }
    return negative(out, sid, UDS_NRC_SERVICE_NOT_SUPPORTED);
}

/* The next step of a deferred read: another NRC 0x78, or the answer. */
static uint16_t deferred_step(sim_ecu_t* ecu, uint32_t now_ms, uint8_t* out)
{
    if (!ecu->deferred || ((int32_t)(now_ms - ecu->deferred_next_ms) < 0)) {
        return 0u;
    }
    if (ecu->deferred_left > 0u) {
        ecu->deferred_left--;
        ecu->deferred_next_ms = now_ms + ecu->pending_interval_ms;
        return negative(out, ecu->deferred_req[0], UDS_NRC_RESPONSE_PENDING);
    }
    ecu->deferred = false;
    uint16_t did = (uint16_t)(((uint16_t)ecu->deferred_req[1] << 8u) | ecu->deferred_req[2]);
    return read_answer(ecu, vehicle_cl250_find(did), ecu->deferred_req, now_ms, out);
}

void sim_ecu_step(sim_ecu_t* ecu, uint32_t now_ms)
{
    can_frame_t f;
    for (uint32_t n = 0u; n < RX_PER_STEP; n++) {
        if (vbus_recv(ecu->bus, ecu->node, &f) != CAN_PORT_OK) {
            break;
        }
        if ((f.id != VEHICLE_CL250_REQUEST_ID) || (f.extended != ECU_IDS_EXTENDED)) {
            continue;
        }
        isotp_frame_t in;
        in.dlc = f.dlc;
        for (uint8_t i = 0u; i < ISOTP_CAN_DL; i++) {
            in.data[i] = f.data[i];
        }
        isotp_on_frame(&ecu->iso, &in, now_ms);
    }

    if (ecu->extended && ((uint32_t)(now_ms - ecu->last_request_ms) >= SIM_ECU_S3_MS)) {
        ecu->extended = false; /* S3server expired */
    }

    uint8_t resp[SIM_ECU_BUF];
    uint16_t n = 0u;
    isotp_n_result_t res;
    uint16_t len;
    if (isotp_take_rx_indication(&ecu->iso, &res, &len) && (res == ISOTP_N_OK)) {
        ecu->requests++;
        if (!ecu->silent && (len > 0u)) {
            ecu->last_request_ms = now_ms;
            n = answer(ecu, ecu->rx_buf, len, now_ms, resp);
        }
        isotp_rx_release(&ecu->iso);
    } else if (!ecu->silent) {
        n = deferred_step(ecu, now_ms, resp);
    }
    if (n > 0u) {
        (void)isotp_send(&ecu->iso, resp, n);
    }
    (void)isotp_take_tx_confirm(&ecu->iso, &res);

    isotp_frame_t out;
    while (vbus_tx_free(ecu->bus, ecu->node) && isotp_poll(&ecu->iso, now_ms, &out)) {
        can_frame_t cf;
        cf.id = VEHICLE_CL250_RESPONSE_ID;
        cf.extended = ECU_IDS_EXTENDED;
        cf.dlc = out.dlc;
        for (uint8_t i = 0u; i < ISOTP_CAN_DL; i++) {
            cf.data[i] = out.data[i];
        }
        (void)vbus_send(ecu->bus, ecu->node, &cf);
    }
}
