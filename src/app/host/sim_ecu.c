#include "app/host/sim_ecu.h"

#include "vehicle_cl250.h"

#include <stddef.h>

#define UDS_SID_RDBI 0x22u
#define UDS_POSITIVE_OFFSET 0x40u
#define UDS_NEGATIVE_SID 0x7Fu
#define UDS_NRC_SERVICE_NOT_SUPPORTED 0x11u
#define UDS_NRC_REQUEST_OUT_OF_RANGE 0x31u
#define UDS_NRC_INCORRECT_LENGTH 0x13u
#define SWEEP_PERIOD_MS 10000u
#define RX_PER_STEP 32u
#define ECU_IDS_EXTENDED (VEHICLE_CL250_REQUEST_ID > CAN_PORT_STD_ID_MAX)

_Static_assert(ISOTP_CAN_DL == CAN_PORT_MAX_DLC, "ISO-TP and CAN frame sizes differ");

bool sim_ecu_init(sim_ecu_t* ecu, vbus_t* bus)
{
    if ((ecu == NULL) || !vbus_attach(bus, &ecu->node)) {
        return false;
    }
    ecu->bus = bus;
    ecu->requests = 0u;
    ecu->silent = false;
    isotp_config_t cfg;
    isotp_default_config(&cfg);
    cfg.padding_enabled = (VEHICLE_CL250_FRAME_DLC == ISOTP_CAN_DL);
    cfg.padding_byte = VEHICLE_CL250_PADDING_BYTE;
    return isotp_init(&ecu->iso, &cfg, ecu->rx_buf, SIM_ECU_BUF, ecu->tx_buf, SIM_ECU_BUF) ==
           ISOTP_OK;
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
    out[0] = UDS_NEGATIVE_SID;
    out[1] = sid;
    out[2] = nrc;
    return 3u;
}

static uint16_t answer(const uint8_t* req, uint16_t len, uint32_t now_ms, uint8_t* out)
{
    if (req[0] != UDS_SID_RDBI) {
        return negative(out, req[0], UDS_NRC_SERVICE_NOT_SUPPORTED);
    }
    if (len != 3u) {
        return negative(out, req[0], UDS_NRC_INCORRECT_LENGTH);
    }
    uint16_t did = (uint16_t)(((uint16_t)req[1] << 8u) | req[2]);
    const vehicle_cl250_did_t* e = vehicle_cl250_find(did);
    if (e == NULL) {
        return negative(out, req[0], UDS_NRC_REQUEST_OUT_OF_RANGE);
    }
    uint32_t raw = synthetic_raw(e, now_ms);
    out[0] = (uint8_t)(UDS_SID_RDBI + UDS_POSITIVE_OFFSET);
    out[1] = req[1];
    out[2] = req[2];
    for (uint8_t i = 0u; i < e->length; i++) {
        uint8_t shift = (uint8_t)(8u * (e->length - 1u - i));
        out[3u + i] = (uint8_t)(raw >> shift);
    }
    return (uint16_t)(3u + e->length);
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

    isotp_n_result_t res;
    uint16_t len;
    if (isotp_take_rx_indication(&ecu->iso, &res, &len) && (res == ISOTP_N_OK)) {
        ecu->requests++;
        if (!ecu->silent && (len > 0u)) {
            uint8_t resp[SIM_ECU_BUF];
            uint16_t n = answer(ecu->rx_buf, len, now_ms, resp);
            (void)isotp_send(&ecu->iso, resp, n);
        }
        isotp_rx_release(&ecu->iso);
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
