#include "features/uds/isotp_core.h"

#include <stddef.h>

/* PCI frame types (ISO 15765-2 §9.6.1, Table 9): high nibble of the first byte. */
#define PCI_SF 0x0u
#define PCI_FF 0x1u
#define PCI_CF 0x2u
#define PCI_FC 0x3u

/* Flow status (§9.6.5.2, Table 18). */
#define FS_CTS 0x0u
#define FS_WAIT 0x1u
#define FS_OVFLW 0x2u

#define FF_DATA_BYTES 6u  /* payload bytes in a First Frame */
#define CF_DATA_BYTES 7u  /* payload bytes in a Consecutive Frame */
#define FC_LEN 3u
#define SN_MASK 0x0Fu
#define ST_MIN_RESERVED_MS 127u

static uint32_t elapsed_ms(uint32_t now_ms, uint32_t since_ms)
{
    return now_ms - since_ms; /* unsigned: wrap-safe */
}

static uint8_t next_sn(uint8_t sn)
{
    return (uint8_t)((sn + 1u) & SN_MASK);
}

static uint16_t min_u16(uint16_t a, uint16_t b)
{
    return (a < b) ? a : b;
}

static void copy_bytes(uint8_t* dst, const uint8_t* src, uint16_t len)
{
    for (uint16_t i = 0u; i < len; i++) {
        dst[i] = src[i];
    }
}

/* Sets the DLC and fills the unused bytes (padding byte, or 0 without padding). */
static void finish_frame(const isotp_link_t* link, isotp_frame_t* out, uint8_t used)
{
    uint8_t fill = link->cfg.padding_enabled ? link->cfg.padding_byte : 0u;
    for (uint8_t i = used; i < ISOTP_CAN_DL; i++) {
        out->data[i] = fill;
    }
    out->dlc = link->cfg.padding_enabled ? (uint8_t)ISOTP_CAN_DL : used;
}

/* ------------------------------------------------------------------------- */
/* Receiver                                                                   */
/* ------------------------------------------------------------------------- */

static void rx_indicate(isotp_link_t* link, isotp_n_result_t result, uint16_t len)
{
    link->rx_ind_pending = true;
    link->rx_ind_result = result;
    link->rx_ind_len = len;
}

static void rx_abort(isotp_link_t* link, isotp_n_result_t result)
{
    link->rx_state = ISOTP_RX_IDLE;
    link->fc_pending = false;
    if (link->rx_error_count < UINT16_MAX) {
        link->rx_error_count++;
    }
    rx_indicate(link, result, 0u);
}

static void queue_fc(isotp_link_t* link, uint8_t status)
{
    link->fc_pending = true;
    link->fc_status = status;
}

static void rx_single_frame(isotp_link_t* link, const isotp_frame_t* frame)
{
    uint8_t sf_dl = (uint8_t)(frame->data[0] & 0x0Fu);
    if ((sf_dl == 0u) || (sf_dl > ISOTP_SF_MAX_LEN) || (frame->dlc < (uint8_t)(sf_dl + 1u))) {
        return; /* invalid SF_DL: ignore (§9.6.2.2) */
    }
    if (link->rx_state == ISOTP_RX_COMPLETE) {
        return; /* previous message not released yet: drop */
    }
    if (link->rx_state == ISOTP_RX_WAIT_CF) {
        rx_abort(link, ISOTP_N_UNEXP_PDU); /* a new SF ends the running reception */
    }
    if (sf_dl > link->rx_cap) {
        if (link->rx_error_count < UINT16_MAX) {
            link->rx_error_count++;
        }
        return;
    }
    copy_bytes(link->rx_buf, &frame->data[1], sf_dl);
    link->rx_len = sf_dl;
    link->rx_state = ISOTP_RX_COMPLETE;
    rx_indicate(link, ISOTP_N_OK, sf_dl);
}

static void rx_first_frame(isotp_link_t* link, const isotp_frame_t* frame, uint32_t now_ms)
{
    if (frame->dlc < ISOTP_CAN_DL) {
        return; /* an FF always fills the frame */
    }
    uint16_t ff_dl = (uint16_t)(((uint16_t)(frame->data[0] & 0x0Fu) << 8) | frame->data[1]);
    if (ff_dl <= ISOTP_SF_MAX_LEN) {
        return; /* 0 = 32-bit escape (out of scope); 1..7 must be an SF: ignore */
    }
    if (link->rx_state == ISOTP_RX_COMPLETE) {
        return; /* previous message not released yet: drop, the sender times out */
    }
    if (link->rx_state == ISOTP_RX_WAIT_CF) {
        rx_abort(link, ISOTP_N_UNEXP_PDU); /* a new FF restarts the reception */
    }
    if (ff_dl > link->rx_cap) {
        if (link->rx_error_count < UINT16_MAX) {
            link->rx_error_count++;
        }
        queue_fc(link, FS_OVFLW); /* §9.6.5.2: refuse, no reception starts */
        return;
    }
    copy_bytes(link->rx_buf, &frame->data[2], FF_DATA_BYTES);
    link->rx_len = ff_dl;
    link->rx_pos = FF_DATA_BYTES;
    link->rx_next_sn = 1u;
    link->rx_block_left = link->cfg.block_size;
    link->rx_timer_ms = now_ms; /* restarted when the FC actually goes out */
    link->rx_state = ISOTP_RX_WAIT_CF;
    queue_fc(link, FS_CTS);
}

static void rx_consecutive_frame(isotp_link_t* link, const isotp_frame_t* frame, uint32_t now_ms)
{
    if (link->rx_state != ISOTP_RX_WAIT_CF) {
        return; /* unexpected CF: ignore (§9.8.3) */
    }
    uint8_t sn = (uint8_t)(frame->data[0] & SN_MASK);
    if (sn != link->rx_next_sn) {
        rx_abort(link, ISOTP_N_WRONG_SN);
        return;
    }
    uint16_t n = min_u16((uint16_t)(link->rx_len - link->rx_pos), CF_DATA_BYTES);
    if (frame->dlc < (uint16_t)(n + 1u)) {
        return; /* too short to carry the expected data: ignore */
    }
    copy_bytes(&link->rx_buf[link->rx_pos], &frame->data[1], n);
    link->rx_pos = (uint16_t)(link->rx_pos + n);
    link->rx_next_sn = next_sn(link->rx_next_sn);
    link->rx_timer_ms = now_ms;

    if (link->rx_pos >= link->rx_len) {
        link->rx_state = ISOTP_RX_COMPLETE;
        rx_indicate(link, ISOTP_N_OK, link->rx_len);
        return;
    }
    if (link->cfg.block_size != 0u) {
        link->rx_block_left--;
        if (link->rx_block_left == 0u) {
            link->rx_block_left = link->cfg.block_size;
            queue_fc(link, FS_CTS);
        }
    }
}

/* ------------------------------------------------------------------------- */
/* Sender                                                                     */
/* ------------------------------------------------------------------------- */

static void tx_finish(isotp_link_t* link, isotp_n_result_t result)
{
    link->tx_state = ISOTP_TX_IDLE;
    link->tx_conf_pending = true;
    link->tx_conf_result = result;
}

static void tx_flow_control(isotp_link_t* link, const isotp_frame_t* frame, uint32_t now_ms)
{
    if ((link->tx_state != ISOTP_TX_WAIT_FC) || (frame->dlc < FC_LEN)) {
        return; /* unexpected or short FC: ignore (§9.8.3) */
    }
    uint8_t fs = (uint8_t)(frame->data[0] & 0x0Fu);
    if (fs == FS_CTS) {
        link->tx_block_size = frame->data[1];
        link->tx_block_left = frame->data[1];
        link->tx_st_min_ms = isotp_st_min_to_ms(frame->data[2]);
        link->tx_wait_count = 0u;
        link->tx_cf_sent = false; /* the first CF of a block may go at once */
        link->tx_timer_ms = now_ms;
        link->tx_state = ISOTP_TX_SEND_CF;
    } else if (fs == FS_WAIT) {
        if (link->tx_wait_count >= link->cfg.wft_max) {
            tx_finish(link, ISOTP_N_WFT_OVRN);
        } else {
            link->tx_wait_count++;
            link->tx_timer_ms = now_ms; /* N_Bs restarts */
        }
    } else if (fs == FS_OVFLW) {
        tx_finish(link, ISOTP_N_BUFFER_OVFLW);
    } else {
        tx_finish(link, ISOTP_N_INVALID_FS);
    }
}

static void build_first(isotp_link_t* link, uint32_t now_ms, isotp_frame_t* out)
{
    if (link->tx_len <= ISOTP_SF_MAX_LEN) {
        out->data[0] = (uint8_t)((PCI_SF << 4) | link->tx_len);
        copy_bytes(&out->data[1], link->tx_buf, link->tx_len);
        finish_frame(link, out, (uint8_t)(link->tx_len + 1u));
        tx_finish(link, ISOTP_N_OK);
        return;
    }
    out->data[0] = (uint8_t)((PCI_FF << 4) | ((link->tx_len >> 8) & 0x0Fu));
    out->data[1] = (uint8_t)(link->tx_len & 0xFFu);
    copy_bytes(&out->data[2], link->tx_buf, FF_DATA_BYTES);
    finish_frame(link, out, ISOTP_CAN_DL);
    link->tx_pos = FF_DATA_BYTES;
    link->tx_next_sn = 1u;
    link->tx_wait_count = 0u;
    link->tx_timer_ms = now_ms; /* N_Bs starts */
    link->tx_state = ISOTP_TX_WAIT_FC;
}

static bool build_consecutive(isotp_link_t* link, uint32_t now_ms, isotp_frame_t* out)
{
    if (link->tx_cf_sent && (elapsed_ms(now_ms, link->tx_timer_ms) < link->tx_st_min_ms)) {
        return false; /* STmin between CFs not over yet */
    }
    uint16_t n = min_u16((uint16_t)(link->tx_len - link->tx_pos), CF_DATA_BYTES);
    out->data[0] = (uint8_t)((PCI_CF << 4) | link->tx_next_sn);
    copy_bytes(&out->data[1], &link->tx_buf[link->tx_pos], n);
    finish_frame(link, out, (uint8_t)(n + 1u));
    link->tx_pos = (uint16_t)(link->tx_pos + n);
    link->tx_next_sn = next_sn(link->tx_next_sn);
    link->tx_timer_ms = now_ms;
    link->tx_cf_sent = true;

    if (link->tx_pos >= link->tx_len) {
        tx_finish(link, ISOTP_N_OK);
    } else if (link->tx_block_size != 0u) {
        link->tx_block_left--;
        if (link->tx_block_left == 0u) {
            link->tx_state = ISOTP_TX_WAIT_FC; /* N_Bs starts now */
        }
    } else {
        /* BS = 0: the receiver wants no further FC, keep sending CFs */
    }
    return true;
}

/* ------------------------------------------------------------------------- */
/* Public API                                                                 */
/* ------------------------------------------------------------------------- */

uint32_t isotp_st_min_to_ms(uint8_t st_min)
{
    if (st_min <= 0x7Fu) {
        return st_min;
    }
    if ((st_min >= 0xF1u) && (st_min <= 0xF9u)) {
        return 1u; /* 100..900 us, rounded up to the ms timebase */
    }
    return ST_MIN_RESERVED_MS;
}

bool isotp_single_frame(const uint8_t* data, uint8_t dlc, uint8_t* len)
{
    if ((data == NULL) || (len == NULL) || (dlc < 2u) || (dlc > ISOTP_CAN_DL) ||
        ((uint8_t)(data[0] >> 4u) != PCI_SF)) {
        return false;
    }
    const uint8_t n = (uint8_t)(data[0] & 0x0Fu);
    if ((n == 0u) || (n > ISOTP_SF_MAX_LEN) || (n >= dlc)) {
        return false;
    }
    *len = n;
    return true;
}

void isotp_default_config(isotp_config_t* cfg)
{
    if (cfg == NULL) {
        return;
    }
    cfg->padding_enabled = false;
    cfg->padding_byte = 0u;
    cfg->block_size = 0u;
    cfg->st_min = 0u;
    cfg->n_bs_ms = ISOTP_DEFAULT_N_BS_MS;
    cfg->n_cr_ms = ISOTP_DEFAULT_N_CR_MS;
    cfg->wft_max = 0u;
}

isotp_status_t isotp_init(isotp_link_t* link, const isotp_config_t* cfg,
                          uint8_t* rx_buf, uint16_t rx_cap,
                          uint8_t* tx_buf, uint16_t tx_cap)
{
    if ((link == NULL) || (cfg == NULL) || (rx_buf == NULL) || (tx_buf == NULL) ||
        (rx_cap == 0u) || (tx_cap == 0u) || (cfg->n_bs_ms == 0u) || (cfg->n_cr_ms == 0u)) {
        return ISOTP_ERR_ARG;
    }
    static const isotp_link_t zero = {0};
    *link = zero;
    link->cfg = *cfg;
    link->rx_buf = rx_buf;
    link->rx_cap = min_u16(rx_cap, ISOTP_FF_MAX_LEN);
    link->tx_buf = tx_buf;
    link->tx_cap = min_u16(tx_cap, ISOTP_FF_MAX_LEN);
    return ISOTP_OK;
}

isotp_status_t isotp_send(isotp_link_t* link, const uint8_t* data, uint16_t len)
{
    if ((link == NULL) || (data == NULL)) {
        return ISOTP_ERR_ARG;
    }
    if ((len == 0u) || (len > link->tx_cap)) {
        return ISOTP_ERR_LENGTH;
    }
    if (link->tx_state != ISOTP_TX_IDLE) {
        return ISOTP_ERR_BUSY;
    }
    copy_bytes(link->tx_buf, data, len);
    link->tx_len = len;
    link->tx_pos = 0u;
    link->tx_state = ISOTP_TX_SEND_FIRST;
    return ISOTP_OK;
}

void isotp_on_frame(isotp_link_t* link, const isotp_frame_t* frame, uint32_t now_ms)
{
    if ((link == NULL) || (frame == NULL) || (frame->dlc == 0u) || (frame->dlc > ISOTP_CAN_DL)) {
        return;
    }
    switch ((uint8_t)(frame->data[0] >> 4)) {
        case PCI_SF:
            rx_single_frame(link, frame);
            break;
        case PCI_FF:
            rx_first_frame(link, frame, now_ms);
            break;
        case PCI_CF:
            rx_consecutive_frame(link, frame, now_ms);
            break;
        case PCI_FC:
            tx_flow_control(link, frame, now_ms);
            break;
        default:
            break; /* reserved PCI type: ignore */
    }
}

bool isotp_poll(isotp_link_t* link, uint32_t now_ms, isotp_frame_t* out)
{
    if ((link == NULL) || (out == NULL)) {
        return false;
    }

    /* Timers first, so a timed-out transfer never emits another frame. */
    if ((link->rx_state == ISOTP_RX_WAIT_CF) &&
        (elapsed_ms(now_ms, link->rx_timer_ms) >= link->cfg.n_cr_ms)) {
        rx_abort(link, ISOTP_N_TIMEOUT_CR);
    }
    if ((link->tx_state == ISOTP_TX_WAIT_FC) &&
        (elapsed_ms(now_ms, link->tx_timer_ms) >= link->cfg.n_bs_ms)) {
        tx_finish(link, ISOTP_N_TIMEOUT_BS);
    }

    /* Receiver's flow control has priority: the peer is waiting for it. */
    if (link->fc_pending) {
        link->fc_pending = false;
        out->data[0] = (uint8_t)((PCI_FC << 4) | link->fc_status);
        out->data[1] = link->cfg.block_size;
        out->data[2] = link->cfg.st_min;
        finish_frame(link, out, FC_LEN);
        if (link->fc_status == FS_CTS) {
            link->rx_timer_ms = now_ms; /* N_Cr runs from the FC */
        }
        return true;
    }

    if (link->tx_state == ISOTP_TX_SEND_FIRST) {
        build_first(link, now_ms, out);
        return true;
    }
    if (link->tx_state == ISOTP_TX_SEND_CF) {
        return build_consecutive(link, now_ms, out);
    }
    return false;
}

bool isotp_take_rx_indication(isotp_link_t* link, isotp_n_result_t* result, uint16_t* length)
{
    if ((link == NULL) || !link->rx_ind_pending) {
        return false;
    }
    link->rx_ind_pending = false;
    if (result != NULL) {
        *result = link->rx_ind_result;
    }
    if (length != NULL) {
        *length = link->rx_ind_len;
    }
    return true;
}

void isotp_rx_release(isotp_link_t* link)
{
    if ((link != NULL) && (link->rx_state == ISOTP_RX_COMPLETE)) {
        link->rx_state = ISOTP_RX_IDLE;
    }
}

bool isotp_take_tx_confirm(isotp_link_t* link, isotp_n_result_t* result)
{
    if ((link == NULL) || !link->tx_conf_pending) {
        return false;
    }
    link->tx_conf_pending = false;
    if (result != NULL) {
        *result = link->tx_conf_result;
    }
    return true;
}

uint16_t isotp_rx_error_count(const isotp_link_t* link)
{
    return (link == NULL) ? 0u : link->rx_error_count;
}

bool isotp_rx_busy(const isotp_link_t* link)
{
    return (link != NULL) && (link->rx_state == ISOTP_RX_WAIT_CF);
}

bool isotp_tx_idle(const isotp_link_t* link)
{
    return (link != NULL) && (link->tx_state == ISOTP_TX_IDLE);
}
