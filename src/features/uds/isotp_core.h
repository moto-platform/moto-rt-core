#ifndef ISOTP_CORE_H
#define ISOTP_CORE_H

/*
 * ISO-TP transport core (ISO 15765-2:2016) for classic CAN, thesis deliverable Ç2.
 *
 * Scope (kept minimal on purpose, see README.md):
 *  - Classic CAN, 8-byte frames; normal and normal-fixed addressing (the CAN ID is the
 *    caller's business, this core only sees the 8 data bytes).
 *  - Single Frame, First Frame (12-bit FF_DL, max 4095 bytes), Consecutive Frame and
 *    Flow Control (CTS / WAIT / OVFLW), block size, STmin, SN wrap.
 *  - Timeouts N_Bs (sender waits for FC) and N_Cr (receiver waits for CF), FC.WAIT
 *    limit N_WFTmax. N_As/N_Ar (driver transmit confirmation) are left to the glue, which
 *    ends a message whose frames were aborted with isotp_abort_tx(N_TIMEOUT_A).
 *  - Out of scope: CAN FD frames, the 32-bit FF_DL escape, extended/mixed addressing,
 *    sending FC.WAIT as a receiver.
 *
 * Pure logic: no HAL, no RTOS, no heap, no recursion; every loop is bounded by a frame
 * length. Time is a caller-supplied millisecond counter; all comparisons are
 * wrap-safe. One isotp_link_t per (request ID, response ID) pair; a link can send and
 * receive at the same time (full duplex), but carries one message per direction.
 *
 * Usage (glue, per pass):
 *   isotp_on_frame(link, &rx_frame, now)   for every CAN frame of this link
 *   while (isotp_poll(link, now, &tx_frame)) transmit(tx_frame)
 *   isotp_take_rx_indication(...) / isotp_take_tx_confirm(...)
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ISOTP_CAN_DL 8u          /* classic CAN frame payload */
#define ISOTP_SF_MAX_LEN 7u      /* SF_DL range 1..7 */
#define ISOTP_FF_MAX_LEN 4095u   /* 12-bit FF_DL */

/* ISO 15765-2 defaults (Table 16): N_Bs and N_Cr timeout values. */
#define ISOTP_DEFAULT_N_BS_MS 1000u
#define ISOTP_DEFAULT_N_CR_MS 1000u

/* Result of an API call (not a protocol result). */
typedef enum {
    ISOTP_OK = 0,
    ISOTP_ERR_ARG,     /* NULL pointer or invalid configuration */
    ISOTP_ERR_LENGTH,  /* length 0, above 4095 or above the buffer */
    ISOTP_ERR_BUSY     /* a message is already being sent on this link */
} isotp_status_t;

/* N_Result (ISO 15765-2 §8.3.7) reported by indications and confirmations. */
typedef enum {
    ISOTP_N_OK = 0,
    ISOTP_N_TIMEOUT_BS,    /* sender: no FC within N_Bs */
    ISOTP_N_TIMEOUT_CR,    /* receiver: no CF within N_Cr */
    ISOTP_N_WRONG_SN,      /* receiver: CF with an unexpected sequence number */
    ISOTP_N_INVALID_FS,    /* sender: FC with a reserved flow status */
    ISOTP_N_UNEXP_PDU,     /* receiver: SF/FF arrived during a reception, which is dropped */
    ISOTP_N_WFT_OVRN,      /* sender: more than N_WFTmax FC.WAIT in a row */
    ISOTP_N_BUFFER_OVFLW,  /* sender: FC.OVFLW received; receiver: message too long */
    ISOTP_N_TIMEOUT_A      /* sender: a frame was not sent within N_As (glue, Ç1) */
} isotp_n_result_t;

typedef struct {
    uint8_t data[ISOTP_CAN_DL];
    uint8_t dlc; /* 0..8 */
} isotp_frame_t;

typedef struct {
    bool padding_enabled;  /* pad every frame to 8 bytes (DLC 8) */
    uint8_t padding_byte;  /* e.g. the vehicle's padding byte from gen/ */
    uint8_t block_size;    /* BS advertised in our FC (0 = no further FC) */
    uint8_t st_min;        /* raw STmin byte advertised in our FC */
    uint16_t n_bs_ms;      /* sender timeout waiting for FC (> 0) */
    uint16_t n_cr_ms;      /* receiver timeout waiting for CF (> 0) */
    uint8_t wft_max;       /* FC.WAIT frames accepted in a row before N_WFT_OVRN */
} isotp_config_t;

typedef enum {
    ISOTP_RX_IDLE = 0,
    ISOTP_RX_WAIT_CF,
    ISOTP_RX_COMPLETE      /* message held in the rx buffer until isotp_rx_release() */
} isotp_rx_state_t;

typedef enum {
    ISOTP_TX_IDLE = 0,
    ISOTP_TX_SEND_FIRST,   /* SF or FF queued */
    ISOTP_TX_WAIT_FC,
    ISOTP_TX_SEND_CF
} isotp_tx_state_t;

/* Link state. Treat as opaque: use the functions below. */
typedef struct {
    isotp_config_t cfg;

    /* receiver */
    uint8_t* rx_buf;
    uint16_t rx_cap;
    isotp_rx_state_t rx_state;
    uint16_t rx_len;        /* FF_DL / SF_DL of the message being received */
    uint16_t rx_pos;
    uint8_t rx_next_sn;
    uint8_t rx_block_left;  /* CFs until the next FC (0 = BS was 0: none) */
    uint32_t rx_timer_ms;   /* start of the running N_Cr period */
    bool fc_pending;
    uint8_t fc_status;      /* flow status of the pending FC */
    bool rx_ind_pending;
    isotp_n_result_t rx_ind_result;
    uint16_t rx_ind_len;
    uint16_t rx_error_count;

    /* sender */
    uint8_t* tx_buf;
    uint16_t tx_cap;
    isotp_tx_state_t tx_state;
    uint16_t tx_len;
    uint16_t tx_pos;
    uint8_t tx_next_sn;
    uint8_t tx_block_size;  /* BS from the last FC.CTS (0 = unlimited) */
    uint8_t tx_block_left;
    uint32_t tx_st_min_ms;  /* STmin from the last FC.CTS, rounded up to ms */
    uint32_t tx_timer_ms;   /* N_Bs start, or time of the last CF */
    bool tx_cf_sent;        /* at least one CF sent since the last FC.CTS */
    uint8_t tx_wait_count;
    bool tx_conf_pending;
    isotp_n_result_t tx_conf_result;
} isotp_link_t;

/* Fills cfg with ISO defaults: no padding, BS 0, STmin 0, N_Bs/N_Cr 1000 ms, N_WFTmax 0. */
void isotp_default_config(isotp_config_t* cfg);

/*
 * Binds caller-owned buffers (static storage) to the link. rx_buf receives messages
 * (its size is the longest message this link accepts); tx_buf holds a copy of the
 * message being sent. Both capacities are capped at 4095.
 */
isotp_status_t isotp_init(isotp_link_t* link, const isotp_config_t* cfg,
                          uint8_t* rx_buf, uint16_t rx_cap,
                          uint8_t* tx_buf, uint16_t tx_cap);

/* N_USData.request: copies the message and queues its first frame. */
isotp_status_t isotp_send(isotp_link_t* link, const uint8_t* data, uint16_t len);

/* Feeds one received CAN frame of this link (both directions share the link). */
void isotp_on_frame(isotp_link_t* link, const isotp_frame_t* frame, uint32_t now_ms);

/*
 * Runs the timers and returns the next frame to transmit, if one is due (a pending FC
 * first, then SF/FF/CF). Call it until it returns false. Frames must be sent in order.
 */
bool isotp_poll(isotp_link_t* link, uint32_t now_ms, isotp_frame_t* out);

/*
 * N_USData.indication: true once per finished reception. On ISOTP_N_OK, *length bytes
 * are in the rx buffer, and the link accepts no new message until isotp_rx_release().
 */
bool isotp_take_rx_indication(isotp_link_t* link, isotp_n_result_t* result, uint16_t* length);

/* Frees the rx buffer after an ISOTP_N_OK indication was consumed. */
void isotp_rx_release(isotp_link_t* link);

/* Ends a transmission under way (any state but idle) with a confirmation of `result`,
 * e.g. ISOTP_N_TIMEOUT_A when the driver aborted its frames. No-op while idle. */
void isotp_abort_tx(isotp_link_t* link, isotp_n_result_t result);

/* N_USData.confirm: true once per finished transmission (success or error). */
bool isotp_take_tx_confirm(isotp_link_t* link, isotp_n_result_t* result);

/* Receptions that ended in an error since init (diagnostics). */
uint16_t isotp_rx_error_count(const isotp_link_t* link);

/* True while a segmented reception is running (FF received, waiting for CFs). It ends
 * with an indication: ISOTP_N_OK, or an error such as ISOTP_N_TIMEOUT_CR. */
bool isotp_rx_busy(const isotp_link_t* link);

/* Ends a reception under way (FF received, waiting for CFs) without an indication,
 * e.g. when the link withheld our Flow Control, so no CF will come. Counted as a
 * reception error. A pending FC is dropped. No-op while no reception runs. */
void isotp_rx_cancel(isotp_link_t* link);

/* True when no message is being sent: isotp_send() would accept a new one. */
bool isotp_tx_idle(const isotp_link_t* link);

/* STmin byte to milliseconds (§9.6.5.4): 0x00-0x7F ms, 0xF1-0xF9 (100-900 us) rounded up
 * to 1 ms, reserved values treated as 0x7F. Exposed for tests. */
uint32_t isotp_st_min_to_ms(uint8_t st_min);

/* Single Frame check for a receiver outside a link (e.g. functional addressing, which
 * carries Single Frames only, §9.6.2): true if `data` (dlc bytes) is a Single Frame with
 * SF_DL 1..ISOTP_SF_MAX_LEN that fits the DLC; *len is then SF_DL and the payload
 * starts at data[1]. */
bool isotp_single_frame(const uint8_t* data, uint8_t dlc, uint8_t* len);

#ifdef __cplusplus
}
#endif

#endif /* ISOTP_CORE_H */
