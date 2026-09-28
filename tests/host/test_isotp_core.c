/*
 * L0 tests for the ISO-TP core (ISO 15765-2, thesis deliverable Ç2).
 * Section numbers refer to ISO 15765-2:2016. No requirement IDs exist yet (Q-006);
 * test names describe the checked behaviour.
 */
#include "features/uds/isotp_core.h"

#include <string.h>
#include <unity.h>

#define BUF_CAP 4095u

static uint8_t rx_buf_a[BUF_CAP];
static uint8_t tx_buf_a[BUF_CAP];
static uint8_t rx_buf_b[BUF_CAP];
static uint8_t tx_buf_b[BUF_CAP];
static isotp_link_t a; /* the side under test */
static isotp_link_t b; /* its peer in loopback tests */
static isotp_config_t cfg;

void setUp(void)
{
    isotp_default_config(&cfg);
    TEST_ASSERT_EQUAL(ISOTP_OK, isotp_init(&a, &cfg, rx_buf_a, BUF_CAP, tx_buf_a, BUF_CAP));
}

void tearDown(void) {}

static isotp_frame_t frame_of(uint8_t dlc, const uint8_t* bytes)
{
    isotp_frame_t f;
    memset(&f, 0, sizeof f);
    f.dlc = dlc;
    memcpy(f.data, bytes, dlc);
    return f;
}

static isotp_frame_t fc(uint8_t fs, uint8_t bs, uint8_t st_min)
{
    const uint8_t d[3] = {(uint8_t)(0x30u | fs), bs, st_min};
    return frame_of(3u, d);
}

static void fill_pattern(uint8_t* p, uint16_t len, uint8_t seed)
{
    for (uint16_t i = 0u; i < len; i++) {
        p[i] = (uint8_t)(seed + i * 7u);
    }
}

/* ------------------------------------------------------------------------- */
/* Configuration and API guards                                              */
/* ------------------------------------------------------------------------- */

static void test_init_rejects_invalid_arguments(void)
{
    isotp_link_t l;
    TEST_ASSERT_EQUAL(ISOTP_ERR_ARG, isotp_init(NULL, &cfg, rx_buf_a, 8u, tx_buf_a, 8u));
    TEST_ASSERT_EQUAL(ISOTP_ERR_ARG, isotp_init(&l, NULL, rx_buf_a, 8u, tx_buf_a, 8u));
    TEST_ASSERT_EQUAL(ISOTP_ERR_ARG, isotp_init(&l, &cfg, NULL, 8u, tx_buf_a, 8u));
    TEST_ASSERT_EQUAL(ISOTP_ERR_ARG, isotp_init(&l, &cfg, rx_buf_a, 0u, tx_buf_a, 8u));
    TEST_ASSERT_EQUAL(ISOTP_ERR_ARG, isotp_init(&l, &cfg, rx_buf_a, 8u, tx_buf_a, 0u));
    isotp_config_t bad = cfg;
    bad.n_bs_ms = 0u;
    TEST_ASSERT_EQUAL(ISOTP_ERR_ARG, isotp_init(&l, &bad, rx_buf_a, 8u, tx_buf_a, 8u));
    bad = cfg;
    bad.n_cr_ms = 0u;
    TEST_ASSERT_EQUAL(ISOTP_ERR_ARG, isotp_init(&l, &bad, rx_buf_a, 8u, tx_buf_a, 8u));
}

static void test_send_rejects_bad_length_and_busy_link(void)
{
    uint8_t msg[20] = {0};
    TEST_ASSERT_EQUAL(ISOTP_ERR_LENGTH, isotp_send(&a, msg, 0u));
    TEST_ASSERT_EQUAL(ISOTP_ERR_ARG, isotp_send(&a, NULL, 3u));

    isotp_link_t small;
    TEST_ASSERT_EQUAL(ISOTP_OK, isotp_init(&small, &cfg, rx_buf_b, 8u, tx_buf_b, 16u));
    TEST_ASSERT_EQUAL(ISOTP_ERR_LENGTH, isotp_send(&small, msg, 17u));

    TEST_ASSERT_EQUAL(ISOTP_OK, isotp_send(&a, msg, 20u));
    TEST_ASSERT_EQUAL(ISOTP_ERR_BUSY, isotp_send(&a, msg, 3u));
}

static void test_capacity_is_capped_at_4095(void)
{
    static uint8_t big[5000];
    isotp_link_t l;
    TEST_ASSERT_EQUAL(ISOTP_OK, isotp_init(&l, &cfg, big, 5000u, big, 5000u));
    TEST_ASSERT_EQUAL(ISOTP_ERR_LENGTH, isotp_send(&l, big, 4096u));
    TEST_ASSERT_EQUAL(ISOTP_OK, isotp_send(&l, big, 4095u));
}

static void test_st_min_decoding(void)
{
    TEST_ASSERT_EQUAL_UINT32(0u, isotp_st_min_to_ms(0x00u));
    TEST_ASSERT_EQUAL_UINT32(10u, isotp_st_min_to_ms(0x0Au));
    TEST_ASSERT_EQUAL_UINT32(127u, isotp_st_min_to_ms(0x7Fu));
    TEST_ASSERT_EQUAL_UINT32(1u, isotp_st_min_to_ms(0xF1u)); /* 100 us, rounded up */
    TEST_ASSERT_EQUAL_UINT32(1u, isotp_st_min_to_ms(0xF9u)); /* 900 us */
    TEST_ASSERT_EQUAL_UINT32(127u, isotp_st_min_to_ms(0x80u)); /* reserved */
    TEST_ASSERT_EQUAL_UINT32(127u, isotp_st_min_to_ms(0xF0u)); /* reserved */
    TEST_ASSERT_EQUAL_UINT32(127u, isotp_st_min_to_ms(0xFAu)); /* reserved */
    TEST_ASSERT_EQUAL_UINT32(127u, isotp_st_min_to_ms(0xFFu)); /* reserved */
}

/* ------------------------------------------------------------------------- */
/* Sender                                                                     */
/* ------------------------------------------------------------------------- */

static void test_single_frame_send_without_padding(void)
{
    const uint8_t msg[3] = {0x22u, 0xF4u, 0x0Cu};
    isotp_frame_t out;
    TEST_ASSERT_EQUAL(ISOTP_OK, isotp_send(&a, msg, 3u));
    TEST_ASSERT_TRUE(isotp_poll(&a, 0u, &out));
    TEST_ASSERT_EQUAL_UINT8(4u, out.dlc);
    const uint8_t expected[4] = {0x03u, 0x22u, 0xF4u, 0x0Cu};
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, out.data, 4u);
    TEST_ASSERT_FALSE(isotp_poll(&a, 0u, &out));

    isotp_n_result_t r;
    TEST_ASSERT_TRUE(isotp_take_tx_confirm(&a, &r));
    TEST_ASSERT_EQUAL(ISOTP_N_OK, r);
    TEST_ASSERT_FALSE(isotp_take_tx_confirm(&a, &r));
}

static void test_single_frame_send_with_padding(void)
{
    cfg.padding_enabled = true;
    cfg.padding_byte = 0xAAu;
    TEST_ASSERT_EQUAL(ISOTP_OK, isotp_init(&a, &cfg, rx_buf_a, BUF_CAP, tx_buf_a, BUF_CAP));
    const uint8_t msg[2] = {0x10u, 0x03u};
    isotp_frame_t out;
    isotp_send(&a, msg, 2u);
    TEST_ASSERT_TRUE(isotp_poll(&a, 0u, &out));
    const uint8_t expected[8] = {0x02u, 0x10u, 0x03u, 0xAAu, 0xAAu, 0xAAu, 0xAAu, 0xAAu};
    TEST_ASSERT_EQUAL_UINT8(8u, out.dlc);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, out.data, 8u);
}

static void test_seven_bytes_is_still_a_single_frame_eight_is_not(void)
{
    uint8_t msg[8] = {0};
    fill_pattern(msg, 8u, 1u);
    isotp_frame_t out;
    isotp_send(&a, msg, 7u);
    TEST_ASSERT_TRUE(isotp_poll(&a, 0u, &out));
    TEST_ASSERT_EQUAL_HEX8(0x07u, out.data[0]);

    isotp_send(&a, msg, 8u);
    TEST_ASSERT_TRUE(isotp_poll(&a, 0u, &out));
    TEST_ASSERT_EQUAL_HEX8(0x10u, out.data[0]); /* FF, FF_DL = 8 */
    TEST_ASSERT_EQUAL_HEX8(0x08u, out.data[1]);
}

static void test_multi_frame_send_segments_and_wraps_sn(void)
{
    enum { LEN = 200 };
    uint8_t msg[LEN] = {0};
    fill_pattern(msg, LEN, 3u);
    isotp_frame_t out;
    TEST_ASSERT_EQUAL(ISOTP_OK, isotp_send(&a, msg, LEN));

    TEST_ASSERT_TRUE(isotp_poll(&a, 0u, &out));
    TEST_ASSERT_EQUAL_HEX8(0x10u, out.data[0]);
    TEST_ASSERT_EQUAL_HEX8(LEN, out.data[1]);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(msg, &out.data[2], 6u);
    TEST_ASSERT_FALSE(isotp_poll(&a, 1u, &out)); /* waits for FC */

    isotp_frame_t cts = fc(0u, 0u, 0u);
    isotp_on_frame(&a, &cts, 2u);

    uint8_t rebuilt[LEN];
    memcpy(rebuilt, msg, 6u);
    uint16_t pos = 6u;
    uint8_t sn = 1u;
    int cfs = 0;
    while (isotp_poll(&a, 2u, &out)) {
        TEST_ASSERT_EQUAL_HEX8(0x20u | sn, out.data[0]);
        uint16_t n = (uint16_t)((LEN - pos) < 7 ? (LEN - pos) : 7);
        memcpy(&rebuilt[pos], &out.data[1], n);
        pos = (uint16_t)(pos + n);
        sn = (uint8_t)((sn + 1u) & 0x0Fu); /* 15 wraps to 0 */
        cfs++;
    }
    TEST_ASSERT_EQUAL_INT(28, cfs); /* (200 - 6) / 7 rounded up */
    TEST_ASSERT_EQUAL_UINT16(LEN, pos);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(msg, rebuilt, LEN);

    isotp_n_result_t r;
    TEST_ASSERT_TRUE(isotp_take_tx_confirm(&a, &r));
    TEST_ASSERT_EQUAL(ISOTP_N_OK, r);
}

static void test_last_cf_without_padding_is_short(void)
{
    uint8_t msg[10] = {0};
    fill_pattern(msg, 10u, 9u);
    isotp_frame_t out;
    isotp_send(&a, msg, 10u);
    isotp_poll(&a, 0u, &out);
    isotp_frame_t cts = fc(0u, 0u, 0u);
    isotp_on_frame(&a, &cts, 0u);
    TEST_ASSERT_TRUE(isotp_poll(&a, 0u, &out));
    TEST_ASSERT_EQUAL_UINT8(5u, out.dlc); /* PCI + remaining 4 bytes */
}

static void test_block_size_waits_for_next_flow_control(void)
{
    uint8_t msg[60] = {0};
    fill_pattern(msg, 60u, 5u);
    isotp_frame_t out;
    isotp_send(&a, msg, 60u);
    isotp_poll(&a, 0u, &out);
    isotp_frame_t cts = fc(0u, 2u, 0u);
    isotp_on_frame(&a, &cts, 0u);

    TEST_ASSERT_TRUE(isotp_poll(&a, 0u, &out));
    TEST_ASSERT_TRUE(isotp_poll(&a, 0u, &out));
    TEST_ASSERT_FALSE(isotp_poll(&a, 5u, &out)); /* block of 2 done: wait for FC */

    isotp_on_frame(&a, &cts, 10u);
    TEST_ASSERT_TRUE(isotp_poll(&a, 10u, &out));
    TEST_ASSERT_EQUAL_HEX8(0x23u, out.data[0]); /* SN continues across blocks */
}

static void test_st_min_spaces_consecutive_frames(void)
{
    uint8_t msg[30] = {0};
    fill_pattern(msg, 30u, 2u);
    isotp_frame_t out;
    isotp_send(&a, msg, 30u);
    isotp_poll(&a, 0u, &out);
    isotp_frame_t cts = fc(0u, 0u, 10u);
    isotp_on_frame(&a, &cts, 100u);

    TEST_ASSERT_TRUE(isotp_poll(&a, 100u, &out));  /* first CF right after the FC */
    TEST_ASSERT_FALSE(isotp_poll(&a, 105u, &out));
    TEST_ASSERT_FALSE(isotp_poll(&a, 109u, &out));
    TEST_ASSERT_TRUE(isotp_poll(&a, 110u, &out));
    TEST_ASSERT_FALSE(isotp_poll(&a, 110u, &out));
}

static void test_reserved_st_min_is_treated_as_127_ms(void)
{
    uint8_t msg[30] = {0};
    isotp_frame_t out;
    isotp_send(&a, msg, 30u);
    isotp_poll(&a, 0u, &out);
    isotp_frame_t cts = fc(0u, 0u, 0x80u);
    isotp_on_frame(&a, &cts, 0u);
    TEST_ASSERT_TRUE(isotp_poll(&a, 0u, &out));
    TEST_ASSERT_FALSE(isotp_poll(&a, 126u, &out));
    TEST_ASSERT_TRUE(isotp_poll(&a, 127u, &out));
}

static void test_missing_flow_control_times_out_n_bs(void)
{
    uint8_t msg[20] = {0};
    isotp_frame_t out;
    isotp_send(&a, msg, 20u);
    isotp_poll(&a, 1000u, &out);
    TEST_ASSERT_FALSE(isotp_poll(&a, 1000u + ISOTP_DEFAULT_N_BS_MS - 1u, &out));
    isotp_n_result_t r;
    TEST_ASSERT_FALSE(isotp_take_tx_confirm(&a, &r));

    TEST_ASSERT_FALSE(isotp_poll(&a, 1000u + ISOTP_DEFAULT_N_BS_MS, &out));
    TEST_ASSERT_TRUE(isotp_take_tx_confirm(&a, &r));
    TEST_ASSERT_EQUAL(ISOTP_N_TIMEOUT_BS, r);

    /* A late FC is ignored and nothing more is sent. */
    isotp_frame_t cts = fc(0u, 0u, 0u);
    isotp_on_frame(&a, &cts, 3000u);
    TEST_ASSERT_FALSE(isotp_poll(&a, 3000u, &out));
    TEST_ASSERT_EQUAL(ISOTP_OK, isotp_send(&a, msg, 3u)); /* link free again */
}

static void test_n_bs_also_guards_the_fc_after_a_block(void)
{
    uint8_t msg[60] = {0};
    isotp_frame_t out;
    isotp_send(&a, msg, 60u);
    isotp_poll(&a, 0u, &out);
    isotp_frame_t cts = fc(0u, 1u, 0u);
    isotp_on_frame(&a, &cts, 0u);
    TEST_ASSERT_TRUE(isotp_poll(&a, 50u, &out)); /* block of 1: N_Bs from here */
    TEST_ASSERT_FALSE(isotp_poll(&a, 50u + ISOTP_DEFAULT_N_BS_MS - 1u, &out));
    isotp_poll(&a, 50u + ISOTP_DEFAULT_N_BS_MS, &out);
    isotp_n_result_t r;
    TEST_ASSERT_TRUE(isotp_take_tx_confirm(&a, &r));
    TEST_ASSERT_EQUAL(ISOTP_N_TIMEOUT_BS, r);
}

static void test_fc_wait_restarts_n_bs_until_wft_max(void)
{
    cfg.wft_max = 2u;
    isotp_init(&a, &cfg, rx_buf_a, BUF_CAP, tx_buf_a, BUF_CAP);
    uint8_t msg[20] = {0};
    isotp_frame_t out;
    isotp_send(&a, msg, 20u);
    isotp_poll(&a, 0u, &out);

    isotp_frame_t wait = fc(1u, 0u, 0u);
    isotp_on_frame(&a, &wait, 900u);
    TEST_ASSERT_FALSE(isotp_poll(&a, 1500u, &out)); /* N_Bs restarted at 900 */
    isotp_n_result_t r;
    TEST_ASSERT_FALSE(isotp_take_tx_confirm(&a, &r));
    isotp_on_frame(&a, &wait, 1800u);
    TEST_ASSERT_FALSE(isotp_take_tx_confirm(&a, &r));
    isotp_on_frame(&a, &wait, 1900u); /* third WAIT in a row: over N_WFTmax = 2 */
    TEST_ASSERT_TRUE(isotp_take_tx_confirm(&a, &r));
    TEST_ASSERT_EQUAL(ISOTP_N_WFT_OVRN, r);
}

static void test_fc_wait_count_resets_after_cts(void)
{
    cfg.wft_max = 1u;
    isotp_init(&a, &cfg, rx_buf_a, BUF_CAP, tx_buf_a, BUF_CAP);
    uint8_t msg[60] = {0};
    isotp_frame_t out;
    isotp_send(&a, msg, 60u);
    isotp_poll(&a, 0u, &out);
    isotp_frame_t wait = fc(1u, 0u, 0u);
    isotp_frame_t cts = fc(0u, 1u, 0u);
    isotp_on_frame(&a, &wait, 1u);
    isotp_on_frame(&a, &cts, 2u);
    TEST_ASSERT_TRUE(isotp_poll(&a, 2u, &out));
    isotp_on_frame(&a, &wait, 3u); /* first WAIT of the new block: allowed */
    isotp_n_result_t r;
    TEST_ASSERT_FALSE(isotp_take_tx_confirm(&a, &r));
}

static void test_fc_overflow_and_invalid_status_abort_the_transfer(void)
{
    uint8_t msg[20] = {0};
    isotp_frame_t out;
    isotp_n_result_t r;

    isotp_send(&a, msg, 20u);
    isotp_poll(&a, 0u, &out);
    isotp_frame_t ovflw = fc(2u, 0u, 0u);
    isotp_on_frame(&a, &ovflw, 1u);
    TEST_ASSERT_TRUE(isotp_take_tx_confirm(&a, &r));
    TEST_ASSERT_EQUAL(ISOTP_N_BUFFER_OVFLW, r);
    TEST_ASSERT_FALSE(isotp_poll(&a, 1u, &out));

    isotp_send(&a, msg, 20u);
    isotp_poll(&a, 2u, &out);
    isotp_frame_t reserved = fc(3u, 0u, 0u);
    isotp_on_frame(&a, &reserved, 3u);
    TEST_ASSERT_TRUE(isotp_take_tx_confirm(&a, &r));
    TEST_ASSERT_EQUAL(ISOTP_N_INVALID_FS, r);
}

static void test_unexpected_or_short_fc_is_ignored(void)
{
    isotp_frame_t out;
    isotp_frame_t cts = fc(0u, 0u, 0u);
    isotp_on_frame(&a, &cts, 0u); /* idle: ignored */
    TEST_ASSERT_FALSE(isotp_poll(&a, 0u, &out));

    uint8_t msg[20] = {0};
    isotp_send(&a, msg, 20u);
    isotp_poll(&a, 0u, &out);
    const uint8_t short_fc[2] = {0x30u, 0x00u};
    isotp_frame_t s = frame_of(2u, short_fc);
    isotp_on_frame(&a, &s, 1u);
    TEST_ASSERT_FALSE(isotp_poll(&a, 1u, &out)); /* still waiting for a valid FC */
}

/* ------------------------------------------------------------------------- */
/* Receiver                                                                   */
/* ------------------------------------------------------------------------- */

static void test_single_frame_reception(void)
{
    const uint8_t d[8] = {0x03u, 0x62u, 0xF4u, 0x0Cu, 0xAAu, 0xAAu, 0xAAu, 0xAAu};
    isotp_frame_t f = frame_of(8u, d);
    isotp_on_frame(&a, &f, 0u);
    isotp_n_result_t r;
    uint16_t len = 0u;
    TEST_ASSERT_TRUE(isotp_take_rx_indication(&a, &r, &len));
    TEST_ASSERT_EQUAL(ISOTP_N_OK, r);
    TEST_ASSERT_EQUAL_UINT16(3u, len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(&d[1], rx_buf_a, 3u);
    TEST_ASSERT_FALSE(isotp_take_rx_indication(&a, &r, &len));
}

static void test_invalid_single_frames_are_ignored(void)
{
    const uint8_t zero_len[2] = {0x00u, 0x11u};
    const uint8_t too_long[8] = {0x08u, 1u, 2u, 3u, 4u, 5u, 6u, 7u};
    const uint8_t short_dlc[3] = {0x05u, 1u, 2u};
    isotp_frame_t f1 = frame_of(2u, zero_len);
    isotp_frame_t f2 = frame_of(8u, too_long);
    isotp_frame_t f3 = frame_of(3u, short_dlc);
    isotp_on_frame(&a, &f1, 0u);
    isotp_on_frame(&a, &f2, 0u);
    isotp_on_frame(&a, &f3, 0u);
    isotp_frame_t empty = frame_of(0u, zero_len);
    isotp_on_frame(&a, &empty, 0u);
    TEST_ASSERT_FALSE(isotp_take_rx_indication(&a, NULL, NULL));
}

static void test_multi_frame_reception_sends_fc_and_reassembles(void)
{
    cfg.block_size = 0u;
    cfg.st_min = 5u;
    isotp_init(&a, &cfg, rx_buf_a, BUF_CAP, tx_buf_a, BUF_CAP);
    enum { LEN = 20 };
    uint8_t msg[LEN] = {0};
    fill_pattern(msg, LEN, 11u);

    const uint8_t ff[8] = {0x10u, LEN, msg[0], msg[1], msg[2], msg[3], msg[4], msg[5]};
    isotp_frame_t f = frame_of(8u, ff);
    isotp_on_frame(&a, &f, 0u);

    isotp_frame_t out;
    TEST_ASSERT_TRUE(isotp_poll(&a, 1u, &out));
    const uint8_t expected_fc[3] = {0x30u, 0x00u, 0x05u};
    TEST_ASSERT_EQUAL_UINT8(3u, out.dlc);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_fc, out.data, 3u);
    TEST_ASSERT_FALSE(isotp_poll(&a, 1u, &out));

    const uint8_t cf1[8] = {0x21u, msg[6], msg[7], msg[8], msg[9], msg[10], msg[11], msg[12]};
    const uint8_t cf2[8] = {0x22u, msg[13], msg[14], msg[15], msg[16], msg[17], msg[18], msg[19]};
    isotp_frame_t c1 = frame_of(8u, cf1);
    isotp_frame_t c2 = frame_of(8u, cf2);
    isotp_on_frame(&a, &c1, 2u);
    TEST_ASSERT_FALSE(isotp_take_rx_indication(&a, NULL, NULL));
    isotp_on_frame(&a, &c2, 3u);

    isotp_n_result_t r;
    uint16_t len = 0u;
    TEST_ASSERT_TRUE(isotp_take_rx_indication(&a, &r, &len));
    TEST_ASSERT_EQUAL(ISOTP_N_OK, r);
    TEST_ASSERT_EQUAL_UINT16(LEN, len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(msg, rx_buf_a, LEN);
}

static void test_receiver_block_size_sends_fc_every_block(void)
{
    cfg.block_size = 2u;
    isotp_init(&a, &cfg, rx_buf_a, BUF_CAP, tx_buf_a, BUF_CAP);
    const uint8_t ff[8] = {0x10u, 40u, 0, 0, 0, 0, 0, 0};
    isotp_frame_t f = frame_of(8u, ff);
    isotp_frame_t out;
    isotp_on_frame(&a, &f, 0u);
    TEST_ASSERT_TRUE(isotp_poll(&a, 0u, &out));
    TEST_ASSERT_EQUAL_HEX8(2u, out.data[1]);

    uint8_t cf[8] = {0x21u, 0, 0, 0, 0, 0, 0, 0};
    isotp_frame_t c = frame_of(8u, cf);
    isotp_on_frame(&a, &c, 1u);
    TEST_ASSERT_FALSE(isotp_poll(&a, 1u, &out));
    c.data[0] = 0x22u;
    isotp_on_frame(&a, &c, 2u);
    TEST_ASSERT_TRUE(isotp_poll(&a, 2u, &out)); /* next FC after BS = 2 CFs */
    TEST_ASSERT_EQUAL_HEX8(0x30u, out.data[0]);
}

static void test_too_long_first_frame_gets_fc_overflow(void)
{
    isotp_link_t small;
    TEST_ASSERT_EQUAL(ISOTP_OK, isotp_init(&small, &cfg, rx_buf_b, 16u, tx_buf_b, 16u));
    const uint8_t ff[8] = {0x10u, 17u, 0, 0, 0, 0, 0, 0};
    isotp_frame_t f = frame_of(8u, ff);
    isotp_frame_t out;
    isotp_on_frame(&small, &f, 0u);
    TEST_ASSERT_TRUE(isotp_poll(&small, 0u, &out));
    TEST_ASSERT_EQUAL_HEX8(0x32u, out.data[0]);
    TEST_ASSERT_EQUAL_UINT16(1u, isotp_rx_error_count(&small));
    TEST_ASSERT_FALSE(isotp_take_rx_indication(&small, NULL, NULL));

    const uint8_t cf[8] = {0x21u, 0, 0, 0, 0, 0, 0, 0};
    isotp_frame_t c = frame_of(8u, cf);
    isotp_on_frame(&small, &c, 1u); /* no reception started: ignored */
    TEST_ASSERT_FALSE(isotp_take_rx_indication(&small, NULL, NULL));
}

static void test_invalid_first_frames_are_ignored(void)
{
    const uint8_t ff_dl_7[8] = {0x10u, 7u, 0, 0, 0, 0, 0, 0};
    const uint8_t escape[8] = {0x10u, 0x00u, 0, 0, 0, 0x20u, 0, 0}; /* 32-bit FF_DL: out of scope */
    const uint8_t short_ff[7] = {0x10u, 20u, 0, 0, 0, 0, 0};
    isotp_frame_t f1 = frame_of(8u, ff_dl_7);
    isotp_frame_t f2 = frame_of(8u, escape);
    isotp_frame_t f3 = frame_of(7u, short_ff);
    isotp_frame_t out;
    isotp_on_frame(&a, &f1, 0u);
    isotp_on_frame(&a, &f2, 0u);
    isotp_on_frame(&a, &f3, 0u);
    TEST_ASSERT_FALSE(isotp_poll(&a, 0u, &out)); /* no FC */
}

static void test_wrong_sequence_number_aborts(void)
{
    const uint8_t ff[8] = {0x10u, 20u, 0, 0, 0, 0, 0, 0};
    const uint8_t cf_sn2[8] = {0x22u, 0, 0, 0, 0, 0, 0, 0};
    isotp_frame_t f = frame_of(8u, ff);
    isotp_frame_t c = frame_of(8u, cf_sn2);
    isotp_frame_t out;
    isotp_on_frame(&a, &f, 0u);
    isotp_poll(&a, 0u, &out);
    isotp_on_frame(&a, &c, 1u);
    isotp_n_result_t r;
    TEST_ASSERT_TRUE(isotp_take_rx_indication(&a, &r, NULL));
    TEST_ASSERT_EQUAL(ISOTP_N_WRONG_SN, r);
    TEST_ASSERT_EQUAL_UINT16(1u, isotp_rx_error_count(&a));
}

static void test_missing_consecutive_frame_times_out_n_cr(void)
{
    const uint8_t ff[8] = {0x10u, 20u, 0, 0, 0, 0, 0, 0};
    isotp_frame_t f = frame_of(8u, ff);
    isotp_frame_t out;
    isotp_on_frame(&a, &f, 0u);
    TEST_ASSERT_TRUE(isotp_poll(&a, 10u, &out)); /* FC out at 10: N_Cr starts here */
    isotp_poll(&a, 10u + ISOTP_DEFAULT_N_CR_MS - 1u, &out);
    TEST_ASSERT_FALSE(isotp_take_rx_indication(&a, NULL, NULL));
    isotp_poll(&a, 10u + ISOTP_DEFAULT_N_CR_MS, &out);
    isotp_n_result_t r;
    TEST_ASSERT_TRUE(isotp_take_rx_indication(&a, &r, NULL));
    TEST_ASSERT_EQUAL(ISOTP_N_TIMEOUT_CR, r);
}

static void test_new_single_frame_during_reception_replaces_it(void)
{
    const uint8_t ff[8] = {0x10u, 20u, 0, 0, 0, 0, 0, 0};
    const uint8_t sf[3] = {0x02u, 0x3Eu, 0x00u};
    isotp_frame_t f = frame_of(8u, ff);
    isotp_frame_t s = frame_of(3u, sf);
    isotp_frame_t out;
    isotp_on_frame(&a, &f, 0u);
    isotp_poll(&a, 0u, &out);
    isotp_on_frame(&a, &s, 1u);
    isotp_n_result_t r;
    uint16_t len = 0u;
    TEST_ASSERT_TRUE(isotp_take_rx_indication(&a, &r, &len));
    TEST_ASSERT_EQUAL(ISOTP_N_OK, r); /* the SF is delivered ... */
    TEST_ASSERT_EQUAL_UINT16(2u, len);
    TEST_ASSERT_EQUAL_UINT16(1u, isotp_rx_error_count(&a)); /* ... the FF reception counted as dropped */
}

static void test_new_first_frame_during_reception_restarts_it(void)
{
    const uint8_t ff[8] = {0x10u, 20u, 0, 0, 0, 0, 0, 0};
    isotp_frame_t f = frame_of(8u, ff);
    isotp_frame_t out;
    isotp_on_frame(&a, &f, 0u);
    isotp_poll(&a, 0u, &out);
    isotp_on_frame(&a, &f, 1u);
    TEST_ASSERT_EQUAL_UINT16(1u, isotp_rx_error_count(&a));
    TEST_ASSERT_TRUE(isotp_poll(&a, 1u, &out)); /* a fresh FC for the new FF */
    TEST_ASSERT_EQUAL_HEX8(0x30u, out.data[0]);
}

static void test_unexpected_consecutive_frame_is_ignored(void)
{
    const uint8_t cf[8] = {0x21u, 1, 2, 3, 4, 5, 6, 7};
    isotp_frame_t c = frame_of(8u, cf);
    isotp_on_frame(&a, &c, 0u);
    TEST_ASSERT_FALSE(isotp_take_rx_indication(&a, NULL, NULL));
    TEST_ASSERT_EQUAL_UINT16(0u, isotp_rx_error_count(&a));
}

static void test_unreleased_message_blocks_new_receptions(void)
{
    const uint8_t sf1[2] = {0x01u, 0x11u};
    const uint8_t sf2[2] = {0x01u, 0x22u};
    isotp_frame_t s1 = frame_of(2u, sf1);
    isotp_frame_t s2 = frame_of(2u, sf2);
    isotp_on_frame(&a, &s1, 0u);
    TEST_ASSERT_TRUE(isotp_take_rx_indication(&a, NULL, NULL));
    isotp_on_frame(&a, &s2, 1u);
    TEST_ASSERT_FALSE(isotp_take_rx_indication(&a, NULL, NULL));
    TEST_ASSERT_EQUAL_HEX8(0x11u, rx_buf_a[0]); /* not overwritten */

    isotp_rx_release(&a);
    isotp_on_frame(&a, &s2, 2u);
    TEST_ASSERT_TRUE(isotp_take_rx_indication(&a, NULL, NULL));
    TEST_ASSERT_EQUAL_HEX8(0x22u, rx_buf_a[0]);
}

static void test_short_consecutive_frame_is_ignored(void)
{
    const uint8_t ff[8] = {0x10u, 20u, 0, 0, 0, 0, 0, 0};
    const uint8_t short_cf[4] = {0x21u, 1, 2, 3}; /* 7 bytes still expected */
    isotp_frame_t f = frame_of(8u, ff);
    isotp_frame_t c = frame_of(4u, short_cf);
    isotp_frame_t out;
    isotp_on_frame(&a, &f, 0u);
    isotp_poll(&a, 0u, &out);
    isotp_on_frame(&a, &c, 1u);
    TEST_ASSERT_FALSE(isotp_take_rx_indication(&a, NULL, NULL));
    TEST_ASSERT_EQUAL_UINT16(0u, isotp_rx_error_count(&a));
}

/* ------------------------------------------------------------------------- */
/* Two links talking to each other                                            */
/* ------------------------------------------------------------------------- */

/* Moves frames both ways until both links are quiet; returns the passes used. */
static int pump(isotp_link_t* x, isotp_link_t* y, uint32_t* now, int max_passes)
{
    int pass = 0;
    for (; pass < max_passes; pass++) {
        bool moved = false;
        isotp_frame_t f;
        while (isotp_poll(x, *now, &f)) {
            isotp_on_frame(y, &f, *now);
            moved = true;
        }
        while (isotp_poll(y, *now, &f)) {
            isotp_on_frame(x, &f, *now);
            moved = true;
        }
        if (!moved && (x->tx_state == ISOTP_TX_IDLE) && (y->tx_state == ISOTP_TX_IDLE) &&
            (x->rx_state != ISOTP_RX_WAIT_CF) && (y->rx_state != ISOTP_RX_WAIT_CF)) {
            break;
        }
        *now += 1u;
    }
    return pass;
}

static void test_loopback_all_lengths_and_flow_parameters(void)
{
    static const uint8_t block_sizes[] = {0u, 1u, 3u, 8u};
    static const uint8_t st_mins[] = {0u, 2u, 0xF5u};
    static const uint16_t lengths[] = {1u, 6u, 7u, 8u, 13u, 14u, 15u, 111u, 112u, 113u, 1000u, 4095u};
    static uint8_t msg[BUF_CAP];
    uint32_t now = 0xFFFFF000u; /* crosses the 32-bit wrap during the run */

    for (size_t bi = 0; bi < sizeof block_sizes; bi++) {
        for (size_t si = 0; si < sizeof st_mins; si++) {
            for (size_t li = 0; li < sizeof lengths / sizeof lengths[0]; li++) {
                isotp_config_t cb;
                isotp_default_config(&cb);
                cb.block_size = block_sizes[bi];
                cb.st_min = st_mins[si];
                cb.padding_enabled = (li % 2u) == 0u;
                cb.padding_byte = 0xCCu;
                isotp_init(&a, &cfg, rx_buf_a, BUF_CAP, tx_buf_a, BUF_CAP);
                isotp_init(&b, &cb, rx_buf_b, BUF_CAP, tx_buf_b, BUF_CAP);

                uint16_t len = lengths[li];
                fill_pattern(msg, len, (uint8_t)(bi * 31u + si * 7u + li));
                TEST_ASSERT_EQUAL(ISOTP_OK, isotp_send(&a, msg, len));
                pump(&a, &b, &now, 100000);

                isotp_n_result_t r;
                uint16_t got = 0u;
                TEST_ASSERT_TRUE(isotp_take_tx_confirm(&a, &r));
                TEST_ASSERT_EQUAL(ISOTP_N_OK, r);
                TEST_ASSERT_TRUE(isotp_take_rx_indication(&b, &r, &got));
                TEST_ASSERT_EQUAL(ISOTP_N_OK, r);
                TEST_ASSERT_EQUAL_UINT16(len, got);
                TEST_ASSERT_EQUAL_HEX8_ARRAY(msg, rx_buf_b, len);
            }
        }
    }
}

static void test_full_duplex_request_and_response_on_one_link_pair(void)
{
    uint8_t req[50];
    uint8_t resp[300];
    fill_pattern(req, 50u, 1u);
    fill_pattern(resp, 300u, 2u);
    isotp_init(&b, &cfg, rx_buf_b, BUF_CAP, tx_buf_b, BUF_CAP);
    uint32_t now = 0u;
    TEST_ASSERT_EQUAL(ISOTP_OK, isotp_send(&a, req, 50u));
    TEST_ASSERT_EQUAL(ISOTP_OK, isotp_send(&b, resp, 300u)); /* both directions at once */
    pump(&a, &b, &now, 1000);

    uint16_t got = 0u;
    TEST_ASSERT_TRUE(isotp_take_rx_indication(&b, NULL, &got));
    TEST_ASSERT_EQUAL_UINT16(50u, got);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(req, rx_buf_b, 50u);
    TEST_ASSERT_TRUE(isotp_take_rx_indication(&a, NULL, &got));
    TEST_ASSERT_EQUAL_UINT16(300u, got);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(resp, rx_buf_a, 300u);
}

static void test_null_arguments_are_harmless(void)
{
    isotp_frame_t out;
    memset(&out, 0, sizeof out);
    isotp_default_config(NULL);
    isotp_on_frame(NULL, &out, 0u);
    isotp_on_frame(&a, NULL, 0u);
    TEST_ASSERT_FALSE(isotp_poll(NULL, 0u, &out));
    TEST_ASSERT_FALSE(isotp_poll(&a, 0u, NULL));
    TEST_ASSERT_FALSE(isotp_take_rx_indication(NULL, NULL, NULL));
    TEST_ASSERT_FALSE(isotp_take_tx_confirm(NULL, NULL));
    isotp_rx_release(NULL);
    TEST_ASSERT_EQUAL_UINT16(0u, isotp_rx_error_count(NULL));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_init_rejects_invalid_arguments);
    RUN_TEST(test_send_rejects_bad_length_and_busy_link);
    RUN_TEST(test_capacity_is_capped_at_4095);
    RUN_TEST(test_st_min_decoding);
    RUN_TEST(test_single_frame_send_without_padding);
    RUN_TEST(test_single_frame_send_with_padding);
    RUN_TEST(test_seven_bytes_is_still_a_single_frame_eight_is_not);
    RUN_TEST(test_multi_frame_send_segments_and_wraps_sn);
    RUN_TEST(test_last_cf_without_padding_is_short);
    RUN_TEST(test_block_size_waits_for_next_flow_control);
    RUN_TEST(test_st_min_spaces_consecutive_frames);
    RUN_TEST(test_reserved_st_min_is_treated_as_127_ms);
    RUN_TEST(test_missing_flow_control_times_out_n_bs);
    RUN_TEST(test_n_bs_also_guards_the_fc_after_a_block);
    RUN_TEST(test_fc_wait_restarts_n_bs_until_wft_max);
    RUN_TEST(test_fc_wait_count_resets_after_cts);
    RUN_TEST(test_fc_overflow_and_invalid_status_abort_the_transfer);
    RUN_TEST(test_unexpected_or_short_fc_is_ignored);
    RUN_TEST(test_single_frame_reception);
    RUN_TEST(test_invalid_single_frames_are_ignored);
    RUN_TEST(test_multi_frame_reception_sends_fc_and_reassembles);
    RUN_TEST(test_receiver_block_size_sends_fc_every_block);
    RUN_TEST(test_too_long_first_frame_gets_fc_overflow);
    RUN_TEST(test_invalid_first_frames_are_ignored);
    RUN_TEST(test_wrong_sequence_number_aborts);
    RUN_TEST(test_missing_consecutive_frame_times_out_n_cr);
    RUN_TEST(test_new_single_frame_during_reception_replaces_it);
    RUN_TEST(test_new_first_frame_during_reception_restarts_it);
    RUN_TEST(test_unexpected_consecutive_frame_is_ignored);
    RUN_TEST(test_unreleased_message_blocks_new_receptions);
    RUN_TEST(test_short_consecutive_frame_is_ignored);
    RUN_TEST(test_loopback_all_lengths_and_flow_parameters);
    RUN_TEST(test_full_duplex_request_and_response_on_one_link_pair);
    RUN_TEST(test_null_arguments_are_harmless);
    return UNITY_END();
}
