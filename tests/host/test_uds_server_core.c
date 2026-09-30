/*
 * L0 tests for the UDS server core (features/uds/uds_server_core, Ç3, ISO 14229-1:2020)
 * with a fake provider and an explicit clock: request handling per service, the NRC
 * order of clause 7.5, functional-request suppression, suppressPosRspMsgIndicationBit,
 * response pending (NRC 0x78, P2*) and the S3 session timeout including the 32-bit
 * clock wrap. Every code, ID, DID, DTC and timing comes from gen/. No requirement IDs
 * yet (Q-006).
 */
#include "features/uds/uds_server_core.h"
#include "platform_uds.h"
#include "uds_iso14229.h"

#include <string.h>
#include <unity.h>

#define HI(v) ((uint8_t)(((v) >> 8u) & 0xFFu))
#define LO(v) ((uint8_t)((v) & 0xFFu))
#define DID_BYTES(d) HI(d), LO(d)
#define GROUP_BYTES(g) ((uint8_t)(((g) >> 16u) & 0xFFu)), ((uint8_t)(((g) >> 8u) & 0xFFu)), \
                       ((uint8_t)((g) & 0xFFu))
#define P2_STAR_REPEAT_MS (PLATFORM_UDS_P2_STAR_SERVER_MAX_MS / 2u)
#define AVAIL ((uint8_t)PLATFORM_UDS_DTC_STATUS_AVAILABILITY_MASK)
#define NEVER 1000000u /* "stays pending" */

_Static_assert(PLATFORM_UDS_DTC_COUNT >= 2u, "the DTC tests need two DTCs");
_Static_assert(PLATFORM_UDS_DID_COUNT >= PLATFORM_UDS_MAX_READ_DIDS, "read tests need DIDs");

/* ------------------------------------------------------------------ fake provider */

typedef enum { MODE_OK = 0, MODE_PENDING, MODE_FAIL } mode_t_;

typedef struct {
    mode_t_ did_mode;
    uint32_t did_pending_left; /* MODE_PENDING: answers PENDING this many more calls */
    mode_t_ clear_mode;
    uint32_t clear_pending_left;
    uint8_t dtc[PLATFORM_UDS_DTC_COUNT];
    uint32_t did_calls;
    uint32_t dtc_calls;
    uint32_t clear_calls;
    uint32_t last_did_idx;
} fake_t;

static fake_t fake;
static uds_server_provider_t provider;
static uds_server_core_t core;
static uint8_t rsp[UDS_SERVER_RSP_MAX];
static uint32_t now;

/* The record the fake provider writes for DID index idx. */
static uint8_t fake_byte(uint32_t idx, uint16_t i)
{
    return (uint8_t)((idx * 16u) + i + 1u);
}

/* read_did / dtc_status take a const ctx: the fake counts calls in its global instead. */
static uds_server_data_t fake_read_did(const void* ctx, uint32_t idx, uint8_t* out, uint16_t len)
{
    TEST_ASSERT_EQUAL_PTR(&fake, ctx);
    fake_t* f = &fake;
    f->did_calls++;
    f->last_did_idx = idx;
    if (f->did_mode == MODE_FAIL) {
        return UDS_SERVER_DATA_FAIL;
    }
    if ((f->did_mode == MODE_PENDING) && (f->did_pending_left > 0u)) {
        f->did_pending_left--;
        return UDS_SERVER_DATA_PENDING;
    }
    for (uint16_t i = 0u; i < len; i++) {
        out[i] = fake_byte(idx, i);
    }
    return UDS_SERVER_DATA_OK;
}

static uint8_t fake_dtc_status(const void* ctx, uint32_t idx)
{
    TEST_ASSERT_EQUAL_PTR(&fake, ctx);
    fake_t* f = &fake;
    f->dtc_calls++;
    return f->dtc[idx];
}

static uds_server_data_t fake_clear(void* ctx)
{
    fake_t* f = (fake_t*)ctx;
    f->clear_calls++;
    if (f->clear_mode == MODE_FAIL) {
        return UDS_SERVER_DATA_FAIL;
    }
    if ((f->clear_mode == MODE_PENDING) && (f->clear_pending_left > 0u)) {
        f->clear_pending_left--;
        return UDS_SERVER_DATA_PENDING;
    }
    for (uint32_t i = 0u; i < PLATFORM_UDS_DTC_COUNT; i++) {
        f->dtc[i] = 0u;
    }
    return UDS_SERVER_DATA_OK;
}

void setUp(void)
{
    memset(&fake, 0, sizeof fake);
    provider.read_did = fake_read_did;
    provider.dtc_status = fake_dtc_status;
    provider.clear_dtcs = fake_clear;
    provider.ctx = &fake;
    now = 10000u;
    memset(rsp, 0xEE, sizeof rsp);
    TEST_ASSERT_TRUE(uds_server_core_init(&core, &provider, now));
}

void tearDown(void)
{
}

/* ------------------------------------------------------------------ helpers */

static uint16_t request(const uint8_t* req, uint16_t len, bool functional)
{
    memset(rsp, 0xEE, sizeof rsp);
    return uds_server_core_on_request(&core, now, req, len, functional, rsp, sizeof rsp);
}

#define PHYS(arr) request((arr), (uint16_t)sizeof(arr), false)
#define FUNC(arr) request((arr), (uint16_t)sizeof(arr), true)

static uint16_t poll_at(uint32_t t)
{
    now = t;
    memset(rsp, 0xEE, sizeof rsp);
    return uds_server_core_poll(&core, now, rsp, sizeof rsp);
}

static void assert_nrc(uint16_t len, uint8_t sid, uint8_t nrc)
{
    TEST_ASSERT_EQUAL_UINT16(UDS_NEGATIVE_RESPONSE_LEN, len);
    TEST_ASSERT_EQUAL_HEX8(UDS_SID_NEGATIVE_RESPONSE, rsp[0]);
    TEST_ASSERT_EQUAL_HEX8(sid, rsp[1]);
    TEST_ASSERT_EQUAL_HEX8(nrc, rsp[2]);
}

static void enter_extended(void)
{
    const uint8_t req[] = {UDS_SID_DIAGNOSTIC_SESSION_CONTROL, UDS_SESSION_EXTENDED};
    TEST_ASSERT_EQUAL_UINT16(6u, PHYS(req));
    TEST_ASSERT_EQUAL_UINT8(UDS_SESSION_EXTENDED, uds_server_core_session(&core));
}

/* A DID value the server does not offer. */
static uint16_t unknown_did(uint16_t not_this)
{
    uint16_t d = 1u;
    while ((platform_uds_find_did(d) != NULL) || (d == not_this)) {
        d++;
    }
    return d;
}

/* A request SID below the positive-response range that the server does not offer. */
static uint8_t unsupported_sid(void)
{
    uint8_t s = 1u;
    while (platform_uds_service_supported(s)) {
        s++;
    }
    TEST_ASSERT_LESS_THAN_UINT8(UDS_POSITIVE_RESPONSE_OFFSET, s);
    return s;
}

/* Expected data record of DID index idx as the core serves it. */
static void expect_record(const uint8_t* got, uint32_t idx)
{
    const uint16_t len = platform_uds_dids[idx].length;
    if (idx == (uint32_t)PLATFORM_UDS_IDX_ACTIVE_DIAGNOSTIC_SESSION) {
        TEST_ASSERT_EQUAL_UINT16(1u, len);
        TEST_ASSERT_EQUAL_HEX8(uds_server_core_session(&core), got[0]);
        return;
    }
    for (uint16_t i = 0u; i < len; i++) {
        TEST_ASSERT_EQUAL_HEX8(fake_byte(idx, i), got[i]);
    }
}

/* ------------------------------------------------------------------ init, getters */

static void test_init_rejects_a_null_core_or_provider_or_function_pointer(void)
{
    uds_server_core_t c;
    uds_server_provider_t p = provider;
    TEST_ASSERT_FALSE(uds_server_core_init(NULL, &provider, 0u));
    TEST_ASSERT_FALSE(uds_server_core_init(&c, NULL, 0u));
    p.read_did = NULL;
    TEST_ASSERT_FALSE(uds_server_core_init(&c, &p, 0u));
    p = provider;
    p.dtc_status = NULL;
    TEST_ASSERT_FALSE(uds_server_core_init(&c, &p, 0u));
    p = provider;
    p.clear_dtcs = NULL;
    TEST_ASSERT_FALSE(uds_server_core_init(&c, &p, 0u));
    TEST_ASSERT_TRUE(uds_server_core_init(&c, &provider, 0u));
}

static void test_init_starts_in_the_default_session_with_zeroed_stats(void)
{
    TEST_ASSERT_EQUAL_UINT8(UDS_SESSION_DEFAULT, uds_server_core_session(&core));
    TEST_ASSERT_FALSE(uds_server_core_pending(&core));
    const uds_server_stats_t* s = uds_server_core_stats(&core);
    TEST_ASSERT_NOT_NULL(s);
    TEST_ASSERT_EQUAL_UINT32(0u, s->requests);
    TEST_ASSERT_EQUAL_UINT32(0u, s->positive);
    TEST_ASSERT_EQUAL_UINT32(0u, s->negative);
    TEST_ASSERT_EQUAL_UINT32(0u, s->suppressed);
    TEST_ASSERT_EQUAL_UINT32(0u, s->response_pending);
    TEST_ASSERT_EQUAL_UINT32(0u, s->pending_expired);
    TEST_ASSERT_EQUAL_UINT32(0u, s->busy);
    TEST_ASSERT_EQUAL_UINT32(0u, s->s3_timeouts);
}

static void test_null_and_malformed_arguments_are_dropped_and_not_counted(void)
{
    const uint8_t req[] = {UDS_SID_TESTER_PRESENT, UDS_TESTER_PRESENT_ZERO_SUBFUNCTION};
    uint8_t big[PLATFORM_UDS_RX_BUFFER + 1u];
    memset(big, UDS_SID_TESTER_PRESENT, sizeof big);

    TEST_ASSERT_EQUAL_UINT16(0u, uds_server_core_on_request(&core, now, req, 0u, false, rsp, sizeof rsp));
    TEST_ASSERT_EQUAL_UINT16(0u, uds_server_core_on_request(&core, now, NULL, 2u, false, rsp, sizeof rsp));
    TEST_ASSERT_EQUAL_UINT16(0u, uds_server_core_on_request(&core, now, big, (uint16_t)sizeof big,
                                                            false, rsp, sizeof rsp));
    TEST_ASSERT_EQUAL_UINT16(0u, uds_server_core_on_request(&core, now, req, 2u, false, NULL, sizeof rsp));
    TEST_ASSERT_EQUAL_UINT16(0u, uds_server_core_on_request(NULL, now, req, 2u, false, rsp, sizeof rsp));
    TEST_ASSERT_EQUAL_UINT16(0u, uds_server_core_poll(NULL, now, rsp, sizeof rsp));
    TEST_ASSERT_EQUAL_UINT16(0u, uds_server_core_poll(&core, now, NULL, sizeof rsp));
    const uds_server_stats_t* s = uds_server_core_stats(&core);
    TEST_ASSERT_EQUAL_UINT32(0u, s->requests);
    TEST_ASSERT_EQUAL_UINT32(0u, s->positive);
    TEST_ASSERT_EQUAL_UINT32(0u, s->negative);
    TEST_ASSERT_EQUAL_UINT32(0u, s->suppressed);
}

static void test_a_request_of_exactly_the_rx_buffer_is_handled_and_one_more_is_dropped(void)
{
    uint8_t big[PLATFORM_UDS_RX_BUFFER + 1u];
    memset(big, 0, sizeof big);
    big[0] = UDS_SID_READ_DATA_BY_IDENTIFIER;
    const uint16_t n = request(big, PLATFORM_UDS_RX_BUFFER, false);
    assert_nrc(n, UDS_SID_READ_DATA_BY_IDENTIFIER, UDS_NRC_INCORRECT_MESSAGE_LENGTH_OR_INVALID_FORMAT);
    TEST_ASSERT_EQUAL_UINT32(1u, uds_server_core_stats(&core)->requests);
    TEST_ASSERT_EQUAL_UINT16(0u, request(big, (uint16_t)sizeof big, false));
    TEST_ASSERT_EQUAL_UINT32(1u, uds_server_core_stats(&core)->requests);
}

static void test_a_response_buffer_smaller_than_the_longest_response_is_refused(void)
{
    const uint8_t req[] = {UDS_SID_TESTER_PRESENT, UDS_TESTER_PRESENT_ZERO_SUBFUNCTION};
    TEST_ASSERT_EQUAL_UINT16(0u, uds_server_core_on_request(&core, now, req, sizeof req, false, rsp,
                                                            UDS_SERVER_RSP_MAX - 1u));
    TEST_ASSERT_EQUAL_UINT16(0u, uds_server_core_poll(&core, now, rsp, UDS_SERVER_RSP_MAX - 1u));
    TEST_ASSERT_EQUAL_UINT32(0u, uds_server_core_stats(&core)->requests);
    TEST_ASSERT_EQUAL_UINT16(2u, uds_server_core_on_request(&core, now, req, sizeof req, false, rsp,
                                                            UDS_SERVER_RSP_MAX));
}

static void test_null_core_getters_are_safe(void)
{
    TEST_ASSERT_EQUAL_UINT8(UDS_SESSION_DEFAULT, uds_server_core_session(NULL));
    TEST_ASSERT_FALSE(uds_server_core_pending(NULL));
    TEST_ASSERT_NULL(uds_server_core_stats(NULL));
}

/* ------------------------------------------------------------------ NRC order (7.5) */

static void test_an_unsupported_sid_is_nrc_service_not_supported(void)
{
    const uint8_t sid = unsupported_sid();
    const uint8_t req[] = {sid, 0x00u};
    assert_nrc(PHYS(req), sid, UDS_NRC_SERVICE_NOT_SUPPORTED);
}

static void test_a_sid_in_the_response_range_is_nrc_service_not_supported(void)
{
    const uint8_t pos_as_request[] = {
        (uint8_t)(UDS_SID_READ_DATA_BY_IDENTIFIER + UDS_POSITIVE_RESPONSE_OFFSET)};
    assert_nrc(PHYS(pos_as_request), pos_as_request[0], UDS_NRC_SERVICE_NOT_SUPPORTED);
    const uint8_t neg_as_request[] = {UDS_SID_NEGATIVE_RESPONSE, UDS_SID_READ_DATA_BY_IDENTIFIER,
                                      UDS_NRC_GENERAL_REJECT};
    assert_nrc(PHYS(neg_as_request), UDS_SID_NEGATIVE_RESPONSE, UDS_NRC_SERVICE_NOT_SUPPORTED);
    TEST_ASSERT_EQUAL_UINT32(2u, uds_server_core_stats(&core)->negative);
}

static void test_clear_in_the_default_session_is_nrc_service_not_supported_in_active_session(void)
{
    const uint8_t req[] = {UDS_SID_CLEAR_DIAGNOSTIC_INFORMATION, GROUP_BYTES(UDS_GROUP_OF_DTC_ALL)};
    assert_nrc(PHYS(req), UDS_SID_CLEAR_DIAGNOSTIC_INFORMATION,
               UDS_NRC_SERVICE_NOT_SUPPORTED_IN_ACTIVE_SESSION);
    TEST_ASSERT_EQUAL_UINT32(0u, fake.clear_calls);
}

static void test_session_control_length_and_sub_function_nrcs_in_order(void)
{
    const uint8_t too_short[] = {UDS_SID_DIAGNOSTIC_SESSION_CONTROL};
    assert_nrc(PHYS(too_short), UDS_SID_DIAGNOSTIC_SESSION_CONTROL,
               UDS_NRC_INCORRECT_MESSAGE_LENGTH_OR_INVALID_FORMAT);
    const uint8_t programming[] = {UDS_SID_DIAGNOSTIC_SESSION_CONTROL, UDS_SESSION_PROGRAMMING};
    assert_nrc(PHYS(programming), UDS_SID_DIAGNOSTIC_SESSION_CONTROL,
               UDS_NRC_SUBFUNCTION_NOT_SUPPORTED);
    /* 0x12 outranks 0x13: an unsupported sub-function with extra bytes */
    const uint8_t programming_long[] = {UDS_SID_DIAGNOSTIC_SESSION_CONTROL, UDS_SESSION_PROGRAMMING,
                                        0x00u};
    assert_nrc(PHYS(programming_long), UDS_SID_DIAGNOSTIC_SESSION_CONTROL,
               UDS_NRC_SUBFUNCTION_NOT_SUPPORTED);
    /* an NRC is sent even when suppressPosRspMsgIndicationBit is set */
    const uint8_t programming_spr[] = {
        UDS_SID_DIAGNOSTIC_SESSION_CONTROL,
        (uint8_t)(UDS_SESSION_PROGRAMMING | UDS_SUPPRESS_POS_RSP_BIT)};
    assert_nrc(PHYS(programming_spr), UDS_SID_DIAGNOSTIC_SESSION_CONTROL,
               UDS_NRC_SUBFUNCTION_NOT_SUPPORTED);
    const uint8_t extended_long[] = {UDS_SID_DIAGNOSTIC_SESSION_CONTROL, UDS_SESSION_EXTENDED, 0x00u};
    assert_nrc(PHYS(extended_long), UDS_SID_DIAGNOSTIC_SESSION_CONTROL,
               UDS_NRC_INCORRECT_MESSAGE_LENGTH_OR_INVALID_FORMAT);
    TEST_ASSERT_EQUAL_UINT8(UDS_SESSION_DEFAULT, uds_server_core_session(&core));
}

static void test_tester_present_length_and_sub_function_nrcs_in_order(void)
{
    const uint8_t too_short[] = {UDS_SID_TESTER_PRESENT};
    assert_nrc(PHYS(too_short), UDS_SID_TESTER_PRESENT,
               UDS_NRC_INCORRECT_MESSAGE_LENGTH_OR_INVALID_FORMAT);
    const uint8_t bad_sub[] = {UDS_SID_TESTER_PRESENT, (uint8_t)(UDS_TESTER_PRESENT_ZERO_SUBFUNCTION + 1u)};
    assert_nrc(PHYS(bad_sub), UDS_SID_TESTER_PRESENT, UDS_NRC_SUBFUNCTION_NOT_SUPPORTED);
    const uint8_t too_long[] = {UDS_SID_TESTER_PRESENT, UDS_TESTER_PRESENT_ZERO_SUBFUNCTION, 0x00u};
    assert_nrc(PHYS(too_long), UDS_SID_TESTER_PRESENT,
               UDS_NRC_INCORRECT_MESSAGE_LENGTH_OR_INVALID_FORMAT);
}

/* ------------------------------------------------------------------ 0x10, 0x3E */

static void test_extended_session_response_carries_p2_and_p2_star_from_gen(void)
{
    const uint8_t req[] = {UDS_SID_DIAGNOSTIC_SESSION_CONTROL, UDS_SESSION_EXTENDED};
    const uint16_t p2 = PLATFORM_UDS_P2_SERVER_MAX_MS / UDS_P2_RESOLUTION_MS;
    const uint16_t p2s = PLATFORM_UDS_P2_STAR_SERVER_MAX_MS / UDS_P2_STAR_RESOLUTION_MS;
    const uint8_t expect[] = {(uint8_t)(UDS_SID_DIAGNOSTIC_SESSION_CONTROL + UDS_POSITIVE_RESPONSE_OFFSET),
                              UDS_SESSION_EXTENDED, HI(p2), LO(p2), HI(p2s), LO(p2s)};
    const uint16_t n = PHYS(req);
    TEST_ASSERT_EQUAL_UINT16(sizeof expect, n);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expect, rsp, sizeof expect);
    TEST_ASSERT_EQUAL_UINT8(UDS_SESSION_EXTENDED, uds_server_core_session(&core));
    TEST_ASSERT_EQUAL_UINT32(1u, uds_server_core_stats(&core)->positive);
}

static void test_switching_back_to_the_default_session_is_positive(void)
{
    enter_extended();
    const uint8_t req[] = {UDS_SID_DIAGNOSTIC_SESSION_CONTROL, UDS_SESSION_DEFAULT};
    TEST_ASSERT_EQUAL_UINT16(6u, PHYS(req));
    TEST_ASSERT_EQUAL_HEX8(UDS_SESSION_DEFAULT, rsp[1]);
    TEST_ASSERT_EQUAL_UINT8(UDS_SESSION_DEFAULT, uds_server_core_session(&core));
}

static void test_session_control_with_suppress_pos_rsp_changes_the_session_and_sends_nothing(void)
{
    const uint8_t req[] = {UDS_SID_DIAGNOSTIC_SESSION_CONTROL,
                           (uint8_t)(UDS_SESSION_EXTENDED | UDS_SUPPRESS_POS_RSP_BIT)};
    TEST_ASSERT_EQUAL_UINT16(0u, PHYS(req));
    TEST_ASSERT_EQUAL_UINT8(UDS_SESSION_EXTENDED, uds_server_core_session(&core));
    TEST_ASSERT_EQUAL_UINT32(1u, uds_server_core_stats(&core)->suppressed);
    TEST_ASSERT_EQUAL_UINT32(0u, uds_server_core_stats(&core)->positive);
}

static void test_tester_present_zero_sub_function_answers_with_the_sub_function(void)
{
    const uint8_t req[] = {UDS_SID_TESTER_PRESENT, UDS_TESTER_PRESENT_ZERO_SUBFUNCTION};
    const uint8_t expect[] = {(uint8_t)(UDS_SID_TESTER_PRESENT + UDS_POSITIVE_RESPONSE_OFFSET),
                              UDS_TESTER_PRESENT_ZERO_SUBFUNCTION};
    TEST_ASSERT_EQUAL_UINT16(sizeof expect, PHYS(req));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expect, rsp, sizeof expect);
}

static void test_tester_present_with_suppress_pos_rsp_is_silent_and_counted(void)
{
    const uint8_t req[] = {UDS_SID_TESTER_PRESENT,
                           (uint8_t)(UDS_TESTER_PRESENT_ZERO_SUBFUNCTION | UDS_SUPPRESS_POS_RSP_BIT)};
    TEST_ASSERT_EQUAL_UINT16(0u, PHYS(req));
    TEST_ASSERT_EQUAL_UINT32(1u, uds_server_core_stats(&core)->suppressed);
    TEST_ASSERT_EQUAL_UINT32(0u, uds_server_core_stats(&core)->positive);
}

/* ------------------------------------------------------------------ 0x22 */

static void test_read_active_session_did_is_served_by_the_core_in_both_sessions(void)
{
    const uint8_t req[] = {UDS_SID_READ_DATA_BY_IDENTIFIER,
                           DID_BYTES(PLATFORM_UDS_DID_ACTIVE_DIAGNOSTIC_SESSION)};
    uint8_t expect[] = {(uint8_t)(UDS_SID_READ_DATA_BY_IDENTIFIER + UDS_POSITIVE_RESPONSE_OFFSET),
                        DID_BYTES(PLATFORM_UDS_DID_ACTIVE_DIAGNOSTIC_SESSION), UDS_SESSION_DEFAULT};
    TEST_ASSERT_EQUAL_UINT16(sizeof expect, PHYS(req));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expect, rsp, sizeof expect);
    enter_extended();
    expect[3] = UDS_SESSION_EXTENDED;
    TEST_ASSERT_EQUAL_UINT16(sizeof expect, PHYS(req));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expect, rsp, sizeof expect);
    TEST_ASSERT_EQUAL_UINT32(0u, fake.did_calls); /* never asked of the provider */
}

static void test_read_of_a_provider_did_returns_the_record_after_the_did_echo(void)
{
    const uint8_t req[] = {UDS_SID_READ_DATA_BY_IDENTIFIER, DID_BYTES(PLATFORM_UDS_DID_SW_VERSION)};
    const uint16_t n = PHYS(req);
    const uint32_t idx = PLATFORM_UDS_IDX_SW_VERSION;
    TEST_ASSERT_EQUAL_UINT16(3u + platform_uds_dids[idx].length, n);
    TEST_ASSERT_EQUAL_HEX8(UDS_SID_READ_DATA_BY_IDENTIFIER + UDS_POSITIVE_RESPONSE_OFFSET, rsp[0]);
    TEST_ASSERT_EQUAL_HEX8(HI(PLATFORM_UDS_DID_SW_VERSION), rsp[1]);
    TEST_ASSERT_EQUAL_HEX8(LO(PLATFORM_UDS_DID_SW_VERSION), rsp[2]);
    expect_record(&rsp[3], idx);
    TEST_ASSERT_EQUAL_UINT32(idx, fake.last_did_idx);
}

static void test_read_of_the_maximum_number_of_dids_concatenates_records_in_request_order(void)
{
    uint8_t req[1u + (2u * PLATFORM_UDS_MAX_READ_DIDS)];
    uint32_t order[PLATFORM_UDS_MAX_READ_DIDS];
    req[0] = UDS_SID_READ_DATA_BY_IDENTIFIER;
    for (uint32_t i = 0u; i < PLATFORM_UDS_MAX_READ_DIDS; i++) {
        order[i] = PLATFORM_UDS_DID_COUNT - 1u - i; /* reverse table order */
        req[1u + (2u * i)] = HI(platform_uds_dids[order[i]].did);
        req[2u + (2u * i)] = LO(platform_uds_dids[order[i]].did);
    }
    const uint16_t n = request(req, (uint16_t)sizeof req, false);
    uint16_t pos = 1u;
    TEST_ASSERT_EQUAL_HEX8(UDS_SID_READ_DATA_BY_IDENTIFIER + UDS_POSITIVE_RESPONSE_OFFSET, rsp[0]);
    for (uint32_t i = 0u; i < PLATFORM_UDS_MAX_READ_DIDS; i++) {
        TEST_ASSERT_EQUAL_HEX8(req[1u + (2u * i)], rsp[pos]);
        TEST_ASSERT_EQUAL_HEX8(req[2u + (2u * i)], rsp[pos + 1u]);
        expect_record(&rsp[pos + 2u], order[i]);
        pos = (uint16_t)(pos + 2u + platform_uds_dids[order[i]].length);
    }
    TEST_ASSERT_EQUAL_UINT16(pos, n);
    TEST_ASSERT_LESS_OR_EQUAL_UINT16(UDS_SERVER_RSP_MAX, n);
}

static void test_read_of_more_dids_than_the_maximum_is_incorrect_length(void)
{
    uint8_t req[1u + (2u * (PLATFORM_UDS_MAX_READ_DIDS + 1u))];
    req[0] = UDS_SID_READ_DATA_BY_IDENTIFIER;
    for (uint32_t i = 0u; i < (PLATFORM_UDS_MAX_READ_DIDS + 1u); i++) {
        req[1u + (2u * i)] = HI(PLATFORM_UDS_DID_SW_VERSION);
        req[2u + (2u * i)] = LO(PLATFORM_UDS_DID_SW_VERSION);
    }
    assert_nrc(request(req, (uint16_t)sizeof req, false), UDS_SID_READ_DATA_BY_IDENTIFIER,
               UDS_NRC_INCORRECT_MESSAGE_LENGTH_OR_INVALID_FORMAT);
    TEST_ASSERT_EQUAL_UINT32(0u, fake.did_calls);
}

static void test_read_with_a_partial_or_missing_did_is_incorrect_length(void)
{
    const uint8_t one[] = {UDS_SID_READ_DATA_BY_IDENTIFIER};
    const uint8_t two[] = {UDS_SID_READ_DATA_BY_IDENTIFIER, HI(PLATFORM_UDS_DID_SW_VERSION)};
    const uint8_t even[] = {UDS_SID_READ_DATA_BY_IDENTIFIER, DID_BYTES(PLATFORM_UDS_DID_SW_VERSION),
                            HI(PLATFORM_UDS_DID_UPTIME)};
    assert_nrc(PHYS(one), UDS_SID_READ_DATA_BY_IDENTIFIER,
               UDS_NRC_INCORRECT_MESSAGE_LENGTH_OR_INVALID_FORMAT);
    assert_nrc(PHYS(two), UDS_SID_READ_DATA_BY_IDENTIFIER,
               UDS_NRC_INCORRECT_MESSAGE_LENGTH_OR_INVALID_FORMAT);
    assert_nrc(PHYS(even), UDS_SID_READ_DATA_BY_IDENTIFIER,
               UDS_NRC_INCORRECT_MESSAGE_LENGTH_OR_INVALID_FORMAT);
}

static void test_read_of_only_unknown_dids_is_request_out_of_range(void)
{
    const uint16_t u1 = unknown_did(0u);
    const uint16_t u2 = unknown_did(u1);
    const uint8_t req[] = {UDS_SID_READ_DATA_BY_IDENTIFIER, DID_BYTES(u1), DID_BYTES(u2)};
    assert_nrc(PHYS(req), UDS_SID_READ_DATA_BY_IDENTIFIER, UDS_NRC_REQUEST_OUT_OF_RANGE);
    TEST_ASSERT_EQUAL_UINT32(0u, fake.did_calls);
}

static void test_read_of_known_and_unknown_dids_leaves_the_unknown_one_out(void)
{
    const uint16_t u = unknown_did(0u);
    const uint8_t req[] = {UDS_SID_READ_DATA_BY_IDENTIFIER, DID_BYTES(u),
                           DID_BYTES(PLATFORM_UDS_DID_UPTIME), DID_BYTES(u)};
    const uint16_t n = PHYS(req);
    TEST_ASSERT_EQUAL_UINT16(3u + platform_uds_dids[PLATFORM_UDS_IDX_UPTIME].length, n);
    TEST_ASSERT_EQUAL_HEX8(HI(PLATFORM_UDS_DID_UPTIME), rsp[1]);
    TEST_ASSERT_EQUAL_HEX8(LO(PLATFORM_UDS_DID_UPTIME), rsp[2]);
    expect_record(&rsp[3], PLATFORM_UDS_IDX_UPTIME);
}

static void test_read_with_a_failing_provider_is_conditions_not_correct(void)
{
    fake.did_mode = MODE_FAIL;
    const uint8_t req[] = {UDS_SID_READ_DATA_BY_IDENTIFIER, DID_BYTES(PLATFORM_UDS_DID_SW_VERSION)};
    assert_nrc(PHYS(req), UDS_SID_READ_DATA_BY_IDENTIFIER, UDS_NRC_CONDITIONS_NOT_CORRECT);
    TEST_ASSERT_FALSE(uds_server_core_pending(&core));
}

/* ------------------------------------------------------------------ 0x19 */

static void set_dtcs(void)
{
    /* DTC 0: every availability bit; DTC 1: TEST_FAILED plus bits the server does not offer */
    fake.dtc[0] = AVAIL;
    fake.dtc[1] = (uint8_t)(UDS_DTC_STATUS_TEST_FAILED | UDS_DTC_STATUS_PENDING_DTC |
                            UDS_DTC_STATUS_WARNING_INDICATOR_REQUESTED);
}

static void assert_dtc_count(uint8_t mask, uint16_t count)
{
    const uint8_t req[] = {UDS_SID_READ_DTC_INFORMATION,
                           UDS_READ_DTC_REPORT_NUMBER_OF_DTC_BY_STATUS_MASK, mask};
    const uint8_t expect[] = {(uint8_t)(UDS_SID_READ_DTC_INFORMATION + UDS_POSITIVE_RESPONSE_OFFSET),
                              UDS_READ_DTC_REPORT_NUMBER_OF_DTC_BY_STATUS_MASK, AVAIL,
                              UDS_DTC_FORMAT_ISO_14229_1, HI(count), LO(count)};
    TEST_ASSERT_EQUAL_UINT16(sizeof expect, PHYS(req));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expect, rsp, sizeof expect);
}

static void test_dtc_count_by_status_mask_counts_only_dtcs_matching_mask_and_availability(void)
{
    set_dtcs();
    assert_dtc_count(UDS_DTC_STATUS_TEST_FAILED, 2u);
    assert_dtc_count(UDS_DTC_STATUS_CONFIRMED_DTC, 1u);
    assert_dtc_count(0xFFu, 2u);
    /* PENDING_DTC is set on DTC 1 but is outside the availability mask */
    assert_dtc_count(UDS_DTC_STATUS_PENDING_DTC, 0u);
    assert_dtc_count(0x00u, 0u);
}

static void test_dtc_by_status_mask_lists_three_byte_dtc_and_masked_status(void)
{
    set_dtcs();
    const uint8_t req[] = {UDS_SID_READ_DTC_INFORMATION, UDS_READ_DTC_REPORT_DTC_BY_STATUS_MASK,
                           UDS_DTC_STATUS_TEST_FAILED};
    const uint16_t n = PHYS(req);
    const uint8_t expect[] = {
        (uint8_t)(UDS_SID_READ_DTC_INFORMATION + UDS_POSITIVE_RESPONSE_OFFSET),
        UDS_READ_DTC_REPORT_DTC_BY_STATUS_MASK, AVAIL,
        GROUP_BYTES(platform_uds_dtcs[0]), AVAIL,
        GROUP_BYTES(platform_uds_dtcs[1]), (uint8_t)(UDS_DTC_STATUS_TEST_FAILED & AVAIL)};
    TEST_ASSERT_EQUAL_UINT16(sizeof expect, n);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expect, rsp, sizeof expect);
}

static void test_dtc_by_status_mask_skips_dtcs_that_do_not_match(void)
{
    set_dtcs();
    const uint8_t req[] = {UDS_SID_READ_DTC_INFORMATION, UDS_READ_DTC_REPORT_DTC_BY_STATUS_MASK,
                           UDS_DTC_STATUS_CONFIRMED_DTC};
    const uint8_t expect[] = {
        (uint8_t)(UDS_SID_READ_DTC_INFORMATION + UDS_POSITIVE_RESPONSE_OFFSET),
        UDS_READ_DTC_REPORT_DTC_BY_STATUS_MASK, AVAIL,
        GROUP_BYTES(platform_uds_dtcs[0]), AVAIL};
    TEST_ASSERT_EQUAL_UINT16(sizeof expect, PHYS(req));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expect, rsp, sizeof expect);
}

static void test_dtc_by_status_mask_with_no_match_is_a_positive_answer_without_records(void)
{
    const uint8_t req[] = {UDS_SID_READ_DTC_INFORMATION, UDS_READ_DTC_REPORT_DTC_BY_STATUS_MASK,
                           0xFFu};
    TEST_ASSERT_EQUAL_UINT16(3u, PHYS(req));
    TEST_ASSERT_EQUAL_HEX8(AVAIL, rsp[2]);
}

static void test_supported_dtc_report_lists_every_dtc_even_with_status_zero(void)
{
    const uint8_t req[] = {UDS_SID_READ_DTC_INFORMATION, UDS_READ_DTC_REPORT_SUPPORTED_DTC};
    TEST_ASSERT_EQUAL_UINT16(3u + (4u * PLATFORM_UDS_DTC_COUNT), PHYS(req));
    for (uint32_t i = 0u; i < PLATFORM_UDS_DTC_COUNT; i++) {
        const uint16_t p = (uint16_t)(3u + (4u * i));
        TEST_ASSERT_EQUAL_HEX8(((platform_uds_dtcs[i] >> 16u) & 0xFFu), rsp[p]);
        TEST_ASSERT_EQUAL_HEX8(((platform_uds_dtcs[i] >> 8u) & 0xFFu), rsp[p + 1u]);
        TEST_ASSERT_EQUAL_HEX8((platform_uds_dtcs[i] & 0xFFu), rsp[p + 2u]);
        TEST_ASSERT_EQUAL_HEX8(0u, rsp[p + 3u]);
    }
    set_dtcs();
    TEST_ASSERT_EQUAL_UINT16(3u + (4u * PLATFORM_UDS_DTC_COUNT), PHYS(req));
    TEST_ASSERT_EQUAL_HEX8(AVAIL, rsp[3u + 3u]);
    TEST_ASSERT_EQUAL_HEX8((UDS_DTC_STATUS_TEST_FAILED & AVAIL), rsp[3u + 4u + 3u]);
    TEST_ASSERT_EQUAL_HEX8(UDS_READ_DTC_REPORT_SUPPORTED_DTC, rsp[1]);
}

static void test_read_dtc_unsupported_sub_functions_are_nrc_sub_function_not_supported(void)
{
    const uint8_t spr_bit[] = {UDS_SID_READ_DTC_INFORMATION,
                               (uint8_t)(UDS_READ_DTC_REPORT_SUPPORTED_DTC | UDS_SUPPRESS_POS_RSP_BIT)};
    const uint8_t spr_count[] = {UDS_SID_READ_DTC_INFORMATION,
                                 (uint8_t)(UDS_READ_DTC_REPORT_NUMBER_OF_DTC_BY_STATUS_MASK |
                                           UDS_SUPPRESS_POS_RSP_BIT),
                                 0xFFu};
    /* 0x03 is none of the offered sub-functions (0x19 has no suppressPosRsp bit) */
    const uint8_t other[] = {UDS_SID_READ_DTC_INFORMATION,
                             (uint8_t)(UDS_READ_DTC_REPORT_DTC_BY_STATUS_MASK + 1u)};
    assert_nrc(PHYS(spr_bit), UDS_SID_READ_DTC_INFORMATION, UDS_NRC_SUBFUNCTION_NOT_SUPPORTED);
    assert_nrc(PHYS(spr_count), UDS_SID_READ_DTC_INFORMATION, UDS_NRC_SUBFUNCTION_NOT_SUPPORTED);
    assert_nrc(PHYS(other), UDS_SID_READ_DTC_INFORMATION, UDS_NRC_SUBFUNCTION_NOT_SUPPORTED);
}

static void test_read_dtc_wrong_lengths_are_incorrect_length(void)
{
    const uint8_t no_sub[] = {UDS_SID_READ_DTC_INFORMATION};
    const uint8_t count_no_mask[] = {UDS_SID_READ_DTC_INFORMATION,
                                     UDS_READ_DTC_REPORT_NUMBER_OF_DTC_BY_STATUS_MASK};
    const uint8_t count_long[] = {UDS_SID_READ_DTC_INFORMATION,
                                  UDS_READ_DTC_REPORT_NUMBER_OF_DTC_BY_STATUS_MASK, 0xFFu, 0x00u};
    const uint8_t list_no_mask[] = {UDS_SID_READ_DTC_INFORMATION, UDS_READ_DTC_REPORT_DTC_BY_STATUS_MASK};
    const uint8_t supported_mask[] = {UDS_SID_READ_DTC_INFORMATION, UDS_READ_DTC_REPORT_SUPPORTED_DTC,
                                      0xFFu};
    const uint8_t nrc = UDS_NRC_INCORRECT_MESSAGE_LENGTH_OR_INVALID_FORMAT;
    assert_nrc(PHYS(no_sub), UDS_SID_READ_DTC_INFORMATION, nrc);
    assert_nrc(PHYS(count_no_mask), UDS_SID_READ_DTC_INFORMATION, nrc);
    assert_nrc(PHYS(count_long), UDS_SID_READ_DTC_INFORMATION, nrc);
    assert_nrc(PHYS(list_no_mask), UDS_SID_READ_DTC_INFORMATION, nrc);
    assert_nrc(PHYS(supported_mask), UDS_SID_READ_DTC_INFORMATION, nrc);
}

/* ------------------------------------------------------------------ 0x14 */

static void test_clear_all_groups_in_the_extended_session_is_positive_and_clears(void)
{
    set_dtcs();
    enter_extended();
    const uint8_t req[] = {UDS_SID_CLEAR_DIAGNOSTIC_INFORMATION, GROUP_BYTES(UDS_GROUP_OF_DTC_ALL)};
    TEST_ASSERT_EQUAL_UINT16(1u, PHYS(req));
    TEST_ASSERT_EQUAL_HEX8(UDS_SID_CLEAR_DIAGNOSTIC_INFORMATION + UDS_POSITIVE_RESPONSE_OFFSET, rsp[0]);
    TEST_ASSERT_EQUAL_UINT32(1u, fake.clear_calls);
    TEST_ASSERT_EQUAL_HEX8(0u, fake.dtc[0]);
}

static void test_clear_of_another_group_is_request_out_of_range(void)
{
    enter_extended();
    const uint32_t other = UDS_GROUP_OF_DTC_ALL ^ 0x01u;
    const uint8_t req[] = {UDS_SID_CLEAR_DIAGNOSTIC_INFORMATION, GROUP_BYTES(other)};
    assert_nrc(PHYS(req), UDS_SID_CLEAR_DIAGNOSTIC_INFORMATION, UDS_NRC_REQUEST_OUT_OF_RANGE);
    TEST_ASSERT_EQUAL_UINT32(0u, fake.clear_calls);
}

static void test_clear_with_a_wrong_length_is_incorrect_length(void)
{
    enter_extended();
    const uint8_t short_req[] = {UDS_SID_CLEAR_DIAGNOSTIC_INFORMATION, GROUP_BYTES(UDS_GROUP_OF_DTC_ALL)};
    const uint8_t three[] = {UDS_SID_CLEAR_DIAGNOSTIC_INFORMATION, short_req[1], short_req[2]};
    const uint8_t five[] = {UDS_SID_CLEAR_DIAGNOSTIC_INFORMATION, short_req[1], short_req[2],
                            short_req[3], 0x00u};
    assert_nrc(PHYS(three), UDS_SID_CLEAR_DIAGNOSTIC_INFORMATION,
               UDS_NRC_INCORRECT_MESSAGE_LENGTH_OR_INVALID_FORMAT);
    assert_nrc(PHYS(five), UDS_SID_CLEAR_DIAGNOSTIC_INFORMATION,
               UDS_NRC_INCORRECT_MESSAGE_LENGTH_OR_INVALID_FORMAT);
    TEST_ASSERT_EQUAL_UINT32(0u, fake.clear_calls);
}

static void test_clear_with_a_failing_provider_is_conditions_not_correct(void)
{
    enter_extended();
    fake.clear_mode = MODE_FAIL;
    const uint8_t req[] = {UDS_SID_CLEAR_DIAGNOSTIC_INFORMATION, GROUP_BYTES(UDS_GROUP_OF_DTC_ALL)};
    assert_nrc(PHYS(req), UDS_SID_CLEAR_DIAGNOSTIC_INFORMATION, UDS_NRC_CONDITIONS_NOT_CORRECT);
    TEST_ASSERT_FALSE(uds_server_core_pending(&core));
}

/* ------------------------------------------------------------------ functional (7.5) */

static void test_functional_unsupported_service_is_silent_and_counted_suppressed(void)
{
    const uint8_t req[] = {unsupported_sid(), 0x00u};
    TEST_ASSERT_EQUAL_UINT16(0u, FUNC(req));
    TEST_ASSERT_EQUAL_UINT32(1u, uds_server_core_stats(&core)->suppressed);
    TEST_ASSERT_EQUAL_UINT32(0u, uds_server_core_stats(&core)->negative);
}

static void test_functional_unknown_did_out_of_range_is_silent(void)
{
    const uint8_t req[] = {UDS_SID_READ_DATA_BY_IDENTIFIER, DID_BYTES(unknown_did(0u))};
    TEST_ASSERT_EQUAL_UINT16(0u, FUNC(req));
    TEST_ASSERT_EQUAL_UINT32(1u, uds_server_core_stats(&core)->suppressed);
}

static void test_functional_unsupported_sub_function_is_silent(void)
{
    const uint8_t req[] = {UDS_SID_DIAGNOSTIC_SESSION_CONTROL, UDS_SESSION_PROGRAMMING};
    TEST_ASSERT_EQUAL_UINT16(0u, FUNC(req));
    TEST_ASSERT_EQUAL_UINT32(1u, uds_server_core_stats(&core)->suppressed);
}

static void test_functional_service_not_in_active_session_is_silent(void)
{
    const uint8_t req[] = {UDS_SID_CLEAR_DIAGNOSTIC_INFORMATION, GROUP_BYTES(UDS_GROUP_OF_DTC_ALL)};
    TEST_ASSERT_EQUAL_UINT16(0u, FUNC(req));
    TEST_ASSERT_EQUAL_UINT32(1u, uds_server_core_stats(&core)->suppressed);
    TEST_ASSERT_EQUAL_UINT32(0u, fake.clear_calls);
}

static void test_functional_incorrect_length_is_still_answered(void)
{
    const uint8_t req[] = {UDS_SID_READ_DATA_BY_IDENTIFIER, HI(PLATFORM_UDS_DID_SW_VERSION)};
    assert_nrc(FUNC(req), UDS_SID_READ_DATA_BY_IDENTIFIER,
               UDS_NRC_INCORRECT_MESSAGE_LENGTH_OR_INVALID_FORMAT);
    TEST_ASSERT_EQUAL_UINT32(1u, uds_server_core_stats(&core)->negative);
    TEST_ASSERT_EQUAL_UINT32(0u, uds_server_core_stats(&core)->suppressed);
}

static void test_functional_conditions_not_correct_is_still_answered(void)
{
    fake.did_mode = MODE_FAIL;
    const uint8_t req[] = {UDS_SID_READ_DATA_BY_IDENTIFIER, DID_BYTES(PLATFORM_UDS_DID_SW_VERSION)};
    assert_nrc(FUNC(req), UDS_SID_READ_DATA_BY_IDENTIFIER, UDS_NRC_CONDITIONS_NOT_CORRECT);
}

static void test_functional_positive_read_is_answered(void)
{
    const uint8_t req[] = {UDS_SID_READ_DATA_BY_IDENTIFIER,
                           DID_BYTES(PLATFORM_UDS_DID_ACTIVE_DIAGNOSTIC_SESSION)};
    TEST_ASSERT_EQUAL_UINT16(4u, FUNC(req));
    TEST_ASSERT_EQUAL_HEX8(UDS_SESSION_DEFAULT, rsp[3]);
    TEST_ASSERT_EQUAL_UINT32(1u, uds_server_core_stats(&core)->positive);
}

static void test_functional_tester_present_with_suppress_pos_rsp_is_silent(void)
{
    const uint8_t req[] = {UDS_SID_TESTER_PRESENT,
                           (uint8_t)(UDS_TESTER_PRESENT_ZERO_SUBFUNCTION | UDS_SUPPRESS_POS_RSP_BIT)};
    TEST_ASSERT_EQUAL_UINT16(0u, FUNC(req));
}

/* ------------------------------------------------------------------ response pending */

static const uint8_t read_sw[] = {UDS_SID_READ_DATA_BY_IDENTIFIER, DID_BYTES(PLATFORM_UDS_DID_SW_VERSION)};

static void test_pending_physical_gets_nrc_0x78_at_once_then_every_half_p2_star(void)
{
    const uint32_t t0 = now;
    fake.did_mode = MODE_PENDING;
    fake.did_pending_left = NEVER;
    assert_nrc(PHYS(read_sw), UDS_SID_READ_DATA_BY_IDENTIFIER, UDS_NRC_RESPONSE_PENDING);
    TEST_ASSERT_TRUE(uds_server_core_pending(&core));
    TEST_ASSERT_EQUAL_UINT32(1u, uds_server_core_stats(&core)->response_pending);
    TEST_ASSERT_EQUAL_UINT32(0u, uds_server_core_stats(&core)->negative); /* 0x78 is not counted */

    TEST_ASSERT_EQUAL_UINT16(0u, poll_at(t0 + 1u));
    TEST_ASSERT_EQUAL_UINT16(0u, poll_at(t0 + P2_STAR_REPEAT_MS - 1u));
    assert_nrc(poll_at(t0 + P2_STAR_REPEAT_MS), UDS_SID_READ_DATA_BY_IDENTIFIER,
               UDS_NRC_RESPONSE_PENDING);
    TEST_ASSERT_EQUAL_UINT16(0u, poll_at(t0 + P2_STAR_REPEAT_MS + 1u));
    TEST_ASSERT_EQUAL_UINT32(2u, uds_server_core_stats(&core)->response_pending);
    TEST_ASSERT_TRUE(uds_server_core_pending(&core));
}

static void test_pending_request_ends_with_the_final_positive_answer_from_poll(void)
{
    const uint32_t t0 = now;
    fake.did_mode = MODE_PENDING;
    fake.did_pending_left = 1u; /* the request call is the one pending call */
    assert_nrc(PHYS(read_sw), UDS_SID_READ_DATA_BY_IDENTIFIER, UDS_NRC_RESPONSE_PENDING);
    const uint16_t n = poll_at(t0 + 5u);
    TEST_ASSERT_EQUAL_UINT16(3u + platform_uds_dids[PLATFORM_UDS_IDX_SW_VERSION].length, n);
    TEST_ASSERT_EQUAL_HEX8(UDS_SID_READ_DATA_BY_IDENTIFIER + UDS_POSITIVE_RESPONSE_OFFSET, rsp[0]);
    expect_record(&rsp[3], PLATFORM_UDS_IDX_SW_VERSION);
    TEST_ASSERT_FALSE(uds_server_core_pending(&core));
    TEST_ASSERT_EQUAL_UINT32(1u, uds_server_core_stats(&core)->positive);
    TEST_ASSERT_EQUAL_UINT16(0u, poll_at(t0 + 6u));
}

static void test_pending_request_becoming_a_failure_ends_with_nrc_conditions_not_correct(void)
{
    const uint32_t t0 = now;
    fake.did_mode = MODE_PENDING;
    fake.did_pending_left = NEVER;
    assert_nrc(PHYS(read_sw), UDS_SID_READ_DATA_BY_IDENTIFIER, UDS_NRC_RESPONSE_PENDING);
    fake.did_mode = MODE_FAIL;
    assert_nrc(poll_at(t0 + 3u), UDS_SID_READ_DATA_BY_IDENTIFIER, UDS_NRC_CONDITIONS_NOT_CORRECT);
    TEST_ASSERT_FALSE(uds_server_core_pending(&core));
}

static void test_pending_request_still_pending_after_p2_star_ends_with_general_reject(void)
{
    const uint32_t t0 = now;
    fake.did_mode = MODE_PENDING;
    fake.did_pending_left = NEVER;
    assert_nrc(PHYS(read_sw), UDS_SID_READ_DATA_BY_IDENTIFIER, UDS_NRC_RESPONSE_PENDING);
    assert_nrc(poll_at(t0 + P2_STAR_REPEAT_MS), UDS_SID_READ_DATA_BY_IDENTIFIER,
               UDS_NRC_RESPONSE_PENDING);
    TEST_ASSERT_EQUAL_UINT16(0u, poll_at(t0 + PLATFORM_UDS_P2_STAR_SERVER_MAX_MS - 1u));
    TEST_ASSERT_TRUE(uds_server_core_pending(&core));
    assert_nrc(poll_at(t0 + PLATFORM_UDS_P2_STAR_SERVER_MAX_MS), UDS_SID_READ_DATA_BY_IDENTIFIER,
               UDS_NRC_GENERAL_REJECT);
    TEST_ASSERT_FALSE(uds_server_core_pending(&core));
    TEST_ASSERT_EQUAL_UINT32(1u, uds_server_core_stats(&core)->pending_expired);
    TEST_ASSERT_EQUAL_UINT32(1u, uds_server_core_stats(&core)->negative);
    TEST_ASSERT_EQUAL_UINT16(0u, poll_at(t0 + PLATFORM_UDS_P2_STAR_SERVER_MAX_MS + 1u));
    /* the server is free again */
    fake.did_mode = MODE_OK;
    TEST_ASSERT_GREATER_THAN_UINT16(3u, PHYS(read_sw));
}

static void test_pending_expiry_is_measured_across_the_clock_wrap(void)
{
    now = UINT32_MAX - 100u;
    TEST_ASSERT_TRUE(uds_server_core_init(&core, &provider, now));
    const uint32_t t0 = now;
    fake.did_mode = MODE_PENDING;
    fake.did_pending_left = NEVER;
    assert_nrc(PHYS(read_sw), UDS_SID_READ_DATA_BY_IDENTIFIER, UDS_NRC_RESPONSE_PENDING);
    TEST_ASSERT_EQUAL_UINT16(0u, poll_at(t0 + P2_STAR_REPEAT_MS - 1u));
    assert_nrc(poll_at(t0 + PLATFORM_UDS_P2_STAR_SERVER_MAX_MS), UDS_SID_READ_DATA_BY_IDENTIFIER,
               UDS_NRC_GENERAL_REJECT);
}

static void test_pending_clear_is_retried_by_poll_until_the_provider_is_done(void)
{
    enter_extended();
    set_dtcs();
    fake.clear_mode = MODE_PENDING;
    fake.clear_pending_left = 2u;
    const uint8_t req[] = {UDS_SID_CLEAR_DIAGNOSTIC_INFORMATION, GROUP_BYTES(UDS_GROUP_OF_DTC_ALL)};
    assert_nrc(PHYS(req), UDS_SID_CLEAR_DIAGNOSTIC_INFORMATION, UDS_NRC_RESPONSE_PENDING);
    TEST_ASSERT_EQUAL_UINT16(0u, poll_at(now + 1u)); /* second pending call */
    TEST_ASSERT_EQUAL_UINT16(1u, poll_at(now + 1u)); /* done */
    TEST_ASSERT_EQUAL_HEX8(UDS_SID_CLEAR_DIAGNOSTIC_INFORMATION + UDS_POSITIVE_RESPONSE_OFFSET, rsp[0]);
    TEST_ASSERT_EQUAL_UINT32(3u, fake.clear_calls);
    TEST_ASSERT_EQUAL_HEX8(0u, fake.dtc[0]);
}

static void test_pending_functional_gets_nrc_0x78_like_a_physical_request_and_the_final_answer(void)
{
    const uint32_t t0 = now;
    fake.did_mode = MODE_PENDING;
    fake.did_pending_left = NEVER;
    assert_nrc(FUNC(read_sw), UDS_SID_READ_DATA_BY_IDENTIFIER, UDS_NRC_RESPONSE_PENDING);
    TEST_ASSERT_TRUE(uds_server_core_pending(&core));
    TEST_ASSERT_EQUAL_UINT16(0u, poll_at(t0 + P2_STAR_REPEAT_MS - 1u));
    assert_nrc(poll_at(t0 + P2_STAR_REPEAT_MS), UDS_SID_READ_DATA_BY_IDENTIFIER,
               UDS_NRC_RESPONSE_PENDING);
    TEST_ASSERT_EQUAL_UINT32(2u, uds_server_core_stats(&core)->response_pending);
    fake.did_mode = MODE_OK;
    const uint16_t n = poll_at(t0 + P2_STAR_REPEAT_MS + 10u);
    TEST_ASSERT_EQUAL_UINT16(3u + platform_uds_dids[PLATFORM_UDS_IDX_SW_VERSION].length, n);
    TEST_ASSERT_EQUAL_HEX8(UDS_SID_READ_DATA_BY_IDENTIFIER + UDS_POSITIVE_RESPONSE_OFFSET, rsp[0]);
    TEST_ASSERT_FALSE(uds_server_core_pending(&core));
}

static void test_pending_functional_that_expires_ends_with_general_reject(void)
{
    const uint32_t t0 = now;
    fake.did_mode = MODE_PENDING;
    fake.did_pending_left = NEVER;
    assert_nrc(FUNC(read_sw), UDS_SID_READ_DATA_BY_IDENTIFIER, UDS_NRC_RESPONSE_PENDING);
    assert_nrc(poll_at(t0 + PLATFORM_UDS_P2_STAR_SERVER_MAX_MS), UDS_SID_READ_DATA_BY_IDENTIFIER,
               UDS_NRC_GENERAL_REJECT);
    TEST_ASSERT_EQUAL_UINT32(1u, uds_server_core_stats(&core)->pending_expired);
}

/* After an NRC 0x78 the tester waits for the final answer: suppressPosRsp and the
 * functional NRC suppression no longer apply to that request. The gen services that can
 * pend (0x22, 0x14) carry neither, so the finished request is put into the state by
 * hand: pending, with the 0x78 already sent. */
static void arm_pending(const uint8_t* req, uint16_t len, bool functional, bool rp_sent)
{
    memcpy(core.req, req, len);
    core.req_len = len;
    core.pending = true;
    core.pending_functional = functional;
    core.pending_since_ms = now;
    core.last_rp_ms = now;
    core.rp_sent = rp_sent;
}

static void test_final_positive_after_nrc_0x78_ignores_suppress_pos_rsp(void)
{
    const uint8_t tp[] = {UDS_SID_TESTER_PRESENT,
                          (uint8_t)(UDS_TESTER_PRESENT_ZERO_SUBFUNCTION | UDS_SUPPRESS_POS_RSP_BIT)};
    arm_pending(tp, sizeof tp, false, true);
    TEST_ASSERT_EQUAL_UINT16(2u, poll_at(now + 1u));
    TEST_ASSERT_EQUAL_HEX8(UDS_SID_TESTER_PRESENT + UDS_POSITIVE_RESPONSE_OFFSET, rsp[0]);
    TEST_ASSERT_EQUAL_UINT32(0u, uds_server_core_stats(&core)->suppressed);
    /* without a 0x78 having gone out, the same request stays suppressed */
    arm_pending(tp, sizeof tp, false, false);
    TEST_ASSERT_EQUAL_UINT16(0u, poll_at(now + 1u));
    TEST_ASSERT_EQUAL_UINT32(1u, uds_server_core_stats(&core)->suppressed);
}

static void test_final_functional_nrc_after_nrc_0x78_is_sent_despite_the_suppression_list(void)
{
    const uint8_t bad[] = {unsupported_sid(), 0x00u};
    arm_pending(bad, sizeof bad, true, true);
    assert_nrc(poll_at(now + 1u), bad[0], UDS_NRC_SERVICE_NOT_SUPPORTED);
    TEST_ASSERT_EQUAL_UINT32(0u, uds_server_core_stats(&core)->suppressed);
    arm_pending(bad, sizeof bad, true, false);
    TEST_ASSERT_EQUAL_UINT16(0u, poll_at(now + 1u));
    TEST_ASSERT_EQUAL_UINT32(1u, uds_server_core_stats(&core)->suppressed);
}

static void test_the_answer_obligation_ends_with_the_pending_request(void)
{
    fake.did_mode = MODE_PENDING;
    fake.did_pending_left = 1u;
    assert_nrc(FUNC(read_sw), UDS_SID_READ_DATA_BY_IDENTIFIER, UDS_NRC_RESPONSE_PENDING);
    TEST_ASSERT_GREATER_THAN_UINT16(0u, poll_at(now + 1u));
    const uint8_t bad[] = {unsupported_sid(), 0x00u};
    TEST_ASSERT_EQUAL_UINT16(0u, FUNC(bad)); /* suppressed again */
    TEST_ASSERT_FALSE(core.rp_sent);
}

static void test_a_physical_request_while_pending_is_busy_repeat_request(void)
{
    fake.did_mode = MODE_PENDING;
    fake.did_pending_left = NEVER;
    assert_nrc(PHYS(read_sw), UDS_SID_READ_DATA_BY_IDENTIFIER, UDS_NRC_RESPONSE_PENDING);
    const uint8_t tp[] = {UDS_SID_TESTER_PRESENT, UDS_TESTER_PRESENT_ZERO_SUBFUNCTION};
    assert_nrc(PHYS(tp), UDS_SID_TESTER_PRESENT, UDS_NRC_BUSY_REPEAT_REQUEST);
    TEST_ASSERT_EQUAL_UINT32(1u, uds_server_core_stats(&core)->busy);
    TEST_ASSERT_TRUE(uds_server_core_pending(&core));
}

static void test_a_functional_request_while_pending_is_dropped(void)
{
    fake.did_mode = MODE_PENDING;
    fake.did_pending_left = NEVER;
    assert_nrc(PHYS(read_sw), UDS_SID_READ_DATA_BY_IDENTIFIER, UDS_NRC_RESPONSE_PENDING);
    const uint8_t tp[] = {UDS_SID_TESTER_PRESENT, UDS_TESTER_PRESENT_ZERO_SUBFUNCTION};
    TEST_ASSERT_EQUAL_UINT16(0u, FUNC(tp));
    TEST_ASSERT_EQUAL_UINT32(1u, uds_server_core_stats(&core)->busy);
    TEST_ASSERT_TRUE(uds_server_core_pending(&core));
}

static void test_tester_present_with_suppress_pos_rsp_while_pending_is_silent_and_counted_busy(void)
{
    fake.did_mode = MODE_PENDING;
    fake.did_pending_left = NEVER;
    assert_nrc(PHYS(read_sw), UDS_SID_READ_DATA_BY_IDENTIFIER, UDS_NRC_RESPONSE_PENDING);
    const uint8_t tp[] = {UDS_SID_TESTER_PRESENT,
                          (uint8_t)(UDS_TESTER_PRESENT_ZERO_SUBFUNCTION | UDS_SUPPRESS_POS_RSP_BIT)};
    TEST_ASSERT_EQUAL_UINT16(0u, PHYS(tp));
    TEST_ASSERT_EQUAL_UINT32(1u, uds_server_core_stats(&core)->busy);
    TEST_ASSERT_TRUE(uds_server_core_pending(&core));
}

/* ------------------------------------------------------------------ S3 (ISO 14229-2) */

static void test_extended_session_falls_back_to_default_after_s3(void)
{
    const uint32_t t0 = now;
    enter_extended();
    TEST_ASSERT_EQUAL_UINT16(0u, poll_at(t0 + PLATFORM_UDS_S3_SERVER_MS - 1u));
    TEST_ASSERT_EQUAL_UINT8(UDS_SESSION_EXTENDED, uds_server_core_session(&core));
    TEST_ASSERT_EQUAL_UINT32(0u, uds_server_core_stats(&core)->s3_timeouts);
    TEST_ASSERT_EQUAL_UINT16(0u, poll_at(t0 + PLATFORM_UDS_S3_SERVER_MS));
    TEST_ASSERT_EQUAL_UINT8(UDS_SESSION_DEFAULT, uds_server_core_session(&core));
    TEST_ASSERT_EQUAL_UINT32(1u, uds_server_core_stats(&core)->s3_timeouts);
    TEST_ASSERT_EQUAL_UINT16(0u, poll_at(t0 + (2u * PLATFORM_UDS_S3_SERVER_MS)));
    TEST_ASSERT_EQUAL_UINT32(1u, uds_server_core_stats(&core)->s3_timeouts); /* counted once */
}

static void test_the_default_session_never_times_out(void)
{
    TEST_ASSERT_EQUAL_UINT16(0u, poll_at(now + (10u * PLATFORM_UDS_S3_SERVER_MS)));
    TEST_ASSERT_EQUAL_UINT32(0u, uds_server_core_stats(&core)->s3_timeouts);
}

static void test_any_request_restarts_s3_including_a_suppressed_tester_present(void)
{
    const uint32_t t0 = now;
    enter_extended();
    now = t0 + PLATFORM_UDS_S3_SERVER_MS - 1u;
    const uint8_t tp[] = {UDS_SID_TESTER_PRESENT,
                          (uint8_t)(UDS_TESTER_PRESENT_ZERO_SUBFUNCTION | UDS_SUPPRESS_POS_RSP_BIT)};
    TEST_ASSERT_EQUAL_UINT16(0u, PHYS(tp));
    TEST_ASSERT_EQUAL_UINT16(0u, poll_at(t0 + PLATFORM_UDS_S3_SERVER_MS));
    TEST_ASSERT_EQUAL_UINT8(UDS_SESSION_EXTENDED, uds_server_core_session(&core));
    TEST_ASSERT_EQUAL_UINT16(0u, poll_at(t0 + PLATFORM_UDS_S3_SERVER_MS - 1u + PLATFORM_UDS_S3_SERVER_MS));
    TEST_ASSERT_EQUAL_UINT8(UDS_SESSION_DEFAULT, uds_server_core_session(&core));
}

static void test_a_pending_request_holds_the_session_until_it_ends(void)
{
    enter_extended();
    const uint32_t t0 = now;
    fake.did_mode = MODE_PENDING;
    fake.did_pending_left = NEVER;
    assert_nrc(PHYS(read_sw), UDS_SID_READ_DATA_BY_IDENTIFIER, UDS_NRC_RESPONSE_PENDING);
    /* polls every half P2-star across more than S3 in total: the session must stay */
    uint32_t t = t0;
    bool expired = false;
    for (uint32_t i = 0u; (i < 10u) && !expired; i++) {
        t += P2_STAR_REPEAT_MS;
        const uint16_t n = poll_at(t);
        TEST_ASSERT_EQUAL_UINT8(UDS_SESSION_EXTENDED, uds_server_core_session(&core));
        expired = !uds_server_core_pending(&core);
        TEST_ASSERT_TRUE((n == 0u) || (n == UDS_NEGATIVE_RESPONSE_LEN));
    }
    TEST_ASSERT_TRUE(expired);
    /* after the request ended, S3 counts from its last poll */
    TEST_ASSERT_EQUAL_UINT16(0u, poll_at(t + PLATFORM_UDS_S3_SERVER_MS - 1u));
    TEST_ASSERT_EQUAL_UINT8(UDS_SESSION_EXTENDED, uds_server_core_session(&core));
    TEST_ASSERT_EQUAL_UINT16(0u, poll_at(t + (2u * PLATFORM_UDS_S3_SERVER_MS)));
    TEST_ASSERT_EQUAL_UINT8(UDS_SESSION_DEFAULT, uds_server_core_session(&core));
}

static void test_s3_is_measured_across_the_clock_wrap(void)
{
    now = UINT32_MAX - 500u;
    TEST_ASSERT_TRUE(uds_server_core_init(&core, &provider, now));
    const uint32_t t0 = now;
    enter_extended();
    TEST_ASSERT_EQUAL_UINT16(0u, poll_at(t0 + PLATFORM_UDS_S3_SERVER_MS - 1u));
    TEST_ASSERT_LESS_THAN_UINT32(t0, now); /* the counter wrapped */
    TEST_ASSERT_EQUAL_UINT8(UDS_SESSION_EXTENDED, uds_server_core_session(&core));
    TEST_ASSERT_EQUAL_UINT16(0u, poll_at(t0 + PLATFORM_UDS_S3_SERVER_MS));
    TEST_ASSERT_EQUAL_UINT8(UDS_SESSION_DEFAULT, uds_server_core_session(&core));
    TEST_ASSERT_EQUAL_UINT32(1u, uds_server_core_stats(&core)->s3_timeouts);
}

static void test_a_session_reset_by_s3_makes_extended_only_services_unavailable(void)
{
    const uint32_t t0 = now;
    enter_extended();
    (void)poll_at(t0 + PLATFORM_UDS_S3_SERVER_MS);
    const uint8_t req[] = {UDS_SID_CLEAR_DIAGNOSTIC_INFORMATION, GROUP_BYTES(UDS_GROUP_OF_DTC_ALL)};
    assert_nrc(PHYS(req), UDS_SID_CLEAR_DIAGNOSTIC_INFORMATION,
               UDS_NRC_SERVICE_NOT_SUPPORTED_IN_ACTIVE_SESSION);
}

/* ------------------------------------------------------------------ length limits, tick */

static void test_read_of_the_same_did_max_read_dids_times_gives_that_many_records(void)
{
    uint8_t req[1u + (2u * PLATFORM_UDS_MAX_READ_DIDS)];
    req[0] = UDS_SID_READ_DATA_BY_IDENTIFIER;
    for (uint32_t i = 0u; i < PLATFORM_UDS_MAX_READ_DIDS; i++) {
        req[1u + (2u * i)] = HI(PLATFORM_UDS_DID_ACTIVE_DIAGNOSTIC_SESSION);
        req[2u + (2u * i)] = LO(PLATFORM_UDS_DID_ACTIVE_DIAGNOSTIC_SESSION);
    }
    const uint16_t rec = 2u + PLATFORM_UDS_DID_ACTIVE_DIAGNOSTIC_SESSION_LENGTH;
    const uint16_t n = request(req, (uint16_t)sizeof req, false);
    TEST_ASSERT_EQUAL_UINT16(1u + (PLATFORM_UDS_MAX_READ_DIDS * rec), n);
    for (uint32_t i = 0u; i < PLATFORM_UDS_MAX_READ_DIDS; i++) {
        const uint16_t p = (uint16_t)(1u + (i * rec));
        TEST_ASSERT_EQUAL_HEX8(HI(PLATFORM_UDS_DID_ACTIVE_DIAGNOSTIC_SESSION), rsp[p]);
        TEST_ASSERT_EQUAL_HEX8(LO(PLATFORM_UDS_DID_ACTIVE_DIAGNOSTIC_SESSION), rsp[p + 1u]);
        TEST_ASSERT_EQUAL_HEX8(UDS_SESSION_DEFAULT, rsp[p + 2u]);
    }
}

static void test_read_of_the_longest_dids_fills_exactly_the_longest_response(void)
{
    uint32_t longest = 0u;
    for (uint32_t i = 0u; i < PLATFORM_UDS_DID_COUNT; i++) {
        if (platform_uds_dids[i].length > platform_uds_dids[longest].length) {
            longest = i;
        }
    }
    TEST_ASSERT_EQUAL_UINT8(PLATFORM_UDS_MAX_DID_LENGTH, platform_uds_dids[longest].length);
    TEST_ASSERT_EQUAL_UINT8(PLATFORM_UDS_DID_SW_VERSION_LENGTH, platform_uds_dids[longest].length);
    uint8_t req[1u + (2u * PLATFORM_UDS_MAX_READ_DIDS)];
    req[0] = UDS_SID_READ_DATA_BY_IDENTIFIER;
    for (uint32_t i = 0u; i < PLATFORM_UDS_MAX_READ_DIDS; i++) {
        req[1u + (2u * i)] = HI(platform_uds_dids[longest].did);
        req[2u + (2u * i)] = LO(platform_uds_dids[longest].did);
    }
    const uint16_t n = request(req, (uint16_t)sizeof req, false);
    TEST_ASSERT_EQUAL_UINT16(UDS_SERVER_RSP_MAX, n);
    TEST_ASSERT_EQUAL_HEX8(UDS_SID_READ_DATA_BY_IDENTIFIER + UDS_POSITIVE_RESPONSE_OFFSET, rsp[0]);
    const uint16_t rec = (uint16_t)(2u + platform_uds_dids[longest].length);
    for (uint32_t i = 0u; i < PLATFORM_UDS_MAX_READ_DIDS; i++) {
        expect_record(&rsp[1u + (i * rec) + 2u], longest);
    }
}

static void test_tick_falls_back_to_the_default_session_after_s3_without_a_poll(void)
{
    const uint32_t t0 = now;
    enter_extended();
    uds_server_core_tick(&core, t0 + PLATFORM_UDS_S3_SERVER_MS - 1u);
    TEST_ASSERT_EQUAL_UINT8(UDS_SESSION_EXTENDED, uds_server_core_session(&core));
    uds_server_core_tick(&core, t0 + PLATFORM_UDS_S3_SERVER_MS);
    TEST_ASSERT_EQUAL_UINT8(UDS_SESSION_DEFAULT, uds_server_core_session(&core));
    TEST_ASSERT_EQUAL_UINT32(1u, uds_server_core_stats(&core)->s3_timeouts);
    uds_server_core_tick(&core, t0 + (3u * PLATFORM_UDS_S3_SERVER_MS));
    TEST_ASSERT_EQUAL_UINT32(1u, uds_server_core_stats(&core)->s3_timeouts);
}

static void test_tick_is_a_no_op_while_a_request_is_pending_and_for_null(void)
{
    enter_extended();
    const uint32_t t0 = now;
    fake.did_mode = MODE_PENDING;
    fake.did_pending_left = NEVER;
    assert_nrc(PHYS(read_sw), UDS_SID_READ_DATA_BY_IDENTIFIER, UDS_NRC_RESPONSE_PENDING);
    uds_server_core_tick(&core, t0 + (2u * PLATFORM_UDS_S3_SERVER_MS));
    TEST_ASSERT_EQUAL_UINT8(UDS_SESSION_EXTENDED, uds_server_core_session(&core));
    TEST_ASSERT_EQUAL_UINT32(0u, uds_server_core_stats(&core)->s3_timeouts);
    uds_server_core_tick(NULL, now);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_init_rejects_a_null_core_or_provider_or_function_pointer);
    RUN_TEST(test_init_starts_in_the_default_session_with_zeroed_stats);
    RUN_TEST(test_null_and_malformed_arguments_are_dropped_and_not_counted);
    RUN_TEST(test_a_request_of_exactly_the_rx_buffer_is_handled_and_one_more_is_dropped);
    RUN_TEST(test_a_response_buffer_smaller_than_the_longest_response_is_refused);
    RUN_TEST(test_null_core_getters_are_safe);
    RUN_TEST(test_an_unsupported_sid_is_nrc_service_not_supported);
    RUN_TEST(test_a_sid_in_the_response_range_is_nrc_service_not_supported);
    RUN_TEST(test_clear_in_the_default_session_is_nrc_service_not_supported_in_active_session);
    RUN_TEST(test_session_control_length_and_sub_function_nrcs_in_order);
    RUN_TEST(test_tester_present_length_and_sub_function_nrcs_in_order);
    RUN_TEST(test_extended_session_response_carries_p2_and_p2_star_from_gen);
    RUN_TEST(test_switching_back_to_the_default_session_is_positive);
    RUN_TEST(test_session_control_with_suppress_pos_rsp_changes_the_session_and_sends_nothing);
    RUN_TEST(test_tester_present_zero_sub_function_answers_with_the_sub_function);
    RUN_TEST(test_tester_present_with_suppress_pos_rsp_is_silent_and_counted);
    RUN_TEST(test_read_active_session_did_is_served_by_the_core_in_both_sessions);
    RUN_TEST(test_read_of_a_provider_did_returns_the_record_after_the_did_echo);
    RUN_TEST(test_read_of_the_maximum_number_of_dids_concatenates_records_in_request_order);
    RUN_TEST(test_read_of_more_dids_than_the_maximum_is_incorrect_length);
    RUN_TEST(test_read_with_a_partial_or_missing_did_is_incorrect_length);
    RUN_TEST(test_read_of_only_unknown_dids_is_request_out_of_range);
    RUN_TEST(test_read_of_known_and_unknown_dids_leaves_the_unknown_one_out);
    RUN_TEST(test_read_with_a_failing_provider_is_conditions_not_correct);
    RUN_TEST(test_dtc_count_by_status_mask_counts_only_dtcs_matching_mask_and_availability);
    RUN_TEST(test_dtc_by_status_mask_lists_three_byte_dtc_and_masked_status);
    RUN_TEST(test_dtc_by_status_mask_skips_dtcs_that_do_not_match);
    RUN_TEST(test_dtc_by_status_mask_with_no_match_is_a_positive_answer_without_records);
    RUN_TEST(test_supported_dtc_report_lists_every_dtc_even_with_status_zero);
    RUN_TEST(test_read_dtc_unsupported_sub_functions_are_nrc_sub_function_not_supported);
    RUN_TEST(test_read_dtc_wrong_lengths_are_incorrect_length);
    RUN_TEST(test_clear_all_groups_in_the_extended_session_is_positive_and_clears);
    RUN_TEST(test_clear_of_another_group_is_request_out_of_range);
    RUN_TEST(test_clear_with_a_wrong_length_is_incorrect_length);
    RUN_TEST(test_clear_with_a_failing_provider_is_conditions_not_correct);
    RUN_TEST(test_functional_unsupported_service_is_silent_and_counted_suppressed);
    RUN_TEST(test_functional_unknown_did_out_of_range_is_silent);
    RUN_TEST(test_functional_unsupported_sub_function_is_silent);
    RUN_TEST(test_functional_service_not_in_active_session_is_silent);
    RUN_TEST(test_functional_incorrect_length_is_still_answered);
    RUN_TEST(test_functional_conditions_not_correct_is_still_answered);
    RUN_TEST(test_functional_positive_read_is_answered);
    RUN_TEST(test_functional_tester_present_with_suppress_pos_rsp_is_silent);
    RUN_TEST(test_pending_physical_gets_nrc_0x78_at_once_then_every_half_p2_star);
    RUN_TEST(test_pending_request_ends_with_the_final_positive_answer_from_poll);
    RUN_TEST(test_pending_request_becoming_a_failure_ends_with_nrc_conditions_not_correct);
    RUN_TEST(test_pending_request_still_pending_after_p2_star_ends_with_general_reject);
    RUN_TEST(test_pending_expiry_is_measured_across_the_clock_wrap);
    RUN_TEST(test_pending_clear_is_retried_by_poll_until_the_provider_is_done);
    RUN_TEST(test_pending_functional_gets_nrc_0x78_like_a_physical_request_and_the_final_answer);
    RUN_TEST(test_final_positive_after_nrc_0x78_ignores_suppress_pos_rsp);
    RUN_TEST(test_final_functional_nrc_after_nrc_0x78_is_sent_despite_the_suppression_list);
    RUN_TEST(test_the_answer_obligation_ends_with_the_pending_request);
    RUN_TEST(test_pending_functional_that_expires_ends_with_general_reject);
    RUN_TEST(test_a_physical_request_while_pending_is_busy_repeat_request);
    RUN_TEST(test_a_functional_request_while_pending_is_dropped);
    RUN_TEST(test_tester_present_with_suppress_pos_rsp_while_pending_is_silent_and_counted_busy);
    RUN_TEST(test_extended_session_falls_back_to_default_after_s3);
    RUN_TEST(test_the_default_session_never_times_out);
    RUN_TEST(test_any_request_restarts_s3_including_a_suppressed_tester_present);
    RUN_TEST(test_a_pending_request_holds_the_session_until_it_ends);
    RUN_TEST(test_s3_is_measured_across_the_clock_wrap);
    RUN_TEST(test_a_session_reset_by_s3_makes_extended_only_services_unavailable);
    RUN_TEST(test_read_of_the_same_did_max_read_dids_times_gives_that_many_records);
    RUN_TEST(test_read_of_the_longest_dids_fills_exactly_the_longest_response);
    RUN_TEST(test_tick_falls_back_to_the_default_session_after_s3_without_a_poll);
    RUN_TEST(test_tick_is_a_no_op_while_a_request_is_pending_and_for_null);
    return UNITY_END();
}
