#ifndef UDS_SERVER_CORE_H
#define UDS_SERVER_CORE_H

/*
 * UDS server core (thesis deliverable Ç3, ISO 14229-1:2020): the pure state machine of
 * rt-core's diagnostic server on the platform bus (D-040). No HAL, services or RTOS:
 * time is passed in, data comes through a provider table, and the caller moves the
 * bytes over ISO-TP. Tested in tests/host/test_uds_server_core.c.
 *
 * Everything the server offers comes from gen/ (platform_uds.h, uds_iso14229.h):
 * services and their sessions, 0x10 / 0x3E / 0x19 sub-functions, DIDs, DTCs, the
 * DTC status availability mask, P2 / P2* / S3 and the functional NRC suppression list.
 *
 * Services (clause):
 *   0x10 DiagnosticSessionControl (10.2)  default / extended; programming -> NRC 0x12
 *   0x3E TesterPresent (10.6)             zeroSubFunction, suppressPosRsp honoured
 *   0x22 ReadDataByIdentifier (11.2)      1..PLATFORM_UDS_MAX_READ_DIDS DIDs
 *   0x19 ReadDTCInformation (12.3)        0x01, 0x02, 0x0A
 *   0x14 ClearDiagnosticInformation (12.2) group 0xFFFFFF only, extended session only
 *
 * NRC order (clause 7.5, general server response behaviour): 0x11 service not
 * supported, 0x7F not in the active session, 0x13 too short for a sub-function, 0x12
 * sub-function not supported, 0x13 wrong length, 0x31 out of range, 0x22 conditions
 * not correct (the provider failed), 0x14 response too long.
 *
 * Functional requests: NRC 0x11 / 0x12 / 0x31 / 0x7E / 0x7F are not sent (gen/ list,
 * clause 7.5). A positive response is not sent when the request set
 * suppressPosRspMsgIndicationBit (0x10, 0x3E). Once NRC 0x78 went out for a request,
 * its final answer is always sent, positive or negative, physical or functional
 * (clause 7.5 / 8.7.3: after 0x78 the tester waits for it).
 *
 * Timing (ISO 14229-2):
 *   - The answer is produced in the same call as the request, well inside P2.
 *   - A provider may report PENDING (for example a future flash-backed DTC clear). The
 *     request then gets NRC 0x78 at once, and again every P2* / 2 while it is pending. uds_server_core_poll() retries the request every call. If it is still
 *     pending P2* after it arrived, it ends with NRC 0x10 (generalReject), so one
 *     request can never hold the server.
 *   - A new request while one is pending gets NRC 0x21 (busyRepeatRequest) when it is
 *     physical and is dropped when it is functional. 0x3E with suppressPosRsp only
 *     restarts S3.
 *   - S3: a non-default session with no request for PLATFORM_UDS_S3_SERVER_MS falls
 *     back to the default session. Every request restarts S3; a pending request holds
 *     it.
 *
 * Memory: static, caller-owned uds_server_core_t (the request copy for a pending
 * request is PLATFORM_UDS_RX_BUFFER bytes). No heap, no recursion, loops bounded by
 * the gen/ table sizes.
 */

#include "platform_uds.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UDS_SERVER_MAX2(a, b) (((a) > (b)) ? (a) : (b))
/* Longest response: 0x22 with MAX_READ_DIDS of the longest DID, 0x19 with every DTC,
 * or the 6-byte 0x50 / 0x59 0x01 answers. */
#define UDS_SERVER_RSP_READ_MAX \
    (1u + (PLATFORM_UDS_MAX_READ_DIDS * (2u + PLATFORM_UDS_MAX_DID_LENGTH)))
#define UDS_SERVER_RSP_DTC_MAX (3u + (4u * PLATFORM_UDS_DTC_COUNT))
#define UDS_SERVER_RSP_MAX \
    UDS_SERVER_MAX2(UDS_SERVER_MAX2(UDS_SERVER_RSP_READ_MAX, UDS_SERVER_RSP_DTC_MAX), 6u)

typedef enum {
    UDS_SERVER_DATA_OK = 0,
    UDS_SERVER_DATA_PENDING, /* not ready yet: the request is retried (NRC 0x78) */
    UDS_SERVER_DATA_FAIL     /* NRC 0x22 conditionsNotCorrect */
} uds_server_data_t;

/* Data access; every function must be non-NULL. Not called from an ISR. */
typedef struct {
    /* Fills the `len`-byte data record of platform_uds_dids[idx]. Never called for
     * ACTIVE_DIAGNOSTIC_SESSION, which the core serves itself. */
    uds_server_data_t (*read_did)(const void* ctx, uint32_t idx, uint8_t* out, uint16_t len);
    /* DTCStatusMask of platform_uds_dtcs[idx]. */
    uint8_t (*dtc_status)(const void* ctx, uint32_t idx);
    /* Clears every DTC record (group 0xFFFFFF). PENDING: called again until done. */
    uds_server_data_t (*clear_dtcs)(void* ctx);
    void* ctx;
} uds_server_provider_t;

typedef struct {
    uint32_t requests;       /* requests handled (including busy and dropped ones) */
    uint32_t positive;       /* positive responses produced */
    uint32_t negative;       /* NRCs produced, excluding 0x78 */
    uint32_t suppressed;     /* answers not sent (suppressPosRsp or functional NRC) */
    uint32_t response_pending; /* NRC 0x78 produced */
    uint32_t pending_expired;  /* pending requests ended with NRC 0x10 */
    uint32_t busy;           /* requests refused or dropped while one was pending */
    uint32_t s3_timeouts;    /* fall-backs to the default session */
} uds_server_stats_t;

/* Treat as opaque. */
typedef struct {
    const uds_server_provider_t* provider;
    uint8_t session;
    uint32_t s3_start_ms;
    bool pending;
    bool pending_functional;
    bool rp_sent;              /* NRC 0x78 went out: the final answer is always sent */
    uint32_t pending_since_ms; /* when the pending request arrived */
    uint32_t last_rp_ms;       /* last NRC 0x78 */
    uint8_t req[PLATFORM_UDS_RX_BUFFER];
    uint16_t req_len;
    uds_server_stats_t stats;
} uds_server_core_t;

/* Default session, nothing pending. False (core unusable) if a provider function or
 * the provider itself is NULL. */
bool uds_server_core_init(uds_server_core_t* core, const uds_server_provider_t* provider,
                          uint32_t now_ms);

/*
 * One complete request (N_USData.indication, len bytes from the SID). Writes the
 * answer to rsp (capacity cap, at least UDS_SERVER_RSP_MAX) and returns its length;
 * 0 means nothing is sent. Requests longer than PLATFORM_UDS_RX_BUFFER and empty
 * requests are dropped.
 */
uint16_t uds_server_core_on_request(uds_server_core_t* core, uint32_t now_ms,
                                    const uint8_t* req, uint16_t len, bool functional,
                                    uint8_t* rsp, uint16_t cap);

/* Call every pass: S3, and the retry / NRC 0x78 / expiry of a pending request.
 * Returns the length of an answer to send now (0: none), like on_request. */

uint16_t uds_server_core_poll(uds_server_core_t* core, uint32_t now_ms, uint8_t* rsp,
                              uint16_t cap);

/* The S3 timer alone, for a pass in which the caller cannot take an answer (its last
 * one is still queued): the session still falls back on time. poll() includes it. */
void uds_server_core_tick(uds_server_core_t* core, uint32_t now_ms);

/* The active session (UDS_SESSION_*). */
uint8_t uds_server_core_session(const uds_server_core_t* core);

bool uds_server_core_pending(const uds_server_core_t* core);

const uds_server_stats_t* uds_server_core_stats(const uds_server_core_t* core);

#ifdef __cplusplus
}
#endif

#endif /* UDS_SERVER_CORE_H */
