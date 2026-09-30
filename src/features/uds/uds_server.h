#ifndef UDS_SERVER_H
#define UDS_SERVER_H

/*
 * UDS server glue (thesis deliverable Ç3): runs uds_server_core on the PLATFORM bus
 * (FDCAN2) only. Nothing here opens, reads or writes the vehicle bus (D-020, D-037):
 * the link is opened with isotp_link_open(), which refuses CAN_PORT_VEHICLE, and every
 * ID comes from gen/platform_uds.h (D-040).
 *
 *   physical   PLATFORM_UDS_PHYS_REQUEST_ID -> PLATFORM_UDS_PHYS_RESPONSE_ID, ISO-TP
 *              (segmented responses and requests), gen/ padding, BS, STmin, N_Bs, N_Cr
 *   functional PLATFORM_UDS_FUNCTIONAL_REQUEST_ID, Single Frames only (ISO 15765-2
 *              clause 9.6: functional addressing carries no segmented message); the
 *              answer goes out physically on the response ID
 *
 * Arbitration: one request at a time. A physical request is taken when the link has
 * one. A functional request is taken only while the server is idle (nothing pending,
 * no answer queued, no physical reception or transmission running); otherwise it is
 * dropped and counted. A functional Single Frame that arrives while one is held is
 * dropped too.
 *
 * Data (provider of the core):
 *   0xF189 SW version   MOTO_RTCORE_VERSION from the build (CMake project VERSION)
 *   0xFD00 tester state services/diag (written by the UDS client glue)
 *   0xFD01 uptime       seconds since uds_server_open()
 *   0xFD1x samples      services/vehicle_signals, never features/
 *   DTCs, 0x14          services/diag; a clear resets DTC records only
 *
 * Queued answers: an answer the link cannot take yet (ISOTP_ERR_BUSY) is retried on the
 * next pass; S3 keeps running meanwhile. It is dropped (tx_expired) once it is
 * P2*server old, so the glue never delivers a stale answer later. Any other link
 * error drops it at once (tx_failed). An answer the link has already accepted can
 * still go out late after a bus stall until N_As exists (Ç1 FDCAN HAL). A functional
 * request that arrives while an answer is queued is dropped, never run late.
 *
 * Every pass also runs diag_supervise(): a missing or stale vehicle-tester status fails
 * VEHICLE_TESTER_LATCHED and reads NOT_RUNNING in 0xFD00 (fail-safe, D-040).
 *
 * Per main-loop pass, after can_if_dispatch(CAN_PORT_PLATFORM, ...):
 *   uds_server_step(server)
 */

#include "features/uds/isotp_link.h"
#include "features/uds/uds_server_core.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t functional_taken;
    uint32_t functional_dropped; /* busy, malformed or not a Single Frame */
    uint32_t tx_busy;            /* passes an answer waited for the link */
    uint32_t tx_failed;          /* answers refused by the link or not delivered (confirm) */
    uint32_t tx_expired;         /* queued answers dropped after P2*server (dead bus) */
    uint32_t rx_errors;          /* physical receptions that failed (N_Cr, SN, ...) */
} uds_server_glue_stats_t;

/* Treat as opaque. Must have static storage duration (can_if receivers). */
typedef struct {
    isotp_can_link_t link;
    uint8_t rx_buf[PLATFORM_UDS_RX_BUFFER];
    uint8_t tx_buf[UDS_SERVER_RSP_MAX];
    uds_server_core_t core;
    uds_server_provider_t provider;
    uint8_t rsp[UDS_SERVER_RSP_MAX];
    uint16_t rsp_len;           /* > 0: an answer waits for the link */
    uint32_t rsp_since_ms;      /* when that answer was produced */
    uint8_t func_req[ISOTP_SF_MAX_LEN];
    uint8_t func_len;           /* > 0: a functional request is held */
    uint32_t open_ms;
    uds_server_glue_stats_t stats;
    bool open;
} uds_server_t;

/* Opens the physical link on CAN_PORT_PLATFORM and registers the functional receiver.
 * can_if_init() must have run; call diag_init() just before the main loop. ISOTP_ERR_ARG
 * if a receiver cannot be registered (the server then stays closed), ISOTP_ERR_BUSY if
 * already open. */
isotp_status_t uds_server_open(uds_server_t* server);

void uds_server_step(uds_server_t* server);

/* The active diagnostic session (UDS_SESSION_*); default when not open. */
uint8_t uds_server_session(const uds_server_t* server);

const uds_server_stats_t* uds_server_stats(const uds_server_t* server);
const uds_server_glue_stats_t* uds_server_glue_stats(const uds_server_t* server);

#ifdef __cplusplus
}
#endif

#endif /* UDS_SERVER_H */
