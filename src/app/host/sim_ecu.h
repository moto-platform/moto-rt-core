#ifndef APP_HOST_SIM_ECU_H
#define APP_HOST_SIM_ECU_H

/*
 * Minimal simulated CL250 engine ECU for the host SIL demo and tests, on the
 * in-process bus, using the DID table from gen/vehicle_cl250.h:
 *   - 0x22 ReadDataByIdentifier: values sweep slowly across each DID's [min, max].
 *   - 0x10 DiagnosticSessionControl 0x01/0x03 and 0x3E TesterPresent (suppress bit
 *     honoured). The extended session falls back to default after SIM_ECU_S3_MS without
 *     a request (ISO 14229-2 S3server).
 *   - With require_session, 0x22 is refused with NRC 0x7F outside the extended session,
 *     like the real ECU needs the extended session for its reads (D-019).
 *   - Anything else gets a negative response.
 * Fault hooks for tests: silent, NRC 0x78 bursts, a segmented (First Frame) answer, a
 * fixed NRC for one DID, and a session drop. It is a stand-in until the live vehicle
 * model of moto-hil-bench (D-035) exists; not firmware.
 */

#include "features/uds/isotp_core.h"
#include "hal/host/vbus.h"

#include <stdbool.h>
#include <stdint.h>

#define SIM_ECU_BUF 64u
#define SIM_ECU_S3_MS 5000u            /* ISO 14229-2 default S3server */
#define SIM_ECU_SEGMENTED_LEN 20u      /* length of the segmented answer (First Frame) */

typedef struct {
    vbus_t* bus;
    uint8_t node;
    isotp_link_t iso;
    uint8_t rx_buf[SIM_ECU_BUF];
    uint8_t tx_buf[SIM_ECU_BUF];
    uint32_t requests;          /* every request received */
    uint32_t reads;             /* 0x22 requests */
    uint32_t session_requests;  /* 0x10 requests */
    uint32_t tester_presents;   /* 0x3E requests */
    bool extended;              /* current session */
    uint32_t last_request_ms;

    /* Test hooks. */
    bool silent;                /* ignore requests (ECU off) */
    bool require_session;       /* 0x22 only in the extended session */
    uint8_t pending_count;      /* answer the next reads with this many NRC 0x78 first */
    uint32_t pending_interval_ms;
    uint16_t pending_did;       /* 0: every read, else only this DID's reads */
    bool segmented_enabled;     /* answer segmented_did with a First Frame */
    uint16_t segmented_did;
    bool nrc_enabled;           /* answer nrc_did with NRC nrc_code */
    uint16_t nrc_did;
    uint8_t nrc_code;

    /* Deferred answer after NRC 0x78. */
    bool deferred;
    uint8_t deferred_req[SIM_ECU_BUF];
    uint16_t deferred_len;
    uint8_t deferred_left;
    uint32_t deferred_next_ms;
} sim_ecu_t;

bool sim_ecu_init(sim_ecu_t* ecu, vbus_t* bus);
void sim_ecu_step(sim_ecu_t* ecu, uint32_t now_ms);

/* Back to the default session at once (ECU reset, S3 expiry). */
void sim_ecu_drop_session(sim_ecu_t* ecu);

#endif /* APP_HOST_SIM_ECU_H */
