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
 *   - 0x19 ReadDTCInformation 0x01/0x02/0x04/0x06 (D-080) from a small stored-DTC table
 *     (default: none stored). The DTC numbers, statuses and record bytes are SYNTHETIC
 *     TEST DATA, not vehicle facts; the 0x04/0x06 answers are long enough to be segmented.
 *   - Anything else gets a negative response.
 * Fault hooks for tests: silent, NRC 0x78 bursts, a segmented (First Frame) answer, a
 * fixed NRC for one DID, and a session drop. It is a stand-in until the live vehicle
 * model of vehicle-hil-bench (D-035) exists; not firmware.
 */

#include "features/uds/isotp_core.h"
#include "hal/host/vbus.h"

#include <stdbool.h>
#include <stdint.h>

#define SIM_ECU_BUF 512u               /* holds the longest 0x19 answer a test asks for */
#define SIM_ECU_S3_MS 5000u            /* ISO 14229-2 default S3server */
#define SIM_ECU_SEGMENTED_LEN 20u      /* length of the segmented answer (First Frame) */
#define SIM_ECU_MAX_DTCS 12u           /* stored DTC table size (the tester reads at most 8) */
#define SIM_ECU_DTC_RECORD_LEN 24u     /* default 0x04 answer length (segmented) */
#define SIM_ECU_DTC_EXT_LEN 14u        /* default 0x06 answer length (segmented) */

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
    uint32_t dtc_requests;      /* 0x19 requests */
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
    bool fixed_enabled;         /* answer fixed_did with fixed_raw instead of the sweep */
    uint16_t fixed_did;
    uint32_t fixed_raw;
    bool nrc_enabled;           /* answer nrc_did with NRC nrc_code */
    uint16_t nrc_did;
    uint8_t nrc_code;

    /* The DIDs of the gen/ warm-idle window (VEHICLE_CL250_DTC_WARM_IDLE_*) answer values
     * that satisfy all its conditions (coolant hot, speed exactly 0, idle rpm). */
    bool warm_idle;

    /* 0x19 (D-080). dtc_count DTCs are stored (synthetic, see sim_ecu_dtc_number()). */
    uint8_t dtc_count;
    uint8_t dtc_status;         /* status byte of every stored DTC (default matches the mask) */
    uint16_t dtc_record_len;    /* length of a 0x04 answer; >= 6 */
    uint16_t dtc_ext_len;       /* length of a 0x06 answer; >= 6 */
    bool dtc_silent;            /* ignore 0x19 requests */
    bool dtc_nrc_enabled;       /* answer every 0x19 with NRC dtc_nrc */
    uint8_t dtc_nrc;
    bool dtc_bad_echo;          /* 0x04/0x06 answers echo a wrong DTC */
    bool dtc_pending_once;      /* answer the first 0x19 with NRC 0x78 and then never: it times out */

    /* Deferred answer after NRC 0x78. */
    bool deferred;
    uint8_t deferred_req[SIM_ECU_BUF];
    uint16_t deferred_len;
    uint8_t deferred_left;
    uint32_t deferred_next_ms;
} sim_ecu_t;

bool sim_ecu_init(sim_ecu_t* ecu, vbus_t* bus);
void sim_ecu_step(sim_ecu_t* ecu, uint32_t now_ms);

/* Synthetic DTC number i (test data): 3 bytes into out. */
void sim_ecu_dtc_number(uint8_t i, uint8_t* out);

/* Back to the default session at once (ECU reset, S3 expiry). */
void sim_ecu_drop_session(sim_ecu_t* ecu);

#endif /* APP_HOST_SIM_ECU_H */
