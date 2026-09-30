#ifndef APP_HOST_SIM_TESTER_H
#define APP_HOST_SIM_TESTER_H

/*
 * Scripted diagnostic tester on the simulated PLATFORM bus (host SIL only, not
 * firmware): drives rt-core's UDS server (features/uds/uds_server, Ç3, D-040) end to
 * end over ISO-TP, the way a workshop tool on the platform bus would. Every request,
 * expected answer and wait is built from gen/ (platform_uds.h, uds_iso14229.h).
 *
 * Script: tester present; session read; 0x14 refused in the default session; extended
 * session with P2 / P2*; a segmented multi-DID read (SW version, vehicle-tester status,
 * uptime, an engine-speed sample that must be VALID); DTC count and list; 0x14 clear;
 * programming session refused; functional tester present (silent), functional read,
 * functional unsupported sub-function (silent); S3 expiry back to the default session.
 * Each answer's first frame must arrive within PLATFORM_UDS_P2_SERVER_MAX_MS of the
 * request's last frame.
 */

#include "features/uds/isotp_core.h"
#include "hal/host/vbus.h"

#include <stdbool.h>
#include <stdint.h>

#define SIM_TESTER_BUF 128u
#define SIM_TESTER_REASON_LEN 128u

typedef enum {
    SIM_TESTER_DELAY = 0,
    SIM_TESTER_SEND,
    SIM_TESTER_WAIT,
    SIM_TESTER_DONE,
    SIM_TESTER_FAILED
} sim_tester_state_t;

typedef struct {
    vbus_t* bus;
    uint8_t node;
    isotp_link_t iso;
    uint8_t rx_buf[SIM_TESTER_BUF];
    uint8_t tx_buf[SIM_TESTER_BUF];
    sim_tester_state_t state;
    uint32_t step;
    uint32_t state_since_ms;
    uint32_t sent_ms;        /* last frame of the request went out */
    bool first_frame_seen;
    uint32_t max_latency_ms; /* worst request -> first answer frame */
    uint32_t steps_passed;
    char reason[SIM_TESTER_REASON_LEN];
} sim_tester_t;

/* Attaches a node to `bus` (the platform bus). False if the bus is full. */
bool sim_tester_init(sim_tester_t* t, vbus_t* bus, uint32_t now_ms);

/* One pass: reads its frames, runs the script, sends due frames. */
void sim_tester_step(sim_tester_t* t, uint32_t now_ms);

bool sim_tester_finished(const sim_tester_t* t);
bool sim_tester_passed(const sim_tester_t* t);
uint32_t sim_tester_step_count(void);

#endif /* APP_HOST_SIM_TESTER_H */
