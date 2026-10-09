#ifndef ISOTP_LINK_H
#define ISOTP_LINK_H

/*
 * ISO-TP link glue (thesis deliverable Ç2): binds one isotp_core link to a CAN port
 * and a (TX ID, RX ID) pair through services/can_if, with time from services/timebase.
 * The same code runs on the host (SIL, mock bus) and on the H7 (D-034).
 *
 * Per main-loop pass:
 *   can_if_dispatch(port, n)      delivers received frames to the link (callback)
 *   isotp_link_step(link)          runs timers, transmits due frames while TX is free
 *   isotp_link_take_rx(...)        N_USData.indication; data via isotp_link_rx_data()
 *   isotp_link_take_tx_confirm()   N_USData.confirm
 *
 * Vehicle bus (D-020, safety rule 2): the only way to open a link on CAN_PORT_VEHICLE
 * is isotp_link_open_vehicle_cl250(); its addressing and padding come from gen/ and
 * cannot be passed in. Its requests must pass vehicle_cl250_request_allowed() and fit
 * in a Single Frame; its frames are checked with vehicle_cl250_frame_allowed() before
 * they are written. These are early rejects with proper errors. The hard control is
 * the fail-closed guard in services/can_if, which every vehicle frame passes as well.
 * Segmented responses (D-059): requests stay Single Frame. The only other frame the
 * generated gate passes is the one FC.CTS vehicle_cl250_fc_cts[] (BS, STmin and padding
 * from gen/). The gate is stateless; this link sends that FC only while a response is
 * expected (isotp_link_expect_response(), armed by the client after its allowed
 * request went out), once per request, for a First Frame with FF_DL <= rx_cap <=
 * VEHICLE_CL250_MAX_FF_DL that starts with the expected positive response SID. Every other Flow Control the core queues is withheld: the
 * reception is cancelled at once (no indication, no N_Cr wait) and
 * isotp_link_take_fc_withheld() reports it, so the client can wait until the ECU has
 * given up its segmented send.
 *
 * The link registers itself as a can_if receiver, so it must have static storage
 * duration.
 *
 * N_As (Ç1): services/can_sm aborts a port's pending TX when no frame was confirmed
 * within N_As, or on a bus-off, and can_if_tx_abort_count() changes. A link whose message
 * is under way then ends it with ISOTP_N_TIMEOUT_A (at its next step, or before it
 * accepts a new message, so a new message is never ended by an older abort). A frame
 * accepted by can_if_write() still counts as sent for the core: a Single Frame whose
 * frame was aborted is already confirmed, and the peer's timeout covers it. N_Ar is the
 * peer's business (we never wait for our own Flow Control to be confirmed).
 */

#include "features/uds/isotp_core.h"
#include "services/can_if.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Frames transmitted per isotp_link_step() at most (bounds the loop; a real FDCAN
 * has 3 TX mailboxes, the host bus has no limit of its own). */
#define ISOTP_LINK_MAX_TX_PER_STEP 16u

typedef struct {
    can_port_id_t port;
    uint32_t tx_id;   /* ID of the frames we send (request ID for a client) */
    uint32_t rx_id;   /* ID of the frames we receive, including FC for our sends */
    bool extended;    /* 29-bit IDs (both) */
} isotp_link_addr_t;

/* Treat as opaque. */
typedef struct {
    isotp_link_addr_t addr;
    isotp_link_t iso;
    const uint8_t* rx_buf;
    uint32_t tx_error_count;
    uint32_t tx_refused_count;
    uint32_t abort_seen;  /* can_if_tx_abort_count() last acted on */
    uint32_t tx_timeouts; /* messages ended with ISOTP_N_TIMEOUT_A */
    uint32_t fc_withheld_count; /* Flow Controls the vehicle link did not send */
    bool fc_withheld; /* one was withheld since isotp_link_take_fc_withheld() */
    bool response_open; /* vehicle link: one FC.CTS may answer the next First Frame */
    uint8_t response_sid; /* ... whose first data byte is this positive response SID */
    bool vehicle;     /* CL250 tester link: D-020 checks apply */
    bool open;
} isotp_can_link_t;

/*
 * Generic link on a port other than CAN_PORT_VEHICLE (ISOTP_ERR_ARG for the vehicle
 * port). Inits the core and registers the RX ID with can_if. ISOTP_ERR_ARG also covers
 * an RX ID that already has a receiver; ISOTP_ERR_BUSY a link that is already open.
 */
isotp_status_t isotp_link_open(isotp_can_link_t* link, const isotp_link_addr_t* addr,
                               const isotp_config_t* cfg, uint8_t* rx_buf, uint16_t rx_cap,
                               uint8_t* tx_buf, uint16_t tx_cap);

/*
 * The vehicle link to the CL250 engine ECU (D-019), everything from gen/vehicle_cl250.h:
 * vehicle port, VEHICLE_CL250_REQUEST_ID -> VEHICLE_CL250_RESPONSE_ID (29-bit), 8-byte
 * frames padded with VEHICLE_CL250_PADDING_BYTE, the D-059 FC block size and STmin,
 * N_Cr = VEHICLE_CL250_RESPONSE_TIMEOUT_BASE_MS, the ISO default N_Bs. rx_cap is
 * clamped to VEHICLE_CL250_MAX_FF_DL: a longer First Frame gets no reception.
 */
isotp_status_t isotp_link_open_vehicle_cl250(isotp_can_link_t* link, uint8_t* rx_buf,
                                             uint16_t rx_cap, uint8_t* tx_buf, uint16_t tx_cap);

/*
 * N_USData.request. On the vehicle link: ISOTP_ERR_ARG if the request gate refuses the
 * service, ISOTP_ERR_LENGTH if it does not fit in a Single Frame (both counted as
 * refused, nothing is queued).
 */
isotp_status_t isotp_link_send(isotp_can_link_t* link, const uint8_t* data, uint16_t len);

void isotp_link_step(isotp_can_link_t* link);

bool isotp_link_take_rx(isotp_can_link_t* link, isotp_n_result_t* result, uint16_t* length);

/* The received message after an ISOTP_N_OK indication, valid until isotp_link_rx_release(). */
const uint8_t* isotp_link_rx_data(const isotp_can_link_t* link);

void isotp_link_rx_release(isotp_can_link_t* link);

bool isotp_link_take_tx_confirm(isotp_can_link_t* link, isotp_n_result_t* result);

/* Frames the core produced that the port refused or could not take (lost; the peer
 * then times out). Includes frames refused by the can_if vehicle guard. */
uint32_t isotp_link_tx_error_count(const isotp_can_link_t* link);

/* Messages ended with ISOTP_N_TIMEOUT_A (N_As abort or bus-off). */
uint32_t isotp_link_tx_timeout_count(const isotp_can_link_t* link);

/* Vehicle link: requests and frames refused by the D-020 checks (never written). A
 * withheld Flow Control is not counted here (see isotp_link_fc_withheld_count()). */
uint32_t isotp_link_tx_refused_count(const isotp_can_link_t* link);

/* Vehicle link (D-059): the client's allowed request went out and it waits for the
 * answer, whose first byte is positive_sid; the next First Frame starting with it may
 * get the one FC.CTS. Fail-closed: only this call arms it, never isotp_link_send().
 * Ignored on other links. */
void isotp_link_expect_response(isotp_can_link_t* link, uint8_t positive_sid);

/* Vehicle link: no answer is expected any more (answered, timed out, latched); no FC
 * goes out until the next isotp_link_expect_response(). */
void isotp_link_close_response(isotp_can_link_t* link);

/* Vehicle link: true once after the link withheld a Flow Control (a First Frame it did
 * not answer; its reception was cancelled). */
bool isotp_link_take_fc_withheld(isotp_can_link_t* link);

/* Vehicle link: Flow Controls withheld since open (saturating). */
uint32_t isotp_link_fc_withheld_count(const isotp_can_link_t* link);

uint16_t isotp_link_rx_error_count(const isotp_can_link_t* link);

/* True while a segmented reception is running on the link (see isotp_rx_busy()). */
bool isotp_link_rx_busy(const isotp_can_link_t* link);

/* True when a new message can go out at once: the link is open, sends nothing, and the
 * port has a free TX mailbox. A client that waits for this never queues a request
 * behind a stuck one (e.g. while no node ACKs on the bus). */
bool isotp_link_tx_ready(const isotp_can_link_t* link);

/* The N_Bs the link was opened with (ms; 0 if the link is not open). A client uses it
 * as its estimate of how long the peer waits for a Flow Control that never comes. */
uint32_t isotp_link_n_bs_ms(const isotp_can_link_t* link);

#ifdef __cplusplus
}
#endif

#endif /* ISOTP_LINK_H */
