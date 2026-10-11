#ifndef FEATURES_CORNERING_LEAN_TX_H
#define FEATURES_CORNERING_LEAN_TX_H

/*
 * Lean glue, comms-task side (D-065 item 2, PR 3b): feeds the speed to the EKF task and
 * sends 0x020 EkfLean. Every function here runs in the comms task only (it reads
 * services/vehicle_signals and writes through services/com, both main-loop only), driven
 * by app/comms.
 */

#include "features/cornering/lean.h"
#include "platform_e2e.h"
#include "services/com.h"
#include "services/com_core.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* km/h per m/s: 0xF40D and 0x021 carry km/h, the lean core works in m/s. */
#define LEAN_KMH_PER_MPS (3.6f)

typedef struct {
    lean_link_t* link;          /* bound by lean_tx_open() */
    com_cycle_t cycle;
    moto_e2e_tx_state_t e2e;    /* committed: advanced only by accepted frames */
    com_msg_stats_t stats;
    bool opened;                /* the only state that publishes speeds or sends 0x020 */
} lean_tx_t;

/*
 * Boot only, before comms_apply_filters(); called through comms_open_lean(). Opens 0x020
 * only when registered is this link's own alive counter, i.e. the counter app/comms
 * supervises (D-065 amendment: 0x020 fails closed without a registered EKF;
 * architecture-guard MAJOR-3: an unrelated counter does not count), and the dedicated
 * replace-on-new TX buffer was registered. False otherwise; tx then stays closed and
 * 0x020 never appears on the bus (receivers see the timeout, the D-042 fallback).
 */
bool lean_tx_open(lean_tx_t* tx, lean_link_t* link, const alive_counter_t* registered);

/* Comms task, every pass after the vehicle client: publishes the 0xF40D sample to the
 * EKF task as physical m/s, absolute stamp (D-051) and state. VALID only when the sample
 * is VALID and inside the gen/ 0x021 speed range (no hand-written limit); else UNUSABLE. */
void lean_tx_publish_speed(lean_tx_t* tx, uint32_t now_ms);

/* Comms task, every pass: sends 0x020 at its gen/ cycle from the newest EKF output
 * (lean_tx_core.h rules). Nothing while closed. */
void lean_tx_step(lean_tx_t* tx, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* FEATURES_CORNERING_LEAN_TX_H */
