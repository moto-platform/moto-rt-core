#ifndef SERVICES_LEAN_HEALTH_H
#define SERVICES_LEAN_HEALTH_H

/*
 * State behind DID 0xFD03 LEAN_HEALTH (D-067, E-17 (a)), like services/diag for 0xFD00 /
 * 0xFD02: the lean glue writes, the UDS server reads, and neither feature includes the
 * other. **Comms task only** (the lean glue's comms side, app/comms and the UDS server
 * all run in it): nothing here is shared with the EKF task, which is why this service
 * needs no atomics.
 *
 * Race freedom (safety review, uds/dids.yaml): the EKF-side fields reach this service
 * only as a copy that lean_tx_step() makes of the record it reads from the output
 * snapshot, in the same call and from the index that read returned. The UDS server never
 * reads the snapshot and keeps no slot index, so it cannot take a record away from 0x020
 * (a second snapshot_read_index() would). The comms flags are comms-task state, set by
 * app/comms right before the UDS server step.
 *
 * Static storage, no heap. The zero state (also after lean_health_init()) is "no record
 * since boot, no comms flag set": RECORD_FRESH, OFFSET_LEARNED and EKF_ALIVE_OK clear,
 * UNLEARNED_S UNKNOWN.
 */

#include "services/lean_health_core.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Boot (lean_tx_open() calls it) and tests: no record, no flags. */
void lean_health_init(void);

/* Comms task, from lean_tx_step(): the newest copy of the EKF's record. NULL is ignored. */
void lean_health_set_record(const lean_health_rec_t* rec);

/* Comms task, every pass before uds_server_step(). NULL is ignored. */
void lean_health_set_comms(const lean_health_comms_t* comms);

/* Comms task (UDS server): builds the 0xFD03 record as of now_ms. False if len differs
 * from the gen/ length or out is NULL. */
bool lean_health_read(uint32_t now_ms, uint8_t* out, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* SERVICES_LEAN_HEALTH_H */
