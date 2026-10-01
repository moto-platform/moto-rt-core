#ifndef SERVICES_VEHICLE_SIGNALS_H
#define SERVICES_VEHICLE_SIGNALS_H

/*
 * Vehicle signal store: the last sample of every CL250 DID in gen/vehicle_cl250.h,
 * indexed like vehicle_cl250_dids[] (vehicle_cl250_did_index_t). The UDS client
 * (features/uds) is the single writer; readers such as the future platform-bus
 * republisher use vehicle_signals_get() and never include features/.
 *
 * The state is computed when a sample is read, from its age and the DID's generated
 * stale_after_ms (D-025, D-029): VALID while age <= stale_after_ms, STALE after that.
 * Nothing stored can keep a value VALID once the poller stops (a sample older than
 * stale_after_ms must not be used or republished as VALID). STALE is sticky: once a
 * read or vehicle_signals_expire() has seen it, the sample stays STALE until the next
 * write, so the 32-bit ms counter wrapping (~49.7 days) cannot make it VALID again.
 *
 * Static storage, no heap. Main loop only (not ISR-safe), like can_if: on the H7 the
 * writer and every reader must run in the same task, or this needs a snapshot lock.
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    VEHICLE_SIGNAL_NONE = 0, /* no sample since init */
    VEHICLE_SIGNAL_VALID,    /* age <= stale_after_ms */
    VEHICLE_SIGNAL_STALE     /* older: must not be used or republished as VALID */
} vehicle_signal_state_t;

typedef struct {
    uint32_t raw;          /* the DID's data bytes, unsigned big-endian */
    float physical;        /* raw through the generated formula, within [min, max] */
    uint32_t timestamp_ms; /* earliest time the ECU took it: the request send time (D-051) */
    uint32_t age_ms;       /* now - timestamp_ms at the time of the read */
    vehicle_signal_state_t state;
} vehicle_signal_sample_t;

/* Forgets every sample and sets the ECU to absent. */
void vehicle_signals_init(void);

/* Stores a sample for DID index idx (writer: the UDS client). False if idx is out of
 * range. The value must come from the generated decoder. */
bool vehicle_signals_write(uint32_t idx, uint32_t raw, float physical, uint32_t timestamp_ms);

/*
 * Reads the sample of DID index idx as of now_ms. False (and *out untouched) if idx is
 * out of range or out is NULL. With no sample yet, the state is VEHICLE_SIGNAL_NONE and
 * the other fields are 0. A timestamp ahead of now_ms reads as a huge age, so STALE.
 */
bool vehicle_signals_get(uint32_t idx, uint32_t now_ms, vehicle_signal_sample_t* out);

/* Marks every sample older than its stale_after_ms as STALE (sticky, see above). The
 * writer calls it every pass, so the flag is set long before the counter can wrap. */
void vehicle_signals_expire(uint32_t now_ms);

/* ECU presence as last reported by the UDS client (D-028 IsEcuPresent). */
void vehicle_signals_set_ecu_present(bool present);
bool vehicle_signals_ecu_present(void);

#ifdef __cplusplus
}
#endif

#endif /* SERVICES_VEHICLE_SIGNALS_H */
