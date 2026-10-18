#ifndef SERVICES_LEAN_HEALTH_TYPES_H
#define SERVICES_LEAN_HEALTH_TYPES_H

/*
 * Types only, no gen/ include: the EKF-side fields of DID 0xFD03 LEAN_HEALTH as they ride
 * in the lean record (E-17 (a), D-067). Kept apart so the 0x020 builder header
 * (features/cornering/lean_tx_core.h) does not depend on the UDS tables.
 */

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool offset_learned;   /* services/imu holds a learnt gyro z offset */
    bool offset_rejected;  /* the last learning window gave reason OFFSET_RANGE */
    uint16_t reject_count; /* imu reject_count, saturated at the gen/ maximum */
    uint16_t unlearned_s;  /* seconds of the current run without an offset, saturated */
} lean_health_ekf_t;

#endif /* SERVICES_LEAN_HEALTH_TYPES_H */
