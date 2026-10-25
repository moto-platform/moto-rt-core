#ifndef SERVICES_TESTER_EPOCH_H
#define SERVICES_TESTER_EPOCH_H

/*
 * The vehicle tester's D-075 stop epoch as a service (D-087 item 2, C3 part 2a): whether an
 * epoch is running and a counter that changes whenever a new one starts. The UDS client glue
 * (features/uds/uds_client, comms task) is the only writer: it sets {open, counter} each
 * step from the client core (the core bumps its own counter where the epoch opens and
 * clears `open` where the ECU is absent). The only reader is services/derived_inputs, which
 * copies the two fields into the snapshot record the derived task reads, so the derived task
 * learns the ride boundary through the same channel as the DIDs and never reads the client.
 *
 * Comms task only, like services/vehicle_signals (no atomics, not ISR-safe). Includes no
 * feature header. The counter is 16 bit and wraps; consumers compare it with != only.
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool open;
    uint16_t counter;
} tester_epoch_t;

/* Writer: the UDS client glue. */
void tester_epoch_set(bool open, uint16_t counter);

/* Reader: the comms task (derived_inputs_publish). Closed with counter 0 before any set. */
tester_epoch_t tester_epoch_get(void);

#ifdef __cplusplus
}
#endif

#endif /* SERVICES_TESTER_EPOCH_H */
