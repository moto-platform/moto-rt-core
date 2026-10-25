#ifndef SERVICES_TESTER_EPOCH_H
#define SERVICES_TESTER_EPOCH_H

/*
 * The vehicle tester's D-075 stop epoch as a service (D-087 item 2, C3 part 2a): whether an
 * epoch is running and a counter that changes whenever a new one starts.
 *
 * Single writer: the UDS client glue (features/uds/uds_client, comms task) and nothing else
 * (CI greps the includes of this service and of services/derived_inputs; any other call of
 * tester_epoch_set() is a review finding). It sets {open, counter} after every client step
 * while the client is open (from the client core: the core bumps its own counter where the
 * epoch opens and clears `open` where the ECU is absent); it is not refreshed while the client
 * is closed, and uds_client_open() resets it to {closed, 0}.
 *
 * Restarting counter is safe: after uds_client_open() the counter restarts at 0 and the first
 * epoch is counter 1 again, possibly the value of the epoch before the re-open. Consumers
 * compare the counter with != only and always see the closed state in between: the re-open
 * publishes {closed, 0} before the first new epoch can open (an epoch needs a session
 * confirmation, which needs requests and answers, many comms passes later), so the derived
 * task closes the old ride at the closed record and opens the new one at the next open. Only a
 * derived task that skipped every record for that whole stretch could miss it; it would still
 * see the closed state or a different counter within one ECU-absent timeout.
 *
 * The only reader is services/derived_inputs, which copies the two fields into the snapshot
 * record the derived task reads, so the derived task learns the ride boundary through the
 * same channel as the DIDs and never reads the client.
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
