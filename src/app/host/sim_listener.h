#ifndef APP_HOST_SIM_LISTENER_H
#define APP_HOST_SIM_LISTENER_H

/*
 * Platform-bus listener (host SIL only, not firmware): watches what rt-core's
 * republisher (features/vehicle_republish, ISSUES D-5, D-056) puts on the simulated
 * platform bus. For 0x021 VehicleSpeed it runs the receiver side of the E2E check
 * (gen/ moto_e2e_check() with the message's DataID, max delta and timeout), as CONN or
 * LINUX would; for 0x021 and 0x110 VehicleEngine it counts frames and the longest gap
 * between two of them. IDs, lengths, cycle times and E2E parameters come from gen/.
 */

#include "hal/host/vbus.h"
#include "moto_e2e.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t frames;
    uint32_t bad_length;
    uint32_t last_ms;
    uint32_t gap_max_ms;
    bool seen;
} sim_listener_msg_t;

typedef struct {
    vbus_t* bus;
    uint8_t node;
    sim_listener_msg_t speed;
    sim_listener_msg_t engine;
    moto_e2e_rx_state_t speed_rx;
    uint32_t speed_e2e_ok;      /* OK (and the first INITIAL) */
    uint32_t speed_e2e_bad;     /* any other status on a received frame */
} sim_listener_t;

/* Attaches the listener to the bus. False if the bus is full. */
bool sim_listener_init(sim_listener_t* l, vbus_t* bus);

/* Reads every frame waiting for the listener. */
void sim_listener_step(sim_listener_t* l, uint32_t now_ms);

/* True if both messages were seen, no 0x021 frame failed its E2E check after the first,
 * every length matched, and no gap reached the 0x021 E2E timeout (3 cycles). */
bool sim_listener_passed(const sim_listener_t* l);

#endif /* APP_HOST_SIM_LISTENER_H */
