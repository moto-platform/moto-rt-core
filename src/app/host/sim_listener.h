#ifndef APP_HOST_SIM_LISTENER_H
#define APP_HOST_SIM_LISTENER_H

/*
 * Platform-bus listener (host SIL only, not firmware): watches what rt-core's
 * republisher (features/vehicle_republish, ISSUES D-5, D-056) puts on the simulated
 * platform bus. For 0x021 VehicleSpeed it runs the receiver side of the E2E check
 * (gen/ moto_e2e_check() with the message's DataID, max delta and timeout), as CONN or
 * LINUX would; for 0x021 and 0x110 VehicleEngine it counts frames and the longest gap
 * between two of them. For the heartbeat 0x081 (features/heartbeat, D-064) it runs the
 * E2E check as SAFETY would and checks NODE_MODE (INIT only on the first frame, then
 * NORMAL or DEGRADED) and that UPTIME never decreases. IDs, lengths, cycle times, choice
 * values and E2E parameters come from gen/.
 */

#include "hal/host/vbus.h"
#include "moto_e2e.h"
#include "platform.h"

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
    sim_listener_msg_t heartbeat;
    moto_e2e_rx_state_t heartbeat_rx;
    uint32_t heartbeat_e2e_ok;  /* OK (and the first INITIAL) */
    uint32_t heartbeat_e2e_bad; /* any other status on a received frame */
    uint32_t heartbeat_normal;  /* frames with NODE_MODE NORMAL */
    uint32_t heartbeat_degraded;
    uint32_t heartbeat_bad_mode;   /* INIT after the first frame, or any other mode */
    uint32_t heartbeat_uptime_back; /* UPTIME lower than in the previous frame */
    uint16_t heartbeat_uptime;
} sim_listener_t;

/* Decodes a 0x081 payload of PLATFORM_HEARTBEAT_RT_CORE_LENGTH bytes. rt-core's gen/
 * has no unpack for its own message, so every field bit is located with the gen/ pack
 * function (pack a message with only that bit set): no hand-written layout (invariant 2). */
void sim_listener_heartbeat_decode(const uint8_t* data, struct platform_heartbeat_rt_core_t* msg);

/* Attaches the listener to the bus. False if the bus is full. */
bool sim_listener_init(sim_listener_t* l, vbus_t* bus);

/* Reads every frame waiting for the listener. */
void sim_listener_step(sim_listener_t* l, uint32_t now_ms);

/* True if all three messages were seen, no 0x021 or 0x081 frame failed its E2E check
 * after the first, every length matched, no gap reached the E2E timeout of 0x021 or
 * 0x081 (3 cycles), and every 0x081 had a valid NODE_MODE and no decreasing UPTIME. */
bool sim_listener_passed(const sim_listener_t* l);

#endif /* APP_HOST_SIM_LISTENER_H */
