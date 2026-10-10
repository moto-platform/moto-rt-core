#include "features/heartbeat/heartbeat_core.h"

#include <stddef.h>

/* Half the ms counter range: a timebase step at least this long is not a forward step. */
#define HALF_RANGE_MS 0x80000000u
#define MS_PER_S 1000u

uint8_t heartbeat_node_mode(bool first_frame, bool fault_active)
{
    uint8_t mode = (uint8_t)PLATFORM_HEARTBEAT_RT_CORE_NODE_MODE_NORMAL_CHOICE;
    if (first_frame) {
        mode = (uint8_t)PLATFORM_HEARTBEAT_RT_CORE_NODE_MODE_INIT_CHOICE;
    } else if (fault_active) {
        mode = (uint8_t)PLATFORM_HEARTBEAT_RT_CORE_NODE_MODE_DEGRADED_CHOICE;
    } else {
        /* NORMAL: no DTC monitor reports a fault (D-064 item 1) */
    }
    return mode;
}

bool heartbeat_fault_active(bool monitor_failed, bool tester_latched, bool tester_not_running,
                            bool vehicle_port_latched, bool ekf_stalled)
{
    return monitor_failed || tester_latched || tester_not_running || vehicle_port_latched ||
           ekf_stalled;
}

void heartbeat_uptime_init(heartbeat_uptime_t* up, uint32_t now_ms)
{
    up->elapsed_ms = now_ms;
    up->last_ms = now_ms;
}

void heartbeat_uptime_update(heartbeat_uptime_t* up, uint32_t now_ms)
{
    const uint32_t delta = now_ms - up->last_ms;
    if (delta < HALF_RANGE_MS) { /* a backward reading adds nothing: never decreases */
        up->elapsed_ms = (delta > (UINT32_MAX - up->elapsed_ms)) ? UINT32_MAX : (up->elapsed_ms + delta);
        up->last_ms = now_ms;
    }
}

void heartbeat_msg(bool first_frame, bool fault_active, uint32_t fault_onsets,
                   const heartbeat_uptime_t* up, struct platform_heartbeat_rt_core_t* msg)
{
    (void)platform_heartbeat_rt_core_init(msg);
    msg->node_mode = heartbeat_node_mode(first_frame, fault_active);
    /* uint32 -> float is exact up to 2^24; above that the encode saturates anyway */
    const float onsets = (float)fault_onsets;
    msg->error_count = platform_heartbeat_rt_core_error_count_encode(onsets);
    const uint32_t whole_s = up->elapsed_ms / MS_PER_S; /* at most 4294967: exact in float */
    const float seconds = (float)whole_s;
    msg->uptime = platform_heartbeat_rt_core_uptime_encode(seconds);
}

bool heartbeat_frame(const struct platform_heartbeat_rt_core_t* msg,
                     const moto_e2e_tx_state_t* committed, moto_e2e_tx_state_t* next,
                     uint8_t out[PLATFORM_HEARTBEAT_RT_CORE_LENGTH])
{
    if ((msg == NULL) || (committed == NULL) || (next == NULL) || (out == NULL)) {
        return false;
    }
    if (platform_heartbeat_rt_core_pack(out, msg, PLATFORM_HEARTBEAT_RT_CORE_LENGTH) !=
        (int)PLATFORM_HEARTBEAT_RT_CORE_LENGTH) {
        return false;
    }
    *next = *committed;
    return platform_heartbeat_rt_core_e2e_protect(out, PLATFORM_HEARTBEAT_RT_CORE_LENGTH, next) ==
           MOTO_E2E_OK;
}
