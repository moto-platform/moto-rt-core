#ifndef FEATURES_HEARTBEAT_CORE_H
#define FEATURES_HEARTBEAT_CORE_H

/*
 * Pure logic of rt-core's heartbeat (D-064): the gen/ platform message 0x081
 * HeartbeatRtCore, its E2E protection and the never-decreasing uptime. No HAL, RTOS or
 * CAN port access: the glue (heartbeat.c) reads services/diag and services/timebase and
 * writes through services/com.
 *
 * Every ID, layout, choice value, cycle time and E2E DataID comes from gen/ (platform.h,
 * platform_e2e.h); this file names no CAN literal (invariant 2).
 */

#include "platform.h"
#include "platform_e2e.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* NODE_MODE (D-064 item 1): INIT for a frame built in the first step after open (before
 * any comms pass completed, so before the monitors of that pass ran), DEGRADED while
 * heartbeat_fault_active(), NORMAL otherwise. NORMAL attests the comms pass, the DTC
 * monitors and, once an EKF is registered with app/comms, the EKF's alive counter
 * (D-064 item 4, D-065 item 2).
 * SAFE_STATE, BOOTLOADER and DIAGNOSTIC are not used until they are defined. */
uint8_t heartbeat_node_mode(bool first_frame, bool fault_active);

/* DEGRADED condition (D-064 item 1): a DTC monitor's last result is failed, or one of
 * the monitored conditions holds although its monitor did not run (architecture-guard
 * MAJOR-1: U3000 and U0001-88 are evaluated in the UDS server step): the vehicle
 * tester's status is latched or missing/stale (NOT_RUNNING), or the vehicle CAN port is
 * latched after its bus-off limit. Or the EKF stalled (D-064 item 4, D-065 item 2):
 * registered but its alive counter did not change within the window, or never changed
 * (app/comms decides; false while no EKF is registered). */
bool heartbeat_fault_active(bool monitor_failed, bool tester_latched, bool tester_not_running,
                            bool vehicle_port_latched, bool ekf_stalled);

/* Time since boot that never decreases (D-064 item 3): the timebase wraps after about
 * 49.7 days, while UPTIME must decrease only on a restart. */
typedef struct {
    uint32_t elapsed_ms; /* since boot, saturating at UINT32_MAX */
    uint32_t last_ms;    /* timebase reading of the last update */
} heartbeat_uptime_t;

/* Anchors the uptime at now_ms, the timebase reading at open; on the target the timebase
 * counts from reset, so this is the time since boot. */
void heartbeat_uptime_init(heartbeat_uptime_t* up, uint32_t now_ms);

/* Adds the wrap-safe delta since the last update (call it at least once per half
 * timebase range; the comms pass runs every ms). */
void heartbeat_uptime_update(heartbeat_uptime_t* up, uint32_t now_ms);

/* 0x081 from its inputs: NODE_MODE as above, ERROR_COUNT the fault onsets since boot
 * (D-064 item 2) and UPTIME the whole seconds since boot, each through the generated
 * encode, which saturates at the signal's maximum (D-056 item 1). */
void heartbeat_msg(bool first_frame, bool fault_active, uint32_t fault_onsets,
                   const heartbeat_uptime_t* up, struct platform_heartbeat_rt_core_t* msg);

/*
 * Packs 0x081 and protects it with the next E2E counter. *next receives the state after
 * this frame; the caller copies it to its committed state only when the port accepted
 * the frame (D-056 item 5), so a refused write leaves no counter gap. False if packing
 * failed.
 */
bool heartbeat_frame(const struct platform_heartbeat_rt_core_t* msg,
                     const moto_e2e_tx_state_t* committed, moto_e2e_tx_state_t* next,
                     uint8_t out[PLATFORM_HEARTBEAT_RT_CORE_LENGTH]);

#ifdef __cplusplus
}
#endif

#endif /* FEATURES_HEARTBEAT_CORE_H */
