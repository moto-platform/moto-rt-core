# features/heartbeat

rt-core's heartbeat on the platform bus (decisions D-005, D-026, D-042 item 3, D-054 item 6, D-056, D-064). Every receiver supervises rt-core through it. safety-node uses rt-core's lean (0x020) only while this frame passes its E2E check and NODE_MODE is NORMAL; otherwise it switches to its own fallback lean (D-042 item 3).

## Responsibility

Sends the gen/ `platform.dbc` message 0x081:

| Message | ID, cycle | Protection | TX path | Receivers |
|---|---|---|---|---|
| `HeartbeatRtCore` | `PLATFORM_HEARTBEAT_RT_CORE_FRAME_ID`, `PLATFORM_HEARTBEAT_RT_CORE_CYCLE_TIME_MS` (0x081, 100 ms) | E2E (D-005, D-026): `platform_heartbeat_rt_core_e2e_protect()`, DataID `PLATFORM_HEARTBEAT_RT_CORE_E2E_DATA_ID`, receiver timeout `PLATFORM_HEARTBEAT_RT_CORE_E2E_TIMEOUT_MS` (300 ms) | dedicated replace-on-new buffer (D-054 item 6, D-056 item 6) | SAFETY, IO, CONN, LINUX |

Every ID, length, choice value, cycle time and E2E parameter comes from gen/ (`platform.h`, `platform_e2e.h`, `platform_uds.h`, defs v0.8.0). The module names no CAN literal.

## Fields (D-064)

- **`NODE_MODE`**
  - **INIT** for a frame built in the first step after `heartbeat_open()`, before any comms pass completed (so before the monitors of that pass ran).
  - **DEGRADED** while `heartbeat_fault_active()`:
    - a DTC monitor's last result is failed (`diag_fault_active()`: U0100-00 VEHICLE_ECU_COMM_LOST, U3000-00 VEHICLE_TESTER_LATCHED, U0001-88 VEHICLE_BUS_OFF_LATCHED), or
    - one of these conditions holds although its monitor did not run (architecture-guard MAJOR-1: U3000-00 supervision and U0001-88 are evaluated in `uds_server_step()`): the vehicle tester status is latched or reads NOT_RUNNING (missing or stale, `diag_vehicle_tester()`), or the vehicle CAN port is latched (`can_sm`). An unknown port state is not a latch: the UDS client then sees no answers and fails U0100-00 on its own, or
    - **the EKF stalled** (`ekf_stalled`, D-064 item 4, D-065 item 2): `app/comms` samples the registered EKF's alive counter (`services/alive`) every pass and passes the verdict to `heartbeat_step()`. Stalled means the counter did not change for more than `COMMS_EKF_ALIVE_WINDOW_MS` (gen/ `PLATFORM_EKF_LEAN_E2E_TIMEOUT_MS`, 60 ms = three EKF cycles), or never changed since the EKF was registered (registered but never ran: no grace time). It is not a DTC: it degrades NODE_MODE but adds no onset. No EKF is registered until D-065 PR 3, so today it is always false; a NULL supervision object reads as stalled.
  - **NORMAL** otherwise.
  - SAFE_STATE, BOOTLOADER and DIAGNOSTIC are not sent until they are defined.
  - **NORMAL attests the EKF only once it is registered.** It attests that the comms pass runs, no DTC monitor reports a fault and, with an EKF registered in `app/comms`, that the EKF completed a cycle within the window. The heartbeat never includes cornering/: it only gets the bool.
  - Consequence (D-064): whenever the ECU stays silent (U0100-00, e.g. a missing or unpowered ECU), safety-node runs on its fallback lean and shows DEGRADED.
  - **Boot window** (safety-reviewer MINOR-1): U0100-00 arms only `VEHICLE_CL250_ECU_ABSENT_TIMEOUT_MS` (3 s) after the client opened, so with an ECU silent from boot the frames after the first (INIT) read NORMAL for up to 3 s, then DEGRADED with `ERROR_COUNT` 1. NORMAL in this window means "no fault detected yet", not "ECU confirmed". No hazard while 0x020 is not published; safety-node's Q-023(4) return hysteresis covers it once it is.
- **`ERROR_COUNT`** (DBC: faults detected since boot): `diag_fault_onsets()`, the number of times a DTC monitor's result went from passed (or not yet reported) to failed. A UDS 0x14 clear neither resets nor re-triggers it. It saturates at the signal maximum through the generated encode (D-056 item 1). A flapping monitor counts every onset, so it can reach the maximum quickly; that is the decided meaning. Conditions that only the heartbeat sees unmonitored (above) degrade NODE_MODE but are not counted.
- **`UPTIME`** (DBC: seconds since boot, saturates; a decrease means a restart): whole seconds of `heartbeat_uptime_t`, which starts at the timebase reading at open (the timebase counts from reset on the target) and adds wrap-safe deltas, saturating at `UINT32_MAX` ms, so the ~49.7-day timebase wrap never makes it decrease. A backward reading adds nothing. The signal saturates at its maximum (about 18.2 h) through the generated encode.

## Timing and order

- **Cycle:** deadline-anchored at the gen/ cycle time (`services/com_core`). The first frame goes out at the first step. A late pass skips missed deadlines, never a burst.
- **Order:** `app/comms` `comms_pass()`: platform `can_sm_step()` → `can_if_dispatch()` → `vehicle_republish_step()` → `heartbeat_step()` → `uds_server_step()`.
  - It sees the platform port's current state, and its E2E frame never waits behind server work (as D-056 item 4 for 0x021).
  - It sees the DTC results that the server's monitors reported in the previous pass (1 ms), and reads their conditions directly as well.
  - The frame therefore proves that the comms pass runs. The EKF task (20 ms, higher priority) is supervised through its alive counter, sampled in the same pass just before `heartbeat_step()`; the stamp of a change is when the comms pass saw it, so a late comms pass delays a stall verdict but never causes one.
  - **EKF stall latency:** DEGRADED goes out at the first heartbeat cycle after more than 60 ms without a bump: at most window + 1 ms + one heartbeat cycle (100 ms) after the EKF's last complete cycle. Back to NORMAL at the first heartbeat cycle after the next bump.
- **Write results** (`services/com`):
  - `CAN_PORT_OK`: counted in `sent`, the E2E counter is committed, the cycle is done.
  - `CAN_PORT_TX_FULL`: the replace cancel of the previous 0x081 is still pending. Counted in `retried`; the next pass builds the frame again with the then-current state and the same counter.
  - Any other status (bus-off or latched platform port): counted in `dropped`, the cycle is given up.

## E2E

Same rules as 0x021 (`features/vehicle_republish/README.md` § E2E, D-056 item 5): the module owns its `moto_e2e_tx_state_t`, `heartbeat_frame()` protects a copy, and the glue commits it only on `CAN_PORT_OK`. A frame replaced in its buffer shows up as a counter gap; a gap longer than the 300 ms timeout makes the receiver resync as INITIAL, never OK (D-026).

## Before 0x020 (safety-reviewer MINOR-3, D-065 item 4)

D-064 item 4 is now mechanical on the heartbeat side: `heartbeat_fault_active()` takes `ekf_stalled` (D-065 PR 1), wired through `app/comms` from a lock-free alive counter (`services/alive`; no shared lock, the EKF runs at another priority), with host tests that a stalled or never-run EKF makes NODE_MODE DEGRADED (`tests/host/test_comms_ekf.c`). What the PRs that publish 0x020 must still do:

- **PR (2)**, the pure lean core: no heartbeat change.
- **PR (3)**, `services/imu` and the 0x020 glue: register the EKF's counter with `comms_ekf_register()` at boot, bump it (`alive_bump()`) only after a complete EKF cycle, and make 0x020's open **fail closed** without `comms_ekf_registered()`, so a lean never goes out unsupervised (architecture-guard on PR 1, MAJOR-2); the integration test "EKF task never runs → DEGRADED" on the SIL program; `safety-reviewer`.

## Open

`heartbeat_open()` sets `opened` only when can_if accepted 0x081's dedicated buffer (D-054 item 6). A refused open (sealed, already listed, full) leaves the heartbeat closed: it sends nothing, never through the Tx FIFO, and its receivers see the E2E timeout, which is the safe side (D-042 fallback).

## Failure behaviour

| Fault | Effect on 0x081 | Receiver |
|---|---|---|
| comms task stalls or rt-core resets | no new frame; the pending one is never replaced | E2E timeout after 300 ms → fallback (D-042) |
| platform port bus-off / stalled | cycles dropped / retried, only the newest frame waits | E2E timeout or INITIAL resync, never OK across the gap |
| ECU silent, client latched or not running, vehicle port latched | DEGRADED | fallback (D-042) |
| UDS server not stepped | DEGRADED still follows the conditions (read directly) | fallback |
| registered EKF stalls or never ran | DEGRADED within 60 ms + one cycle; NORMAL after its next complete cycle | fallback (D-042) |

## Memory

- **RAM:** `heartbeat_t` (cycle, E2E state, uptime, three counters, two flags), owned by the caller: about 36 B. The EKF supervision (`comms_ekf_t`, 16 B) and the counter (`alive_counter_t`, 4 B) are owned by the setup, not the heartbeat. `services/diag` adds the per-DTC monitor results and the onset count (bss +7 B). Stack per step is under 100 B. No heap; the only loop is bounded by `PLATFORM_UDS_DTC_COUNT`.
- **Flash** (M7 release, `arm-none-eabi-size` of `moto_rtcore_logic` + `moto_rtcore_fw`, 2026-10-09): text 10904 → 11524 B (+620 B) for the heartbeat, `services/com` and the diag additions. The gen/ functions the linker keeps add about 154 B (pack, init, two encodes, the 0x081 E2E protect). D-065 PR 1: 12018 → 12370 B (+352 B: `services/snapshot` 144 with its defensive index checks, `alive` 42, `alive_core` 54, the `app/comms` supervision and the extra input), bss unchanged.

## Coverage (D-046: 100 % target for heartbeat/E2E)

- `heartbeat.c`: 100 % of lines. One branch is not taken: the "frame not built" side of `built ? data : NULL`. `heartbeat_frame()` fails only if gen/ `platform_heartbeat_rt_core_pack()` refuses a buffer of exactly `PLATFORM_HEARTBEAT_RT_CORE_LENGTH` bytes, which it never does; if it did, `com_send()` would drop the cycle like a bus-off (tested in `test_com`).
- `heartbeat_core.c`: one line and branch not taken, the same pack failure inside `heartbeat_frame()`, kept as a defensive check of the gen/ contract (as in the republisher).
- `services/diag.c`: the saturation of the onset count at `UINT32_MAX` is not exercised (it needs 2³² onsets); the 8-bit `ERROR_COUNT` saturates long before.

## Tests

- **`tests/host/test_heartbeat_core.c`:** INIT / DEGRADED / NORMAL; each single fault condition; uptime from the anchor, across the timebase wrap (accumulating and saturating), backward readings; whole seconds; `ERROR_COUNT` and `UPTIME` at and past their signal maxima; frames pass the receiver E2E check, the counter moves only when committed, a wrong DataID fails the CRC; NULL arguments.
- **`tests/host/test_heartbeat.c`** (vbus, manual clock, client and server not stepped):
  - one frame per gen/ cycle from the first pass, INIT only on the first, E2E INITIAL then OK, nothing on the vehicle bus
  - UPTIME in whole seconds since boot
  - DEGRADED while a monitor fails, NORMAL after it passes, one onset per transition
  - a 0x14 clear neither hides the fault nor changes `ERROR_COUNT`
  - DEGRADED for a stopped client, a latched client and a latched vehicle port with no monitor report (MAJOR-1); an unknown port state is not a latch
  - after a stall only the newest frame goes out, never OK across the gap; a platform bus-off drops cycles and the receiver resyncs INITIAL
  - a refused open (sealed, already listed) and NULL send nothing
- **`tests/host/test_diag.c`:** onset counting, clear and init.
- **`tests/host/test_comms_ekf.c`** (real comms pass, client, ECU, server; the EKF simulated by bumping its counter every gen/ EKF cycle): no EKF registered leaves NODE_MODE as before; a registered EKF that never ran reads DEGRADED from the first cycle, also when its count was nonzero at registration; a running EKF reads NORMAL, a stall reads DEGRADED after more than the window and within one heartbeat cycle, a resume reads NORMAL within one cycle; two missed EKF cycles are not a stall; a NULL supervision object reads DEGRADED; NULL or a second registration is refused.
- **`tests/host/test_alive.c`**, **`test_snapshot.c`**, **`test_snapshot_stress.c`** (two threads, also under ThreadSanitizer): the services the EKF supervision and exchange are built on (`src/services/README.md`).
- **`tests/host/test_can_sm.c`** (real comms pass, client and server stepped):
  - 0x081 at its cycle time next to the client, the server and the republisher
  - the boot window: with the ECU silent, INIT, then NORMAL until U0100-00 arms, DEGRADED within one cycle after `VEHICLE_CL250_ECU_ABSENT_TIMEOUT_MS` with `ERROR_COUNT` 1
  - U3000-00's two writers (client status, server supervision): three client stalls past the max age give `ERROR_COUNT` 1, 2, 3, NORMAL after each resume, no extra onset
- **SIL** (`moto_rtcore_host`, ctest `sil_moto_rtcore_host_demo`): `sim_listener` checks every 0x081 (E2E, NODE_MODE, UPTIME monotonic, gap below the E2E timeout); the demo fails otherwise.

## HIL scenarios (with the board, moto-hil-bench)

- `heartbeat_rt_core_reset`: reset rt-core during a ride. Expected: safety-node sees the E2E timeout within 300 ms + one cycle and shows DEGRADED; after the reset the first 0x081 is INIT with a smaller UPTIME, and safety-node returns to rt-core's lean only per its Q-023(4) hysteresis.
- `heartbeat_ecu_silent_degraded`: silence the restbus ECU. Expected: NODE_MODE DEGRADED within `VEHICLE_CL250_ECU_ABSENT_TIMEOUT_MS` + one cycle, `ERROR_COUNT` +1, NORMAL again within one cycle after the ECU answers.
- `heartbeat_boot_ecu_absent`: power rt-core with the restbus ECU off. Expected: INIT, NORMAL for at most `VEHICLE_CL250_ECU_ABSENT_TIMEOUT_MS`, then DEGRADED with `ERROR_COUNT` 1; safety-node stays on its fallback per its Q-023(4) hysteresis.
- `heartbeat_stale_pending_frame`: block the platform bus past the 300 ms E2E timeout, then stop the comms task. The late pending 0x081 must read INITIAL at the receiver, never OK.
- `heartbeat_platform_stall`: as `republisher_platform_stall`, for 0x081 (newest frame only, INITIAL after the gap).
- `heartbeat_ekf_stall` (with D-065 PR 3): suspend the EKF task for 200 ms during a ride. Expected: NODE_MODE DEGRADED within 60 ms + one 0x081 cycle of the suspension, safety-node on its fallback; NORMAL again within one cycle after the EKF resumes; the comms pass timing unchanged throughout.
