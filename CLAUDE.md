# CLAUDE.md — moto-rt-core

@.claude/PLATFORM-RULES.md

## What this repo is

The main domain controller firmware running on **STM32H7**. CAN read/write, telemetry collection, logging, sensor fusion, UDS/ISO-TP/bootloader, XCP calibration, cornering safety EKF estimation, context classification, virtual dyno computation, the anomaly score's ESP safety-net rules (see below) — all of it lives here, as separate modules under `features/`.

## Directory structure (layered architecture — do not mix)

```
/src/hal/         → hardware abstraction (CAN, IMU, GPS, sensors)
/src/services/     → signal pool, logging manager, EKF fusion, time base
/src/features/      → independent function modules (listed below)
```

Modules under `features/` **do not know about each other**, they only look at `services/`. If you delete a module, the others must keep working. Do not break this rule when adding a new feature.

### Modules under features/ (current plan)
- `uds/` — UDS server + client, ISO-TP transport layer
- `bootloader/` — OTA, A/B bank, rollback
- `xcp/` — calibration interface
- `heartbeat/` — rt-core's 0x081 heartbeat (D-064)
- `cornering/` — **ONLY Layer 2 (EKF estimation): µ, mass, center of gravity, lean angle.** Clipped to physical ranges, published over CAN. Deterministic decision/warning/LED triggering is NOT HERE — that's in `moto-safety-node`, isolated. Don't break this separation; rt-core never makes the cornering-warning decision, it only feeds parameters.
- `dyno/` — virtual dynamometer
- `context/` — context classification (road type, surface, riding event) — the producer of the context data bus
- `anomaly-safety-net/` — **NOT ML**, just a handful of fixed rules (engine temperature threshold, RPM spike, etc.). The actual anomaly model lives on the Raspi (`moto-linux-node`); this is only a guarantee that things don't drop to zero if the Raspi crashes. Don't make this heavier — staying rule-based is a deliberate decision.

## SAFETY-CRITICAL RULES (never violate, no exceptions)

1. `cornering/` (EKF estimation only, in this repo) never makes a cornering-warning/LED decision — that decision lives in `moto-safety-node`. This repo only computes µ/mass/lean angle and publishes it to CAN.
2. This repo NEVER writes to the ECU (engine control unit) in any way; the ECU is read-only (D-037). rt-core is the **only** tester on the vehicle bus (FDCAN1, D-021): it polls the DIDs in `uds/vehicle_cl250.yaml` and sends only what that file's `tester_policy` allows (D-020; session control 0x01/0x03 only). The policy may only be narrowed; widening it or codegen's golden copy (D-027) needs the user's approval. Everything goes through the generated `vehicle_cl250_request_allowed()` / `vehicle_cl250_frame_allowed()` gates and the fixed `can_if` guard. It republishes decoded vehicle signals on the platform bus. The UDS **server** runs only on the platform bus (FDCAN2) (own DIDs, DTCs, bootloader). If a task would widen this boundary, don't do it; ask the user.
3. Dynamic memory allocation (`malloc`/`new`) is not used on safety-critical paths (`cornering/` decision layer, watchdog) — static/stack allocation is preferred. Full MISRA-C compliance is the goal, but this is the minimum rule.
4. Interference independence: a delay/crash in a low-priority module like `anomaly-safety-net/` must NEVER affect the timing of the `cornering/` decision loop — set up task prioritization accordingly.
5. Messages sent to the safety node (platform CAN 0x010-0x07F) and the heartbeat are E2E-protected (D-005); E2E functions come from `gen/`, never hand-written.

## Dependencies

Reads signal/UDS definitions from `moto-vehicle-defs` (submodule: `external/moto-vehicle-defs`, pinned to a tag; generated code at `external/moto-vehicle-defs/gen/c/<node>/`). Bridges to `moto-connectivity-node` (ESP32-S3) via SPI/UART.

## Build

CMake + STM32CubeMX (HAL) + arm-none-eabi-gcc (D-007). RTOS: FreeRTOS/CMSIS-RTOS2 (D-012). Target: STM32H7 (H743/H723 family).
- `cubemx/` is generated code — only write into `USER CODE` blocks.
- Pure logic (EKF, ISO-TP, UDS state machine, E2E, dyno) is written without a HAL dependency and tested under `tests/host/` with Unity. This plus Renode is the path forward while there's no H7 hardware.
- Skeleton: `/repo-bootstrap`; new feature: `/feature-module`.
- Commands (CMake >= 3.20 + Ninja; `git submodule update --init` first, defs pinned to `v0.8.0`):
  - `cmake --preset host-tests && cmake --build --preset host-tests && ctest --preset host-tests` (Unity, ASan + UBSan, plus the 2 s SIL smoke run and the `--uds-scenario` UDS server run)
  - `cmake --preset host-tsan && cmake --build --preset host-tsan && ctest --preset host-tsan` (ThreadSanitizer: the snapshot/alive tests, including the two-thread stress test)
  - SIL program (D-034): `build/host-tests/src/moto_rtcore_host` (in-process bus + simulated CL250 ECU); on Linux `--vcan vcan0`
  - `cmake --preset target-m7-debug && cmake --build --preset target-m7-debug` (also `target-m7-release`); `-DARM_TOOLCHAIN_DIR=...` if `arm-none-eabi-gcc` is not on PATH
- The target presets cross-compile only the pure logic (`moto_rtcore_logic`) for Cortex-M7 (flags shared by H743/H723). The board choice decides the CubeMX project (`cubemx/<board>.ioc`), startup and linker script; do not guess it.
- Current modules and layers:
  - `features/uds/isotp_core` (ISO-TP, Ç2)
  - `features/uds/isotp_link`: the core on a port + ID pair. `isotp_link_open_vehicle_cl250()` is the only vehicle-port link and takes its IDs and padding from gen/.
  - `features/uds/uds_client` + `uds_client_core` (Ç3): the single read-only vehicle tester. It keeps the session (0x10 03, 0x3E 80) and polls the gen/ DIDs into `services/vehicle_signals`, the gen/ `priority` first (D-043); a DID whose last read timed out, or whose last answer came later than its period, competes as normal with no NRC 0x78 extension until it answers in time or is skipped (D-050, D-051); a sample is stamped with the send time of its DID's oldest unanswered read (D-051). Requests are Single Frames; a segmented answer gets the one gen/ FC.CTS only while the link expects the answer to its own allowed request, once, FF_DL <= 64 (D-059), and is then "service unavailable" for a table DID. Fail-closed latch with a reason (`uds_client_fault()`: gate, guard, or a second tester seen on the request IDs or the gen/ OBD functional watch IDs). ISO 14229 codes come from gen/ `uds_iso14229.h`.
  - `features/uds/uds_server` + `uds_server_core` (Ç3, D-040): rt-core's diagnostic server on the **platform bus only** (0x710/0x718, functional 0x7DF): 0x10 / 0x3E / 0x22 / 0x19 / 0x14, NRCs, P2 / P2* / S3, all from gen/ `platform_uds.h`. It serves the 0xFD02 health DID (D-055: step counters from `services/diag`, per-port CAN state from `services/can_sm`, UNKNOWN when not known) and reports DTC U0001-88 while the vehicle port is latched. It never touches the vehicle port and never includes `uds_client.h`.
  - `features/vehicle_republish` (ISSUES D-5, D-056): republishes `services/vehicle_signals` on the platform bus as gen/ 0x021 `VehicleSpeed` (E2E, dedicated replace-on-new TX buffer; not a safety-node input, D-041/D-048) and 0x110 `VehicleEngine` (Tx FIFO), each at its gen/ cycle time. NONE/STALE/out-of-range samples go out INVALID with value 0 (no clamping), and the speed AGE saturates at 2550 ms when INVALID. The E2E counter advances only with accepted frames. It runs in `comms_pass()` after the platform dispatch and before the UDS server.
  - `features/heartbeat` (D-064): rt-core's gen/ 0x081 `HeartbeatRtCore` every 100 ms (E2E, dedicated replace-on-new TX buffer), the input of safety-node's D-042 fallback. NODE_MODE: INIT for the frame of the first step, DEGRADED while a DTC monitor's last result is failed or its condition holds unmonitored (vehicle tester latched or not running, vehicle port latched), else NORMAL; also DEGRADED while a registered EKF is stalled or never ran (`ekf_stalled` from `app/comms`, D-064 item 4; no EKF is registered until D-065 PR 3). ERROR_COUNT: DTC failure onsets since boot; UPTIME: never decreasing seconds. Runs in `comms_pass()` after the republisher, before the UDS server.
  - `services/diag`: DTC memory (RAM, level-triggered), the vehicle-tester status and the client's step counters (fresh within 500 ms, D-055); the client glue writes them, the server reads and clears the DTCs and reports its own monitors. Each monitor's last result and the onset count are kept apart from the status bytes (a 0x14 clear changes neither; D-064, the heartbeat). It includes no other service.
  - `services/com` + `com_core` (D-056 item 7): the shared cycle grid and platform send path (`com_send()`: OK / TX_FULL retry / drop) of the republisher and the heartbeat; the senders keep their E2E state.
  - `services/timebase` (task-safe, `hal/hal_time.h`)
  - `services/snapshot` (D-065): the SPSC triple buffer between tasks of different priority; index-only, no lock/wait/retry (never a seqlock). Atomics behind `hal/hal_atomic.h`.
  - `services/alive` + `alive_core` (D-064 item 4, D-065): a task's lock-free alive counter and the supervisor's pure stall monitor (baseline at registration, never-ran = stalled, sticky).
  - `services/vehicle_signals`: the last raw/physical sample per gen/ DID. VALID/STALE is derived at read time from `stale_after_ms`.
  - `services/can_if`: CAN RX routing, plus the **fixed, fail-closed D-020 vehicle-bus guard** that every vehicle frame passes. Never add a bypass. A bus-off or latched port refuses writes with `CAN_PORT_ERR_IO` (not a guard refusal); `can_if_apply_filters()` turns the receiver table into the port's acceptance filters, after handing the port its dedicated replace-on-new TX IDs (`can_if_register_tx_dedicated()`, platform port only, D-056).
  - `services/can_sm` + `can_sm_core` (Ç1): CAN error state machine per port (error active/passive/bus-off, backoff recovery, N_As abort). The vehicle port latches on its 5th bus-off (D-030, D-054); the platform port never latches. `can_sm_state_known()` tells a read state from an unknown one (D-055).
  - `app/comms`: the portable comms pass (client, then the republisher and the heartbeat, then the server), shared by the host program and the future H7 task. It owns the EKF alive supervision (`comms_ekf_t`: zeroed = no EKF; NULL reads as stalled) and passes `ekf_stalled` to the heartbeat; 0x020's sender must not open without `comms_ekf_registered()` (D-065 PR 3).
  - `hal/can_types.h`, `hal/can_port.h` (services and app only; FDCAN requirements in `hal/README.md`), `hal/hal_time.h`, `hal/hal_atomic.h` (services only; H7 requirements in `hal/README.md`), and the host port `hal/host/`
- Libraries:
  - `moto_rtcore_logic` (pure) and `moto_rtcore_fw` (services + feature glue, HAL interfaces only) are also cross-compiled for M7.
  - `moto_rtcore_hal_host` and `moto_rtcore_host` build natively only.
- CI:
  - host tests + SIL smoke, and the SocketCAN test (skipped without `vcan0`); the ThreadSanitizer job (`host-tsan`)
  - the layering grep: no `hal/` include under `src/features/`, `hal/hal_atomic.h` only from `services/` and `hal/`
  - both cross builds
  - cppcheck, all blocking: warning / portability / performance, and style + the MISRA C:2012 addon (without the host-only code). Deviations: `misra/README.md` + `misra/suppressions.txt`; add one only with a reason.
  - coverage (`host-coverage` preset, gcovr): floors 95 % lines / 80 % branches on `src/` without the host-only code
  - the defs pin must be a release tag

## Context

Full architecture: `../moto-vehicle-defs/docs/ARCHITECTURE.md` (summary) · detail: `../moto-vehicle-defs/docs/hardware-architecture.md` (section index in `docs/README.md`) section 2-5 (general), section 5b (subsystems — except blind spot, which is in `moto-io-node`), section 8 (rationale for repo separation).
