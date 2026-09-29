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
- Commands (CMake >= 3.20 + Ninja; `git submodule update --init` first, defs pinned to `v0.1.0`):
  - `cmake --preset host-tests && cmake --build --preset host-tests && ctest --preset host-tests` (Unity, ASan + UBSan, plus the 2 s SIL smoke run)
  - SIL program (D-034): `build/host-tests/src/moto_rtcore_host` (in-process bus + simulated CL250 ECU); on Linux `--vcan vcan0`
  - `cmake --preset target-m7-debug && cmake --build --preset target-m7-debug` (also `target-m7-release`); `-DARM_TOOLCHAIN_DIR=...` if `arm-none-eabi-gcc` is not on PATH
- The target presets cross-compile only the pure logic (`moto_rtcore_logic`) for Cortex-M7 (flags shared by H743/H723). The board choice decides the CubeMX project (`cubemx/<board>.ioc`), startup and linker script; do not guess it.
- Current modules and layers:
  - `features/uds/isotp_core` (ISO-TP, Ç2)
  - `features/uds/isotp_link`: the core on a port + ID pair. `isotp_link_open_vehicle_cl250()` is the only vehicle-port link and takes its IDs and padding from gen/.
  - `features/uds/uds_client` + `uds_client_core` (Ç3): the single read-only vehicle tester. It keeps the session (0x10 03, 0x3E 80) and polls the gen/ DIDs into `services/vehicle_signals`. Single Frame responses only (Q-020). Fail-closed latch with a reason (`uds_client_fault()`: gate, guard, or a second tester seen on the request IDs). `uds_iso14229.h` is a temporary header of generic ISO 14229 codes until gen/ has them.
  - `services/timebase`
  - `services/vehicle_signals`: the last raw/physical sample per gen/ DID. VALID/STALE is derived at read time from `stale_after_ms`.
  - `services/can_if`: CAN RX routing, plus the **fixed, fail-closed D-020 vehicle-bus guard** that every vehicle frame passes. Never add a bypass.
  - `hal/can_types.h`, `hal/can_port.h` (services and app only), `hal/hal_time.h`, and the host port `hal/host/`
- Libraries:
  - `moto_rtcore_logic` (pure) and `moto_rtcore_fw` (services + feature glue, HAL interfaces only) are also cross-compiled for M7.
  - `moto_rtcore_hal_host` and `moto_rtcore_host` build natively only.
- CI:
  - host tests + SIL smoke, and the SocketCAN test (skipped without `vcan0`)
  - the layering grep: no `hal/` include under `src/features/`
  - both cross builds
  - cppcheck, all blocking: warning / portability / performance, and style + the MISRA C:2012 addon (without the host-only code). Deviations: `misra/README.md` + `misra/suppressions.txt`; add one only with a reason.
  - coverage (`host-coverage` preset, gcovr): floors 95 % lines / 80 % branches on `src/` without the host-only code
  - the defs pin must be a release tag

## Context

Full architecture: `../moto-vehicle-defs/docs/ARCHITECTURE.md` (summary) · detail: `../moto-vehicle-defs/docs/hardware-architecture.md` (section index in `docs/README.md`) section 2-5 (general), section 5b (subsystems — except blind spot, which is in `moto-io-node`), section 8 (rationale for repo separation).
