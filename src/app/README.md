# src/app

Task and loop setup. Module periods and priorities are registered here, with the rationale for safety-relevant ones (freedom from interference).

- **`comms` (Ç1, portable):** `comms_pass(client, server)` is the fixed order of rt-core's CAN work, for the host program and the H7 comms task alike: `can_sm_step(VEHICLE)`, `can_if_dispatch(VEHICLE)`, `uds_client_step()`, then `can_sm_step(PLATFORM)`, `can_if_dispatch(PLATFORM)`, `vehicle_republish_step()`, `uds_server_step()`. One task runs both UDS roles and the republisher, the client first (uds README, MINOR-3 and MINOR-C; the republisher's place: D-056 item 4). `comms_apply_filters()` sets both ports' acceptance filters (and the platform port's dedicated TX buffers) once the client, the server and the republisher opened. It includes services and feature headers only, never `hal/`.
- **`host/` (D-034):** `moto_rtcore_host`, rt-core as a host program (SIL). It runs the same `services/` and `features/` code as the target. One pass per ms: the simulated peers answer what rt-core sent in the last pass, then `comms_pass()`. The summary prints each port's state, bus-offs, recoveries, N_As aborts and blocked writes.
  - Default: in-process bus plus `sim_ecu`, a minimal simulated CL250 ECU. It answers 0x10 01/03, 0x3E and 0x22 from the gen/ DID table, and here it needs the extended session for its reads, like the real ECU (D-019). Tests also use its fault hooks: silent, NRC 0x78 bursts, a segmented answer, a fixed NRC, and a session drop. It stands in until the moto-hil-bench live model exists (D-035).
  - `--vcan IF` (Linux): the vehicle port on SocketCAN, with an external peer. Interfaces not named `vcan*` also need `--allow-real-bus`, because a second tester next to a real rt-core would break D-021. The `can_if` vehicle guard applies either way.
  - The loop runs the UDS client (`features/uds/uds_client`, Ç3): session, tester present, and the gen/ DIDs into `services/vehicle_signals`. Once a second it prints the signal table (raw, physical, age, VALID/STALE).
  - The platform port is always an in-process bus. The UDS server (`features/uds/uds_server`, Ç3, D-040) runs on it. With `--uds-scenario`, `sim_tester` (a scripted platform-bus tester) runs a full diagnostic session: session, multi-frame multi-DID read, DTC count/list/clear, functional requests, NRCs, the P2 check and S3 expiry. The program exits 0 when every step passes (ctest `sil_moto_rtcore_host_uds_server`, about 6 s).
  - The republisher (`features/vehicle_republish`, D-056) sends 0x021 and 0x110 on the platform bus; `sim_listener` checks every 0x021 with the receiver E2E check, the frame lengths and the longest gap of both, and the summary prints them.
  - With `--duration-ms N` it exits 0 when at least `--min-responses` DID reads were decoded, the client did not latch as failed and the listener passed (ctest `sil_moto_rtcore_host_demo`: 2 s, 20 reads, 40 frames of each message with E2E OK).
- **Target:** FreeRTOS/CMSIS-RTOS2 tasks (D-012), called from the CubeMX `USER CODE` blocks. Empty until the CubeMX project exists. The comms task calls `comms_pass()` every 1 ms; its period plus release jitter must stay within `VEHICLE_CL250_CLIENT_STEP_MAX_MS` (D-053).

```bash
./build/host-tests/src/moto_rtcore_host                  # simulated ECU, Ctrl-C to stop
./build/host-tests/src/moto_rtcore_host --vcan vcan0     # Linux
```
