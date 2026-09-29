# src/app

Task and loop setup. Module periods and priorities are registered here, with the rationale for safety-relevant ones (freedom from interference).

- **`host/` (D-034):** `moto_rtcore_host`, rt-core as a host program (SIL). It runs the same `services/` and `features/` code as the target. One pass per ms: `can_if_dispatch()`, then `uds_client_step()`.
  - Default: in-process bus plus `sim_ecu`, a minimal simulated CL250 ECU. It answers 0x10 01/03, 0x3E and 0x22 from the gen/ DID table, and here it needs the extended session for its reads, like the real ECU (D-019). Tests also use its fault hooks: silent, NRC 0x78 bursts, a segmented answer, a fixed NRC, and a session drop. It stands in until the moto-hil-bench live model exists (D-035).
  - `--vcan IF` (Linux): the vehicle port on SocketCAN, with an external peer. Interfaces not named `vcan*` also need `--allow-real-bus`, because a second tester next to a real rt-core would break D-021. The `can_if` vehicle guard applies either way.
  - The loop runs the UDS client (`features/uds/uds_client`, Ç3): session, tester present, and the gen/ DIDs into `services/vehicle_signals`. Once a second it prints the signal table (raw, physical, age, VALID/STALE).
  - With `--duration-ms N` it exits 0 when at least `--min-responses` DID reads were decoded and the client did not latch as failed (ctest `sil_moto_rtcore_host_demo`: 2 s, 20 reads).
- **Target:** FreeRTOS/CMSIS-RTOS2 tasks (D-012), called from the CubeMX `USER CODE` blocks. Empty until the CubeMX project exists.

```bash
./build/host-tests/src/moto_rtcore_host                  # simulated ECU, Ctrl-C to stop
./build/host-tests/src/moto_rtcore_host --vcan vcan0     # Linux
```
