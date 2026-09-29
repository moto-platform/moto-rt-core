# src/app

Task and loop setup. Module periods and priorities are registered here, with the rationale for safety-relevant ones (freedom from interference).

- **`host/` (D-034):** `moto_rtcore_host`, rt-core as a host program (SIL). It runs the same `services/` and `features/` code as the target. One pass per ms: `can_if_dispatch()`, then the link steps.
  - Default: in-process bus plus `sim_ecu`, a minimal simulated CL250 ECU that answers 0x22 from the gen/ DID table. It stands in until the moto-hil-bench live model exists (D-035).
  - `--vcan IF` (Linux): the vehicle port on SocketCAN, with an external peer. Interfaces not named `vcan*` also need `--allow-real-bus`, because a second tester next to a real rt-core would break D-021. The `can_if` vehicle guard applies either way.
  - The loop is a transport smoke demo, not the UDS client (Ç3). It reads the gen/ DIDs round-robin, one request in flight, and every request passes the D-020 gates.
  - With `--duration-ms N` it exits 0 when at least `--min-responses` answers were decoded (ctest `sil_moto_rtcore_host_demo`).
- **Target:** FreeRTOS/CMSIS-RTOS2 tasks (D-012), called from the CubeMX `USER CODE` blocks. Empty until the CubeMX project exists.

```bash
./build/host-tests/src/moto_rtcore_host                  # simulated ECU, Ctrl-C to stop
./build/host-tests/src/moto_rtcore_host --vcan vcan0     # Linux
```
