# src/services

Shared services the feature modules talk through. Features never include each other or `hal/`, only services (CI checks the `hal/` part). `can_if.h` exposes only the CAN types (`hal/can_types.h`), not the port functions. The same code runs on the host and on the H7 (D-034).

| Service | Status | What |
|---|---|---|
| `timebase` | done | `timebase_now_ms()` over `hal_time.h`, plus wrap-safe `timebase_elapsed_ms()` / `timebase_expired()` |
| `can_if` | done | The one reader of the CAN ports and the one TX path of features (≈ AUTOSAR CanIf as a concept, D-006). Receivers register per (port, ID, format), up to `CAN_IF_MAX_RECEIVERS` = 12 in a static table. `can_if_dispatch(port, max)` drains a port and calls them; frames with no receiver are counted. **Vehicle-bus guard (D-020, safety rule 2):** fixed and fail-closed. `CAN_PORT_VEHICLE` accepts only 29-bit `VEHICLE_CL250_REQUEST_ID` frames with DLC `VEHICLE_CL250_FRAME_DLC` that pass `vehicle_cl250_frame_allowed()`. Everything else gets `CAN_PORT_ERR_REFUSED` and is counted. |
| `vehicle_signals` | done | The last sample of every CL250 DID (raw, physical, and the send time of its DID's oldest unanswered read, D-051, so the age includes the round trip), written only by the UDS client (`features/uds`). Readers such as the future platform-bus republisher call `vehicle_signals_get(idx, now, &s)`. The state VALID / STALE / NONE is derived at read time from the generated `stale_after_ms` (D-025, D-029), so nothing stored can keep a value VALID. STALE is sticky until the next write (`vehicle_signals_expire()` every client step), so the ms counter wrap cannot revive a value. It also holds ECU presence. Static table, main loop only. |
| `diag` | done | rt-core's own DTC memory and the vehicle-tester status (≈ AUTOSAR Dem as a concept, D-040). DTCs are the gen/ table (`platform_uds.h`). Monitors report every pass (level-triggered): failed sets testFailed + confirmedDTC, passed clears testFailed. Only availability-mask bits are set. `diag_dtc_clear_all()` (UDS 0x14) zeroes the records, and an active condition sets them again on the next report. It never changes the condition itself (e.g. the UDS client latch). RAM only until the H7 flash driver. Writer: the UDS client glue. Reader: the UDS server glue. Main loop only. |
| signal pool, com (generated pack/unpack + E2E from `gen/c/rt_core/`), log | not yet | |

`can_if` runs in the main loop only and is not ISR-safe. On the H7, the FDCAN ISR will only queue frames, and `can_if_dispatch()` will drain that queue.
