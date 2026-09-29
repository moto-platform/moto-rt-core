# src/services

Shared services the feature modules talk through. Features never include each other or `hal/`, only services (CI checks the `hal/` part). `can_if.h` exposes only the CAN types (`hal/can_types.h`), not the port functions. The same code runs on the host and on the H7 (D-034).

| Service | Status | What |
|---|---|---|
| `timebase` | done | `timebase_now_ms()` over `hal_time.h`, plus wrap-safe `timebase_elapsed_ms()` / `timebase_expired()` |
| `can_if` | done | The one reader of the CAN ports and the one TX path of features (≈ AUTOSAR CanIf as a concept, D-006). Receivers register per (port, ID, format), up to `CAN_IF_MAX_RECEIVERS` = 8 in a static table. `can_if_dispatch(port, max)` drains a port and calls them; frames with no receiver are counted. **Vehicle-bus guard (D-020, safety rule 2):** fixed and fail-closed. `CAN_PORT_VEHICLE` accepts only 29-bit `VEHICLE_CL250_REQUEST_ID` frames with DLC `VEHICLE_CL250_FRAME_DLC` that pass `vehicle_cl250_frame_allowed()`. Everything else gets `CAN_PORT_ERR_REFUSED` and is counted. |
| signal pool, com (generated pack/unpack + E2E from `gen/c/rt_core/`), diag, log | not yet | |

`can_if` runs in the main loop only and is not ISR-safe. On the H7, the FDCAN ISR will only queue frames, and `can_if_dispatch()` will drain that queue.
