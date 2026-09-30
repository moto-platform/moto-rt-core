# features/uds

UDS server (platform bus, FDCAN2) and UDS client (vehicle bus, FDCAN1, the only tester per D-021) on top of an ISO-TP transport layer. Thesis deliverables Ç2 (ISO-TP) and Ç3 (UDS).

Status:
- Done:
  - the ISO-TP core (`isotp_core.{h,c}`)
  - the link glue that binds it to a CAN port and an ID pair (`isotp_link.{h,c}`)
  - the UDS client, the CL250 vehicle poller (`uds_client.{h,c}`, `uds_client_core.{h,c}`, Ç3)
- Next: the UDS server on the platform bus (Ç3).

## ISO-TP core (`isotp_core.h`)

**Responsibility.** ISO 15765-2:2016 segmentation and reassembly for classic CAN. It covers:
- Single Frame, First Frame and Consecutive Frame, with the 12-bit FF_DL (up to 4095 bytes).
- Flow Control CTS / WAIT / OVFLW, block size and STmin.
- The N_Bs and N_Cr timeouts, and the N_WFTmax limit on FC.WAIT.

It is pure logic with no HAL, RTOS or heap, and is tested in `tests/host/test_isotp_core.c`.

**Out of scope.**
- CAN FD frames and the 32-bit FF_DL escape.
- Extended and mixed addressing.
- Sending FC.WAIT as a receiver.
- The N_As / N_Ar transmit confirmation, which belongs to the glue.

**Inputs.**
- The CAN frames of one link, one (request ID, response ID) pair, via `isotp_on_frame()`.
- A millisecond timebase.
- `isotp_send()` requests.

**Outputs.**
- Frames to transmit via `isotp_poll()`, in order.
- `N_USData.indication` via `isotp_take_rx_indication()` and `N_USData.confirm` via `isotp_take_tx_confirm()`.

**Configuration** (`isotp_config_t`, per link):
- Padding on/off and the padding byte. The vehicle link uses `VEHICLE_CL250_PADDING_BYTE` from `gen/`.
- The BS and STmin that we advertise.
- N_Bs and N_Cr, which default to 1000 ms (ISO default).
- N_WFTmax.

**Timing.** Call `isotp_poll()` every pass, and only while a CAN TX mailbox is free: a frame it returns counts as sent. STmin values below 1 ms (0xF1–0xF9) are rounded up to 1 ms. Reserved STmin values count as 127 ms (§9.6.5.4).

**Memory.** Buffers are supplied by the caller (static storage), up to 4095 bytes each. `isotp_link_t` itself holds no buffer.

**Failure behaviour.**
- Invalid PDUs are ignored:
  - SF_DL of 0 or above 7, or a DLC too short
  - FF_DL of 7 or less, or the escape form
  - an unexpected CF or FC
  - a short FC or CF
- A wrong SN or an N_Cr expiry ends the reception. It is reported as `ISOTP_N_WRONG_SN` / `ISOTP_N_TIMEOUT_CR` and counted in `isotp_rx_error_count()`.
- An SF or FF that arrives during a reception ends that reception (counted, `ISOTP_N_UNEXP_PDU`), and the new message is processed.
- A message longer than the rx buffer gets FC.OVFLW and no reception starts.
- A received message stays in the buffer until `isotp_rx_release()`. New messages are dropped until then.
- The sender aborts on:
  - N_Bs expiry: `ISOTP_N_TIMEOUT_BS`
  - FC.OVFLW: `ISOTP_N_BUFFER_OVFLW`
  - a reserved flow status: `ISOTP_N_INVALID_FS`
  - more than N_WFTmax FC.WAIT in a row: `ISOTP_N_WFT_OVRN`

**Requirement IDs.** None yet (Q-006: where requirements live). The tests are named after the behaviour and the ISO clause.

## ISO-TP link glue (`isotp_link.h`)

**Responsibility.** Binds one core link to a (port, TX ID, RX ID, format) tuple.
- It registers the RX ID with `services/can_if`, and takes its time from `services/timebase`.
- `isotp_link_step()` runs the timers and writes due frames while `can_if_tx_free()`. It writes at most `ISOTP_LINK_MAX_TX_PER_STEP` = 16 frames per call.
- The same objects run in the host SIL program and on the H7 (D-034).

**Vehicle link.** `isotp_link_open_vehicle_cl250(link, buffers)` is the only way to open a link on `CAN_PORT_VEHICLE`. The generic `isotp_link_open()` refuses that port. Everything comes from `gen/vehicle_cl250.h` (D-019) and cannot be passed in:
- `VEHICLE_CL250_REQUEST_ID` → `_RESPONSE_ID`, 29-bit
- 8-byte frames padded with `VEHICLE_CL250_PADDING_BYTE`
- the ISO default BS, STmin, N_Bs and N_Cr

**D-020 on the vehicle bus**, in two layers (safety review of this change, finding B1):
1. **Hard guard in `services/can_if`.** It is fixed and fail-closed, and every frame for `CAN_PORT_VEHICLE` passes it, whichever feature sends it. It passes only 29-bit frames on `VEHICLE_CL250_REQUEST_ID`, with DLC `VEHICLE_CL250_FRAME_DLC`, whose bytes pass the generated `vehicle_cl250_frame_allowed()`. Anything else returns `CAN_PORT_ERR_REFUSED` and is counted (`can_if_tx_refused_count()`). Features cannot bypass it: they see only `can_types.h`, and CI fails on a `hal/` include under `src/features/`.
2. **Early rejects in the vehicle link**, so the caller gets a proper error:
   - `isotp_link_send()` returns `ISOTP_ERR_ARG` for a service that `vehicle_cl250_request_allowed()` refuses.
   - It returns `ISOTP_ERR_LENGTH` for a request longer than 7 bytes, which would need a First Frame.
   - Frames are checked with `vehicle_cl250_frame_allowed()` before `can_if_write()`.
   - All of these are counted in `isotp_link_tx_refused_count()`.

**Known gap (open question):** the generated frame gate passes Single Frames only, so the vehicle link can never send a Flow Control. A segmented response from the ECU therefore ends in `ISOTP_N_TIMEOUT_CR`.
- All current CL250 DIDs fit in a Single Frame (at most 5 bytes).
- 0x19 with more than one DTC, and OBD 0x09 (VIN), pass the request gate but always need several frames, so they will not work in practice.
- Until this is decided, the Ç3 client (see "UDS client" below) does the following:
  - treats `ISOTP_N_TIMEOUT_CR` as "service unavailable"
  - applies `VEHICLE_CL250_DID_SKIP_COOLDOWN_MS`, so one failing request cannot hold the single in-flight slot
  - waits at least N_Bs after an aborted segmented response
- Letting FC.CTS through widens the D-020 gate. It needs the user's approval and a versioned moto-vehicle-defs change, reviewed by the safety-reviewer. The reviewer's conditions:
  - byte-exact FC.CTS with fixed BS/STmin and padding
  - sent only while a reception is running for an allowed request
  - a capped FF_DL
  - tester requests stay Single Frame
- How that would be split: the `can_if` guard is stateless and would keep only the byte-exact FC match. The "only during an active reception for an allowed request" condition has to be a link-state check in the vehicle link.

**Failure behaviour.**
- A full TX mailbox leaves frames in the core until the next step; nothing is lost.
- A frame the port refuses (`can_if_write` ≠ OK) is lost and counted (`isotp_link_tx_error_count()`). The peer then times out.
- N_As/N_Ar need a TX-complete confirmation from the FDCAN driver and come with the H7 HAL. Until then, a frame accepted by the port counts as sent.

**Memory.** `isotp_can_link_t` plus buffers supplied by the caller (static storage). The link registers itself with `can_if` as the receiver context, so it must have static storage duration.

**Tests.** `tests/host/test_isotp_link.c` runs on the in-process bus with a manual clock. It covers:
- the CL250 wire format and the gen/ values
- every gen/ DID answered by the simulated ECU
- the request and frame gates, including the FC gap
- other IDs and formats being ignored
- BS/STmin segmentation over the bus
- a full mailbox, N_Cr, the per-step bound, and argument checks

## UDS client, the vehicle poller (`uds_client.h`, `uds_client_core.h`)

**Responsibility.** rt-core is the single vehicle-bus tester, and it only reads (D-021, D-037). The client keeps the extended session with the CL250 ECU and polls the DIDs of `gen/vehicle_cl250.h` into `services/vehicle_signals`.
- `uds_client_core` is the pure state machine: no HAL, services or RTOS, time passed in. Tested in `tests/host/test_uds_client_core.c`.
- `uds_client` is the glue: the vehicle ISO-TP link, `services/timebase`, and the `vehicle_signals` writes. Tested end to end in `tests/host/test_uds_client.c`.

**What it sends.** Only three requests, all built from gen/:
- `VEHICLE_CL250_SESSION_SID` / `_SUBFUNCTION` (0x10 03)
- `VEHICLE_CL250_TESTER_PRESENT_SID` / `_SUBFUNCTION` (0x3E 80)
- 0x22 plus a DID of `vehicle_cl250_dids[]`

Every request still passes `vehicle_cl250_request_allowed()` in the link and the `can_if` guard. It never sends 0x19 or OBD 0x09, which need multi-frame responses (Q-020).

**Temporary header.** `uds_iso14229.h` holds the generic ISO 14229-1 codes that gen/ does not provide yet (user decision, 2026-09-29):
- the 0x22 request SID
- the 0x7F negative response
- NRC 0x78, 0x7E and 0x7F

No vehicle fact lives there. The positive 0x62 check stays in the generated `vehicle_cl250_parse_response()`. Remove the header once moto-vehicle-defs generates these codes.

**Inputs.**
- The link's `N_USData.indication`.
- `isotp_link_rx_busy()`, true while a segmented reception runs.
- `timebase_now_ms()`.

**Outputs.**
- One sample per DID in `services/vehicle_signals`:
  - `raw`: the data bytes, big-endian
  - `physical`: from the generated parser, formula and range
  - the receive timestamp
- VALID/STALE is derived at read time from the generated `stale_after_ms`. A stalled poller therefore cannot leave a value VALID.
- ECU presence (`vehicle_signals_ecu_present()`).
- Republishing on the platform bus is not in this module.

**Timing.** Every value is a `VEHICLE_CL250_*` from gen/:
- **Session.** 0x10 03 every `SESSION_RETRY_INTERVAL_MS` until the positive `SESSION_POSITIVE_SID` response with the echoed sub-function. No DID is read before that.
- **Tester present.** 0x3E 80 every `TESTER_PRESENT_PERIOD_MS`, also before the session is up (verified legacy behaviour). The response is suppressed, so it never takes the in-flight slot.
- **Reads.** Round-robin over the DIDs that are due (`poll_period_ms` since their last request) and not skipped. The scan starts after the last DID served, so a DID cannot starve. One request is in flight (`REQUESTS_IN_FLIGHT` = 1). When several requests are due, the order is session, then tester present, then reads.
- **Response timeout.**
  - `RESPONSE_TIMEOUT_BASE_MS`.
  - NRC 0x78 for the pending SID restarts it, doubled, up to `RESPONSE_TIMEOUT_MAX_MS` (100 → 200 → 400 → 800 → 1600 → 2000).
  - A request never waits more than `RESPONSE_TIMEOUT_MAX_MS` in total since it was sent, so an ECU that answers 0x78 forever cannot hold the slot. The doubling and this total cap are the legacy rule (`docs/legacy-telemetry-notes.md`).
- **Skip.** `MAX_CONSECUTIVE_TIMEOUTS` timeouts in a row skip a DID for `DID_SKIP_COOLDOWN_MS`. A decoded answer or an NRC resets the count.
- **ECU present.** True while any answer (positive or NRC) came within `ECU_ABSENT_TIMEOUT_MS`.

**Failure behaviour.**
- **Other NRCs** for the pending SID end the request (legacy, user decision 2026-09-29). They are not a timeout, and the DID keeps its schedule. NRCs for another SID, for example tester present, never end or extend the pending read.
- **Session lost.** NRC 0x7E or 0x7F on any request, or the ECU going absent, marks the session down. It is then re-established the same way. Only 0x10 03 is ever sent, never 0x10 01 or 0x10 02.
- **Q-020, segmented responses.**
  - The link cannot send the FC, so a segmented response ends in `ISOTP_N_TIMEOUT_CR` after N_Cr.
  - Any failed reception counts as "service unavailable". The DID goes into skip cooldown at once, and nothing is sent for the link's N_Bs (`isotp_link_n_bs_ms()`), so the ECU has given up its segmented send first.
  - While a reception runs, nothing is sent, and the base timeout pauses. The total cap still applies.
  - The link's receive buffer (`UDS_CLIENT_RX_BUF` = 64) is larger than a Single Frame, so a First Frame starts a reception instead of being dropped silently.
- **Late, malformed or foreign answers** (another DID, out of range, longer than a Single Frame) are counted as `unexpected`. The request then ends by timeout.
- **TX readiness.** No request is produced while `isotp_link_tx_ready()` is false (the link is sending, or the mailbox is full because no node ACKs: DLC unplugged, ECU off, cold-crank brownout). The timers keep running. At most one stuck request goes out when the bus frees. A request the link refuses as busy is dropped without a latch (`uds_client_core_not_sent()`).
- **Fail-closed latch** (`uds_client_fault()` reports the first reason). The client sends nothing more until it is opened again, and its session reads down:
  - `UDS_CLIENT_FAULT_GATE`: the link refused a request (the D-020 gate or the Single Frame length). Only a bug can cause it.
  - `UDS_CLIENT_FAULT_GUARD`: the `can_if` vehicle guard refused any frame.
  - `UDS_CLIENT_FAULT_FOREIGN_TESTER`: a frame on `VEHICLE_CL250_REQUEST_ID` or `VEHICLE_CL250_FALLBACK_REQUEST_ID`. The controller never receives its own frames, so this is a second tester (D-021, same as connectivity-node, D-030). `uds_client_open()` refuses to run without this watch (`ISOTP_ERR_ARG` when `can_if` has no room).
  - The FC.CTS that the link drops for a segmented response is expected, and does not latch.
- **Sticky STALE.** `vehicle_signals_expire()` runs every step. Once a sample is STALE it stays STALE until the next write, so the 32-bit ms counter wrapping (about 49.7 days) cannot make an old value VALID. ECU presence also needs a new answer after absence.
- **Known limits.**
  - A First Frame with FF_DL above `UDS_CLIENT_RX_BUF` (64) is dropped silently by the core. The request ends by timeout and counts towards the skip limit. No CL250 DID does this.
  - Each segmented response stops polling for about N_Cr + N_Bs (about 2 s), so every DID goes STALE. This is the cost of the Q-020 deferral.
  - A DID answered with a permanent NRC (for example 0x31) is polled at its full rate. This is legacy behaviour, and the schedule bounds it.
  - The foreign-tester watch covers the two physical request IDs from gen/. A generic OBD dongle that uses functional addressing (0x7DF, or 0x18DB33F1 for 29-bit) is not seen. Those IDs belong in defs (`/signal-change`) before rt-core can watch them.
  - `uds_client_core_not_sent()` clears only the pending slot. The dropped request's schedule stays advanced, so a dropped tester present waits one full period. The path is defensive: `tx_ready` already requires the link to be idle.
  - On the H7, the FDCAN acceptance filters must pass `VEHICLE_CL250_REQUEST_ID` and `VEHICLE_CL250_FALLBACK_REQUEST_ID`. Otherwise the foreign-tester watch is deaf. This is an Ç1 HAL requirement, checked on target by `uds_client_foreign_tester`.
  - N_As (a TX that is never confirmed) and bus-off recovery (`VEHICLE_CL250_BUS_OFF_BACKOFF_*`, D-030's latch after 5 bus-offs) come with the H7 FDCAN HAL (Ç1).
- **Counters** (`uds_client_stats()`): requests, reads, timeouts, NRC, response pending, unavailable, skips, unexpected, session starts and losses.

**Memory.** `uds_client_t` is 348 B on the M7: the link, 64 + 8 B of buffers, and the core. It must have static storage duration. `vehicle_signals` adds 81 B, and the glue adds 4 B (the foreign-frame counter). Flash is about 2 kB: core 1094 B, glue 622 B, service 252 B (release build, 2026-09-29, after the safety fixes). There is no heap, and every loop is bounded by the DID count or a frame length.

**Integration notes.**
- Call `uds_client_step()` once per main-loop pass, after `can_if_dispatch(CAN_PORT_VEHICLE, ...)`.
- On the H7, the writer and every `vehicle_signals` reader must share one task, or the store needs a snapshot lock.
- When rt-core polls on the vehicle, connectivity-node's temporary poller must be off (`CONN_VEHICLE_TESTER=0`, D-021/D-023/D-030). There are never two testers.

**Requirement IDs.** None yet (Q-006). The tests are named after the behaviour and the gen/ value they check.

**Reviews (2026-09-29).**
- architecture-guard: the poller goes in `features/uds` and the value store in `services/vehicle_signals`. The gen/ gap was filled with a temporary local header, by user decision.
- vss-schema-guardian: CLEAN (defs v0.1.0).
- safety-reviewer: no blocker. Findings and their status:

| ID | Finding | Status |
|---|---|---|
| M1 | A full TX mailbox made `isotp_link_send` return BUSY, which latched the client until reboot | Fixed: `tx_ready` input, BUSY is not a fault |
| M2 | No foreign-tester detection; D-021 was enforced only by procedure | Fixed: watch on both request IDs, `FOREIGN_TESTER` latch |
| M3 | VALID/STALE aliased after 2^32 ms | Fixed: sticky STALE plus `vehicle_signals_expire()` every step |
| m1 | `ecu_seen` was never cleared, so presence came back after the wrap | Fixed: cleared on absence |
| m2 | `session_up` stayed true after the latch | Fixed: the latch drops the session |
| m3 | A First Frame with FF_DL > 64 was dropped silently | Documented (known limits) |
| m4 | A segmented response silences polling for about 2 s | Documented (known limits), HIL scenario |
| m5 | The latch reason was not exposed | Fixed: `uds_client_fault()` |
| m6 | `uds_client_sample_t s` was not initialised | Fixed |
| m7 | A permanent NRC is polled at full rate | Documented (legacy, user decision) |
| m8 | `moto_rtcore_host --allow-real-bus` is now a full tester | Fixed: the warning text says so, and the summary prints the fault |
| — | N_As, bus-off backoff | Deferred to the H7 HAL (Ç1) |

- Checks after the fixes (2026-09-29):
  - ctest 9/9 (ASan + UBSan)
  - cppcheck and MISRA clean
  - coverage 98.2 % lines / 90.0 % branches; only the defensive GATE/BUSY branches in `uds_client.c` are not covered
  - both M7 cross builds clean
- Re-review after the fixes (2026-09-30):
  - vss-schema-guardian: CLEAN.
  - safety-reviewer: every fix confirmed, no blocker. Four new MINOR findings:
    - MINOR-1: presence did not age out while latched. Fixed: the absence check runs before the latch return; tested.
    - MINOR-2, MINOR-3, MINOR-4 (functional addressing, `not_sent` schedule, H7 filters): documented under known limits.

**Follow-ups.** The client is merged (rt-core#5 and #6). What remains:
- Move the codes in `uds_iso14229.h` into gen/ with a defs `/signal-change`, then delete the header (D-039).
- Q-021: functional request IDs in defs, so the foreign-tester watch covers generic OBD dongles.
- N_As, bus-off backoff and FDCAN filters that pass both request IDs: the H7 HAL (Ç1).
- A platform-bus republisher that reads `services/vehicle_signals` and maps NONE/STALE to INVALID (speed E2E to safety-node, D-021).

## Proposed HIL scenarios (moto-hil-bench, once the host schema exists)

- `isotp_vehicle_segmented_response_refused` (vehicle bus, today's behaviour):
  - The simulated ECU answers a 0x22 request with a First Frame.
  - Pass when rt-core sends no Flow Control, the request ends in `ISOTP_N_TIMEOUT_CR` after N_Cr, and the next Single Frame request succeeds.
  - Do **not** loosen the D-020 gate to make a segmented vehicle response pass; that needs the FC decision above.
- `isotp_segmented_transfer` (platform bus, e.g. against the future UDS server link):
  - A message longer than 7 bytes, with BS = 2 and STmin = 5 ms.
  - Pass when the complete payload arrives, the time between CFs inside a block is at least 5 ms, and there is no `N_TIMEOUT_*`.
  - Fault variants:
    - Drop a middle CF: expect `ISOTP_N_WRONG_SN` on the next one.
    - Drop the last CF: expect `ISOTP_N_TIMEOUT_CR` after N_Cr.
  - In both cases the next message must succeed.
- `uds_client_vehicle_poll_nominal` (vehicle bus, UDS client):
  - The live ECU model answers 0x10 03 and 0x22 for every gen/ DID, and needs the extended session for its reads.
  - Pass when the tester's first frame is 0x10 03 and a tester present goes out every `VEHICLE_CL250_TESTER_PRESENT_PERIOD_MS` (±1 loop period).
  - Over 60 s every DID stays VALID: age ≤ `stale_after_ms` on the platform bus once the republisher exists, or in the rt-core log until then.
  - Every tester frame passes `vehicle_cl250_frame_allowed()`.
- `uds_client_vehicle_dlc_unplug_replug` (vehicle bus, UDS client, needs the H7 N_As abort):
  - Disconnect CAN_H/L for 2 s and for 10 s.
  - Pass when there is no latch, bus load stays bounded while unplugged, and every DID is VALID within `SESSION_RETRY_INTERVAL_MS` + `stale_after_ms` after reconnecting.
- `uds_client_cold_crank_brownout`: the ECU stops ACKing for 200–500 ms. Same pass criteria.
- `uds_client_foreign_tester`: a second tester sends 0x22 on `VEHICLE_CL250_REQUEST_ID`. Pass when rt-core stops sending within one loop and reports `UDS_CLIENT_FAULT_FOREIGN_TESTER`.
- `uds_client_vehicle_ecu_off_on` (vehicle bus, UDS client):
  - The model goes silent for 10 s (ignition off), then comes back in its default session.
  - Pass when:
    - every DID turns STALE within its `stale_after_ms`
    - the ECU is reported absent after `VEHICLE_CL250_ECU_ABSENT_TIMEOUT_MS`
    - while absent only 0x10 03 (every `SESSION_RETRY_INTERVAL_MS`) and 0x3E 80 go out
    - after power-on the session is re-established and every DID is VALID within `SESSION_RETRY_INTERVAL_MS` + 1 s
