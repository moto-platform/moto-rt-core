# features/uds

UDS server (platform bus, FDCAN2) and UDS client (vehicle bus, FDCAN1, the only tester per D-021) on top of an ISO-TP transport layer. Thesis deliverables Ç2 (ISO-TP) and Ç3 (UDS).

Status:
- Done:
  - the ISO-TP core (`isotp_core.{h,c}`)
  - the link glue that binds it to a CAN port and an ID pair (`isotp_link.{h,c}`)
  - the UDS client, the CL250 vehicle poller (`uds_client.{h,c}`, `uds_client_core.{h,c}`, Ç3)
  - the UDS server on the platform bus (`uds_server.{h,c}`, `uds_server_core.{h,c}`, Ç3, D-040)
- ISO 14229 codes come from gen/ `uds_iso14229.h` (moto-vehicle-defs v0.3.1, D-040); the server contract (IDs, timing, services, DIDs, DTCs) from gen/ `platform_uds.h`.

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

**ISO codes.** The generic ISO 14229-1 codes (0x22, 0x7F, NRC 0x78/0x7E/0x7F) come from the generated `uds_iso14229.h` (D-040). The temporary local header of D-039 is gone. The positive 0x62 check stays in the generated `vehicle_cl250_parse_response()`.

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
- **Reads (D-043).** Of the DIDs that are due (`poll_period_ms` since their last request) and not skipped, the lowest gen/ `priority` value goes first (`VEHICLE_CL250_PRIORITY_HIGH`, today only 0xF40D), then table order. One request is in flight (`REQUESTS_IN_FLIGHT` = 1). When several requests are due, the order is session, then tester present, then reads.
  - Priority changes the order only, never what is sent: every request still passes the D-020 gates.
  - **No starvation (nominal case).** While every request holds the slot for at most `ASSUMED_ROUND_TRIP_MS`, every DID gets a new sample within its `stale_after_ms`. The bound is non-preemptive fixed priority: period + one blocking round trip + higher-priority interference + its own round trip ≤ `stale_after_ms`. Today: 0xF40D 140/300, 0xF40C 110/150, 0xF411 300/600, 0xF405 960/2400, 0xF442 1000/2400 ms.
    - The defs codegen checks it from the defs release after v0.3.1 on (`yaml_checks.did_sample_gap_bounds`, Unreleased in v0.3.1). `test_the_gen_table_meets_its_gap_bound` recomputes it from gen/ here, and `test_no_did_starves_*` checks the scheduler against it: 20 ms and 1 ms round trips, 40 seeded runs with random phases and round trips of 1-20 ms, and a poller that steps every 10 ms, so an answer is seen up to one step after it arrives (ISSUES E-6 n4). That model needs every `poll_period_ms` to be a multiple of the target step (asserted in the test); otherwise each gap gets up to one more step of release jitter; every request is checked to hold the slot for at most `ASSUMED_ROUND_TRIP_MS`.
    - `ASSUMED_ROUND_TRIP_MS` must cover the ECU's answer **plus one `uds_client_step()` period** of the target task, since a request holds the slot until the step that sees the answer. The 0.8 polling budget already caps it at 21 ms with today's table, so the D-029 bench measurement (ECU round trip + target step) must re-run `make check` in defs. Tester present (once per `TESTER_PRESENT_PERIOD_MS`) uses one step without holding the slot and is not in the model.
  - After a **timeout** the DID's period restarts at the timeout. Otherwise a silent 0xF40D (period 100 ms = the base timeout) would be due again at once and, served first, hold the slot until it is skipped.
  - **Fault mode (D-050, ISSUES E-5).** A DID whose last read timed out (`consecutive_timeouts > 0`) is faulty until it answers or is skipped. Meanwhile it competes in the normal class whatever its gen/ priority, and NRC 0x78 does not extend its read: the read ends `RESPONSE_TIMEOUT_BASE_MS` (100 ms) after it was sent. An answer or the skip gives it its gen/ priority and the 0x78 extension back.
    - Why the hold time, not only the class: one request is in flight, so a faulty read starves the others by how long it holds the slot. Demotion alone would leave an endless-0x78 0xF40D at 2000 ms per attempt.
    - Result: after the first failing attempt, a silent 0xF40D or endless NRC 0x78 on it leaves every other DID's request gap within its `stale_after_ms`. For 0xF40C that is its period + the base timeout = 50 + 100 = 150 ms, exactly its `stale_after_ms` (still VALID: STALE needs age > `stale_after_ms`), so round-trip jitter and the target step period come on top. Simulated over 30 s: with a silent 0xF40D RPM is never STALE; with endless 0x78 RPM is STALE about 25 % of the time (the fresh attempts), down from about 63 %.
    - The 150 ms relies on 0xF40C being the first normal DID in the gen/ table: another normal DID ahead of it would add one round trip. The defs codegen bound covers the nominal case only.
    - Tests: `test_silent_speed_keeps_the_others_within_stale_after_ms`, `test_endless_0x78_on_speed_keeps_the_others_within_stale_after_ms` (20 ms and 1 ms round trips, 30 s with several skip cycles) and `test_faulty_speed_with_random_round_trips_adds_only_the_jitter` (20 seeded runs, 1-20 ms); each faulty attempt is checked to hold the slot for at most the base timeout.
  - **Not covered by D-050** (fail-safe: the samples go STALE, since VALID/STALE is decided by sample age in `services/vehicle_signals`, never by the scheduler):
    - The **fresh attempt**: the first failing read, and the first after each skip cooldown, may hold the slot up to `RESPONSE_TIMEOUT_MAX_MS` (2000 ms) with endless 0x78.
    - A **slow but answering** high-priority DID (ISSUES E-7): if 0xF40D answers after a 0x78, later than its 100 ms period, it never times out, keeps its priority and the extension, and is due again when its answer lands. Every normal DID then starves with no skip and no counter. An ECU that alternates answers and endless 0x78 keeps clearing the fault state the same way. Nominal "no starvation" assumes the round trip stays within `ASSUMED_ROUND_TRIP_MS`.
    - A **segmented reception** pauses the base timeout (up to N_Cr), and the abort hold follows; the DID is then skipped, so this happens once per cooldown.
    - An NRC carries the SID, not the DID. A 0x78 that the ECU keeps sending for a timed-out read extends the next 0x22 read, up to `RESPONSE_TIMEOUT_MAX_MS` from that read's request; that DID is then faulty too (`test_a_late_0x78_extends_the_next_read_only_up_to_the_cap`).
    - A late positive answer to a timed-out read can be accepted as the answer to the next request of the same DID. Its sample is stamped on arrival, so its age is underestimated by up to one timeout (ISSUES E-7). D-050 makes this more likely, since a faulty read ends after 100 ms.
    - **Assumption:** a faulty read is abandoned 100 ms after the ECU said "response pending", and a new 0x22 may go out while the ECU is still busy. Whether the CL250 then answers NRC 0x21, ignores it or drops the session is unknown. It stays an assumption until it is seen on the real bus (D-029 bench); a HIL ECU model cannot show it.
- **Response timeout.**
  - `RESPONSE_TIMEOUT_BASE_MS`.
  - NRC 0x78 for the pending SID restarts it, doubled, up to `RESPONSE_TIMEOUT_MAX_MS` (100 → 200 → 400 → 800 → 1600 → 2000).
  - A request never waits more than `RESPONSE_TIMEOUT_MAX_MS` in total since it was sent, so an ECU that answers 0x78 forever cannot hold the slot. The doubling and this total cap are the legacy rule (`docs/legacy-telemetry-notes.md`).
  - Except for a faulty DID's read (D-050): its 0x78 is counted but restarts nothing, so the read ends `RESPONSE_TIMEOUT_BASE_MS` after it was sent.
- **Skip.** `MAX_CONSECUTIVE_TIMEOUTS` timeouts in a row skip a DID for `DID_SKIP_COOLDOWN_MS`. A decoded answer, an NRC other than 0x78, or the skip resets the count, and with it the D-050 fault state.
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
- **TX readiness.** No request is produced while `isotp_link_tx_ready()` is false (the link is sending, or the mailbox is full because no node ACKs: DLC unplugged, ECU off, cold-crank brownout). The timers keep running. At most one stuck request goes out when the bus frees. A request the link refuses as busy is dropped without a latch (`uds_client_core_not_sent()`). It is not counted in `requests`, and its schedule is restored: a read's DID, the session retry or tester present is due again at once instead of losing a period (ISSUES E-6 n1/n2). The path is defensive: `tx_ready` already requires the link to be idle, and a refused request puts no frame on the bus.
- **Fail-closed latch** (`uds_client_fault()` reports the first reason). The client sends nothing more until it is opened again, and its session reads down:
  - `UDS_CLIENT_FAULT_GATE`: the link refused a request (the D-020 gate or the Single Frame length). Only a bug can cause it.
  - `UDS_CLIENT_FAULT_GUARD`: the `can_if` vehicle guard refused any frame.
  - `UDS_CLIENT_FAULT_FOREIGN_TESTER`: a frame on `VEHICLE_CL250_REQUEST_ID`, `VEHICLE_CL250_FALLBACK_REQUEST_ID` or one of the OBD functional request IDs in `vehicle_cl250_functional_watch[]` (0x7DF, 0x18DB33F1; Q-021 → D-040, watch-only, never sent). The controller never receives its own frames, so this is a second tester, for example a generic OBD dongle (D-021, same as connectivity-node, D-030). `uds_client_open()` refuses to run without these watches (`ISOTP_ERR_ARG` when `can_if` has no room).
  - The FC.CTS that the link drops for a segmented response is expected, and does not latch.
- **Sticky STALE.** `vehicle_signals_expire()` runs every step. Once a sample is STALE it stays STALE until the next write, so the 32-bit ms counter wrapping (about 49.7 days) cannot make an old value VALID. ECU presence also needs a new answer after absence.
- **Known limits.**
  - A First Frame with FF_DL above `UDS_CLIENT_RX_BUF` (64) is dropped silently by the core. The request ends by timeout and counts towards the skip limit. No CL250 DID does this.
  - Each segmented response stops polling for about N_Cr + N_Bs (about 2 s), so every DID goes STALE. This is the cost of the Q-020 deferral.
  - A DID answered with a permanent NRC (for example 0x31) is polled at its full rate. This is legacy behaviour, and the schedule bounds it.
  - On the H7, the FDCAN acceptance filters must pass `VEHICLE_CL250_REQUEST_ID`, `VEHICLE_CL250_FALLBACK_REQUEST_ID` and every `vehicle_cl250_functional_watch[]` ID. Otherwise the foreign-tester watch is deaf. This is an Ç1 HAL requirement, checked on target by `uds_client_foreign_tester`.
  - N_As (a TX that is never confirmed) and bus-off recovery (`VEHICLE_CL250_BUS_OFF_BACKOFF_*`, D-030's latch after 5 bus-offs) come with the H7 FDCAN HAL (Ç1).
- **Counters** (`uds_client_stats()`): requests, reads, timeouts, NRC, response pending, unavailable, skips, unexpected, session starts and losses.
- **Diagnostics (D-040).** Every step reports to `services/diag`, level-triggered:
  - DTC `VEHICLE_ECU_COMM_LOST` (U0100-00): the ECU is absent, counted only once `ECU_ABSENT_TIMEOUT_MS` has passed since open (a sticky flag, so the ms counter wrap cannot disarm it).
  - DTC `VEHICLE_TESTER_LATCHED` (U3000-00): the latch.
  - The 0xFD00 status: ECU present, session up, latched, latch reason (gen/ values).
  - A 0x14 clear on the platform bus resets the DTC records only. The next step sets them again while the condition lasts, and the latch is never released by it.

**Memory.** `uds_client_t` is 364 B on the M7 (2026-10-01: 360 B after E-4/D-050, +4 B for the E-6 `not_sent` request kind; 348 B before the D-040 diagnostics): the link, 64 + 8 B of buffers, and the core. It must have static storage duration. `vehicle_signals` adds 81 B, and the glue adds 4 B (the foreign-frame counter). Flash is about 2 kB: core 1286 B (1094 B before D-050, 1198 B before E-6), glue 622 B, service 252 B (release build; core 2026-10-01, the rest 2026-09-29). There is no heap, and every loop is bounded by the DID count or a frame length.

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
    - MINOR-2, MINOR-3, MINOR-4 (functional addressing, `not_sent` schedule, H7 filters): documented under known limits. MINOR-3 fixed later (ISSUES E-6 n2): `not_sent` restores every kind of request.

**Review of D-050 (2026-10-01).** vss-schema-guardian: CLEAN (defs v0.3.1). safety-reviewer: no blocker; the fault-state lifecycle, the 150 ms gap and the unchanged traffic were confirmed.
- MAJOR-1, a slow but answering high-priority DID still starves the others (there before D-050): documented under "Not covered", user decision ISSUES E-7.
- MINOR-2 (STALE share quantified), MINOR-3 (late 0x78 test, HIL wording), MINOR-4 (segmented exception), MINOR-6 (abandon-after-0x78 assumption): applied here.
- MINOR-5, late answer credited to the next request: documented, ISSUES E-7.
- NIT-7 (line length) fixed, NIT-9 (table order) documented. Tests added: the late 0x78 on the next read, and 0x78 on the session request while a DID is faulty.

**Follow-ups.** The client is merged (rt-core#5 and #6); the ISO codes and Q-021 are done (D-040). What remains:
- N_As, bus-off backoff and FDCAN filters that pass the request and watch IDs: the H7 HAL (Ç1).
- A platform-bus republisher that reads `services/vehicle_signals` and maps NONE/STALE to INVALID (speed E2E to safety-node, D-021).

## UDS server, platform bus (`uds_server.h`, `uds_server_core.h`)

**Responsibility.** rt-core's own diagnostic server on the platform bus (FDCAN2), for a workshop tool, the Raspi or the HIL host (Ç3, D-040). It serves rt-core's DIDs and DTCs. It never opens, reads or writes the vehicle bus: the link is opened with `isotp_link_open()`, which refuses `CAN_PORT_VEHICLE`, and nothing was added to the vehicle port or the `can_if` guard (D-020, D-037).
- `uds_server_core` is the pure state machine: no HAL, services or RTOS, time passed in, data through a provider table. Tested in `tests/host/test_uds_server_core.c`.
- `uds_server` is the glue: the platform ISO-TP link, the functional receiver, `services/timebase`, and the providers (`services/diag`, `services/vehicle_signals`). Tested end to end in `tests/host/test_uds_server.c`, and in SIL by `moto_rtcore_host --uds-scenario`.
- It never includes `uds_client.h`. The client publishes its state through `services/diag`, so either module can be removed without breaking the other (architecture-guard, 2026-09-30).

**Addressing and transport** (all from gen/ `platform_uds.h`):
- physical `PLATFORM_UDS_PHYS_REQUEST_ID` (0x710) → `PLATFORM_UDS_PHYS_RESPONSE_ID` (0x718), 11-bit, segmented in both directions, padding `PLATFORM_UDS_PADDING_BYTE`, BS / STmin / N_Bs / N_Cr from gen/, requests up to `PLATFORM_UDS_RX_BUFFER` (64) bytes (longer: FC.OVFLW)
- functional `PLATFORM_UDS_FUNCTIONAL_REQUEST_ID` (0x7DF): Single Frames only (a functional First Frame is dropped), answered physically on 0x718

**Services** (ISO 14229-1 clause; sessions from gen/):

| SID | Service | Sessions | Notes |
|---|---|---|---|
| 0x10 | DiagnosticSessionControl (10.2) | default, extended | 0x01 / 0x03. 0x02 programming → NRC 0x12 until the bootloader (Ç5). The answer carries P2 (1 ms units) and P2* (10 ms units) from gen/. suppressPosRsp honoured |
| 0x3E | TesterPresent (10.6) | default, extended | zeroSubFunction; suppressPosRsp honoured |
| 0x22 | ReadDataByIdentifier (11.2) | default, extended | 1 to `PLATFORM_UDS_MAX_READ_DIDS` (4) DIDs, answered in request order. Unknown DIDs are left out; NRC 0x31 only if none is known |
| 0x19 | ReadDTCInformation (12.3) | default, extended | 0x01 count, 0x02 by status mask, 0x0A supported DTCs. Status masked by the availability mask 0x09. 0x19 has no suppressPosRsp bit, so 0x81 and above get NRC 0x12 |
| 0x14 | ClearDiagnosticInformation (12.2) | **extended only** | group 0xFFFFFF only, else NRC 0x31. Clears rt-core's RAM DTC records, never the UDS client latch, never anything on the vehicle |

**DIDs:**
- 0xF186 active session, served by the core
- 0xF189 SW version: `MOTO_RTCORE_VERSION` = the CMake project version, NUL-padded to 12 bytes
- 0xFD00 vehicle-tester status: byte 0 = ECU present / session up / latched bits, byte 1 = latch reason
- 0xFD01 uptime in seconds since `uds_server_open()`
- 0xFD10–0xFD14 the five CL250 samples from `services/vehicle_signals`: [state 0 NONE / 1 VALID / 2 STALE][age ms, big-endian, saturates at 0xFFFF, 0xFFFF for NONE][raw, big-endian]

**DTCs** (`services/diag`, RAM only until the H7 flash driver, D-040):
- 0xC10000 U0100-00, CL250 ECU communication lost
- 0xF00000 U3000-00, vehicle UDS client latched

**NRC order** (clause 7.5):
- 0x11 service not supported
- 0x7F service not in the active session
- 0x13 too short for a sub-function
- 0x12 sub-function not supported
- 0x13 wrong length
- 0x31 request out of range
- 0x22 conditions not correct (a provider failed)
- 0x14 response too long (not reachable with the gen/ sizes; kept as a guard)

**Functional requests.** NRC 0x11, 0x12, 0x31, 0x7E and 0x7F are not sent (gen/ `platform_uds_nrc_suppressed_functional()`, clause 7.5), and suppressPosRsp is honoured. NRC 0x78 is sent as for a physical request. Once a 0x78 went out, the final answer is always sent, even with suppressPosRsp or a functionally suppressed NRC, because the tester is waiting for it. A functional request is taken only while the server is idle: nothing pending, no answer queued, no physical reception or transmission running. Otherwise it is dropped and counted (`uds_server_glue_stats()`).

**Timing** (ISO 14229-2, values from gen/):
- **P2.** The answer is built in the same `uds_server_step()` as the request. In SIL the worst request-to-first-frame time was 2 ms, against a P2 of 50 ms.
- **P2\*.** A provider may answer PENDING, for example a future flash-backed clear. The request then gets NRC 0x78 at once and again every P2\* / 2. When the provider is ready, the final answer follows. If it is still pending P2\* after the request arrived, the request ends with NRC 0x10, so one request cannot hold the server.
- **Busy.** While a request is pending, a new physical request gets NRC 0x21, a functional one is dropped, and 0x3E 80 only restarts S3.
- **S3.** A non-default session with no request for `PLATFORM_UDS_S3_SERVER_MS` falls back to the default session. Every request restarts S3; a pending request holds it.

**Failure behaviour.**
- An empty request, one longer than the receive buffer, or a failed reception (N_Cr, wrong SN) gets no answer and is counted.
- **Queued answer.**
  - While the link is busy (`ISOTP_ERR_BUSY`), the answer stays queued and is retried every pass (`tx_busy`). S3 keeps running meanwhile (`uds_server_core_tick()`).
  - It is dropped once it is P2\*server old (`tx_expired`), so the glue never delivers a stale answer later.
  - An answer the link already accepted (frames in the core or the mailbox) can still go out late after a bus stall until N_As exists (Ç1 FDCAN HAL; safety re-review MINOR-B).
  - A functional request that arrives while an answer is queued is dropped and counted, never run late (safety re-review MINOR-A).
  - Any other link error drops it at once (`tx_failed`), as does a failed N_USData.confirm.
- **Vehicle-tester status is fail-safe** (safety review MAJOR-1). `services/diag` stamps the client's status. If none has come since boot, or the last one is older than `PLATFORM_UDS_VEHICLE_TESTER_STATUS_MAX_AGE_MS` (500 ms):
  - 0xFD00 reads FAULT = NOT_RUNNING (4) with every flag clear.
  - `diag_supervise()`, called by the server every pass, fails U3000-00.
  - The stale flag is sticky until a new status arrives, so the ms wrap cannot revive an old one.
  - A client that failed to open or stopped therefore never looks healthy.
- A provider that fails gives NRC 0x22.
- If a gen/ DID has no source here, or its layout changes (for example the vehicle sample length), reading it gives NRC 0x22 rather than a guessed value.
- **No security access.** There is no 0x27 yet. The extended session is the only barrier before 0x14, and it clears RAM DTC records only. Treat the platform bus as unauthenticated: nothing reachable from the Raspi or the phone may assume a tester was authenticated.

**Memory.**
- M7 RAM: `uds_server_t` is 468 B (link, 64 B rx, 57 B tx, core with a 64 B request copy, 57 B answer buffer). `services/diag` is 12 B.
- M7 flash (release, 2026-09-30, after the safety fixes): core 1396 B, glue 1030 B, diag 304 B, gen tables 270 B, `isotp_single_frame()` about 40 B.
- `CAN_IF_MAX_RECEIVERS` went from 8 to 12:
  - vehicle: the link, 2 physical and 2 functional watches
  - platform: the server's physical and functional receivers
  - room for the SIL tester and for growth
- No heap. Every loop is bounded by a gen/ table size or the request length.

**Integration.**
- Call `diag_init(now)` as the last setup step before the main loop (the opens do not use `diag`). Then, every pass, run `uds_client_step()` **before** `uds_server_step()`, and `uds_server_step()` after `can_if_dispatch(CAN_PORT_PLATFORM, ...)`.
  - Why the order matters (safety re-review MINOR-C): the client's first status must arrive before `diag_supervise()` runs.
  - If the server ran first and more than `PLATFORM_UDS_VEHICLE_TESTER_STATUS_MAX_AGE_MS` passed between `diag_init()` and the first pass, U3000-00 would be confirmed at every boot.
  - That error is in the fail-safe direction, but it is a false fault.
- **Ç1 requirement, one comms task** (safety review MINOR-3):
  - `can_if`, `services/diag`, `services/vehicle_signals`, `uds_client` and `uds_server` run in the same FreeRTOS task.
  - Every `can_if_register_rx()` happens before the scheduler starts.
  - If this is ever split, those services need a critical section or a snapshot. Otherwise the 0xFD00 status, a sample, or the one-pass 0x14 window can tear.
- **Ç1 requirement, FDCAN2 queueing** (safety review MINOR-4):
  - One pass can queue up to 9 server frames (First Frame + 8 CF at STmin 0), about 2.5 ms at 500 kbit/s. That is well inside the 3 × 20 ms E2E timeout, but it is a priority inversion.
  - FDCAN2 TX must use the Tx-Queue (priority) mode, or keep dedicated TX buffers for the safety range 0x010-0x08F.
  - The RX filters must route 0x700-0x7FF to FIFO1 and the safety range and heartbeats to FIFO0.
  - The FDCAN2 filters must pass 0x710 and 0x7DF.

**Known limits.**
- 0xFD01 uptime wraps to 0 after about 49.7 days (32-bit ms / 1000). It is informational only. The server and glue counters also wrap; they are not saturating like `can_if`'s.
- **The functional watch IDs are not verified on the CL250** (safety review MINOR-7). OEM traffic on 0x7DF or 0x18DB33F1 would latch the client at boot. The latch is fail-closed, so this costs availability, not safety. Include both IDs in the Q-001 listen-only probe before the first rt-core ride (D-040), and log the unrouted and watch counters in the first on-bike session.
- **No 0x27 yet.** This is acceptable while 0x14 only clears RAM DTCs. Before the DTC memory becomes flash-backed (0x14 flash wear), or before 0x10 02, 0x11, 0x2E, 0x31 or 0x34-0x37 are offered, 0x27 or an equivalent is mandatory (D-040).

**Requirement IDs.** None yet (Q-006).

**Reviews of the UDS server (2026-09-30).**
- architecture-guard (plan): OK with changes, all applied.
  - The server stays in `features/uds`, the DTC store goes in `services/diag`, and the server never includes the client.
  - DTCs are level-triggered.
  - A test checks that the vehicle gate still refuses 0x14 and 0x10 02.
- vss-schema-guardian: CLEAN. Every ID, DID, DTC, SID, NRC and timing comes from gen/. The pin must be a tag before merge: defs v0.2.0.
- safety-reviewer: no blocker. The vehicle-bus invariant holds: no path from the server or the diag service to `CAN_PORT_VEHICLE`, the gates are byte-identical, and the functional watch is RX-only. Findings and their status:

| ID | Finding | Status |
|---|---|---|
| MAJOR-1 | 0xFD00 and the DTCs read "healthy" when the client never opened or stopped | Fixed: defs `FAULT = NOT_RUNNING` + `max_age_ms`; stamped, sticky-stale status in `services/diag`; `diag_supervise()` fails U3000-00; tests |
| MINOR-1 | A queued answer blocked S3 and the pending expiry and was retried on any error | Fixed: retry on BUSY only, drop after P2\* (`tx_expired`), `uds_server_core_tick()` while queued; tests |
| MINOR-2 | Functional requests never got NRC 0x78 (ISO sends it) | Fixed: 0x78 for functional too; after a 0x78 the final answer is always sent; tests |
| MINOR-3 | The single-task assumption is not enforced | Documented as a Ç1 integration requirement (above) |
| MINOR-4 | FDCAN2 TX/RX queueing could delay safety frames | Documented as a Ç1 HAL requirement (above); HIL `uds_server_bus_flood_isolation` |
| MINOR-5 | Hand-written SF parsing, `7u`, `12u` | Fixed: `isotp_single_frame()` in the core, `ISOTP_SF_MAX_LEN`, gen `PLATFORM_UDS_DID_<NAME>_LENGTH` |
| MINOR-6 | 0xF186 assumed a 1-byte record | Fixed: NRC 0x22 if the gen/ length is not 1 |
| MINOR-7 | Functional watch IDs unverified on the bike | Documented (known limits, D-040 Q-001 probe) |
| MINOR-8 | The uptime DID and counters wrap | Documented (known limits) |
| MINOR-9 | The defs pin is not a tag | Pinned to v0.2.0 before merge |

- safety-reviewer re-review of the fixes: no blocker, no major. Every fix above is confirmed. New findings:

| ID | Finding | Status |
|---|---|---|
| MINOR-A | A functional request that arrived while an answer was queued stayed held and ran up to P2\* late | Fixed: dropped and counted; test |
| MINOR-B | "A dead bus never delivers a stale answer" holds for the glue queue only; the link's accepted frames wait for N_As | Documented; HIL `uds_server_bus_off_recovery` depends on N_As (Ç1) |
| MINOR-C | The boot grace depends on the client-before-server step order | Documented (integration) |

- Checks after the fixes: see the PR description (ctest, coverage, MISRA, cross builds, SIL).


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
- `uds_client_vehicle_faulty_speed` (vehicle bus, UDS client, D-050; ISSUES E-6):
  - The ECU model leaves 0xF40D unanswered for 30 s, then answers NRC 0x78 forever on it for 30 s, then recovers.
  - Pass when only one request is ever in flight, only `tester_policy` services go out, every other DID stays VALID except in the window of each fresh 0xF40D attempt (and at most a few ms per faulty attempt for 0xF40C, see "Reads"; about 25 % RPM STALE in the endless-0x78 phase), and 0xF40D is VALID within `stale_after_ms` after recovery plus at most one skip cooldown.
  - Variants: 0xF40D answering after 0x78 at 150-250 ms (ISSUES E-7, today every normal DID starves), and 0x78 sent on the ECU's own clock after the timeout.
  - This checks rt-core against a model. How the real CL250 behaves needs a capture on the bike.
- `uds_client_cold_crank_brownout`: the ECU stops ACKing for 200–500 ms. Same pass criteria.
- `uds_client_foreign_tester`: a second tester sends 0x22 on `VEHICLE_CL250_REQUEST_ID`. Pass when rt-core stops sending within one loop and reports `UDS_CLIENT_FAULT_FOREIGN_TESTER`.
- `uds_client_vehicle_ecu_off_on` (vehicle bus, UDS client):
  - The model goes silent for 10 s (ignition off), then comes back in its default session.
  - Pass when:
    - every DID turns STALE within its `stale_after_ms`
    - the ECU is reported absent after `VEHICLE_CL250_ECU_ABSENT_TIMEOUT_MS`
    - while absent only 0x10 03 (every `SESSION_RETRY_INTERVAL_MS`) and 0x3E 80 go out
    - after power-on the session is re-established and every DID is VALID within `SESSION_RETRY_INTERVAL_MS` + 1 s
- `uds_server_platform_session` (platform bus, UDS server; the SIL `--uds-scenario` script on the bench):
  - The HIL host is the tester on 0x710 / 0x7DF, and rt-core polls the live ECU model on the vehicle bus at the same time.
  - Pass when:
    - every answer's first frame comes within P2 (50 ms)
    - 0x14 is refused in the default session and accepted in the extended one
    - functional NRCs 0x11 / 0x12 / 0x31 / 0x7E / 0x7F never appear
    - the session drops to default within S3 + 1 loop period
    - no frame appears on the vehicle bus except the client's gen/ requests
- `uds_server_vehicle_status` (both buses):
  - Unplug the vehicle DLC for 10 s.
  - Pass when 0xFD00 reports the ECU absent, U0100-00 is testFailed and confirmed after `ECU_ABSENT_TIMEOUT_MS`, and after reconnecting testFailed clears while confirmed stays until 0x14.
- `uds_server_no_client`: rt-core boots with the vehicle client failed to open. Pass when 0xFD00 reads NOT_RUNNING at once, and U3000-00 is testFailed within `PLATFORM_UDS_VEHICLE_TESTER_STATUS_MAX_AGE_MS` + one loop.
- `uds_server_bus_flood_isolation`:
  - The tester floods 0x710 / 0x7DF at 100 % load and requests segmented 0x22 reads at STmin 0, while rt-core sends 0x020-0x022 E2E frames.
  - Pass when safety-node sees no E2E timeout or counter jump, and the client's poll schedule is unchanged.
- `uds_server_port_mapping` (first H7 bring-up):
  - The platform tester exercises every server service.
  - Pass when the vehicle bus carries only 29-bit 0x18DA10F1 frames that pass the gate. This catches an FDCAN1/FDCAN2 swap.
- `uds_server_bus_off_recovery` (needs N_As from the Ç1 FDCAN HAL):
  - Platform bus-off for 10 s in the extended session.
  - Pass when, after recovery, no stale answer appears, the session is default, and new requests are answered.
- `uds_server_dongle_latch` (both buses):
  - A frame on 0x7DF on the vehicle bus.
  - Pass when the client latches with `FOREIGN_TESTER`, 0xFD00 reports latched with reason 3, U3000-00 is testFailed, and after 0x14 it is testFailed again within one loop while the client stays silent.

