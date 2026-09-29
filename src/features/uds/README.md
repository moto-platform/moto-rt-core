# features/uds

UDS server (platform bus, FDCAN2) and UDS client (vehicle bus, FDCAN1, the only tester per D-021) on top of an ISO-TP transport layer. Thesis deliverables Ç2 (ISO-TP) and Ç3 (UDS).

Status:
- Done: the ISO-TP core (`isotp_core.{h,c}`), and the link glue that binds it to a CAN port and an ID pair (`isotp_link.{h,c}`).
- Next: the UDS server (Ç3) and the UDS client (the vehicle poller).

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
- Until this is decided, the Ç3 client must:
  - treat `ISOTP_N_TIMEOUT_CR` as "service unavailable"
  - apply `VEHICLE_CL250_DID_SKIP_COOLDOWN_MS`, so one failing request cannot hold the single in-flight slot
  - wait at least N_Bs after an aborted segmented response
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
