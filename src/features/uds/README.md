# features/uds

UDS server (platform bus, FDCAN2) and UDS client (vehicle bus, FDCAN1, the only tester per D-021) on top of an ISO-TP transport layer. Thesis deliverables Ç2 (ISO-TP) and Ç3 (UDS).

Status: only the ISO-TP core exists (`isotp_core.{h,c}`). The glue that binds it to the CAN HAL and the timebase, the UDS server and the UDS client come next.

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

## Proposed HIL scenario (moto-hil-bench, once the host schema exists)

`isotp_segmented_did_read`: the simulated ECU answers a 0x22 request with a response longer than 7 bytes, using BS = 2 and STmin = 5 ms. Pass when the complete payload reaches the signal pool, the time between CFs is at least 5 ms, and there is no `N_TIMEOUT_*`. Fault variants: drop a middle CF (expect `ISOTP_N_WRONG_SN` on the next one) or the last CF (expect `ISOTP_N_TIMEOUT_CR` after N_Cr). In both cases the next request must succeed.
