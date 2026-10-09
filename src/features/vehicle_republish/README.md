# features/vehicle_republish

Platform-bus republisher of the CL250 vehicle signals (ISSUES D-5; decisions D-021, D-041, D-048, D-056). rt-core is the only vehicle-bus tester (D-037): what it reads into `services/vehicle_signals`, every other node gets from the platform bus.

## Responsibility

Reads the last sample of every gen/ DID (`vehicle_signals_get()`) and the ECU presence. Sends them as two gen/ `platform.dbc` messages:

| Message | ID, cycle | Protection | TX path | Receivers | Content |
|---|---|---|---|---|---|
| `VehicleSpeed` | `PLATFORM_VEHICLE_SPEED_FRAME_ID`, `PLATFORM_VEHICLE_SPEED_CYCLE_TIME_MS` (0x021, 50 ms) | E2E (D-005, D-026): `platform_vehicle_speed_e2e_protect()`, DataID `PLATFORM_VEHICLE_SPEED_E2E_DATA_ID` | dedicated replace-on-new buffer (safety-range ID, D-054 item 6) | CONN, LINUX (not SAFETY since D-048) | `VEHICLE_SPEED_VALID`, `VEHICLE_SPEED` from DID 0xF40D, `VEHICLE_SPEED_AGE` |
| `VehicleEngine` | `PLATFORM_VEHICLE_ENGINE_FRAME_ID`, `PLATFORM_VEHICLE_ENGINE_CYCLE_TIME_MS` (0x110, 50 ms) | none (state range) | Tx FIFO | CONN, LINUX | `ENGINE_SPEED` (0xF40C), `BATTERY_VOLTAGE` (0xF442), `COOLANT_TEMP` (0xF405), `THROTTLE_POS` (0xF411), one VALID bit each, `ECU_PRESENT` |

0x021 is **not a safety input**. The Layer 1 cornering decision needs no speed (D-041). The DBC says no safety decision may use 0x021. rt-core's own EKF uses the sample with the D-048 age rule, never this frame.

Every ID, length, scale, range, choice, cycle time and E2E parameter comes from gen/ (`platform.h`, `platform_e2e.h`, `vehicle_cl250.h`, defs v0.5.0). The module names no CAN literal.

## Mapping (D-056 item 2)

- **VALID:** a sample that is VALID (age ≤ the DID's `stale_after_ms`, D-025; 300 ms for 0xF40D, D-048) and inside the DBC signal's physical range (`*_is_in_phys_range()`, which also rejects NaN and ±inf) is sent with its value (generated `*_encode()`, rounding, D-056 item 1) and its VALID bit set.
- **INVALID:** NONE, STALE, non-finite or out of range all give an INVALID bit and value 0.
  - **No clamping.** The 0xF40D range equals the 0x021 range (0-255 km/h, D-048), so an out-of-range value is a fault, not a reading. The "clamp at 255" of ISSUES D-5 became "out of range → INVALID" in D-056.
- **`VEHICLE_SPEED_AGE`:** the sample age through the generated encode. It is rounded to the nearest 10 ms and saturates at 2550 ms. An INVALID speed sends the saturated value, so it never reads as fresh.
  - The age counts from the send time of the DID's oldest unanswered read (D-051), which is before the ECU took the sample.
  - VALID always uses the unrounded age.
  - The time the frame then waits in its TX buffer (at most one write, see below) is not included. A consumer's worst case for a VALID 0x021 is therefore `stale_after_ms` + one cycle + one pass (about 351 ms). The DBC comment of `VEHICLE_SPEED_AGE` ("since rt-core received the source sample") is to be aligned with D-051 and this in a defs patch (ISSUES E-14).
- **`ECU_PRESENT`:** copied from `vehicle_signals_ecu_present()`.
- **0x110 raw values:** they equal the DID raw values, because the DBC uses the J1979 source scaling. The tests check every raw value.

## Timing and order

- **Cycle:** deadline-anchored at the gen/ cycle times. Both messages go out in the same pass. The first frames go out at the first step, INVALID until samples exist. A late pass skips the missed deadlines and never sends a burst (`com_cycle_done()` of `services/com_core`, D-056 item 7; the write and its result classification are `com_send()`).
- **Order:** `app/comms` `comms_pass()`: platform `can_sm_step()` → `can_if_dispatch()` → `vehicle_republish_step()` → `uds_server_step()` (D-056 item 4).
  - The step sends the samples the client wrote in the same pass and sees the platform port's current state.
  - Its E2E frame never waits behind server work.
  - The comms pass period (1 ms, `src/app/README.md`) therefore bounds 0x021's freshness. The 20 ms EKF work (0x020) must not join this task (CLAUDE.md safety rule 4).
- **Write results:**
  - `CAN_PORT_OK`: counted in `sent`, and the cycle is done.
  - `CAN_PORT_TX_FULL`: the replace cancel of the previous 0x021 is still pending, or the Tx FIFO has its one element (hal README erratum). Counted in `retried`, and the frame is re-packed with the then-current sample in the next pass. On a stalled bus, 0x021 therefore alternates between "written" and "cancelled", and its buffer always holds the newest frame.
  - Any other status (bus-off or latched port, `services/can_sm`): counted in `dropped`. The cycle is given up, with no retry storm.

## E2E (D-056 item 5)

- The module owns 0x021's `moto_e2e_tx_state_t`. `vehicle_republish_speed_frame()` protects a copy, and the glue commits it only when `can_if_write()` returned `CAN_PORT_OK`.
- A refused write leaves no counter gap, and its retry carries the same counter.
- A frame replaced in its buffer shows up at the receiver as a counter gap. A gap longer than the 3-cycle timeout makes the receiver resync as INITIAL, never OK (D-026).
- A replace that comes too late lets the old frame out. The new one follows with the next counter, so there is no gap.

## Open

`vehicle_republish_open()` sets `opened` only when can_if accepted 0x021's dedicated buffer; a refused open (sealed, already listed, full) leaves the republisher closed and `vehicle_republish_step()` sends nothing, so 0x021 never goes through the Tx FIFO (safety review MAJOR-1).

## Freshness bound

- A 0x021 frame is never older than one write of this module, because the buffer is replace-on-new (hal README).
- If the comms task stalls, nothing replaces the pending frame. Only the receiver's E2E timeout (150 ms) then keeps it from being used as OK. That is HIL scenario `platform_comms_starved_e2e`.
- A 0x110 frame (Tx FIFO) can wait until N_As (1000 ms) on a stalled bus and then be aborted. It is state-range data with no E2E, and its receivers apply their own timeouts.

## Memory

- **RAM:** `vehicle_republish_t` (two cycles, one E2E state, six counters) is 44 B, owned by the caller. bss does not change. Stack per step is about 150 B (five samples and one frame). There is no heap, and loops are bounded by `VEHICLE_CL250_DID_COUNT`.
- **Flash** (M7 release, `arm-none-eabi-size` of `moto_rtcore_logic` + `moto_rtcore_fw`, 2026-10-03): text 10108 → 10904 B (+796 B; +936 B over v0.7.0) for the core, the glue and the comms change (host-only code is not in these libraries).
  - The gen/ functions the linker keeps from `moto_defs_gen` add about 792 B: the two pack/init functions, 6 encode and 5 `is_in_phys_range` functions, the 0x021 E2E protect, and the CRC.
  - The whole gen `platform.c` archive is 4 KB, but `--gc-sections` drops the unused functions.

## Coverage (D-046)

- `vehicle_republish.c`: 100 % of lines. Two branches are not taken: the "frame not built" side at both writes. `vehicle_republish_*_frame()` fails only if gen/ `*_pack()` refuses a buffer of exactly `*_LENGTH` bytes, which it never does. If it ever did, the cycle would be dropped like a bus-off.
- `vehicle_republish_core.c`: 98 % of lines. The single missing line and branch is the same pack failure inside `vehicle_republish_speed_frame()`, kept as a defensive check of the gen/ contract.
- The E2E path itself (protect, commit only on `CAN_PORT_OK`, retry, drop) is fully covered.

## Tests

- **`tests/host/test_vehicle_republish_core.c`:**
  - the VALID, INVALID, NONE and STALE mapping, the 255 km/h limit with no clamping, NaN and ±inf
  - AGE saturation (2549, 2550, 2556 ms, and huge values) and rounding (294 → 290, 295 → 300)
  - every 0xF40D value
  - every engine DID raw value arriving unchanged on 0x110
  - the E2E check and counter, CRC, and DataID
  - the cycle grid, missed deadlines, and the ms counter wrap
- **`tests/host/test_vehicle_republish.c`** (vbus, manual clock):
  - the cycle times from the first pass
  - frame bytes equal the core's for the current samples, and turn INVALID at `stale_after_ms`
  - TX_FULL retried in the next pass with no counter gap
  - bus-off dropping one cycle at a time, then an INITIAL resync
  - after a stall, only the newest 0x021, never OK across the gap
  - a late replace leaves no gap
  - VALID at exactly `stale_after_ms`, INVALID one ms later
  - TX_FULL longer than a cycle: one frame, the grid kept
  - a refused open sends nothing; the counters saturate
  - nothing on the vehicle bus
- **`tests/host/test_can_sm.c`:** the comms pass sends both messages at their cycle times next to the UDS client and server.
- **SIL** (`moto_rtcore_host`, ctest `sil_moto_rtcore_host_demo`): `app/host/sim_listener` checks every 0x021 with the receiver E2E check, the lengths, and that no gap reaches the 0x021 E2E timeout. The demo fails otherwise.

HIL scenarios (with the board, moto-hil-bench):
- `republisher_speed_stale_invalid`: silence 0xF40D on the restbus ECU. Within 300 ms plus one cycle, 0x021 turns INVALID with AGE 2550 and never shows a small AGE.
- `republisher_platform_stall`: plus the hal README dedicated-buffer scenarios.
- `republisher_error_passive_age`: rt-core alone on FDCAN2 without ACK, then ACK restored. The wire time of every 0x021 against its AGE: wire age ≤ AGE + one cycle + jitter, and the first frame after the gap is INITIAL.
- `republisher_fifo_contention`: a long multi-frame 0xFD02 answer during 0x110 cycles: P2 holds, the CF order is kept, the 0x110 jitter stays under one pass.
