# src/hal

HAL-free interfaces that the layers above use, plus one implementation per platform. Only `services/` include these headers. `features/` go through `services/`.

| Interface | What |
|---|---|
| `can_types.h` | Classic CAN frame type, logical ports (`CAN_PORT_VEHICLE` = FDCAN1, `CAN_PORT_PLATFORM` = FDCAN2), status codes, frame/ID checks (`can_frame.c`). Visible to features through `services/can_if.h`. |
| `can_port.h` | Non-blocking write/read/tx-free per port, plus the controller supervision of Ç1: `can_port_get_state()` (TEC, REC, error passive, bus-off, wrapping bus-off and TX-done counters, pending TX), `can_port_recover()`, `can_port_tx_abort()`, `can_port_set_filters()`. None of them sends a frame. `services/` and the app setup only, never features: the vehicle-bus guard lives in `services/can_if`, the state machine in `services/can_sm`. |
| `hal_time.h` | Monotonic millisecond counter (wraps after ~49.7 days). |

## Implementations

- **`host/` (D-034):** the host platform layer for the SIL program and the mock-bus tests. Native builds only.
  - `vbus`: in-process virtual CAN bus, up to 6 nodes. A frame goes to every node except its sender. It has static RX queues and counts overruns. Fault injection: a full TX mailbox, a TX stall (frames wait in 3 TX buffers and go out late when un-stalled, unless aborted), bus-off (counted per entry; the node neither sends nor receives until `vbus_recover()`), TEC/REC, and exact-match acceptance filters.
  - `can_port_host`: binds each logical port to a `vbus` node or, on Linux, to a SocketCAN interface (`vcan0`, `can0`, ...). The SocketCAN socket is raw and non-blocking, carries classic frames only, and skips RTR, error and FD frames. Filters map to `CAN_RAW_FILTER`; a raw socket shows no controller state, so SocketCAN reports a healthy, idle controller: on a real interface (`--allow-real-bus`) the D-030 latch, N_As and the one-TX-buffer rule are not enforced, and the kernel's `restart-ms` recovers bus-offs without limit (the program warns; a development path only, never the vehicle tester). On the vbus, the vehicle port's TX is free only when nothing is pending, like the H7's one TX buffer.
  - `hal_time_host`: CLOCK_MONOTONIC, or a manual clock for tests and fast simulation.
- **`stm32/`:** not yet. FDCAN and the tick timer come once the board (H743/H723, Q-019) and its CubeMX project exist. Nothing above this layer includes STM32 HAL headers.

## H7 FDCAN port requirements (Ç1, ISSUES D-2)

What `hal/stm32/can_port_fdcan.c` must provide. The logic above it (`services/can_sm`, `services/can_if`, the ISO-TP links) is done and host-tested.

| Item | FDCAN1, vehicle bus (`CAN_PORT_VEHICLE`) | FDCAN2, platform bus (`CAN_PORT_PLATFORM`) |
|---|---|---|
| Mode | Classic CAN, 500 kbit/s (`VEHICLE_CL250_BITRATE`), automatic retransmission on | Classic CAN, 500 kbit/s (D-009), automatic retransmission on |
| TX | One dedicated TX buffer: `can_port_tx_free()` only when it is empty (one request in flight, D-021) | Dedicated TX buffers (TXBC.NDTB) for the safety range 0x010-0x07F and the heartbeats, one per ID, replace-on-new (abort the old pending frame, then write): an E2E frame is never older than one cycle and never waits behind other traffic. Everything else (UDS 0x718, ...) in a **Tx FIFO** (not Tx-Queue mode), so frames of one ID keep their order (ISO-TP CF sequence numbers, E2E counters); the FIFO head and the dedicated buffers still arbitrate by ID (D-054, safety review MAJOR-3) |
| RX filters | Exactly the `can_if` receivers (`can_if_apply_filters()` after all receivers are registered): `VEHICLE_CL250_RESPONSE_ID` and the watch IDs (both request IDs and `vehicle_cl250_functional_watch[]`) | The `can_if` receivers; the gen/ platform UDS IDs (0x7xx) into RX FIFO1, everything else FIFO0 |
| Filter elements | Exact match: standard filters SFT dual-ID (or classic with a full mask), extended filters likewise with XIDAM = 0x1FFFFFFF; SFEC/EFEC select FIFO0/FIFO1 | Same |
| Non-matching / remote frames | Rejected (GFC ANFS/ANFE, RRFS/RRFE) | Rejected |
| Start order | GFC, SIDFC, XIDFC, XIDAM, TXBC are written with CCCR.INIT = 1 and CCE = 1: init, register every receiver, apply filters, then clear INIT. TX stays recessive until then (Q-018 parity); writes are refused while CCCR.INIT or PSR.BO is set | Same |
| `can_port_read()` | FIFO0, then FIFO1; FxOM blocking mode, a lost message (RFxL) counted as an overrun | Same |
| ISR | IE.BOE and IE.TCE (TXBTIE per buffer, or diff TXBTO: one IR.TC can cover several buffers). Copies received frames into a static queue, counts bus-off **entries** (IR.BO with PSR.BO = 1) and TX completions; nothing else | Same |
| `can_port_get_state()` | ECR.TEC, ECR.REC (ECR.RP → `error_passive` and REC 128), PSR.EP/BO, the ISR counters, pending = TXBRP bits. Never fails on a bound port (a failure refuses TX in `can_sm`) | Same |
| `can_port_recover()` | Clear CCCR.INIT (the controller rejoins after 128 × 11 recessive bits). `can_sm` calls it only with nothing pending | Same |
| `can_port_tx_abort()` | Non-blocking: set TXBCR for every pending buffer and return; completion shows as cleared TXBRP in the next `get_state()` (`can_sm` repeats the abort and defers a recovery while anything is pending, counted in `recover_deferred`). A frame already in arbitration can still complete: `tx_done` counts TXBTO only. HIL: confirm that a cancel completes while CCCR.INIT = 1 (bus-off) | Same |
| Message RAM | FDCAN1 and FDCAN2 share it: lay out both ranges (filters, RX FIFOs, TX buffers) so they never overlap | Same |

N_As (`CAN_SM_TX_TIMEOUT_MS`) is per port and counts any TX progress, so on FDCAN2 a starved frame is bounded only by N_As after the port's last progress. It is not the freshness bound of E2E frames: their dedicated, replace-on-new buffers are.

The bus-off count lives in RAM (D-054). Keeping it across a reset (a no-init or backup-SRAM section, like conn's RTC record, D-030) needs the linker script and comes with the board. Until then a fault that causes a bus-off and then a reset gets 5 new bus-offs per boot (accepted, D-054).
