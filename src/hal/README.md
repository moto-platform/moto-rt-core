# src/hal

HAL-free interfaces that the layers above use, plus one implementation per platform. Only `services/` include these headers. `features/` go through `services/`.

| Interface | What |
|---|---|
| `can_types.h` | Classic CAN frame type, logical ports (`CAN_PORT_VEHICLE` = FDCAN1, `CAN_PORT_PLATFORM` = FDCAN2), status codes, frame/ID checks (`can_frame.c`). Visible to features through `services/can_if.h`. |
| `can_port.h` | Non-blocking write/read/tx-free per port. `services/` and the app setup only, never features: the vehicle-bus guard lives in `services/can_if`. |
| `hal_time.h` | Monotonic millisecond counter (wraps after ~49.7 days). |

## Implementations

- **`host/` (D-034):** the host platform layer for the SIL program and the mock-bus tests. Native builds only.
  - `vbus`: in-process virtual CAN bus, up to 4 nodes. A frame goes to every node except its sender. It has static RX queues, counts overruns, and can make a node's TX mailbox look full (fault injection).
  - `can_port_host`: binds each logical port to a `vbus` node or, on Linux, to a SocketCAN interface (`vcan0`, `can0`, ...). The SocketCAN socket is raw and non-blocking, carries classic frames only, and skips RTR, error and FD frames.
  - `hal_time_host`: CLOCK_MONOTONIC, or a manual clock for tests and fast simulation.
- **`stm32/`:** not yet. FDCAN and the tick timer come once the board (H743/H723, Q-019) and its CubeMX project exist. Nothing above this layer includes STM32 HAL headers.
