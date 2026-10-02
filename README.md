# moto-rt-core

Main domain controller firmware for the [moto-platform](https://github.com/moto-platform) motorcycle platform, running on an **STM32H7** (H743/H723 family; the board is not decided yet). It owns CAN on both buses, UDS/ISO-TP, logging, sensor fusion and the cornering EKF estimation (see `CLAUDE.md` for the scope and the safety rules).

Signal, CAN and UDS definitions come only from [moto-vehicle-defs](https://github.com/moto-platform/moto-vehicle-defs) (submodule `external/moto-vehicle-defs`, pinned to `v0.4.0`).

## Layout

```
src/hal/         HAL-free interfaces (can_port, hal_time); host/ = host platform layer
                 (in-process bus, SocketCAN, monotonic ms); stm32/ once the board is chosen
src/services/    timebase, can_if (CAN RX routing + D-020 vehicle guard), vehicle_signals
                 (last sample per DID), diag (DTC memory + vehicle-tester status);
                 signal pool, com, log -- not yet
src/features/    independent function modules (uds/: ISO-TP core + link glue, UDS client =
                 the CL250 vehicle poller, UDS server = rt-core's diagnostics on the
                 platform bus only)
src/app/         host/ = moto_rtcore_host SIL program; target task setup -- not yet
tests/host/      Unity host tests (ctest)
cmake/           arm-none-eabi toolchain file
external/        moto-vehicle-defs (generated C in gen/c/rt_core/)
```

## Build and test

Requires CMake >= 3.20, Ninja, a native C compiler and (for the target build) `arm-none-eabi-gcc`.

```bash
git submodule update --init
cmake --preset host-tests && cmake --build --preset host-tests && ctest --preset host-tests
cmake --preset target-m7-debug && cmake --build --preset target-m7-debug
# toolchain outside PATH: cmake --preset target-m7-debug -DARM_TOOLCHAIN_DIR=/path/to/bin
```

The host tests run with AddressSanitizer and UndefinedBehaviorSanitizer. The native
build also produces `build/host-tests/src/moto_rtcore_host`: rt-core as a host program
(SIL, D-034). It runs the UDS client against an in-process bus with a simulated CL250
ECU and prints the signal table once a second. The UDS server answers on a separate
in-process platform bus; `--uds-scenario` drives a scripted diagnostic session against it.
On Linux, `--vcan vcan0` puts the vehicle port on SocketCAN. ctest runs it for 2 s as
a smoke test and once with `--uds-scenario`. The target
presets cross-compile the pure logic for Cortex-M7 (`-mcpu=cortex-m7 -mfpu=fpv5-d16
-mfloat-abi=hard`); the CubeMX project, startup code and linker script are added once
the board is chosen.

## License

MIT, see `LICENSE` (D-036).
