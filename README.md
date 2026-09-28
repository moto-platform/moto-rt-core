# moto-rt-core

Main domain controller firmware for the [moto-platform](https://github.com/moto-platform) motorcycle platform, running on an **STM32H7** (H743/H723 family; the board is not decided yet). It owns CAN on both buses, UDS/ISO-TP, logging, sensor fusion and the cornering EKF estimation (see `CLAUDE.md` for the scope and the safety rules).

Signal, CAN and UDS definitions come only from [moto-vehicle-defs](https://github.com/moto-platform/moto-vehicle-defs) (submodule `external/moto-vehicle-defs`, pinned to `v0.1.0`).

## Layout

```
src/hal/         hardware abstraction (thin wrappers over the STM32 HAL) -- not yet
src/services/    signal pool, com, diag, timebase, log -- not yet
src/features/    independent function modules (uds/ today: the ISO-TP core)
src/app/         task/loop setup, called from the CubeMX project -- not yet
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

The host tests run with AddressSanitizer and UndefinedBehaviorSanitizer. The target
presets cross-compile the pure logic for Cortex-M7 (`-mcpu=cortex-m7 -mfpu=fpv5-d16
-mfloat-abi=hard`); the CubeMX project, startup code and linker script are added once
the board is chosen.

## License

Not decided yet.
