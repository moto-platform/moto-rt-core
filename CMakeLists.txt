cmake_minimum_required(VERSION 3.20)

# moto-rt-core: STM32H7 domain controller firmware (D-001, D-007).
# Two build flavours, selected by CMakePresets.json:
#   host-tests : native compiler, pure (HAL-free) logic + Unity tests under ctest
#   target-m7  : arm-none-eabi-gcc (cmake/arm-none-eabi.cmake), the same pure logic
#                cross-compiled for Cortex-M7. The CubeMX project, startup code and
#                linker script arrive once the board (H743 or H723) is decided.
project(moto_rt_core VERSION 0.2.0 LANGUAGES C) # semver, tagged vX.Y.Z (D-036)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(MOTO_BUILD_HOST_TESTS "Build the Unity host tests (native toolchain only)" OFF)

set(MOTO_DEFS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/moto-vehicle-defs)
set(MOTO_DEFS_GEN ${MOTO_DEFS_DIR}/gen/c/rt_core)
if(NOT EXISTS ${MOTO_DEFS_GEN}/vehicle_cl250.h)
    message(FATAL_ERROR "Missing ${MOTO_DEFS_GEN}. Run: git submodule update --init")
endif()

# Warnings are errors for our own code. Generated and third-party code is built
# with its own flags (see tests/host).
set(MOTO_WARNINGS -Wall -Wextra -Werror -Wshadow -Wconversion -Wsign-conversion
                  -Wstrict-prototypes -Wmissing-prototypes -Wundef -pedantic)

add_subdirectory(src)

if(MOTO_BUILD_HOST_TESTS)
    if(CMAKE_CROSSCOMPILING)
        message(FATAL_ERROR "MOTO_BUILD_HOST_TESTS needs the native toolchain (preset host-tests)")
    endif()
    enable_testing()
    add_subdirectory(tests/host)
endif()
