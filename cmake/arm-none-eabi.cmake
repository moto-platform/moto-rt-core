# Toolchain file: arm-none-eabi-gcc for the STM32H7 (Cortex-M7, D-001/D-007).
# H743 and H723 share these core/FPU flags; the board-specific parts (CubeMX HAL,
# startup, linker script) are added once the board is decided.
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Honour a toolchain outside PATH, e.g. -DARM_TOOLCHAIN_DIR=~/.platformio/packages/toolchain-gccarmnoneeabi/bin
set(ARM_TOOLCHAIN_DIR "" CACHE PATH "Directory holding arm-none-eabi-gcc (empty = PATH)")
if(ARM_TOOLCHAIN_DIR)
    set(_prefix "${ARM_TOOLCHAIN_DIR}/arm-none-eabi-")
else()
    set(_prefix "arm-none-eabi-")
endif()

set(CMAKE_C_COMPILER   ${_prefix}gcc)
set(CMAKE_ASM_COMPILER ${_prefix}gcc)
set(CMAKE_AR           ${_prefix}ar)
set(CMAKE_OBJCOPY      ${_prefix}objcopy)
set(CMAKE_SIZE         ${_prefix}size)

# No executable can link without the board's linker script yet: test the compiler
# with a static library instead.
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(MOTO_CPU_FLAGS "-mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard")
set(CMAKE_C_FLAGS_INIT "${MOTO_CPU_FLAGS} -ffunction-sections -fdata-sections -fno-common")
set(CMAKE_ASM_FLAGS_INIT "${MOTO_CPU_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS_INIT "--specs=nano.specs -Wl,--gc-sections")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
