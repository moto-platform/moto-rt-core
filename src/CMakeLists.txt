# Pure, HAL-free logic: built by every preset, unit-tested on the host (tests/host)
# and cross-compiled for the Cortex-M7. Nothing here may include HAL/RTOS headers.
add_library(moto_rtcore_logic STATIC
    features/uds/isotp_core.c
)
target_include_directories(moto_rtcore_logic PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_compile_options(moto_rtcore_logic PRIVATE ${MOTO_WARNINGS})

# Generated moto-vehicle-defs code for this node (D-003). Header-only use for now.
add_library(moto_defs_gen INTERFACE)
target_include_directories(moto_defs_gen INTERFACE ${MOTO_DEFS_GEN})
