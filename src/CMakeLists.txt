# Pure, HAL-free logic: built by every preset, unit-tested on the host (tests/host)
# and cross-compiled for the Cortex-M7. Nothing here may include HAL/RTOS headers.
add_library(moto_rtcore_logic STATIC
    features/uds/isotp_core.c
    hal/can_frame.c
)
target_include_directories(moto_rtcore_logic PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_compile_options(moto_rtcore_logic PRIVATE ${MOTO_WARNINGS})

# Generated moto-vehicle-defs code for this node (D-003): the CL250 DID table and the
# D-020 request/frame gates. Built with its own flags (generated code).
add_library(moto_defs_gen STATIC
    ${MOTO_DEFS_GEN}/vehicle_cl250.c
)
target_include_directories(moto_defs_gen PUBLIC ${MOTO_DEFS_GEN})

# Firmware code above the HAL: services/ and the feature glue. It only uses the HAL
# interfaces (hal/can_port.h, hal/hal_time.h), so the same objects serve the host
# program and the H7; each platform links its own hal/ implementation.
add_library(moto_rtcore_fw STATIC
    services/timebase.c
    services/can_if.c
    features/uds/isotp_link.c
)
target_link_libraries(moto_rtcore_fw PUBLIC moto_rtcore_logic moto_defs_gen)
target_compile_options(moto_rtcore_fw PRIVATE ${MOTO_WARNINGS})

# Host platform layer + SIL program (D-034): native builds only.
if(NOT CMAKE_CROSSCOMPILING)
    add_library(moto_rtcore_hal_host STATIC
        hal/host/vbus.c
        hal/host/can_port_host.c
        hal/host/hal_time_host.c
    )
    target_link_libraries(moto_rtcore_hal_host PUBLIC moto_rtcore_logic)
    target_compile_definitions(moto_rtcore_hal_host PRIVATE _POSIX_C_SOURCE=200809L)
    target_compile_options(moto_rtcore_hal_host PRIVATE ${MOTO_WARNINGS})

    add_library(moto_rtcore_sim STATIC
        app/host/sim_ecu.c
    )
    target_link_libraries(moto_rtcore_sim PUBLIC moto_rtcore_hal_host moto_defs_gen)
    target_compile_options(moto_rtcore_sim PRIVATE ${MOTO_WARNINGS})

    add_executable(moto_rtcore_host app/host/main.c)
    target_link_libraries(moto_rtcore_host PRIVATE moto_rtcore_fw moto_rtcore_sim moto_rtcore_hal_host)
    target_compile_definitions(moto_rtcore_host PRIVATE _POSIX_C_SOURCE=200809L)
    target_compile_options(moto_rtcore_host PRIVATE ${MOTO_WARNINGS})
endif()
