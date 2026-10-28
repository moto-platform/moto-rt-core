# Pure, HAL-free logic: built by every preset, unit-tested on the host (tests/host)
# and cross-compiled for the Cortex-M7. Nothing here may include HAL/RTOS headers.
add_library(vehicle_rtcore_logic STATIC
    features/uds/isotp_core.c
    features/uds/dtc_snapshot_core.c
    features/uds/uds_client_core.c
    features/uds/uds_server_core.c
    features/vehicle_republish/vehicle_republish_core.c
    features/heartbeat/heartbeat_core.c
    hal/can_frame.c
    services/can_sm_core.c
    services/com_core.c
    services/alive_core.c
    services/imu_core.c
    services/lean_health_core.c
    features/cornering/lean_core.c
    features/cornering/lean_tx_core.c
    features/derived/derived_core.c
    features/derived/derived_gear.c
    features/derived/derived_ride.c
)
target_include_directories(vehicle_rtcore_logic PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)
# The UDS cores and the republisher core use the generated tables, gates, timings,
# platform pack functions and E2E (pure C).
target_link_libraries(vehicle_rtcore_logic PUBLIC vehicle_defs_gen)
target_compile_options(vehicle_rtcore_logic PRIVATE ${VEHICLE_WARNINGS})
# The lean core (D-065 item 1) uses sinf/cosf/sqrtf: no errno side effect, IEEE semantics
# kept (never -ffast-math; lean_core.c refuses to build otherwise), and libm.
target_compile_options(vehicle_rtcore_logic PRIVATE -fno-math-errno)
target_link_libraries(vehicle_rtcore_logic PUBLIC m)

# Generated vehicle-defs code for this node (D-003): the CL250 DID table, the
# D-020 request/frame gates, the platform UDS tables, and the platform.dbc pack/encode
# functions with the E2E protection (D-005, D-026). Built with its own flags.
add_library(vehicle_defs_gen STATIC
    ${VEHICLE_DEFS_GEN}/vehicle_cl250.c
    ${VEHICLE_DEFS_GEN}/vehicle_cl250_derived.c
    ${VEHICLE_DEFS_GEN}/platform_uds.c
    ${VEHICLE_DEFS_GEN}/platform.c
    ${VEHICLE_DEFS_GEN}/platform_e2e.c
    ${VEHICLE_DEFS_GEN}/vehicle_e2e.c
)
target_include_directories(vehicle_defs_gen PUBLIC ${VEHICLE_DEFS_GEN})

# Firmware code above the HAL: services/ and the feature glue. It only uses the HAL
# interfaces (hal/can_port.h, hal/hal_time.h, hal/hal_atomic.h, hal/imu.h), so the same objects serve the host
# program and the H7; each platform links its own hal/ implementation.
add_library(vehicle_rtcore_fw STATIC
    services/timebase.c
    services/can_if.c
    services/can_sm.c
    services/vehicle_signals.c
    services/diag.c
    services/com.c
    services/snapshot.c
    services/alive.c
    services/imu.c
    services/lean_health.c
    services/derived_inputs.c
    services/tester_epoch.c
    features/uds/isotp_link.c
    features/uds/uds_client.c
    features/uds/uds_server.c
    features/vehicle_republish/vehicle_republish.c
    features/heartbeat/heartbeat.c
    features/cornering/lean.c
    features/cornering/lean_tx.c
    features/derived/derived.c
    app/comms.c
)
target_link_libraries(vehicle_rtcore_fw PUBLIC vehicle_rtcore_logic vehicle_defs_gen)
# DID 0xF189 (SW version) reports the CMake project version (D-036, D-040).
target_compile_definitions(vehicle_rtcore_fw PRIVATE VEHICLE_RTCORE_VERSION="${PROJECT_VERSION}")
target_compile_options(vehicle_rtcore_fw PRIVATE ${VEHICLE_WARNINGS})

# Host platform layer + SIL program (D-034): native builds only.
if(NOT CMAKE_CROSSCOMPILING)
    add_library(vehicle_rtcore_hal_host STATIC
        hal/host/vbus.c
        hal/host/can_port_host.c
        hal/host/hal_time_host.c
        hal/host/hal_atomic_host.c
        hal/host/imu_host.c
    )
    target_link_libraries(vehicle_rtcore_hal_host PUBLIC vehicle_rtcore_logic)
    target_compile_definitions(vehicle_rtcore_hal_host PRIVATE _POSIX_C_SOURCE=200809L)
    target_compile_options(vehicle_rtcore_hal_host PRIVATE ${VEHICLE_WARNINGS})

    add_library(vehicle_rtcore_sim STATIC
        app/host/sim_ecu.c
        app/host/sim_tester.c
        app/host/sim_listener.c
    )
    target_link_libraries(vehicle_rtcore_sim PUBLIC vehicle_rtcore_hal_host vehicle_defs_gen)
    target_compile_options(vehicle_rtcore_sim PRIVATE ${VEHICLE_WARNINGS})

    add_executable(vehicle_rtcore_host app/host/main.c)
    target_link_libraries(vehicle_rtcore_host PRIVATE vehicle_rtcore_fw vehicle_rtcore_sim vehicle_rtcore_hal_host)
    target_compile_definitions(vehicle_rtcore_host PRIVATE _POSIX_C_SOURCE=200809L)
    target_compile_options(vehicle_rtcore_host PRIVATE ${VEHICLE_WARNINGS})
endif()
