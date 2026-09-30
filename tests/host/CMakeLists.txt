# L0 host tests: Unity + ctest, native compiler, sanitizers from the host-tests preset.
include(FetchContent)
FetchContent_Declare(unity
    GIT_REPOSITORY https://github.com/ThrowTheSwitch/Unity.git
    GIT_TAG v2.6.0
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(unity)

# moto_host_test(<name> [libraries...]): one Unity executable per test file.
function(moto_host_test name)
    add_executable(${name} ${name}.c)
    target_link_libraries(${name} PRIVATE ${ARGN} moto_rtcore_logic unity)
    target_compile_options(${name} PRIVATE -Wall -Wextra -Werror)
    add_test(NAME ${name} COMMAND ${name})
endfunction()

moto_host_test(test_isotp_core)
moto_host_test(test_vbus moto_rtcore_hal_host)
moto_host_test(test_timebase moto_rtcore_fw moto_rtcore_hal_host)
moto_host_test(test_can_if moto_rtcore_fw moto_rtcore_hal_host)
moto_host_test(test_isotp_link moto_rtcore_fw moto_rtcore_sim moto_rtcore_hal_host)
moto_host_test(test_uds_client_core)
moto_host_test(test_vehicle_signals moto_rtcore_fw)
moto_host_test(test_uds_client moto_rtcore_fw moto_rtcore_sim moto_rtcore_hal_host)
moto_host_test(test_uds_server_core)
moto_host_test(test_diag moto_rtcore_fw)
moto_host_test(test_uds_server moto_rtcore_fw moto_rtcore_sim moto_rtcore_hal_host)

# SocketCAN backend: Linux only; skips (exit 77) when no vcan0 interface exists.
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    moto_host_test(test_can_port_socketcan moto_rtcore_hal_host)
    set_tests_properties(test_can_port_socketcan PROPERTIES SKIP_RETURN_CODE 77)
endif()

# SIL smoke: the host program against the simulated ECU, real time, 2 s.
add_test(NAME sil_moto_rtcore_host_demo
         COMMAND moto_rtcore_host --duration-ms 2000 --min-responses 20 --quiet)

# SIL: the UDS server on the platform bus, driven end to end by the scripted tester
# (session, multi-frame reads, DTCs, functional requests, P2 check, S3 expiry; ~6 s).
add_test(NAME sil_moto_rtcore_host_uds_server
         COMMAND moto_rtcore_host --uds-scenario --quiet)
