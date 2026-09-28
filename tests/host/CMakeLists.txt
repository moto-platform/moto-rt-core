# L0 host tests: Unity + ctest, native compiler, sanitizers from the host-tests preset.
include(FetchContent)
FetchContent_Declare(unity
    GIT_REPOSITORY https://github.com/ThrowTheSwitch/Unity.git
    GIT_TAG v2.6.0
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(unity)

function(moto_host_test name)
    add_executable(${name} ${name}.c)
    target_link_libraries(${name} PRIVATE moto_rtcore_logic unity)
    target_compile_options(${name} PRIVATE -Wall -Wextra -Werror)
    add_test(NAME ${name} COMMAND ${name})
endfunction()

moto_host_test(test_isotp_core)
