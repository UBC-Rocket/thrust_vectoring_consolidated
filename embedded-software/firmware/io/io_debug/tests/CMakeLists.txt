cmake_minimum_required(VERSION 3.22)
project(io_debug_tests LANGUAGES C)

enable_testing()

# Unity stays as a built library (same pattern as lib/messages/tests).
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../../lib/unity ${CMAKE_BINARY_DIR}/unity)

# Build the portable formatter source directly + the test with its fake
# transport. We deliberately do NOT add_subdirectory(io_debug): that lib
# links io_sys (a firmware INTERFACE target carrying HAL-adjacent include
# paths). For the host test we only need the io_sys *headers*, added below.
add_executable(test_io_debug
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/io_debug.c
    ${CMAKE_CURRENT_SOURCE_DIR}/test_io_debug.c
)

target_include_directories(test_io_debug PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../io_sys/include
)

target_link_libraries(test_io_debug PRIVATE unity)

set_target_properties(test_io_debug PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
)

# Catch lurking warnings that hide bugs.
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
    target_compile_options(test_io_debug PRIVATE
        -Wall -Wextra -Wpedantic
        -Wno-unused-parameter
    )
endif()

add_test(NAME io_debug_tests COMMAND test_io_debug)
