cmake_minimum_required(VERSION 3.22)

project(app_tests LANGUAGES C)

enable_testing()

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../lib/unity ${CMAKE_BINARY_DIR}/unity)

# state_exchange is compiled directly (not via the app library) so it can be
# built for the host with IO_TEST_HOOKS; the shared region and the intercore
# signal stub live in the test itself.
add_executable(test_state_exchange
    test_state_exchange.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../shared/state_exchange.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../io/io_intercore_slot/src/io_intercore_slot.c
)

target_include_directories(test_state_exchange PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../io/io_intercore_slot/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../io/io_sys/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../lib/state_estimation/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../lib/controls/include
)

target_compile_definitions(test_state_exchange PRIVATE IO_TEST_HOOKS)

target_link_libraries(test_state_exchange PRIVATE unity m)

add_test(NAME state_exchange_tests COMMAND test_state_exchange)
