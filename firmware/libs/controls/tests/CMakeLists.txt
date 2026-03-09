cmake_minimum_required(VERSION 3.22)

project(controls_tests LANGUAGES C)

enable_testing()

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../state_estimation ${CMAKE_BINARY_DIR}/state_estimation)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/.. ${CMAKE_BINARY_DIR}/controls)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../unity ${CMAKE_BINARY_DIR}/unity)

add_executable(test_controls
    test_pid.c
    test_flight_controller.c
    test_main.c
)

target_link_libraries(test_controls PRIVATE controls state_estimation unity m)

add_test(NAME controls_tests COMMAND test_controls)
