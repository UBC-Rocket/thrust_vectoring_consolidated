cmake_minimum_required(VERSION 3.22)

project(controls_tests LANGUAGES C)

enable_testing()

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../state_estimation ${CMAKE_BINARY_DIR}/state_estimation)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/.. ${CMAKE_BINARY_DIR}/controls)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../unity ${CMAKE_BINARY_DIR}/unity)

# PID tests (links against controls library)
add_executable(test_controls
    test_pid.c
    test_main.c
)
target_link_libraries(test_controls PRIVATE controls state_estimation unity m)
add_test(NAME controls_tests COMMAND test_controls)

# Flight controller equation tests (includes .c directly to access static functions)
add_executable(test_equations
    test_flight_controller.c
    ../src/pid.c
)
target_include_directories(test_equations PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../state_estimation/include
)
target_link_libraries(test_equations PRIVATE state_estimation unity m)
add_test(NAME equation_tests COMMAND test_equations)
