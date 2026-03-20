cmake_minimum_required(VERSION 3.22)

project(state_estimation_tests LANGUAGES C)

enable_testing()

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/.. ${CMAKE_BINARY_DIR}/state_estimation)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../unity ${CMAKE_BINARY_DIR}/unity)

add_executable(test_state_estimation
    test_quaternion.c
    test_matrix.c
    test_ekf.c
    test_eskf_trajectory.c
    test_main.c
)

target_link_libraries(test_state_estimation PRIVATE state_estimation unity m)

add_test(NAME state_estimation_tests COMMAND test_state_estimation)
