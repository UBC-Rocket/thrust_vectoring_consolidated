cmake_minimum_required(VERSION 3.22)

project(sensors_tests LANGUAGES C)

enable_testing()

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/.. ${CMAKE_BINARY_DIR}/sensors)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../unity ${CMAKE_BINARY_DIR}/unity)

add_executable(test_sensors
    test_main.c
    test_icm40609_builders.c
)

target_link_libraries(test_sensors PRIVATE sensors unity m)

add_test(NAME sensors_tests COMMAND test_sensors)
