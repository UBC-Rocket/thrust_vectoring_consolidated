cmake_minimum_required(VERSION 3.22)

project(trilateration_tests LANGUAGES C)

enable_testing()

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/.. ${CMAKE_BINARY_DIR}/trilateration)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../unity ${CMAKE_BINARY_DIR}/unity)

add_executable(test_trilateration
    test_trilat.c
    test_main.c
)

target_link_libraries(test_trilateration PRIVATE trilateration unity m)

add_test(NAME trilateration_tests COMMAND test_trilateration)
