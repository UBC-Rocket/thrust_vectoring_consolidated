cmake_minimum_required(VERSION 3.22)
project(test_cobs LANGUAGES C)

enable_testing()

add_library(unity STATIC ../../../lib/unity/src/unity.c)
target_include_directories(unity PUBLIC ../../../lib/unity/src)

add_executable(test_cobs
    test_cobs.c
    ../src/cobs.c
)
target_include_directories(test_cobs PRIVATE ../include)
target_link_libraries(test_cobs PRIVATE unity)
set_target_properties(test_cobs PROPERTIES C_STANDARD 11 C_STANDARD_REQUIRED ON)

add_test(NAME cobs_tests COMMAND test_cobs)
