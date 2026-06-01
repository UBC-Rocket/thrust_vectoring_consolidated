cmake_minimum_required(VERSION 3.22)
project(cobs LANGUAGES C)

add_library(cobs STATIC src/cobs.c)
target_include_directories(cobs PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
set_target_properties(cobs PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
)
