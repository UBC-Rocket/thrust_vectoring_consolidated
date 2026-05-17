cmake_minimum_required(VERSION 3.22)
project(timestamp LANGUAGES C)

add_library(timestamp STATIC src/timestamp.c)

target_include_directories(timestamp
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

set_target_properties(timestamp PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
)
