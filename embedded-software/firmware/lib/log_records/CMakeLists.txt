cmake_minimum_required(VERSION 3.22)
project(log_records LANGUAGES C)

add_library(log_records STATIC
    src/log_frame.c
)

target_include_directories(log_records
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

set_target_properties(log_records PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
)
