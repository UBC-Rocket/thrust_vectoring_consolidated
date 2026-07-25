cmake_minimum_required(VERSION 3.22)
project(io_intercore_slot LANGUAGES C)

add_library(io_intercore_slot STATIC
    src/io_intercore_slot.c
)

target_include_directories(io_intercore_slot
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

set_target_properties(io_intercore_slot PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
)
