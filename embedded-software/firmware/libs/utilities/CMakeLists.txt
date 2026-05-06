cmake_minimum_required(VERSION 3.22)

project(utilities LANGUAGES C)

add_library(utilities STATIC
    src/clamp.c
)

target_include_directories(utilities
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

set_target_properties(utilities PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
)
