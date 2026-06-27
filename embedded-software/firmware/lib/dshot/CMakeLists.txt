cmake_minimum_required(VERSION 3.22)

add_library(dshot STATIC)

target_include_directories(dshot
    PUBLIC
        include/
    PRIVATE
        src/
)

target_sources(dshot
    PRIVATE
        src/bdshot.c
)

set_target_properties(dshot PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
)