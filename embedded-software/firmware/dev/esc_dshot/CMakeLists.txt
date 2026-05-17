cmake_minimum_required(VERSION 3.22)
project(dev_esc_dshot LANGUAGES C)

add_library(dev_esc_dshot STATIC
    src/dev_esc_dshot.c
)

target_include_directories(dev_esc_dshot PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(dev_esc_dshot
    PUBLIC
        io_sys
    PRIVATE
        io
)

set_target_properties(dev_esc_dshot PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
)
