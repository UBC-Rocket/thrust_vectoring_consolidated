cmake_minimum_required(VERSION 3.22)

project(dev_esc_dshot LANGUAGES C)

add_library(dev_esc_dshot STATIC)

target_sources(dev_esc_dshot
    PRIVATE
        src/bdshot.c
        src/bdshot_ral.h
        src/hal_to_ll_translator.h
)

target_include_directories(dev_esc_dshot
    PUBLIC
        include/
    PRIVATE
        src/
)

target_link_libraries(dev_esc_dshot
    PUBLIC
        io_sys
    PRIVATE
        io
        dshot
        stm32cubemx
)

set_target_properties(dev_esc_dshot PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
)
