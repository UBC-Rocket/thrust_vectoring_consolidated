cmake_minimum_required(VERSION 3.22)
project(dev_radio_rfd900 LANGUAGES C)

add_library(dev_radio_rfd900 STATIC
    src/dev_radio_rfd900.c
)

target_include_directories(dev_radio_rfd900 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(dev_radio_rfd900
    PUBLIC
        io_sys
    PRIVATE
        io
)

set_target_properties(dev_radio_rfd900 PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
)
