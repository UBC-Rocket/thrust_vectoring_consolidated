cmake_minimum_required(VERSION 3.22)
project(dev_baro_ms5611 LANGUAGES C)

add_library(dev_baro_ms5611 STATIC
    src/dev_baro_ms5611.c
)

target_include_directories(dev_baro_ms5611 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(dev_baro_ms5611
    PUBLIC
        io_sys
    PRIVATE
        io
        sensors
)

set_target_properties(dev_baro_ms5611 PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
)
