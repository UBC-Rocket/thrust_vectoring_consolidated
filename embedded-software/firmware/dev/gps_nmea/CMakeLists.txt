cmake_minimum_required(VERSION 3.22)
project(dev_gps_nmea LANGUAGES C)

add_library(dev_gps_nmea STATIC
    src/dev_gps_nmea.c
)

target_include_directories(dev_gps_nmea PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(dev_gps_nmea
    PUBLIC
        io_sys
        lwgps
    PRIVATE
        io
)

set_target_properties(dev_gps_nmea PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
)
