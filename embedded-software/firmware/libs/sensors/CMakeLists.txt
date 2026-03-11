cmake_minimum_required(VERSION 3.22)
project(sensors LANGUAGES C)

add_library(sensors STATIC
    src/bmi088_accel.c
    src/bmi088_gyro.c
    src/ms5611_baro.c
    src/ms5607_baro.c
)

target_include_directories(sensors
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(sensors
    PUBLIC collections gnss-protocol
    PRIVATE m
)

set_target_properties(sensors PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
)
