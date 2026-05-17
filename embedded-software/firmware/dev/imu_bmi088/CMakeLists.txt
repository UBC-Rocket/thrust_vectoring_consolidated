cmake_minimum_required(VERSION 3.22)
project(dev_imu_bmi088 LANGUAGES C)

add_library(dev_imu_bmi088 STATIC
    src/dev_imu_bmi088.c
)

target_include_directories(dev_imu_bmi088 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(dev_imu_bmi088
    PUBLIC
        io_sys
    PRIVATE
        io
        sensors
)

set_target_properties(dev_imu_bmi088 PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
)
