cmake_minimum_required(VERSION 3.22)
project(dev_servo_feetech LANGUAGES C)

add_library(dev_servo_feetech STATIC
    src/dev_servo_feetech.c
)

target_include_directories(dev_servo_feetech PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(dev_servo_feetech
    PUBLIC
        io_sys
    PRIVATE
        io
)

set_target_properties(dev_servo_feetech PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
)
