cmake_minimum_required(VERSION 3.22)
project(dev_servo_dynamixel LANGUAGES C)

add_library(dev_servo_dynamixel STATIC
    src/dxl_hal.c
    src/dev_servo_dynamixel.c
)

target_include_directories(dev_servo_dynamixel PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_include_directories(dev_servo_dynamixel PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(dev_servo_dynamixel
    PUBLIC
        dynamixel_proto
        io_sys
    PRIVATE
        io
)

if(TARGET stm32cubemx)
    target_link_libraries(dev_servo_dynamixel PRIVATE stm32cubemx)
endif()

set_target_properties(dev_servo_dynamixel PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
)
