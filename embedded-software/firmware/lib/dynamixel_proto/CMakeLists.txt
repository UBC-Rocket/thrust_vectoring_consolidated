cmake_minimum_required(VERSION 3.22)
project(dynamixel_proto LANGUAGES C)

add_library(dynamixel_proto STATIC
    src/dxl_packet.c
)

target_include_directories(dynamixel_proto PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

set_target_properties(dynamixel_proto PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
)
