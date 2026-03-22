cmake_minimum_required(VERSION 3.22)

project(controls LANGUAGES C)

add_library(controls STATIC
    src/pid.c
    src/flight_controller.c
    src/pwm.c
)

target_include_directories(controls
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(controls
    PUBLIC
        state_estimation
    PRIVATE
        m
)

set_target_properties(controls PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
)
