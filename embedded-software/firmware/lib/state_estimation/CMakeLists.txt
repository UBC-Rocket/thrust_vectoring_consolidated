cmake_minimum_required(VERSION 3.22)

project(state_estimation LANGUAGES C)

add_library(state_estimation STATIC
    src/ekf.c
    src/state_math.c
    src/matrix.c
    src/quaternion.c
    src/body.c
    src/calibration.c
)

target_include_directories(state_estimation
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(state_estimation PRIVATE m)

set_target_properties(state_estimation PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
)
