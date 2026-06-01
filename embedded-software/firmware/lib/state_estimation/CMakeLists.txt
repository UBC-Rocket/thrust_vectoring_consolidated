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

# ekf.c declares `static timeline_event_t events[ESKF_MAX_TIMELINE_EVENTS]`
# inside eskf_process. Default is 16,000 events (≈384 KB) — sized for the
# host-side trajectory tests. On embedded we only ever see ~20 events per
# call (a single drain batch). Shrink to 256 (≈6 KB) when building against
# the CubeMX target (i.e. firmware build); leave the default for host tests.
if(TARGET stm32cubemx)
    target_compile_definitions(state_estimation PRIVATE ESKF_MAX_TIMELINE_EVENTS=256)
endif()

set_target_properties(state_estimation PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
)
