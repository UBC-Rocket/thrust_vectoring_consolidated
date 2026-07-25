cmake_minimum_required(VERSION 3.22)

project(state_estimation LANGUAGES C)

add_library(state_estimation STATIC
    src/ekf.c
    src/state_math.c
    src/matrix.c
    src/quaternion.c
    src/body.c
    src/calibration.c
    src/tilt_kf.c
)

target_include_directories(state_estimation
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(state_estimation PRIVATE m)

# CMSIS-DSP is only available in the firmware (cross-compile) builds — it's
# pulled in via add_subdirectory from each per-core CMakeLists. Host tests
# don't build CMSIS-DSP (they don't need to), so matrix.c falls back to the
# naive impls. Gate both linkage and the #define together.
if(TARGET cmsis_dsp)
    target_link_libraries(state_estimation PRIVATE cmsis_dsp)
    target_compile_definitions(state_estimation PRIVATE STATE_EST_USE_CMSIS_DSP)
endif()

# ekf.c declares `static timeline_event_t events[ESKF_MAX_TIMELINE_EVENTS]`
# inside eskf_process. Default is 16,000 events (≈384 KB) — sized for the
# host-side trajectory tests. On embedded we only ever see ~20 events per
# call (a single drain batch). Shrink to 256 (≈6 KB) when building against
# the CubeMX target (i.e. firmware build); leave the default for host tests.
if(TARGET stm32cubemx)
    target_compile_definitions(state_estimation PRIVATE ESKF_MAX_TIMELINE_EVENTS=256)
endif()

# Yaw output: OFF (default) pins the published yaw to FIXED_YAW — correct for
# the gimbal test rig (yaw is physically constrained) and safe when no working
# magnetometer is present. ON publishes the filter's live, mag-driven yaw
# estimate; requires a working, calibrated mag. Forwarded from the top-level
# build via CM4_SERVO_CMAKE_ARG (the ESKF runs on CM4).
option(ESKF_ESTIMATE_YAW "Publish estimated yaw instead of the FIXED_YAW override (needs a working mag)" OFF)
if(ESKF_ESTIMATE_YAW)
    target_compile_definitions(state_estimation PRIVATE ESKF_ESTIMATE_YAW)
endif()

set_target_properties(state_estimation PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
)
