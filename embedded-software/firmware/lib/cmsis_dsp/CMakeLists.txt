cmake_minimum_required(VERSION 3.22)
project(cmsis_dsp LANGUAGES C)

#
# CMSIS-DSP — ARM-software/CMSIS-DSP v1.16.2, vendored as flat files.
#
# We compile the per-module umbrella TUs (Source/<Module>/<Module>.c, each of
# which #includes every arm_*_f32 / arm_*_q15 / etc. in that module). Picking
# umbrellas keeps the build simple — adding a new arm_xxx use requires no
# CMake edit. The linker GCs unreferenced functions out of the final .elf as
# long as -ffunction-sections / -fdata-sections / --gc-sections are on (they
# are, via mx-generated.cmake's compile/link options).
#
# Excluded modules (no current use, skip to keep build time down):
#   Bayes, ComplexMath, Distance, Filtering, Interpolation, SVM, Transform,
#   Window, *F16 variants (we use f32 everywhere — no half precision in the
#   EKF or control loop).
#
# Per-core flags (-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 for CM4,
# -mcpu=cortex-m7 -mfpu=fpv5-d16 for CM7) propagate from each per-core build
# via the toolchain — CMSIS-DSP's source picks SIMD/MAC paths based on the
# __ARM_FEATURE_DSP / __FPU_PRESENT macros the compiler defines from those
# flags. No per-core source selection needed on our side.
#

add_library(cmsis_dsp STATIC
    Source/BasicMathFunctions/BasicMathFunctions.c
    Source/CommonTables/CommonTables.c
    Source/ControllerFunctions/ControllerFunctions.c
    Source/FastMathFunctions/FastMathFunctions.c
    Source/MatrixFunctions/MatrixFunctions.c
    Source/QuaternionMathFunctions/QuaternionMathFunctions.c
    Source/StatisticsFunctions/StatisticsFunctions.c
    Source/SupportFunctions/SupportFunctions.c
)

target_include_directories(cmsis_dsp
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Include
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/PrivateInclude
)

# CMSIS-DSP needs cmsis_compiler.h from Drivers/CMSIS/Include (exposed by the
# CubeMX-generated stm32cubemx INTERFACE target). PUBLIC so consumers that
# `#include "arm_math.h"` — which transitively pulls cmsis_compiler.h —
# inherit the include path. stm32cubemx is INTERFACE-only, so this
# propagates include dirs + compile defines but adds nothing to link lines.
if(TARGET stm32cubemx)
    target_link_libraries(cmsis_dsp PUBLIC stm32cubemx)
endif()

# DISABLEFLOAT16: we never use f16 — also silences warnings in shared
# headers that conditionally compile half-precision overloads. The
# umbrella TUs we picked don't pull in *F16.c so this is belt-and-braces.
target_compile_definitions(cmsis_dsp PRIVATE
    DISABLEFLOAT16
)

# CMSIS-DSP has a handful of -Wunused-* warnings we don't want polluting
# our build logs (we'll never patch upstream).
target_compile_options(cmsis_dsp PRIVATE
    -Wno-unused-parameter
    -Wno-sign-compare
    -Wno-unused-variable
    -Wno-unused-function
    -Wno-missing-field-initializers
)

set_target_properties(cmsis_dsp PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
)
