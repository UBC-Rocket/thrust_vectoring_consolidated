cmake_minimum_required(VERSION 3.22)
project(nanopb_runtime LANGUAGES C)

# nanopb 0.4.x runtime — three source files + a handful of headers.
# Vendored under embedded-software/libs/rocket-protocol/nanopb/ (alongside
# the rocket-protocol library that also uses it). This CMakeLists exists
# so the firmware can pull in just the runtime without dragging the rest
# of rocket-protocol along.
set(NANOPB_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../libs/rocket-protocol/nanopb")

add_library(nanopb_runtime STATIC
    ${NANOPB_SRC_DIR}/pb_common.c
    ${NANOPB_SRC_DIR}/pb_encode.c
    ${NANOPB_SRC_DIR}/pb_decode.c
)

target_include_directories(nanopb_runtime PUBLIC ${NANOPB_SRC_DIR})

set_target_properties(nanopb_runtime PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
)
