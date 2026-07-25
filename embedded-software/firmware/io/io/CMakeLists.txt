cmake_minimum_required(VERSION 3.22)
project(io LANGUAGES C)

# io is a header-only contract library. Implementations live in the per-core
# Io/ directories of the firmware project (and, eventually, in a SIL backend).
add_library(io INTERFACE)

target_include_directories(io INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
