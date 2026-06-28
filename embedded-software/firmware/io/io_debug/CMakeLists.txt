cmake_minimum_required(VERSION 3.22)
project(io_debug LANGUAGES C)

# Portable debug-console formatter. The transport (io_debug_init /
# io_debug_write) is an unresolved external here — provided per-target by
# io/h747/CM{4,7}/io_debug.c and resolved at the per-core executable link.
add_library(io_debug STATIC
    src/io_debug.c
)

# io_sys carries the public header io_sys/io_debug.h (system-service surface
# that APP + DEV are allowed to include).
target_link_libraries(io_debug PUBLIC io_sys)

set_target_properties(io_debug PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
)
