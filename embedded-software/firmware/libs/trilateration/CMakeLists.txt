cmake_minimum_required(VERSION 3.22)
project(trilateration LANGUAGES C)

add_library(trilateration STATIC
    src/trilat.c
)

target_include_directories(trilateration
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(trilateration PRIVATE m)

set_target_properties(trilateration PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
)