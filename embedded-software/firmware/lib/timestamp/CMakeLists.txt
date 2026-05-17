cmake_minimum_required(VERSION 3.22)
project(timestamp LANGUAGES C)

add_library(timestamp STATIC src/timestamp.c)

target_include_directories(timestamp
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Pick up CMSIS / device headers (CoreDebug, DWT, SystemCoreClock) when the
# firmware project provides the stm32cubemx interface target. The .c uses
# STM32H7 / H5 / F4 macros to select the right device header.
if(TARGET stm32cubemx)
    target_link_libraries(timestamp PRIVATE stm32cubemx)
endif()

set_target_properties(timestamp PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
)
