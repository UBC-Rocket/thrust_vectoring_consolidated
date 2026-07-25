cmake_minimum_required(VERSION 3.22)
project(io_uart_dma_cm LANGUAGES C)

add_library(io_uart_dma_cm STATIC
    src/io_uart_dma_cm.c
)

target_include_directories(io_uart_dma_cm
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

if(TARGET stm32cubemx)
    target_link_libraries(io_uart_dma_cm PUBLIC stm32cubemx)
endif()

set_target_properties(io_uart_dma_cm PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
)
