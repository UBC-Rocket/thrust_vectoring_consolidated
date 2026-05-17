cmake_minimum_required(VERSION 3.22)
project(io_spi_queue LANGUAGES C)

add_library(io_spi_queue STATIC
    src/io_spi_queue.c
)

target_include_directories(io_spi_queue PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(io_spi_queue
    PUBLIC
        io_sys     # io_status_t lives here
)

# Inherit HAL include paths + symbols when the firmware project defined the
# stm32cubemx interface target. Standalone / host builds link whatever they
# need manually.
if(TARGET stm32cubemx)
    target_link_libraries(io_spi_queue PUBLIC stm32cubemx)
endif()

set_target_properties(io_spi_queue PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
)
