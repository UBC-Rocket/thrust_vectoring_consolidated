cmake_minimum_required(VERSION 3.22)
project(io_sys LANGUAGES C)

# io_sys is the "system services" half of the IO contract — headers that the
# APP layer is allowed to include (timestamp, intercore, status LED, SD,
# scheduler-start, shared types). Transport headers (SPI/UART/EXTI/DShot) sit
# in io/ and are visible to the DEV layer only.
add_library(io_sys INTERFACE)

target_include_directories(io_sys INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
