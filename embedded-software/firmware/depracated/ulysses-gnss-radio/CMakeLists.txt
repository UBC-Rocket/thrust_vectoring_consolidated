cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME ulysses-gnss-radio)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Shared libraries
set(FW_LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../libs)
add_subdirectory(${FW_LIBS_DIR}/lwgps ${CMAKE_BINARY_DIR}/lwgps)
add_subdirectory(${FW_LIBS_DIR}/gnss-protocol ${CMAKE_BINARY_DIR}/gnss-protocol)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/spi_slave.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/spi_slave_test.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/radio_driver.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/gps.c
    # Debug UART support - injection and logging (conditional compilation via #ifdef DEBUG)
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/debug_uart.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/uart_callbacks.c

)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc

)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    lwgps
    gnss-protocol

    # Add user defined libraries
)

# Debug builds: enable float printf support for debug logging (adds ~4KB flash)
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    $<$<CONFIG:Debug>:-u>
    $<$<CONFIG:Debug>:_printf_float>
)
