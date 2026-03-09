cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Add our own CMake module directory to the search path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# There are no dynamic linkers for our embedded system. We must
# statically link all libraries.
set(BUILD_SHARED_LIBS OFF)

# Set the project name
set(CMAKE_PROJECT_NAME ulysses)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})

option(ULYSSES_ENABLE_DEBUG_LOGGING "Enable serial debug logging" ON)
option(ULYSSES_USE_USB "Enable USB support" OFF)

# Display a summary of the build configuration
message("Build type: " ${CMAKE_BUILD_TYPE})
message("Debug logging: " ${ULYSSES_ENABLE_DEBUG_LOGGING})
message("USB support: " ${ULYSSES_USE_USB})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Add dependencies
include(dependencies)

set(PWM_OUTPUT_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/motor_drivers/pwm_output.c
)

set(SERVO_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/motor_drivers/servo_driver.c
)

set(ESC_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/motor_drivers/esc_driver.c
)

set(DEBUG_MODULE_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/debug/log.c
)

set(RTOS_TASKS_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/tasks/controls.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/tasks/mission_manager.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/tasks/state_estimation.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/tasks/debug_logging.c
)

set(SPI_DRIVER_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/spi_drivers/SPI_queue.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/spi_drivers/SPI_device_common.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/spi_drivers/bmi088_accel_device.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/spi_drivers/bmi088_gyro_device.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/spi_drivers/ms5611_device.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/spi_drivers/ms5607_device.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/spi_drivers/spi1_bus.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/spi_drivers/gnss_radio_master.c
)

# Shared libraries
set(TVR_LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../libs)
set(FW_LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../libs)
add_subdirectory(${TVR_LIBS_DIR}/rocket-protocol ${CMAKE_BINARY_DIR}/rocket-protocol)
add_subdirectory(${FW_LIBS_DIR}/state_estimation ${CMAKE_BINARY_DIR}/state_estimation)
add_subdirectory(${FW_LIBS_DIR}/controls ${CMAKE_BINARY_DIR}/controls)
add_subdirectory(${FW_LIBS_DIR}/sensors ${CMAKE_BINARY_DIR}/sensors)

# Enable ARM hardware memory barrier for sensor ring buffers (static inline in headers)
target_compile_definitions(sensors INTERFACE SENSORS_USE_ARM_BARRIER)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${RTOS_TASKS_SRC}
    ${SPI_DRIVER_SRC}
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/sensors_init.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/timestamp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/state_exchange.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/utils/gnss_utils.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/interrupts.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/timing.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/SD_logging/log_writer.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/SD_logging/log_service.c
    ${DEBUG_MODULE_SRC}
    ${ALGO_SRC}

    ${PWM_OUTPUT_SRC}
    ${SERVO_SRC}
    ${ESC_SRC}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc/spi_drivers
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    BMI_ACC_Chip_Select_GPIO_Port=BMI_ACC_CS_GPIO_Port
    BMI_ACC_Chip_Select_Pin=BMI_ACC_CS_Pin
    BMI_GYRO_Chip_Select_GPIO_Port=BMI_GYRO_CS_GPIO_Port
    BMI_GYRO_Chip_Select_Pin=BMI_GYRO_CS_Pin
    $<$<BOOL:${ULYSSES_USE_USB}>:ULYSSES_USE_USB>
    $<$<BOOL:${ULYSSES_ENABLE_DEBUG_LOGGING}>:ULYSSES_ENABLE_DEBUG_LOGGING>
)

# Enable -ffast-math for EKF math sources (FMA contraction, relaxed NaN handling)
target_compile_options(state_estimation PRIVATE -ffast-math)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    rocket-protocol
    state_estimation
    controls
    sensors

    # Add user defined libraries
    printf::printf
)
