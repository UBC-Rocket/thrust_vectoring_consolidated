cmake_minimum_required(VERSION 3.22)

#
# CM4 build — flight controller.
#
# Layers, bottom-up:
#   1. HW  — CubeMX-generated Core/ + HAL + FreeRTOS (via mx-generated.cmake)
#   2. IO  — contract + helpers in ../../io/, per-target impls in ../../io/h747/CM4/
#   3. DEV — device drivers under ../../dev/
#   4. APP — tasks + shared services in ../../app/
#   5. utility libs under ../../lib/ (used by app + dev + state estimation)
#

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()
message("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

set(STM32_MCU_FLAGS "-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard")

set(STM32_LINKER_SCRIPT stm32h745xx_flash_CM4.ld)
set(STM32_LINKER_OPTION )

set(CMAKE_PROJECT_NAME ulysses-fc_CM4)
project(${CMAKE_PROJECT_NAME})

add_executable(${CMAKE_PROJECT_NAME})

include("mx-generated.cmake")

set(CMAKE_CXX_COMPILER_FORCED "true")
enable_language(C ASM)

# ------------------------------------------------------------------
# Path roots
# ------------------------------------------------------------------
set(FW_ROOT    ${CMAKE_CURRENT_SOURCE_DIR}/../..)
set(IO_DIR     ${FW_ROOT}/io)
set(DEV_DIR    ${FW_ROOT}/dev)
set(LIB_DIR    ${FW_ROOT}/lib)
set(APP_DIR    ${FW_ROOT}/app)

# ------------------------------------------------------------------
# IO layer
# ------------------------------------------------------------------
add_subdirectory(${IO_DIR}/io           io_build)
add_subdirectory(${IO_DIR}/io_sys       io_sys_build)
add_subdirectory(${IO_DIR}/io_spi_queue io_spi_queue_build)
add_subdirectory(${IO_DIR}/io_uart_dma_cm io_uart_dma_cm_build)
add_subdirectory(${IO_DIR}/io_intercore_slot io_intercore_slot_build)

# ------------------------------------------------------------------
# Utility libs
# ------------------------------------------------------------------
add_subdirectory(${LIB_DIR}/collections      collections_build)
add_subdirectory(${LIB_DIR}/timestamp        timestamp_build)
add_subdirectory(${LIB_DIR}/sensors          sensors_build)
add_subdirectory(${LIB_DIR}/state_estimation state_estimation_build)
add_subdirectory(${LIB_DIR}/controls         controls_build)
add_subdirectory(${LIB_DIR}/gnss-protocol    gnss_protocol_build)
add_subdirectory(${LIB_DIR}/lwgps            lwgps_build)
add_subdirectory(${FW_ROOT}/comms/nanopb     nanopb_build)
add_subdirectory(${FW_ROOT}/comms/cobs       cobs_build)
add_subdirectory(${LIB_DIR}/messages         messages_build)

# ------------------------------------------------------------------
# DEV layer (CM4 owns sensing + comms)
# ------------------------------------------------------------------
add_subdirectory(${DEV_DIR}/imu_bmi088    dev_imu_bmi088_build)
add_subdirectory(${DEV_DIR}/imu_icm40609  dev_imu_icm40609_build)
add_subdirectory(${DEV_DIR}/baro_ms5611   dev_baro_ms5611_build)
add_subdirectory(${DEV_DIR}/gps_nmea      dev_gps_nmea_build)
add_subdirectory(${DEV_DIR}/radio_rfd900  dev_radio_rfd900_build)

# ------------------------------------------------------------------
# APP layer
# ------------------------------------------------------------------
add_subdirectory(${APP_DIR}               app_build)

# ------------------------------------------------------------------
# Per-target IO impl sources (live under io/h747/CM4)
# ------------------------------------------------------------------
set(IO_IMPL_DIR ${IO_DIR}/h747/CM4)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${IO_IMPL_DIR}
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${IO_IMPL_DIR}/io_spi.c
    ${IO_IMPL_DIR}/io_uart.c
    ${IO_IMPL_DIR}/io_exti.c
    ${IO_IMPL_DIR}/io_sd.c
    ${IO_IMPL_DIR}/io_status_led.c
    ${IO_IMPL_DIR}/io_timestamp.c
    ${IO_IMPL_DIR}/io_intercore.c
    ${IO_IMPL_DIR}/io_init.c
)

target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    SENSORS_USE_ARM_BARRIER
)

# mx-generated.cmake uses the plain target_link_libraries signature on this
# target, so we must too (CMake forbids mixing keyword + plain on one target).
target_link_libraries(${CMAKE_PROJECT_NAME}
    io
    io_sys
    io_spi_queue
    io_uart_dma_cm
    io_intercore_slot
    sensors
    state_estimation
    controls
    timestamp
    collections
    lwgps
    gnss-protocol
    cobs
    messages
    dev_imu_bmi088
    dev_imu_icm40609
    dev_baro_ms5611
    dev_gps_nmea
    dev_radio_rfd900
    app_shared
    app_cm4
)

list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

if((CMAKE_C_STANDARD EQUAL 90) OR (CMAKE_C_STANDARD EQUAL 99))
    message(ERROR "Generated code requires C11 or higher")
endif()
