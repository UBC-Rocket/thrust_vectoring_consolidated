cmake_minimum_required(VERSION 3.22)

#
# CM7 build — controls core.
#

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()
message("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

set(STM32_MCU_FLAGS "-mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard")

set(STM32_LINKER_SCRIPT stm32h745xx_flash_CM7.ld)
set(STM32_LINKER_OPTION )

set(CMAKE_PROJECT_NAME ulysses-fc_CM7)
project(${CMAKE_PROJECT_NAME})

add_executable(${CMAKE_PROJECT_NAME})

include("mx-generated.cmake")

set(CMAKE_CXX_COMPILER_FORCED "true")
enable_language(C ASM)

set(FW_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)
set(IO_DIR  ${FW_ROOT}/io)
set(DEV_DIR ${FW_ROOT}/dev)
set(LIB_DIR ${FW_ROOT}/lib)
set(APP_DIR ${FW_ROOT}/app)

# IO layer
add_subdirectory(${IO_DIR}/io          io_build)
add_subdirectory(${IO_DIR}/io_sys      io_sys_build)
add_subdirectory(${IO_DIR}/io_intercore_slot io_intercore_slot_build)

# Utility libs
add_subdirectory(${LIB_DIR}/collections      collections_build)
add_subdirectory(${LIB_DIR}/timestamp        timestamp_build)
add_subdirectory(${LIB_DIR}/log_records      log_records_build)
add_subdirectory(${LIB_DIR}/state_estimation state_estimation_build)
add_subdirectory(${LIB_DIR}/controls         controls_build)

# DEV layer (CM7 owns actuators)
add_subdirectory(${DEV_DIR}/servo_feetech dev_servo_feetech_build)
add_subdirectory(${DEV_DIR}/esc_dshot     dev_esc_dshot_build)

# APP layer
add_subdirectory(${APP_DIR} app_build)

# Per-target IO impl sources (live under io/h747/CM7)
set(IO_IMPL_DIR ${IO_DIR}/h747/CM7)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${IO_IMPL_DIR}
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${IO_IMPL_DIR}/io_dshot_tim.c
    ${IO_IMPL_DIR}/io_uart.c
    ${IO_IMPL_DIR}/io_status_led.c
    ${IO_IMPL_DIR}/io_timestamp.c
    ${IO_IMPL_DIR}/io_intercore.c
    ${IO_IMPL_DIR}/io_init.c
)

target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    SENSORS_USE_ARM_BARRIER
)

target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE
    io
    io_sys
    io_intercore_slot
    controls
    state_estimation
    log_records
    timestamp
    collections
    dev_servo_feetech
    dev_esc_dshot
    app_shared
    app_cm7
)

list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

if((CMAKE_C_STANDARD EQUAL 90) OR (CMAKE_C_STANDARD EQUAL 99))
    message(ERROR "Generated code requires C11 or higher")
endif()
