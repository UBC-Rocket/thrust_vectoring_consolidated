cmake_minimum_required(VERSION 3.22)
project(dev_baro_ms5611 LANGUAGES C)

add_library(dev_baro_ms5611 STATIC
    src/dev_baro_ms5611.c
)

target_include_directories(dev_baro_ms5611 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(dev_baro_ms5611
    PUBLIC
        io_sys
    PRIVATE
        io
        sensors
)

# HAL headers (HAL_Delay) + FreeRTOS (xTaskCreateStatic / vTaskDelay) come
# from the CubeMX target. Guarded with if(TARGET ...) to mirror the other
# dev drivers' CMake — the driver is only ever built in the CM4 firmware
# context where stm32cubemx is present.
if(TARGET stm32cubemx)
    target_link_libraries(dev_baro_ms5611 PRIVATE stm32cubemx)
endif()

# BARO_RELAX_CRC: proceed when the MS5611 PROM CRC fails but C1..C6 are
# sane. Strict (CRC enforced) by default; opt in only for bring-up boards
# with the known word-0 read quirk. See baro_ms5611_init for the rationale.
option(BARO_RELAX_CRC "Relax MS5611 PROM CRC gate (bring-up boards)" OFF)
if(BARO_RELAX_CRC)
    target_compile_definitions(dev_baro_ms5611 PRIVATE BARO_RELAX_CRC=1)
endif()

set_target_properties(dev_baro_ms5611 PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
)
