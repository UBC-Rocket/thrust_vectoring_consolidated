cmake_minimum_required(VERSION 3.22)
project(dev_mag_mmc5983 LANGUAGES C)

add_library(dev_mag_mmc5983 STATIC
    src/dev_mag_mmc5983.c
)

target_include_directories(dev_mag_mmc5983 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(dev_mag_mmc5983
    PUBLIC
        io_sys
    PRIVATE
        io
)

# FreeRTOS headers for xTaskNotifyFromISR — only when CubeMX is on the
# build (the dev driver compiles host-side as a no-op notify too).
if(TARGET stm32cubemx)
    target_link_libraries(dev_mag_mmc5983 PRIVATE stm32cubemx)
endif()

set_target_properties(dev_mag_mmc5983 PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
)
