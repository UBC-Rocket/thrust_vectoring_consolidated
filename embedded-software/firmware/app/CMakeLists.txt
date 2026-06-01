cmake_minimum_required(VERSION 3.22)
project(app LANGUAGES C)

#
# Shared layer — built into both cores.
#
add_library(app_shared STATIC
    shared/state_exchange.c
    shared/log_service.c
    shared/crash_dump.c
    shared/shared_memory.c
)
target_include_directories(app_shared
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(app_shared
    PUBLIC
        state_estimation
        controls
        io_intercore_slot
        io_sys             # APP only depends on system services, not transports
)
set_target_properties(app_shared PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
)

#
# CM4 task set. Built only if the CM4 driver libs are available
# (the per-core CMakeLists adds them before invoking us).
#
if(TARGET dev_imu_bmi088 AND TARGET dev_imu_icm40609 AND TARGET dev_baro_ms5611
        AND TARGET dev_gps_nmea AND TARGET dev_radio_rfd900)
    add_library(app_cm4 STATIC
        cm4/state_estimation_task.c
        cm4/mission_manager_task.c
        cm4/sd_log_task.c
        cm4/dev_init_cm4.c
        cm4/app_init_cm4.c
        cm4/messages_vcp.c
    )
    target_include_directories(app_cm4
        PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    target_link_libraries(app_cm4
        PUBLIC
            app_shared
            io_sys
            io
            cobs
            messages
            dev_imu_bmi088
            dev_imu_icm40609
            dev_baro_ms5611
            dev_gps_nmea
            dev_radio_rfd900
    )
    # mission_manager_task decodes incoming radio frames via rp_packet_decode.
    if(TARGET rocket-protocol)
        target_link_libraries(app_cm4 PUBLIC rocket-protocol)
    endif()
    # FreeRTOS + CMSIS includes for xTaskCreate / osDelay etc.
    if(TARGET stm32cubemx)
        target_link_libraries(app_cm4 PRIVATE stm32cubemx)
    endif()
    set_target_properties(app_cm4 PROPERTIES
        C_STANDARD 11
        C_STANDARD_REQUIRED ON
    )
endif()

#
# CM7 task set. Built only if actuator driver libs are available.
#
if(TARGET dev_servo_feetech AND TARGET dev_esc_dshot)
    add_library(app_cm7 STATIC
        cm7/controls_task.c
        cm7/app_init_cm7.c
        cm7/dev_init_cm7.c
        cm7/messages_udp.c
    )
    target_include_directories(app_cm7
        PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    target_link_libraries(app_cm7
        PUBLIC
            app_shared
            io_sys
            messages
            dev_servo_feetech
            dev_esc_dshot
    )
    if(TARGET stm32cubemx)
        target_link_libraries(app_cm7 PRIVATE stm32cubemx)
    endif()
    set_target_properties(app_cm7 PROPERTIES
        C_STANDARD 11
        C_STANDARD_REQUIRED ON
    )
endif()
