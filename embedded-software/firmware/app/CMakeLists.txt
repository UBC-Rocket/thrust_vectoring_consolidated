cmake_minimum_required(VERSION 3.22)
project(app LANGUAGES C)

# When ON (default), LPUART1/VCP is a plain-text debug console (io_debug →
# CoolTerm at 115200 8-N-1) and the binary "messages" VCP channel is left
# without a sink (its records still route to SD/UDP). Turn OFF to restore the
# binary messages-console on VCP for the host messages-console CLI.
# Only one of the two can own the single STLINK-V3 VCP at a time.
option(DEBUG_TEXT_CONSOLE "LPUART1/VCP = plain-text debug console (CoolTerm)" ON)

# When ON (bring-up default), CM7 never arms the ESC driver: the 800 Hz control
# ISR, controller math, and control_output publishing all run, but every
# esc_dshot_motor_set_throttle() is rejected inside the driver (bdshot.c gates
# on is_armed) so no DShot pulses reach the motors, and the boot-time ESC
# selftest ramp is skipped. Turn OFF only for powered motor tests with props
# removed — there is no armed-state gating in the ISR yet, and the controller
# commands hover-level thrust (T_cmd ≈ m·g) whenever it runs.
option(BENCH_NO_MOTORS "Bring-up: never arm ESCs; control loop runs, DShot output suppressed" ON)

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
        io_debug           # io_debug_printf — text debug console (a system service)
)
# log_service.c branches on CORE_CM4 / CORE_CM7 to pick the right side of the
# SRAM4 log-handoff ring. stm32cubemx is the INTERFACE target that injects
# those macros (mx-generated.cmake). PRIVATE link — we don't want every
# downstream of app_shared to also pull in CMSIS/HAL/FreeRTOS headers.
if(TARGET stm32cubemx)
    target_link_libraries(app_shared PRIVATE stm32cubemx)
endif()
set_target_properties(app_shared PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
)

#
# CM4 task set. Built only if the CM4 driver libs are available
# (the per-core CMakeLists adds them before invoking us).
#
if(TARGET dev_imu_bmi088 AND TARGET dev_imu_icm40609 AND TARGET dev_baro_ms5611
        AND TARGET dev_gps_nmea AND TARGET dev_radio_rfd900 AND TARGET dev_mag_mmc5983
        AND (TARGET dev_servo_feetech OR TARGET dev_servo_dynamixel))
    add_library(app_cm4 STATIC
        cm4/state_estimation_task.c
        cm4/mission_manager_task.c
        cm4/telemetry_task.c
        cm4/sd_log_task.c
        cm4/actuator_task.c
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
            storage
            dev_imu_bmi088
            dev_imu_icm40609
            dev_baro_ms5611
            dev_mag_mmc5983
            dev_gps_nmea
            dev_radio_rfd900
    )
    if(TARGET dev_servo_dynamixel)
        target_link_libraries(app_cm4 PUBLIC dev_servo_dynamixel)
        target_compile_definitions(app_cm4 PRIVATE USE_DYNAMIXEL_SERVO)
    elseif(TARGET dev_servo_feetech)
        target_link_libraries(app_cm4 PUBLIC dev_servo_feetech)
    endif()
    # mission_manager_task decodes incoming radio frames via rp_packet_decode.
    if(TARGET rocket-protocol)
        target_link_libraries(app_cm4 PUBLIC rocket-protocol)
    endif()
    # FreeRTOS + CMSIS includes for xTaskCreate / osDelay etc.
    if(TARGET stm32cubemx)
        target_link_libraries(app_cm4 PRIVATE stm32cubemx)
    endif()
    if(DEBUG_TEXT_CONSOLE)
        target_compile_definitions(app_cm4 PRIVATE DEBUG_TEXT_CONSOLE)
    endif()
    set_target_properties(app_cm4 PROPERTIES
        C_STANDARD 11
        C_STANDARD_REQUIRED ON
    )
endif()

#
# CM7 task set. Built only if the ESC driver lib is available.
#
if(TARGET dev_esc_dshot)
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
            dev_esc_dshot
    )
    if(TARGET stm32cubemx)
        target_link_libraries(app_cm7 PRIVATE stm32cubemx)
    endif()
    if(DEBUG_TEXT_CONSOLE)
        # CM7 forwards its io_debug_printf text to CM4's VCP over the SRAM4
        # console ring (see app_init_cm7.c / io/h747/CM7/io_debug.c).
        target_compile_definitions(app_cm7 PRIVATE DEBUG_TEXT_CONSOLE)
    endif()
    if(BENCH_NO_MOTORS)
        target_compile_definitions(app_cm7 PRIVATE BENCH_NO_MOTORS)
    endif()
    set_target_properties(app_cm7 PROPERTIES
        C_STANDARD 11
        C_STANDARD_REQUIRED ON
    )
endif()
