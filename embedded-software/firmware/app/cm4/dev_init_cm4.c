/**
 * @file    dev_init_cm4.c
 * @brief   DEV-layer phase for CM4. Sensors, comms, and the servo bus.
 *
 * UBC Rocket, 2026
 */
#include "app/app_init.h"
#include "app/sensors_init.h"
#include "app/actuators_init.h"
#include "io_sys/io_test_hooks.h"
#include "io_sys/io_status_led.h"

#ifdef USE_DYNAMIXEL_SERVO
#include "dev_servo_dynamixel.h"
#else
#include "dev_servo_feetech.h"
#endif

static sensors_t   s_sensor_handles;
static actuators_t s_actuator_handles;
static bool        s_init_done;

IO_TEST_HOOK_RW(s_sensor_handles,   sensors_t,   dev_init_cm4_handles)
IO_TEST_HOOK_RW(s_init_done,        bool,        dev_init_cm4_init_done)

void dev_init_cm4(void) {
    if (s_init_done) return;

    /* BMI088 (SPI4) and MMC5983 mag (SPI1) are SKIPPED for bring-up: the
     * first blocking SPI transaction to either hangs CM4 before anything
     * else (console, tasks) comes up. Suspected cause: CubeMX default
     * prescaler /2 leaves both buses far over the sensors' 10 MHz max
     * (SPI4: 120/2 = 60 MHz, SPI1: 64/2 = 32 MHz — see CM4/Core/Src/spi.c;
     * SPI3/ICM was already fixed to /4 = 16 MHz during its bring-up).
     * Their _get() handles stay NULL; all consumers are NULL-safe.
     * TODO: retune SPI1/SPI4 prescalers, verify on scope, then re-enable. */
    /* imu_bmi088_init(); */
    imu_icm40609_init();
    baro_ms5611_init();
    /* mag_mmc5983_init(); */
    gps_nmea_init();
    radio_rfd900_init();

#ifdef USE_DYNAMIXEL_SERVO
    servo_dynamixel_init();
    s_actuator_handles.servos = servo_dynamixel_get();
#else
    servo_feetech_init();
    s_actuator_handles.servos = servo_feetech_get();
#endif

    s_sensor_handles.bmi088   = imu_bmi088_get();
    s_sensor_handles.icm40609 = imu_icm40609_get();
    s_sensor_handles.baro     = baro_ms5611_get();
    s_sensor_handles.mag      = mag_mmc5983_get();
    s_sensor_handles.gps      = gps_nmea_get();
    s_sensor_handles.radio    = radio_rfd900_get();
    s_init_done = true;

    io_status_led_set(IO_LED_YELLOW, true);
}

const sensors_t *sensors_handles(void) {
    return s_init_done ? &s_sensor_handles : NULL;
}

const actuators_t *actuators_handles(void) {
    return s_init_done ? &s_actuator_handles : NULL;
}

void sensors_bind_state_estimation_task(io_task_handle_t task) {
    if (!s_init_done) return;
    imu_icm40609_set_notify (s_sensor_handles.icm40609, task, SENSORS_NOTIFY_IMU_PRIMARY);
    imu_bmi088_set_notify   (s_sensor_handles.bmi088,   task, SENSORS_NOTIFY_IMU_SECONDARY);
    baro_ms5611_set_notify  (s_sensor_handles.baro,     task, SENSORS_NOTIFY_BARO);
    mag_mmc5983_set_notify  (s_sensor_handles.mag,      task, SENSORS_NOTIFY_MAG);
    gps_nmea_set_notify     (s_sensor_handles.gps,      task, SENSORS_NOTIFY_GPS);
}
