/**
 * @file    dev_init_cm4.c
 * @brief   DEV-layer phase for CM4. Each driver owns its own singleton
 *          storage; this file only triggers init and assembles the
 *          ergonomic view-struct.
 *
 * UBC Rocket, 2026
 */
#include "app/app_init.h"
#include "app/sensors_init.h"
#include "io_sys/io_test_hooks.h"
#include "io_sys/io_status_led.h"   /* TEMP bring-up tracer */

static sensors_t s_handles;
static bool      s_init_done;

/* Test-only access to module-private state. No-op in production builds. */
IO_TEST_HOOK_RW(s_handles,   sensors_t, dev_init_cm4_handles)
IO_TEST_HOOK_RW(s_init_done, bool,      dev_init_cm4_init_done)

void dev_init_cm4(void) {
    if (s_init_done) return;

    imu_bmi088_init();
    imu_icm40609_init();
    baro_ms5611_init();
    mag_mmc5983_init();
    gps_nmea_init();
    radio_rfd900_init();

    s_handles.bmi088   = imu_bmi088_get();
    s_handles.icm40609 = imu_icm40609_get();
    s_handles.baro     = baro_ms5611_get();
    s_handles.mag      = mag_mmc5983_get();
    s_handles.gps      = gps_nmea_get();
    s_handles.radio    = radio_rfd900_get();
    s_init_done = true;

    /* TEMP bring-up tracer: YELLOW solid = dev_init_cm4 completed (no sensor
     * init blocked). Remove after bring-up. */
    io_status_led_set(IO_LED_YELLOW, true);
}

const sensors_t *sensors_handles(void) {
    return s_init_done ? &s_handles : NULL;
}

void sensors_bind_state_estimation_task(io_task_handle_t task) {
    if (!s_init_done) return;
    imu_icm40609_set_notify (s_handles.icm40609, task, SENSORS_NOTIFY_IMU_PRIMARY);
    imu_bmi088_set_notify   (s_handles.bmi088,   task, SENSORS_NOTIFY_IMU_SECONDARY);
    baro_ms5611_set_notify  (s_handles.baro,     task, SENSORS_NOTIFY_BARO);
    mag_mmc5983_set_notify  (s_handles.mag,      task, SENSORS_NOTIFY_MAG);
    gps_nmea_set_notify     (s_handles.gps,      task, SENSORS_NOTIFY_GPS);
}
