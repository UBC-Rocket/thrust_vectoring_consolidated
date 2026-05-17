/**
 * @file    sensors_init.h
 * @brief   View-struct + notify wiring for CM4 device drivers.
 *
 * The driver singletons are instantiated by @ref dev_init_cm4 (declared in
 * app_init.h). This header only exposes ergonomics for tasks that need
 * pointers to those singletons.
 *
 * UBC Rocket, 2026
 */
#ifndef APP_SENSORS_INIT_H
#define APP_SENSORS_INIT_H

#include "dev_imu_bmi088.h"
#include "dev_imu_icm40609.h"
#include "dev_baro_ms5611.h"
#include "dev_gps_nmea.h"
#include "dev_radio_rfd900.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    imu_bmi088_t   *bmi088;
    imu_icm40609_t *icm40609;
    baro_ms5611_t  *baro;
    gps_nmea_t     *gps;
    radio_rfd900_t *radio;
} sensors_t;

/**
 * @brief Return pointers to the singletons (NULL if dev_init_cm4 wasn't run).
 */
const sensors_t *sensors_handles(void);

/**
 * @brief Wire each sensor driver's notify target to a single task. Call from
 *        the state-estimation task once it knows its own handle.
 */
void sensors_bind_state_estimation_task(io_task_handle_t task);

/* Notification bits used by the sensor drivers when bound. */
#define SENSORS_NOTIFY_IMU_PRIMARY    (1U << 0)   /* ICM-40609  */
#define SENSORS_NOTIFY_IMU_SECONDARY  (1U << 1)   /* BMI088     */
#define SENSORS_NOTIFY_BARO           (1U << 2)
#define SENSORS_NOTIFY_GPS            (1U << 3)

#ifdef __cplusplus
}
#endif

#endif /* APP_SENSORS_INIT_H */
