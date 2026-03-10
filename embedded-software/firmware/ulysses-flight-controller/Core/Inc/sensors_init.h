/**
 * @file sensors_init.h
 * @brief Consolidated sensor initialization for all SPI sensors.
 *
 * This module provides a single entry point for initializing all sensors
 * before the FreeRTOS scheduler starts. This approach:
 *
 * - Avoids long periods with interrupts disabled inside tasks
 * - Ensures sensors are ready before tasks need them
 * - Centralizes initialization for easier debugging and modification
 * - Provides clear status reporting for each sensor
 *
 * Usage:
 *   Call sensors_init() from main() after MX_SPI*_Init() and before
 *   osKernelStart(). Check the returned status to verify sensors are ready.
 *
 * UBC Rocket, Jan 2026
 */

#ifndef SENSORS_INIT_H
#define SENSORS_INIT_H

#include "stm32h5xx_hal.h"
#include "spi_drivers/ms5611_poller.h"
#include "spi_drivers/ms5607_poller.h"
#include "sensors/sensor_config.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* Initialization Status                                                      */
/* -------------------------------------------------------------------------- */

/**
 * @brief Sensor initialization status flags.
 *
 * Each bit indicates whether a sensor initialized successfully.
 * Check individual bits to determine which sensors are available.
 */
typedef struct {
    bool accel_ok;      /**< BMI088 accelerometer initialized successfully */
    bool gyro_ok;       /**< BMI088 gyroscope initialized successfully */
    bool baro_ok;       /**< MS5611 barometer initialized successfully */
    bool baro2_ok;      /**< MS5607 barometer 2 initialized successfully */
    uint8_t accel_err;  /**< Accelerometer error code (0 = success) */
    uint8_t gyro_err;   /**< Gyroscope error code (0 = success) */
    uint8_t baro_err;   /**< Barometer error code (0 = success) */
    uint8_t baro2_err;  /**< Barometer 2 error code (0 = success) */
} sensors_init_status_t;

/* -------------------------------------------------------------------------- */
/* Initialization API                                                         */
/* -------------------------------------------------------------------------- */

/**
 * @brief Initialize all SPI sensors with specific configuration.
 *
 * This function performs the following:
 *   1. Initialize SPI job queues
 *   2. Initialize sample ring buffers
 *   3. Initialize BMI088 accelerometer (blocking SPI)
 *   4. Initialize BMI088 gyroscope (blocking SPI)
 *   5. Initialize MS5611 barometer poller (blocking SPI for PROM read)
 *   6. Set device ready flags
 *
 * @param config Sensor configuration (range, ODR, etc.). Pass NULL for defaults.
 * @return Status structure indicating success/failure for each sensor.
 *
 * @note This function uses blocking SPI operations and takes several
 *       hundred milliseconds to complete (due to sensor reset delays).
 *       Call before starting the FreeRTOS scheduler.
 */
sensors_init_status_t sensors_init_with_config(const sensor_system_config_t *config);

/**
 * @brief Initialize all SPI sensors with default configuration.
 *
 * Convenience wrapper that uses SENSOR_SYSTEM_CONFIG_DEFAULT.
 *
 * @return Status structure indicating success/failure for each sensor.
 */
sensors_init_status_t sensors_init(void);

/**
 * @brief Get the current sensor system configuration.
 *
 * Returns the configuration used during initialization.
 * Useful for logging or runtime inspection.
 *
 * @return Pointer to the active configuration (read-only).
 */
const sensor_system_config_t *sensors_get_config(void);

/**
 * @brief Get the sensor initialization status.
 *
 * Returns a pointer to the status structure populated during sensors_init().
 * Read-only after boot (no mutex needed).
 *
 * @return Pointer to the init status (read-only).
 */
const sensors_init_status_t *sensors_get_init_status(void);

/**
 * @brief Check if all critical sensors initialized successfully.
 *
 * @param status Status returned by sensors_init().
 * @return true if accelerometer and gyroscope are both OK.
 *
 * @note Barometer is not considered critical since state estimation
 *       can work with IMU alone (with accumulated drift).
 */
static inline bool sensors_init_critical_ok(const sensors_init_status_t *status)
{
    return status->accel_ok && status->gyro_ok;
}

/**
 * @brief Check if all sensors initialized successfully.
 *
 * @param status Status returned by sensors_init().
 * @return true if all sensors are OK.
 */
static inline bool sensors_init_all_ok(const sensors_init_status_t *status)
{
    return status->accel_ok && status->gyro_ok && status->baro_ok && status->baro2_ok;
}

/* -------------------------------------------------------------------------- */
/* Accessors                                                                  */
/* -------------------------------------------------------------------------- */

/**
 * @brief Get pointer to the barometer poller instance.
 *
 * The state estimation task needs this to call ms5611_poller_tick().
 *
 * @return Pointer to the global barometer poller.
 */
ms5611_poller_t *sensors_get_baro_poller(void);

/**
 * @brief Get pointer to the second barometer (MS5607) poller instance.
 *
 * The state estimation task needs this to call ms5607_poller_tick().
 *
 * @return Pointer to the global barometer 2 poller.
 */
ms5607_poller_t *sensors_get_baro2_poller(void);

#ifdef __cplusplus
}
#endif

#endif /* SENSORS_INIT_H */

