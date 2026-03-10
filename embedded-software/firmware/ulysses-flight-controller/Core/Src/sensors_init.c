/**
 * @file sensors_init.c
 * @brief Consolidated sensor initialization implementation.
 *
 * UBC Rocket, Jan 2026
 */

#include "sensors_init.h"
#include "spi_drivers/SPI_device_interactions.h"
#include "spi_drivers/ms5611_poller.h"
#include "spi_drivers/ms5607_poller.h"
#include "main.h"

/* External SPI handles from main.c */
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi4;

/* MS5611 poller instance - owned by this module, used by state estimation task */
static ms5611_poller_t g_baro_poller;

/* MS5607 poller instance - owned by this module, used by state estimation task */
static ms5607_poller_t g_baro2_poller;

/* Active configuration - stored after initialization */
static sensor_system_config_t g_active_config;

/* Initialization status - read-only after sensors_init() returns */
static sensors_init_status_t g_init_status = {0};

/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */

sensors_init_status_t sensors_init_with_config(const sensor_system_config_t *config)
{
    sensors_init_status_t status = {
        .accel_ok = false,
        .gyro_ok = false,
        .baro_ok = false,
        .baro2_ok = false,
        .accel_err = 0xFF,
        .gyro_err = 0xFF,
        .baro_err = 0xFF,
        .baro2_err = 0xFF
    };

    /* Use default config if none provided */
    if (config == NULL) {
        sensor_system_config_t default_cfg = SENSOR_SYSTEM_CONFIG_DEFAULT;
        g_active_config = default_cfg;
    } else {
        g_active_config = *config;
    }

    /*
     * Step 1: Initialize SPI job queues
     *
     * The job queues must be initialized before any sensor operations.
     * jobq_spi_2: IMU (BMI088) and barometer 1 (MS5611)
     * jobq_spi_4: Barometer 2 (MS5607)
     */

    jobq_spi_2.spi_bus = &hspi2;
    jobq_spi_2.spi_busy = false;
    jobq_spi_2.head = 0;
    jobq_spi_2.tail = 0;
    jobq_spi_2.last_submit_status = HAL_OK;

    jobq_spi_4.spi_bus = &hspi4;
    jobq_spi_4.spi_busy = false;
    jobq_spi_4.head = 0;
    jobq_spi_4.tail = 0;
    jobq_spi_4.last_submit_status = HAL_OK;

    /*
     * Step 2: Initialize sample ring buffers
     *
     * These buffers transfer sensor data from ISR context to task context.
     * Initialize head/tail to 0 for empty state.
     */
    bmi088_acc_sample_ring.head = 0;
    bmi088_acc_sample_ring.tail = 0;

    bmi088_gyro_sample_ring.head = 0;
    bmi088_gyro_sample_ring.tail = 0;

    ms5611_sample_ring.head = 0;
    ms5611_sample_ring.tail = 0;

    ms5607_sample_ring.head = 0;
    ms5607_sample_ring.tail = 0;

    /*
     * Step 3: Initialize BMI088 Accelerometer with configuration
     *
     * This is a blocking operation that takes ~600ms due to soft reset delay.
     */
    status.accel_err = bmi088_accel_init_with_config(&hspi2,
                                                      BMI_ACC_Chip_Select_GPIO_Port,
                                                      BMI_ACC_Chip_Select_Pin,
                                                      &accel,
                                                      &g_active_config.accel);

    if (status.accel_err == 0) {
        bmi088_accel_ready = true;
        status.accel_ok = true;
    } else {
        bmi088_accel_ready = false;
    }

    /*
     * Step 4: Initialize BMI088 Gyroscope with configuration
     *
     * This is a blocking operation that takes ~100ms due to soft reset delay.
     */
    status.gyro_err = bmi088_gyro_init_with_config(&hspi2,
                                                    BMI_GYRO_Chip_Select_GPIO_Port,
                                                    BMI_GYRO_Chip_Select_Pin,
                                                    &gyro,
                                                    &g_active_config.gyro);

    if (status.gyro_err == 0) {
        bmi088_gyro_ready = true;
        status.gyro_ok = true;
    } else {
        bmi088_gyro_ready = false;
    }

    /*
     * Step 5: Initialize MS5611 Barometer Poller with configuration
     *
     * This is a blocking operation that:
     *   - Resets the device
     *   - Reads and validates PROM calibration data
     *   - Configures the poller state machine
     */
    ms5611_poller_init(&g_baro_poller,
                       &hspi2,
                       BARO1_CS_GPIO_Port,
                       BARO1_CS_Pin,
                       g_active_config.baro.osr,
                       g_active_config.baro.odr_hz);

    /* Check if PROM was read successfully by verifying calibration data */
    if (g_baro_poller.dev.C[1] != 0 || g_baro_poller.dev.C[2] != 0) {
        status.baro_ok = true;
        status.baro_err = 0;
    } else {
        status.baro_err = 1;  /* PROM read failed */
    }

    /*
     * Step 6: Initialize MS5607 Barometer 2 Poller with configuration
     *
     * This is a blocking operation that:
     *   - Resets the device
     *   - Reads and validates PROM calibration data
     *   - Configures the poller state machine
     */
    ms5607_poller_init(&g_baro2_poller,
                       &hspi4,
                       BARO2_CS_GPIO_Port,
                       BARO2_CS_Pin,
                       g_active_config.baro2.osr,
                       g_active_config.baro2.odr_hz);

    /* Check if PROM was read successfully by verifying calibration data */
    if (g_baro2_poller.dev.C[1] != 0 || g_baro2_poller.dev.C[2] != 0) {
        status.baro2_ok = true;
        status.baro2_err = 0;
    } else {
        status.baro2_err = 1;  /* PROM read failed */
    }

    g_init_status = status;
    return status;
}

sensors_init_status_t sensors_init(void)
{
    return sensors_init_with_config(NULL);
}

const sensors_init_status_t *sensors_get_init_status(void)
{
    return &g_init_status;
}

const sensor_system_config_t *sensors_get_config(void)
{
    return &g_active_config;
}

/* -------------------------------------------------------------------------- */
/* Accessors for the barometer poller                                         */
/* -------------------------------------------------------------------------- */

/**
 * @brief Get pointer to the barometer poller instance.
 *
 * The state estimation task needs this to call ms5611_poller_tick().
 *
 * @return Pointer to the global barometer poller.
 */
ms5611_poller_t *sensors_get_baro_poller(void)
{
    return &g_baro_poller;
}

/**
 * @brief Get pointer to the second barometer (MS5607) poller instance.
 *
 * The state estimation task needs this to call ms5607_poller_tick().
 *
 * @return Pointer to the global barometer 2 poller.
 */
ms5607_poller_t *sensors_get_baro2_poller(void)
{
    return &g_baro2_poller;
}
