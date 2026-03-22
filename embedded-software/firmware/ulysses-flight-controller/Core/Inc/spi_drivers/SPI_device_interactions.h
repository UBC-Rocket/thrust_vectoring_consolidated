/**
 * @file SPI_device_interactions.h
 * @brief Device initialization and interrupt handlers for SPI sensors.
 *
 * This header provides:
 * - Device initialization functions (blocking, for use before scheduler)
 * - Interrupt handlers (for EXTI callbacks)
 * - Global instance declarations (legacy - see sensor_context.h for modern approach)
 *
 * UBC Rocket, Jan 2026
 */

#ifndef SPI_DEVICE_INTERACTIONS_H
#define SPI_DEVICE_INTERACTIONS_H

#include "stm32h5xx_hal.h"
#include "SPI_queue.h"
#include "sensors/bmi088_accel.h"
#include "sensors/bmi088_gyro.h"
#include "sensors/ms5611_baro.h"
#include "sensors/ms5607_baro.h"
#include "ms5611_poller.h"
#include "ms5607_poller.h"
#include "sensors/sensor_config.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* Device Constants                                                           */
/* -------------------------------------------------------------------------- */

/** Expected chip ID values for device identification */
#define BMI088_ACC_CHIP_ID_VALUE   0x1E
#define BMI088_GYRO_CHIP_ID_VALUE  0x0F

/** Task notification flags for sensor data ready events */
#define BMI088_ACCEL_SAMPLE_FLAG  (1U << 0)
#define BMI088_GYRO_SAMPLE_FLAG   (1U << 1)
#define MS5611_BARO_SAMPLE_FLAG   (1U << 2)
#define MS5607_BARO2_SAMPLE_FLAG  (1U << 3)

/* -------------------------------------------------------------------------- */
/* Global Instances                                                           */
/* -------------------------------------------------------------------------- */
/*
 * These globals provide backward compatibility. For new code or dual-core
 * migration, consider using sensor_context.h which groups related state.
 */

/** Sample ring buffers - ISR produces, state estimation task consumes */
extern bmi088_accel_sample_queue_t bmi088_acc_sample_ring;
extern bmi088_gyro_sample_queue_t bmi088_gyro_sample_ring;
extern ms5611_sample_queue_t ms5611_sample_ring;
extern ms5607_sample_queue_t ms5607_sample_ring;

/** SPI job queues for each bus */
extern spi_job_queue_t jobq_spi_1;
extern spi_job_queue_t jobq_spi_2;
extern spi_job_queue_t jobq_spi_4;

/** Device configuration structs */
extern bmi088_accel_t accel;
extern bmi088_gyro_t gyro;

/** Device ready flags - set after successful initialization */
extern volatile bool bmi088_accel_ready;
extern volatile bool bmi088_gyro_ready;

/* -------------------------------------------------------------------------- */
/* Device Initialization Functions                                            */
/* -------------------------------------------------------------------------- */
/*
 * These functions perform blocking initialization using polling SPI.
 * Call them before starting the FreeRTOS scheduler (from main.c or a
 * dedicated sensors_init() function).
 *
 * Return codes:
 *   0 = success
 *   non-zero = error (device-specific)
 */

/**
 * @brief Initialize BMI088 accelerometer with configuration.
 *
 * @param hspi    SPI handle.
 * @param cs_port Chip select GPIO port.
 * @param cs_pin  Chip select GPIO pin.
 * @param dev     Device struct to populate with configuration.
 * @param config  Sensor configuration (range, ODR, interrupts).
 * @return 0 on success, error code otherwise.
 */
uint8_t bmi088_accel_init_with_config(SPI_HandleTypeDef *hspi,
                                      GPIO_TypeDef *cs_port,
                                      uint16_t cs_pin,
                                      bmi088_accel_t *dev,
                                      const bmi088_accel_config_t *config);

/**
 * @brief Initialize BMI088 accelerometer with default configuration.
 *
 * Convenience wrapper that uses BMI088_ACCEL_CONFIG_DEFAULT.
 *
 * @param hspi    SPI handle.
 * @param cs_port Chip select GPIO port.
 * @param cs_pin  Chip select GPIO pin.
 * @param dev     Device struct to populate with configuration.
 * @return 0 on success, error code otherwise.
 */
uint8_t bmi088_accel_init(SPI_HandleTypeDef *hspi,
                          GPIO_TypeDef *cs_port,
                          uint16_t cs_pin,
                          bmi088_accel_t *dev);

/**
 * @brief Initialize BMI088 gyroscope with configuration.
 *
 * @param hspi    SPI handle.
 * @param cs_port Chip select GPIO port.
 * @param cs_pin  Chip select GPIO pin.
 * @param dev     Device struct to populate with configuration.
 * @param config  Sensor configuration (range, ODR, interrupts).
 * @return 0 on success, error code otherwise.
 */
uint8_t bmi088_gyro_init_with_config(SPI_HandleTypeDef *hspi,
                                     GPIO_TypeDef *cs_port,
                                     uint16_t cs_pin,
                                     bmi088_gyro_t *dev,
                                     const bmi088_gyro_config_t *config);

/**
 * @brief Initialize BMI088 gyroscope with default configuration.
 *
 * Convenience wrapper that uses BMI088_GYRO_CONFIG_DEFAULT.
 *
 * @param hspi    SPI handle.
 * @param cs_port Chip select GPIO port.
 * @param cs_pin  Chip select GPIO pin.
 * @param dev     Device struct to populate with configuration.
 * @return 0 on success, error code otherwise.
 */
uint8_t bmi088_gyro_init(SPI_HandleTypeDef *hspi,
                         GPIO_TypeDef *cs_port,
                         uint16_t cs_pin,
                         bmi088_gyro_t *dev);

/**
 * @brief Initialize MS5611 barometer.
 *
 * Sequence:
 *   1. Send Reset (0x1E)
 *   2. Wait ≥2.8 ms (PROM reload)
 *   3. Read PROM[0..7] (each 16-bit)
 *   4. Validate CRC-4
 *
 * @param hspi    SPI handle.
 * @param cs_port Chip select GPIO port.
 * @param cs_pin  Chip select GPIO pin.
 * @param dev     Device struct to populate with calibration data.
 * @return 0 on success, error code otherwise.
 */
uint8_t ms5611_init(SPI_HandleTypeDef *hspi,
                    GPIO_TypeDef *cs_port,
                    uint16_t cs_pin,
                    ms5611_t *dev);

/* -------------------------------------------------------------------------- */
/* Interrupt Handlers                                                         */
/* -------------------------------------------------------------------------- */
/*
 * These functions are called from EXTI GPIO callbacks when sensor
 * data-ready interrupts fire. They submit SPI read jobs to the queue.
 */

/**
 * @brief Handle BMI088 accelerometer data-ready interrupt.
 *
 * Submits an SPI job to read the latest accelerometer data.
 * Call from HAL_GPIO_EXTI_Rising_Callback when BMI_ACC_INT_1_Pin fires.
 */
void bmi088_accel_interrupt(void);

/**
 * @brief Handle BMI088 gyroscope data-ready interrupt.
 *
 * Submits an SPI job to read the latest gyroscope data.
 * Call from HAL_GPIO_EXTI_Rising_Callback when BMI_GYRO_INT_1_Pin fires.
 */
void bmi088_gyro_interrupt(void);

#ifdef __cplusplus
}
#endif

#endif /* SPI_DEVICE_INTERACTIONS_H */
