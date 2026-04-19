/**
 * @file sensor_context.h
 * @brief Sensor context structures to reduce global state coupling.
 *
 * This header defines context structures that group related sensor state,
 * making ownership explicit and simplifying dual-core partitioning.
 *
 * Design Goals:
 * - Explicit ownership: each context owns its buffers and state
 * - Testability: contexts can be instantiated independently for unit tests
 * - Dual-core ready: state grouped by core affinity
 * - Reduced coupling: pass contexts instead of accessing globals
 *
 * Migration Path:
 * 1. Define contexts here (done)
 * 2. Instantiate contexts in SPI_device_common.c (existing globals become members)
 * 3. Update functions to accept context pointers
 * 4. Eventually remove extern declarations from SPI_device_interactions.h
 *
 * UBC Rocket, Jan 2026
 */

#ifndef SENSOR_CONTEXT_H
#define SENSOR_CONTEXT_H

#include "stm32l4xx_hal.h"
#include "SPI_queue.h"
#include "sensors/bmi088_accel.h"
#include "sensors/bmi088_gyro.h"
#include "sensors/ms5611_baro.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* Accelerometer Context                                                      */
/* -------------------------------------------------------------------------- */

/**
 * @brief Complete context for BMI088 accelerometer.
 *
 * Groups all accelerometer-related state:
 * - Device configuration (range, ODR, scale factors)
 * - Sample ring buffer
 * - Ready flag
 * - Hardware identifiers (for ISR routing)
 */
typedef struct {
    /** Device configuration and state */
    bmi088_accel_t config;

    /** Sample ring buffer - producer: ISR, consumer: state estimation task */
    bmi088_accel_sample_queue_t samples;

    /** True when device is initialized and ready for interrupt-driven operation */
    volatile bool ready;

    /** Chip select GPIO port */
    GPIO_TypeDef *cs_port;

    /** Chip select GPIO pin */
    uint16_t cs_pin;

    /** Pointer to SPI job queue for this sensor */
    spi_job_queue_t *job_queue;
} accel_context_t;

/* -------------------------------------------------------------------------- */
/* Gyroscope Context                                                          */
/* -------------------------------------------------------------------------- */

/**
 * @brief Complete context for BMI088 gyroscope.
 */
typedef struct {
    /** Device configuration and state */
    bmi088_gyro_t config;

    /** Sample ring buffer - producer: ISR, consumer: state estimation task */
    bmi088_gyro_sample_queue_t samples;

    /** True when device is initialized and ready for interrupt-driven operation */
    volatile bool ready;

    /** Chip select GPIO port */
    GPIO_TypeDef *cs_port;

    /** Chip select GPIO pin */
    uint16_t cs_pin;

    /** Pointer to SPI job queue for this sensor */
    spi_job_queue_t *job_queue;
} gyro_context_t;

/* -------------------------------------------------------------------------- */
/* Barometer Context                                                          */
/* -------------------------------------------------------------------------- */

/**
 * @brief Complete context for MS5611 barometer.
 *
 * Note: Barometer uses polling (no interrupt pin), so the poller state
 * machine is included in the context.
 */
typedef struct {
    /** Device configuration including PROM calibration data */
    ms5611_t config;

    /** Sample ring buffer - producer: poller callback, consumer: state estimation */
    ms5611_sample_queue_t samples;

    /** True when device is initialized */
    volatile bool ready;

    /** Chip select GPIO port */
    GPIO_TypeDef *cs_port;

    /** Chip select GPIO pin */
    uint16_t cs_pin;

    /** Pointer to SPI job queue for this sensor */
    spi_job_queue_t *job_queue;

    /* Poller state machine fields (previously in ms5611_poller_t) */
    ms5611_osr_t osr;           /**< Oversampling setting */
    uint32_t odr_hz;            /**< Output data rate */
    uint32_t conv_us;           /**< Conversion time for current OSR */
    uint32_t period_us;         /**< Sample period in microseconds */
    uint32_t t_next_action_us;  /**< Timestamp for next action */
    uint32_t t_cycle_start_us;  /**< Start of current D1+D2 cycle */
    uint32_t seq;               /**< Monotonically increasing sample sequence */
    bool fresh;                 /**< True when new P/T data available */

    /** Poller state machine state */
    enum {
        BARO_STATE_IDLE,
        BARO_STATE_CONV_D1,
        BARO_STATE_WAIT_D1,
        BARO_STATE_READ_D1,
        BARO_STATE_CONV_D2,
        BARO_STATE_WAIT_D2,
        BARO_STATE_READ_D2,
        BARO_STATE_READY
    } state;
} baro_context_t;

/* -------------------------------------------------------------------------- */
/* Combined Sensor Subsystem Context                                          */
/* -------------------------------------------------------------------------- */

/**
 * @brief Combined context for all sensors on a single SPI bus.
 *
 * This structure groups all sensors that share an SPI bus, making it clear
 * what state needs to move together for dual-core partitioning.
 *
 * For dual-core:
 * - Core 0 might own SPI2 sensors (IMU + baro for state estimation)
 * - Core 1 might own SPI1 sensors (external/payload sensors)
 */
typedef struct {
    /** SPI bus job queue */
    spi_job_queue_t job_queue;

    /** HAL SPI handle */
    SPI_HandleTypeDef *spi_handle;

    /** Accelerometer context */
    accel_context_t accel;

    /** Gyroscope context */
    gyro_context_t gyro;

    /** Barometer context */
    baro_context_t baro;
} sensor_bus_context_t;

/* -------------------------------------------------------------------------- */
/* Context Initialization Helpers                                             */
/* -------------------------------------------------------------------------- */

/**
 * @brief Initialize accelerometer context to safe defaults.
 * @param ctx Context to initialize.
 * @param cs_port Chip select GPIO port.
 * @param cs_pin Chip select GPIO pin.
 * @param job_queue Pointer to SPI job queue.
 */
static inline void accel_context_init(accel_context_t *ctx,
                                      GPIO_TypeDef *cs_port,
                                      uint16_t cs_pin,
                                      spi_job_queue_t *job_queue)
{
    ctx->ready = false;
    ctx->cs_port = cs_port;
    ctx->cs_pin = cs_pin;
    ctx->job_queue = job_queue;
    ctx->samples.head = 0;
    ctx->samples.tail = 0;
}

/**
 * @brief Initialize gyroscope context to safe defaults.
 */
static inline void gyro_context_init(gyro_context_t *ctx,
                                     GPIO_TypeDef *cs_port,
                                     uint16_t cs_pin,
                                     spi_job_queue_t *job_queue)
{
    ctx->ready = false;
    ctx->cs_port = cs_port;
    ctx->cs_pin = cs_pin;
    ctx->job_queue = job_queue;
    ctx->samples.head = 0;
    ctx->samples.tail = 0;
}

/**
 * @brief Initialize barometer context to safe defaults.
 */
static inline void baro_context_init(baro_context_t *ctx,
                                     GPIO_TypeDef *cs_port,
                                     uint16_t cs_pin,
                                     spi_job_queue_t *job_queue)
{
    ctx->ready = false;
    ctx->cs_port = cs_port;
    ctx->cs_pin = cs_pin;
    ctx->job_queue = job_queue;
    ctx->samples.head = 0;
    ctx->samples.tail = 0;
    ctx->state = BARO_STATE_IDLE;
    ctx->seq = 0;
    ctx->fresh = false;
}

/**
 * @brief Initialize a complete sensor bus context.
 * @param ctx Bus context to initialize.
 * @param spi_handle HAL SPI handle for this bus.
 */
static inline void sensor_bus_context_init(sensor_bus_context_t *ctx,
                                           SPI_HandleTypeDef *spi_handle)
{
    ctx->spi_handle = spi_handle;
    ctx->job_queue.spi_bus = spi_handle;
    ctx->job_queue.spi_busy = false;
    ctx->job_queue.head = 0;
    ctx->job_queue.tail = 0;
    ctx->job_queue.last_submit_status = HAL_OK;

    /* Individual sensors init with pointers to the shared job queue */
    /* Note: CS pins need to be set by caller based on hardware config */
    ctx->accel.job_queue = &ctx->job_queue;
    ctx->gyro.job_queue = &ctx->job_queue;
    ctx->baro.job_queue = &ctx->job_queue;
}

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_CONTEXT_H */

