/**
 * @file ms5611_poller.h
 * @brief Non-blocking polling state machine for MS5611 barometer.
 *
 * The MS5611 has no interrupt pin, so we use a state machine to perform
 * the D1 (pressure) and D2 (temperature) conversion cycles without blocking.
 *
 * Usage:
 *   1. Call ms5611_poller_init() once at startup (after ms5611_init())
 *   2. Call ms5611_poller_tick() periodically (e.g., every 1ms from EKF task)
 *   3. When fresh data is available, it's pushed to the sample ring buffer
 *
 * UBC Rocket, Jan 2026
 */

#ifndef MS5611_POLLER_H
#define MS5611_POLLER_H

#include "MS5611_baro.h"
#include "stm32h5xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* Poller State Machine                                                       */
/* -------------------------------------------------------------------------- */

/**
 * @brief State machine states for MS5611 polling.
 */
typedef enum {
    MS5611_POLLER_IDLE,      /**< Waiting to start next conversion cycle */
    MS5611_POLLER_CONV_D1,   /**< Started D1 (pressure) conversion */
    MS5611_POLLER_WAIT_D1,   /**< Waiting for D1 conversion to complete */
    MS5611_POLLER_READ_D1,   /**< Reading D1 result */
    MS5611_POLLER_CONV_D2,   /**< Started D2 (temperature) conversion */
    MS5611_POLLER_WAIT_D2,   /**< Waiting for D2 conversion to complete */
    MS5611_POLLER_READ_D2,   /**< Reading D2 result */
    MS5611_POLLER_READY      /**< Fresh data available */
} ms5611_poller_state_t;

/**
 * @brief MS5611 non-blocking poller context.
 *
 * This structure holds all state needed for non-blocking operation:
 * - Configuration (OSR, ODR)
 * - Timing information
 * - State machine state
 * - Device data (calibration, raw readings, computed values)
 */
typedef struct {
    /* Configuration */
    ms5611_osr_t osr;           /**< Oversampling ratio setting */
    uint32_t odr_hz;            /**< Target output data rate in Hz */
    uint32_t conv_us;           /**< Conversion time for current OSR (microseconds) */
    uint32_t period_us;         /**< Sample period = 1e6/odr_hz (microseconds) */

    /* Timing */
    uint32_t t_next_action_us;  /**< Timestamp for next action */
    uint32_t t_cycle_start_us;  /**< Start time of current D1+D2 cycle */

    /* State machine */
    ms5611_poller_state_t state;

    /* Device data */
    ms5611_t dev;               /**< Device struct with calibration and readings */
    uint8_t rx_buf[4];          /**< Scratch buffer for SPI reads */
    bool fresh;                 /**< True when new P/T data is available */
    uint32_t seq;               /**< Monotonically increasing sample sequence */
} ms5611_poller_t;

/* -------------------------------------------------------------------------- */
/* Poller API                                                                 */
/* -------------------------------------------------------------------------- */

/**
 * @brief Initialize the MS5611 poller state machine.
 *
 * This function:
 *   1. Resets the device
 *   2. Reads and validates PROM calibration data
 *   3. Configures the poller with specified OSR and ODR
 *
 * @param p         Pointer to poller context to initialize.
 * @param hspi      SPI handle for communication.
 * @param cs_port   Chip select GPIO port.
 * @param cs_pin    Chip select GPIO pin.
 * @param osr       Oversampling ratio (affects precision and conversion time).
 * @param odr_hz    Target output data rate in Hz.
 *
 * @note This function blocks during PROM read (a few milliseconds).
 *       Call before starting the scheduler or from a task.
 */
void ms5611_poller_init(ms5611_poller_t *p,
                        SPI_HandleTypeDef *hspi,
                        GPIO_TypeDef *cs_port,
                        uint16_t cs_pin,
                        ms5611_osr_t osr,
                        uint32_t odr_hz);

/**
 * @brief Advance the poller state machine.
 *
 * Call this function periodically (recommended: every 1ms from the state
 * estimation task). The function checks timestamps and advances through
 * the conversion states as needed.
 *
 * When a complete D1+D2 cycle finishes, the compensated pressure and
 * temperature are computed and pushed to the sample ring buffer.
 *
 * @param p Pointer to poller context.
 *
 * @note Non-blocking - returns immediately after checking state.
 */
void ms5611_poller_tick(ms5611_poller_t *p);

/**
 * @brief Fetch the latest computed pressure/temperature.
 *
 * @param p     Pointer to poller context.
 * @param out   Destination for device data (optional, can be NULL).
 * @param seq   Destination for sequence number (optional, can be NULL).
 * @return true if fresh data was available, false otherwise.
 *
 * @note This clears the 'fresh' flag. Subsequent calls return false until
 *       the next conversion cycle completes.
 */
bool ms5611_fetch_latest(ms5611_poller_t *p, ms5611_t *out, uint32_t *seq);

/**
 * @brief Change ODR/OSR settings at runtime.
 *
 * @param p       Pointer to poller context.
 * @param osr     New oversampling ratio.
 * @param odr_hz  New output data rate in Hz.
 */
void ms5611_poller_set_rate(ms5611_poller_t *p, ms5611_osr_t osr, uint32_t odr_hz);

#ifdef __cplusplus
}
#endif

#endif /* MS5611_POLLER_H */

