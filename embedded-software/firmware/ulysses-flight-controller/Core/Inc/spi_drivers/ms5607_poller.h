/**
 * @file ms5607_poller.h
 * @brief Non-blocking polling state machine for MS5607 barometer.
 *
 * The MS5607 has no interrupt pin, so we use a state machine to perform
 * the D1 (pressure) and D2 (temperature) conversion cycles without blocking.
 *
 * Usage:
 *   1. Call ms5607_poller_init() once at startup (after ms5607_init())
 *   2. Call ms5607_poller_tick() periodically (e.g., every 1ms from EKF task)
 *   3. When fresh data is available, it's pushed to the sample ring buffer
 *
 * UBC Rocket, Jan 2026
 */

#ifndef MS5607_POLLER_H
#define MS5607_POLLER_H

#include "sensors/ms5607_baro.h"
#include "stm32l4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* Poller State Machine                                                       */
/* -------------------------------------------------------------------------- */

/**
 * @brief State machine states for MS5607 polling.
 *
 * Flow: IDLE → WAIT_D1 → READ_D1 → WAIT_D2 → READ_D2 → (loop)
 *
 * The READ states transition immediately after job submission;
 * actual parsing happens in DMA callbacks.
 */
typedef enum {
    MS5607_POLLER_IDLE,      /**< Waiting to start next conversion cycle */
    MS5607_POLLER_WAIT_D1,   /**< Waiting for D1 conversion to complete */
    MS5607_POLLER_READ_D1,   /**< D1 read submitted, transition to D2 convert */
    MS5607_POLLER_WAIT_D2,   /**< Waiting for D2 conversion to complete */
    MS5607_POLLER_READ_D2    /**< D2 read submitted, compute on callback completion */
} ms5607_poller_state_t;

/**
 * @brief MS5607 non-blocking poller context.
 *
 * Manages asynchronous sampling via DMA job queue.
 * Non-blocking operation - call ms5607_poller_tick() from main loop or task.
 */
typedef struct {
    /** Device state with PROM calibration and readings */
    ms5607_t dev;

    /** Current oversampling ratio setting */
    ms5607_osr_t osr;

    /** Conversion time for current OSR (microseconds) */
    uint32_t conv_us;

    /** Target output data rate in Hz */
    uint32_t odr_hz;

    /** Sample period = 1e6/odr_hz (microseconds) */
    uint32_t period_us;

    /** Current FSM state */
    ms5607_poller_state_t state;

    /** Start time of current D1+D2 cycle */
    uint32_t t_cycle_start_us;

    /** Timestamp for next state transition */
    uint32_t t_next_action_us;

    /** True when new P/T data is available */
    bool fresh;

    /** Monotonically increasing sample sequence */
    uint32_t seq;
} ms5607_poller_t;

/* -------------------------------------------------------------------------- */
/* Poller API                                                                 */
/* -------------------------------------------------------------------------- */

/**
 * @brief Initialize the MS5607 poller state machine.
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
void ms5607_poller_init(ms5607_poller_t *p,
                        SPI_HandleTypeDef *hspi,
                        GPIO_TypeDef *cs_port,
                        uint16_t cs_pin,
                        ms5607_osr_t osr,
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
void ms5607_poller_tick(ms5607_poller_t *p);

/**
 * @brief Fetch the latest computed pressure/temperature.
 *
 * @param p     Pointer to poller context.
 * @param out   Destination for device data (optional, can be NULL).
 * @param seq   Destination for sequence number (optional, can be NULL).
 * @return true if fresh sample was available, false otherwise.
 *
 * @note This clears the 'fresh' flag. Subsequent calls return false until
 *       the next conversion cycle completes.
 *
 * @note For streaming applications, prefer using the ring buffer directly
 *       via ms5607_sample_dequeue() to avoid missing samples.
 */
bool ms5607_fetch_latest(ms5607_poller_t *p, ms5607_t *out, uint32_t *seq);

/**
 * @brief Change ODR/OSR settings at runtime.
 *
 * @param p       Pointer to poller context.
 * @param osr     New oversampling ratio.
 * @param odr_hz  New output data rate in Hz.
 */
void ms5607_poller_set_rate(ms5607_poller_t *p, ms5607_osr_t osr, uint32_t odr_hz);

#ifdef __cplusplus
}
#endif

#endif /* MS5607_POLLER_H */
