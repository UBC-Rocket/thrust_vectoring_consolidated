/**
 * @file timestamp.h
 * @brief High-resolution timestamp utilities using DWT cycle counter.
 *
 * Provides microsecond-resolution timestamps with proper overflow handling.
 * The DWT->CYCCNT counter is 32-bit and wraps at ~17 seconds at 250MHz.
 * This module tracks overflows to provide a continuous 64-bit timestamp.
 *
 * Usage:
 *   1. Call timestamp_init() once at startup (after SystemClock_Config).
 *   2. Call timestamp_us() to get the current microsecond timestamp.
 *   3. Call timestamp_update() periodically (e.g., from SysTick) to track overflows.
 *
 * Thread Safety:
 *   - timestamp_us() is safe to call from ISR and task contexts.
 *   - timestamp_update() should be called from a single context (e.g., SysTick ISR).
 *
 * @note Requires DWT (Data Watchpoint and Trace) unit, available on Cortex-M3/M4/M7/M33.
 *
 * UBC Rocket, Benedikt Howard, 2025
 */

#ifndef TIMESTAMP_H
#define TIMESTAMP_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32h5xx.h"

/* -------------------------------------------------------------------------- */
/* Configuration                                                              */
/* -------------------------------------------------------------------------- */

/**
 * @brief Maximum time between timestamp_update() calls in microseconds.
 *
 * At 250MHz, CYCCNT wraps every ~17.18 seconds (2^32 / 250e6).
 * Call timestamp_update() at least every 10 seconds to be safe.
 * SysTick at 1kHz calling this every tick is more than sufficient.
 */
#define TIMESTAMP_UPDATE_INTERVAL_US    (10000000UL)  /* 10 seconds */

/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */

/**
 * @brief Initialize the timestamp module.
 *
 * Enables the DWT cycle counter and precomputes the divisor for
 * efficient cycle-to-microsecond conversion.
 *
 * @note Must be called after SystemClock_Config() so SystemCoreClock is valid.
 */
void timestamp_init(void);

/**
 * @brief Get current timestamp in microseconds.
 *
 * Returns a monotonically increasing 64-bit timestamp that handles
 * CYCCNT overflow correctly (when timestamp_update() is called regularly).
 *
 * @return Current time in microseconds since system start.
 *
 * @note Safe to call from ISR context.
 * @note For backwards compatibility, also provides timestamp_us() returning uint32_t.
 */
uint64_t timestamp_us64(void);

/**
 * @brief Get current timestamp in microseconds (32-bit version).
 *
 * This is a convenience function that returns the lower 32 bits.
 * Wraps every ~71.5 minutes. For timing durations < 35 minutes,
 * simple subtraction works correctly even across wrap.
 *
 * @return Current time in microseconds (lower 32 bits).
 *
 * @note Safe to call from ISR context.
 */
static inline uint32_t timestamp_us(void) {
    return (uint32_t)timestamp_us64();
}

/**
 * @brief Update overflow tracking.
 *
 * Must be called periodically (at least every 10 seconds) to detect
 * CYCCNT overflow. Recommended: call from SysTick ISR.
 *
 * @note Should be called from a single context to avoid race conditions.
 */
void timestamp_update(void);

/**
 * @brief Compute elapsed microseconds between two 32-bit timestamps.
 *
 * Handles single wrap-around correctly. Only valid if actual elapsed
 * time is less than ~35 minutes (half of 32-bit wrap period).
 *
 * @param start Starting timestamp (from timestamp_us()).
 * @param end   Ending timestamp (from timestamp_us()).
 * @return Elapsed time in microseconds.
 */
static inline uint32_t timestamp_elapsed_us(uint32_t start, uint32_t end) {
    return end - start;  /* Works correctly due to unsigned arithmetic */
}

/**
 * @brief Check if a deadline has passed.
 *
 * @param deadline Deadline timestamp (from timestamp_us()).
 * @return True if current time >= deadline.
 */
static inline bool timestamp_deadline_passed(uint32_t deadline) {
    uint32_t now = timestamp_us();
    /* Signed comparison handles wrap-around within half the period */
    return (int32_t)(now - deadline) >= 0;
}

#endif /* TIMESTAMP_H */

