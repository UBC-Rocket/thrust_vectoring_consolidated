/**
 * @file    io_timestamp.h
 * @brief   Monotonic system time, shared between cores.
 *
 * On the H747 the source is a free-running TIM at 10 MHz (0.1 µs / tick)
 * extended to 48 bits via a software upper-word in shared SRAM4. CM4 owns
 * the overflow interrupt; both cores read.
 *
 * UBC Rocket, 2026
 */
#ifndef IO_TIMESTAMP_H
#define IO_TIMESTAMP_H

#include "io_sys/io_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise the timestamp source.
 *        On CM4 this configures and starts the TIM, enables the overflow
 *        interrupt, and zeroes the shared upper word.
 *        On CM7 this only attaches to the shared upper word — the TIM must
 *        already be running, which the CM4 boot sequence guarantees.
 */
void io_timestamp_init(void);

/**
 * @brief Free-running monotonic timestamp, microseconds since CM4 boot.
 *        Resolution = 0.1 µs (1 tick); reported to caller in 1 µs units
 *        (rounded down).
 */
uint64_t io_timestamp_us(void);

/**
 * @brief Free-running monotonic timestamp, in raw 0.1 µs ticks. Use when
 *        sub-µs resolution matters (e.g. SPI sample stamping). Wraps after
 *        ~325 days.
 */
uint64_t io_timestamp_ticks_100ns(void);

static inline uint32_t io_timestamp_ms(void) {
    return (uint32_t)(io_timestamp_us() / 1000ULL);
}

/**
 * @brief IRQ hook called from main.c's HAL_TIM_PeriodElapsedCallback
 *        dispatcher (USER CODE block) on every TIM period-elapsed event.
 *        Filters internally for the timestamp TIM (TIM13 on H747).
 *
 * The htim parameter is opaque to keep this header free of STM32 types;
 * the per-target impl casts back to `TIM_HandleTypeDef *`.
 */
void io_timestamp_on_tim_period_elapsed(void *htim);

#ifdef __cplusplus
}
#endif

#endif /* IO_TIMESTAMP_H */
