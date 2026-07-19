/**
 * @file    dev_baro_ms5611.h
 * @brief   MS5611 driver. Opaque singleton.
 *
 * UBC Rocket, 2026
 */
#ifndef DEV_BARO_MS5611_H
#define DEV_BARO_MS5611_H

#include "io_sys/io_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint64_t t_us;
    float    pressure_pa;
    float    temp_c;
} baro_sample_t;

typedef struct baro_ms5611 baro_ms5611_t;

bool           baro_ms5611_init      (void);

/**
 * @brief Create the MS5611 worker task and start the TIM7 conversion pacer.
 *        Call from app_init (in the same window as the other tasks, right
 *        before the scheduler starts) — NOT from baro_ms5611_init. Creating a
 *        task pre-scheduler leaves BASEPRI raised and freezes HAL_GetTick();
 *        see dev_mag_mmc5983.h. Idempotent; returns false if init hasn't run,
 *        task creation fails, or the timer won't start.
 */
bool           baro_ms5611_start     (void);

/**
 * @brief TIM7 update-IRQ hook. Call from HAL_TIM_PeriodElapsedCallback with
 *        the fired timer handle; on a TIM7 match it notifies the baro worker
 *        to run one conversion step. ISR-safe (notifies only, no SPI). No-op
 *        for any other timer.
 */
void           baro_ms5611_on_tim_period_elapsed(void *htim);

baro_ms5611_t *baro_ms5611_get       (void);
size_t         baro_ms5611_drain     (baro_ms5611_t *d, baro_sample_t *out, size_t max);
void           baro_ms5611_set_notify(baro_ms5611_t *d, io_task_handle_t t, uint32_t bit);

#ifdef __cplusplus
}
#endif

#endif /* DEV_BARO_MS5611_H */
