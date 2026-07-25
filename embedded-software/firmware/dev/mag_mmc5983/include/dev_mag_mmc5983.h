/**
 * @file    dev_mag_mmc5983.h
 * @brief   MEMSIC MMC5983MA magnetometer driver. Opaque singleton.
 *
 * SPI device on IO_SPI_MMC (= SPI1 on the H747 board), DRDY input on
 * IO_EXTI_MMC_INT. Default config: 100 Hz continuous mode with periodic
 * SET (every 1 measurement) so offset stays cancelled. 18-bit raw
 * counts are converted to gauss (1 mG / LSB on the ±8 G range) on the
 * way into the ring.
 *
 * UBC Rocket, 2026
 */
#ifndef DEV_MAG_MMC5983_H
#define DEV_MAG_MMC5983_H

#include "io_sys/io_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint64_t t_us;
    /** Magnetic field in gauss, sensor body frame (chip XYZ as marked).
     *  Apply the board's mounting matrix in the consumer if needed. */
    float    mx, my, mz;
    float    temp_c;
} mag_sample_t;

typedef struct mag_mmc5983 mag_mmc5983_t;

bool            mag_mmc5983_init       (void);

/**
 * @brief Create the SET/RESET service task. Call from app_init (after the
 *        DEV phase, in the same window the other tasks are created) —
 *        NOT from mag_mmc5983_init.
 *
 * xTaskCreateStatic takes a FreeRTOS critical section. Called before the
 * scheduler starts, that leaves BASEPRI raised (uxCriticalNesting is not
 * yet zero), masking the HAL-timebase IRQ and freezing HAL_GetTick(). If
 * any pre-scheduler code then blocks on a HAL timeout it hangs forever.
 * Deferring task creation to app_init keeps it in the harmless window
 * right before vTaskStartScheduler(). Idempotent; returns false if init
 * hasn't run or the task couldn't be created.
 */
bool            mag_mmc5983_start      (void);

mag_mmc5983_t  *mag_mmc5983_get        (void);
size_t          mag_mmc5983_drain      (mag_mmc5983_t *d,
                                         mag_sample_t *out, size_t max);
void            mag_mmc5983_set_notify (mag_mmc5983_t *d,
                                         io_task_handle_t t, uint32_t bit);

/**
 * @brief Most recent (M_set + M_reset)/2 hard-iron offset estimate, in
 *        gauss, sensor body frame. Updated every SET/RESET pair, ~100 Hz.
 *
 * @param d         Driver handle (NULL → returns false).
 * @param out_g     Filled with the three-axis offset on success.
 * @return          true if at least one SET/RESET pair has completed since
 *                  init (i.e. the value in out_g is meaningful), false
 *                  otherwise.
 *
 * The state-estimation task uses this for auto-calibration: once the EKF
 * declares itself calibrated, it snapshots this value into the storage
 * subsystem so subsequent boots come up with the offset pre-loaded
 * instead of having to wait another cal window.
 */
bool            mag_mmc5983_get_offset (mag_mmc5983_t *d, float out_g[3]);

#ifdef __cplusplus
}
#endif

#endif /* DEV_MAG_MMC5983_H */
