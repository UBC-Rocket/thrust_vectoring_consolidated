/**
 * @file    dev_imu_icm40609.h
 * @brief   ICM-40609 driver. Opaque singleton.
 *
 * UBC Rocket, 2026
 */
#ifndef DEV_IMU_ICM40609_H
#define DEV_IMU_ICM40609_H

#include "io_sys/io_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint64_t t_us;
    float    ax, ay, az;     ///< m/s²
    float    gx, gy, gz;     ///< rad/s
    float    temp_c;
} icm_sample_t;

typedef struct imu_icm40609 imu_icm40609_t;

bool             imu_icm40609_init      (void);
imu_icm40609_t  *imu_icm40609_get       (void);
size_t           imu_icm40609_drain     (imu_icm40609_t *d, icm_sample_t *out, size_t max);
void             imu_icm40609_set_notify(imu_icm40609_t *d, io_task_handle_t t, uint32_t bit);

/** Last raw WHO_AM_I byte read during init() (bring-up diagnostic). 0x3B =
 *  correct part; 0xEE = SPI read never completed; 0x00 / 0xFF = no response /
 *  floating bus; any other value = wired but mis-clocked. */
uint8_t          imu_icm40609_last_who_am_i(void);

/** Which init() step failed (bring-up diagnostic). 0 = success; 1..11 = the
 *  numbered step that returned false; 0xFF = init() never ran. Step 4 is the
 *  WHO_AM_I identity check (pair with imu_icm40609_last_who_am_i()). */
uint8_t          imu_icm40609_init_fail_step(void);

/** Runtime data-path counters (bring-up): DATA_READY EXTI fires, async reads
 *  parsed+queued, async reads errored. Any pointer may be NULL. drdy==0 means
 *  INT1 never fired. */
void             imu_icm40609_dbg_counts(uint32_t *drdy, uint32_t *rd_ok, uint32_t *rd_err);

/** INT routing registers read back after init (bring-up): INT_CONFIG (expect
 *  0x03) and INT_SOURCE0 (expect 0x08). Confirms the DATA_READY→INT1 routing
 *  latched on the chip. Either pointer may be NULL. */
void             imu_icm40609_dbg_int_regs(uint8_t *int_config, uint8_t *int_source0);

#ifdef __cplusplus
}
#endif

#endif /* DEV_IMU_ICM40609_H */
