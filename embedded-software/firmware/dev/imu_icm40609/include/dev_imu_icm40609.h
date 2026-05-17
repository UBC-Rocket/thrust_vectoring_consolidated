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

#ifdef __cplusplus
}
#endif

#endif /* DEV_IMU_ICM40609_H */
