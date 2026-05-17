/**
 * @file    dev_imu_bmi088.h
 * @brief   BMI088 driver. Opaque singleton: storage lives in the .c, callers
 *          obtain a pointer via imu_bmi088_get().
 *
 * UBC Rocket, 2026
 */
#ifndef DEV_IMU_BMI088_H
#define DEV_IMU_BMI088_H

#include "io_sys/io_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint64_t t_us;
    float    ax, ay, az;     ///< m/s²,  sensor frame
    float    gx, gy, gz;     ///< rad/s, sensor frame
    float    temp_c;
} imu_sample_t;

/* Opaque — struct defined in the .c. */
typedef struct imu_bmi088 imu_bmi088_t;

bool             imu_bmi088_init      (void);
imu_bmi088_t    *imu_bmi088_get       (void);   /* NULL before init */
size_t           imu_bmi088_drain     (imu_bmi088_t *d, imu_sample_t *out, size_t max);
void             imu_bmi088_set_notify(imu_bmi088_t *d, io_task_handle_t task, uint32_t bit);

#ifdef __cplusplus
}
#endif

#endif /* DEV_IMU_BMI088_H */
