/**
 * @file    calibration.h
 * @brief   Stationary calibration for the ESKF.
 */
#ifndef CALIBRATION_H
#define CALIBRATION_H

#include "state_estimation/ekf.h"

/**
 * @brief Process one calibration sample (stationary assumption).
 *
 * Accumulates gyro and/or accel readings per IMU to estimate initial biases.
 * Calibration completes when all active IMUs have enough samples.
 *
 * @param eskf     ESKF instance.
 * @param gyro     Raw gyro reading [rad/s], or NULL.
 * @param accel    Raw accel reading [g], or NULL.
 * @param imu_idx  Which IMU this sample belongs to.
 */
void eskf_calibration_update(eskf_t *eskf, const float gyro[3],
                             const float accel[3], uint8_t imu_idx);

/**
 * @brief Process one magnetometer calibration sample.
 */
void eskf_calibration_update_mag(eskf_t *eskf, const float mag[3]);

#endif /* CALIBRATION_H */
