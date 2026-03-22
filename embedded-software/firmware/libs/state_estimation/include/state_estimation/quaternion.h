/**
 * @file    quaternion.h
 * @brief   ESKF orientation functions: nominal propagation, Jacobians, observation models.
 */
#ifndef QUATERNION_H
#define QUATERNION_H

#include "state_estimation/ekf.h"

/**
 * @brief Propagate nominal quaternion: q_nom = q_nom ⊗ q(ω_corr * dt).
 */
void eskf_propagate_nominal(orientation_eskf_state_t *state,
                            const float gyro_corr[3], float dt);

/**
 * @brief Compute error-state transition matrix F_d (ESKF_ERR_DIM x ESKF_ERR_DIM).
 *
 * The gyro bias sensitivity (-I₃·dt) is placed at the columns for imu_idx's
 * gyro bias block: cols [3+6*imu_idx .. 5+6*imu_idx].
 *
 * @param gyro_corr  Bias-corrected gyro [rad/s].
 * @param dt         Time step [s].
 * @param F_d        Output transition matrix.
 * @param imu_idx    Which IMU's gyro is being processed.
 */
void eskf_get_F(const float gyro_corr[3], float dt,
                float F_d[ESKF_ERR_DIM][ESKF_ERR_DIM], uint8_t imu_idx);

/**
 * @brief Predict accel observation from nominal quaternion.
 *        a_pred = R(q_nom)^T * expected_g
 */
void eskf_predict_accel(const float q_nom[4], const float expected_g[3],
                        float accel_pred[3]);

/**
 * @brief Compute accelerometer measurement Jacobian H (3 x ESKF_ERR_DIM).
 *
 * Cols 0-2: [a_pred]ₓ (attitude error sensitivity)
 * Cols [6+6*imu_idx .. 8+6*imu_idx]: -R^T (accel bias sensitivity for this IMU)
 *
 * @param imu_idx  Which IMU's accel is being processed.
 */
void eskf_get_H_accel(const float q_nom[4], const float expected_g[3],
                      float H[3][ESKF_ERR_DIM], uint8_t imu_idx);

/**
 * @brief Predict magnetometer observation from nominal quaternion.
 *        m_pred = R(q_nom)^T * mag_ref
 */
void eskf_predict_mag(const float q_nom[4], const float mag_ref[3],
                      float mag_pred[3]);

/**
 * @brief Compute magnetometer measurement Jacobian H (3 x ESKF_ERR_DIM).
 *
 * Cols 0-2: [m_pred]ₓ. All bias cols are zero (mag doesn't depend on IMU biases).
 */
void eskf_get_H_mag(const float q_nom[4], const float mag_ref[3],
                    float H[3][ESKF_ERR_DIM]);

/**
 * @brief Convert quaternion to Euler angles [roll, pitch, yaw] in degrees.
 */
void quat_to_euler(const float q[4], float euler[3]);

#endif /* QUATERNION_H */
