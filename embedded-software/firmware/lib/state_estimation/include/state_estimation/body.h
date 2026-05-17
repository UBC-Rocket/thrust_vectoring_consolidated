/**
 * @file    body.h
 * @brief   Body-frame state transitions and measurement models (position/velocity).
 */
#ifndef BODY_H
#define BODY_H

#include <stdint.h>
#include "state_estimation/ekf.h"

/**
 * @brief Transform body-frame accelerometer to nav-frame linear acceleration.
 *
 * Rotates accel (in g) to nav frame via quaternion, converts to m/s²,
 * and subtracts gravity.
 *
 * @param a_body  Body-frame accel [g].
 * @param q       Body-to-nav quaternion [w,x,y,z].
 * @param a_nav   Output nav-frame linear acceleration [m/s²].
 */
void transform_accel_data(const float a_body[3], const float q[4],
                          float a_nav[3]);

/**
 * @brief Predict body state forward one step.
 *
 * pos_new = pos + vel*dt + 0.5*a*dt²
 * vel_new = vel + a*dt
 *
 * @param state   Body state (read position/velocity).
 * @param dt      Time step [s].
 * @param a_nav   Nav-frame linear acceleration [m/s²].
 * @param out_p   Output predicted position.
 * @param out_v   Output predicted velocity.
 */
void state_transition_body(const body_state_t *state, float dt,
                           const float a_nav[3],
                           float out_p[3], float out_v[3]);

/**
 * @brief Compute 6x6 state Jacobian for body: F = [[I, dt*I], [0, I]].
 */
void get_state_jacobian_body(float dt, float F[6][6]);

/**
 * @brief Sparse covariance prediction exploiting F = [[I,dt*I],[0,I]].
 *
 * @param P       Input covariance (6x6).
 * @param Q       Process noise (6x6).
 * @param dt      Time step.
 * @param P_pred  Output predicted covariance (6x6).
 */
void predict_covar_body_sparse(const float P[6][6], const float Q[6][6],
                               float dt, float P_pred[6][6]);

/**
 * @brief Convert barometer pressure (centipascals) to height [m].
 */
float pressure_to_height(int32_t pressure_centi);

#endif /* BODY_H */
