#ifndef QUATERNION_H
#define QUATERNION_H

#include <state_estimation/ekf.h>

void state_transition_orientation(
    quaternion_state *state, // previous state
    float time_step,
    float g[3], // gyro data
    float out_q[4]
);

void get_state_jacobian_orientation(
    float g[3],
    float dT,
    float j[4][4]
);

void predict_accel_from_quat(const float q[4], float accel_pred[3], float predicted_g[3]);

void get_h_jacobian_quaternion(
    float q[4],
    float expected_g[3],
    float h_jacobian[3][4]
);

void quat_to_euler(float q[4], float e[3]);

void quat_mult(const float q1[4], const float q2[4], float out[4]);

#endif