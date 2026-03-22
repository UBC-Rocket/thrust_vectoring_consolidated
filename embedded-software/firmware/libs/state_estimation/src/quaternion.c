/**
 * @file    quaternion.c
 * @brief   ESKF orientation: nominal propagation, error-state Jacobians,
 *          accelerometer and magnetometer observation models.
 */
#include "state_estimation/quaternion.h"
#include "state_estimation/state.h"
#include "state_estimation/matrix.h"
#include <math.h>
#include <string.h>

/* ========================================================================
 * Nominal State Propagation
 * ====================================================================== */

void eskf_propagate_nominal(orientation_eskf_state_t *state,
                            const float gyro_corr[3], float dt)
{
    float rv[3] = { gyro_corr[0] * dt,
                    gyro_corr[1] * dt,
                    gyro_corr[2] * dt };

    quaternion_t dq;
    quaternion_from_rotation_vector(rv, &dq);

    quaternion_t q_old;
    q_old.w = state->q_nom[0];
    q_old.x = state->q_nom[1];
    q_old.y = state->q_nom[2];
    q_old.z = state->q_nom[3];

    quaternion_t q_new;
    quaternion_multiply(&q_old, &dq, &q_new);

    state->q_nom[0] = q_new.w;
    state->q_nom[1] = q_new.x;
    state->q_nom[2] = q_new.y;
    state->q_nom[3] = q_new.z;

    normalize(state->q_nom);
}

/* ========================================================================
 * Error-State Transition Matrix
 * ====================================================================== */

void eskf_get_F(const float gyro_corr[3], float dt,
                float F_d[ESKF_ERR_DIM][ESKF_ERR_DIM], uint8_t imu_idx)
{
    memset(F_d, 0, sizeof(float) * ESKF_ERR_DIM * ESKF_ERR_DIM);

    /* Identity */
    for (int i = 0; i < ESKF_ERR_DIM; i++) F_d[i][i] = 1.0f;

    /* -[ω]ₓ * dt  (top-left 3x3) */
    float wdt[3] = { gyro_corr[0] * dt,
                     gyro_corr[1] * dt,
                     gyro_corr[2] * dt };

    F_d[0][1] +=  wdt[2];   /* +wz*dt */
    F_d[0][2] += -wdt[1];   /* -wy*dt */
    F_d[1][0] += -wdt[2];   /* -wz*dt */
    F_d[1][2] +=  wdt[0];   /* +wx*dt */
    F_d[2][0] +=  wdt[1];   /* +wy*dt */
    F_d[2][1] += -wdt[0];   /* -wx*dt */

    /* -I₃ * dt at this IMU's gyro bias columns */
    int bg_col = 3 + 6 * imu_idx;
    F_d[0][bg_col + 0] = -dt;
    F_d[1][bg_col + 1] = -dt;
    F_d[2][bg_col + 2] = -dt;
}

/* ========================================================================
 * Observation Models
 * ====================================================================== */

static void rotate_nav_to_body(const float q[4], const float v_nav[3],
                               float v_body[3])
{
    quaternion_t qt;
    qt.w = q[0]; qt.x = q[1]; qt.y = q[2]; qt.z = q[3];

    rotation_matrix_t R;
    quaternion_to_rotation_matrix(&qt, &R);

    for (int i = 0; i < 3; i++) {
        v_body[i] = R.R[0][i] * v_nav[0]
                  + R.R[1][i] * v_nav[1]
                  + R.R[2][i] * v_nav[2];
    }
}

void eskf_predict_accel(const float q_nom[4], const float expected_g[3],
                        float accel_pred[3])
{
    rotate_nav_to_body(q_nom, expected_g, accel_pred);
}

void eskf_predict_mag(const float q_nom[4], const float mag_ref[3],
                      float mag_pred[3])
{
    rotate_nav_to_body(q_nom, mag_ref, mag_pred);
}

/* ========================================================================
 * Measurement Jacobians
 * ====================================================================== */

void eskf_get_H_accel(const float q_nom[4], const float expected_g[3],
                      float H[3][ESKF_ERR_DIM], uint8_t imu_idx)
{
    memset(H, 0, sizeof(float) * 3 * ESKF_ERR_DIM);

    /* Predicted accel in body frame */
    float a_pred[3];
    eskf_predict_accel(q_nom, expected_g, a_pred);

    /* Cols 0..2: [a_pred]ₓ */
    H[0][0] =  0.0f;       H[0][1] = -a_pred[2]; H[0][2] =  a_pred[1];
    H[1][0] =  a_pred[2];  H[1][1] =  0.0f;      H[1][2] = -a_pred[0];
    H[2][0] = -a_pred[1];  H[2][1] =  a_pred[0]; H[2][2] =  0.0f;

    /* Accel bias columns left as zero.
     * The normalized observation model h = normalize(R^T * g + b_a) has
     * ∂h/∂b_a = (I - a_hat * a_hat^T) / |a|, which zeroes out the component
     * along gravity.  Using the full -R^T here would cause divergence.
     * Accel bias is estimated during calibration, not via measurement update.
     */
    (void)imu_idx;
}

void eskf_get_H_mag(const float q_nom[4], const float mag_ref[3],
                    float H[3][ESKF_ERR_DIM])
{
    memset(H, 0, sizeof(float) * 3 * ESKF_ERR_DIM);

    float m_pred[3];
    eskf_predict_mag(q_nom, mag_ref, m_pred);

    /* Cols 0..2: [m_pred]ₓ */
    H[0][0] =  0.0f;       H[0][1] = -m_pred[2]; H[0][2] =  m_pred[1];
    H[1][0] =  m_pred[2];  H[1][1] =  0.0f;      H[1][2] = -m_pred[0];
    H[2][0] = -m_pred[1];  H[2][1] =  m_pred[0]; H[2][2] =  0.0f;

    /* All bias cols zero — mag doesn't depend on IMU biases */
}

/* ========================================================================
 * Utilities
 * ====================================================================== */

void quat_to_euler(const float q[4], float euler[3])
{
    float w = q[0], x = q[1], y = q[2], z = q[3];

    float sinr = 2.0f * (w * x + y * z);
    float cosr = 1.0f - 2.0f * (x * x + y * y);
    euler[0] = atan2f(sinr, cosr) * 180.0f / (float)M_PI;

    float sinp = 2.0f * (w * y - z * x);
    if (fabsf(sinp) >= 1.0f)
        euler[1] = copysignf((float)M_PI / 2.0f, sinp) * 180.0f / (float)M_PI;
    else
        euler[1] = asinf(sinp) * 180.0f / (float)M_PI;

    float siny = 2.0f * (w * z + x * y);
    float cosy = 1.0f - 2.0f * (y * y + z * z);
    euler[2] = atan2f(siny, cosy) * 180.0f / (float)M_PI;
}
