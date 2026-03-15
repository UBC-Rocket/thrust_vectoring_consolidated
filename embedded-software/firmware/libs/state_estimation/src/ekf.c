/**
 * @file ekf.c
 * @brief Extended Kalman Filter for orientation (quaternion) and body (position/velocity).
 *
 * Noise parameters derived from BMI088 datasheet (DS066-00, Rev 1.7):
 *   Gyroscope noise density:      0.014 deg/s/sqrt(Hz)  (Table 3)
 *   Accelerometer noise density:  175 ug/sqrt(Hz)       (Table 1)
 *
 * GPS measurement noise assumes a u-blox receiver with ~2.5m CEP horizontal.
 */

#include "state_estimation/ekf.h"
#include "state_estimation/body.h"
#include "state_estimation/matrix.h"
#include "state_estimation/quaternion.h"
#include "state_estimation/state.h"
#include "math.h"
#include "string.h"
#include <stdbool.h>

static EKF ekf;

/* ========================================================================== */
/* BMI088 datasheet noise parameters                                          */
/* ========================================================================== */

/* Gyroscope: 0.014 deg/s/sqrt(Hz) = 2.443e-4 rad/s/sqrt(Hz) */
#define GYRO_NOISE_DENSITY_RAD_S   2.443e-4f
/* Effective noise bandwidth at 400 Hz ODR / 116 Hz BW setting */
#define GYRO_BW_HZ                 116.0f
/* sigma^2_gyro = noise_density^2 * BW = 6.92e-6 (rad/s)^2 */
#define GYRO_NOISE_VAR             (GYRO_NOISE_DENSITY_RAD_S * GYRO_NOISE_DENSITY_RAD_S * GYRO_BW_HZ)

/* Accelerometer: 175 ug/sqrt(Hz) */
#define ACCEL_NOISE_DENSITY_G      175e-6f
/* Effective noise bandwidth at 800 Hz ODR, normal mode (~280 Hz) */
#define ACCEL_BW_HZ                280.0f
/* sigma^2_accel in g^2 (for normalized accel measurement in orientation EKF) */
#define ACCEL_NOISE_VAR_G2         (ACCEL_NOISE_DENSITY_G * ACCEL_NOISE_DENSITY_G * ACCEL_BW_HZ)

/* sigma_accel in m/s^2 for body EKF process noise */
#define GRAV_MPS2                  9.807f
#define ACCEL_NOISE_DENSITY_MS2    (ACCEL_NOISE_DENSITY_G * GRAV_MPS2)
/* sigma^2_accel in (m/s^2)^2 */
#define ACCEL_NOISE_VAR_MS2        (ACCEL_NOISE_DENSITY_MS2 * ACCEL_NOISE_DENSITY_MS2 * ACCEL_BW_HZ)

/* GPS measurement noise: u-blox CEP ~2.5m horizontal, ~5m vertical */
#define GPS_NOISE_VAR_XY           6.25f    /* (2.5 m)^2 */
#define GPS_NOISE_VAR_Z            25.0f    /* (5.0 m)^2 */

/*
 * MS5611 barometer noise at OSR 4096: ~0.012 mbar RMS (datasheet Table 3)
 * Altitude sensitivity ~8.43 m/mbar at sea level => ~0.1 m altitude RMS
 */
#define BARO_NOISE_VAR_M2          0.01f    /* (0.1 m)^2 */

/* ========================================================================== */
/* EKF tuning constants                                                       */
/* ========================================================================== */

#define EKF_DEBUG_LOG_PERIOD             50U
#define MAX_QUAT_VARIANCE                0.5f
#define ACCEL_UPDATE_MIN_NORM_G          0.85f
#define ACCEL_UPDATE_MAX_NORM_G          1.15f
#define ACCEL_UPDATE_MAX_GYRO_NORM_RAD_S 3.0f

static uint32_t s_orientation_tick_count = 0;
static uint32_t s_body_tick_count = 0;

/* Measurement noise matrices (constant, derived from datasheet) */
static const float R_orientation[3][3] = {
    {ACCEL_NOISE_VAR_G2, 0, 0},
    {0, ACCEL_NOISE_VAR_G2, 0},
    {0, 0, ACCEL_NOISE_VAR_G2}
};

static const float R_body[3][3] = {
    {0.005, 0, 0},
    {0, 0.005, 0},
    {0, 0, 0.005}
};

/* ========================================================================== */
/* Static helpers                                                             */
/* ========================================================================== */

static uint8_t ekf_accel_update_valid(float a_norm_g)
{
    return (a_norm_g >= ACCEL_UPDATE_MIN_NORM_G) &&
           (a_norm_g <= ACCEL_UPDATE_MAX_NORM_G);
}

static uint8_t ekf_gyro_update_valid(float g_norm_rad_s)
{
    return g_norm_rad_s <= ACCEL_UPDATE_MAX_GYRO_NORM_RAD_S;
}

static void ekf_preserve_predicted_yaw(const float q_pred[4], float q_corr[4])
{
    float norm_corr = sqrtf(q_corr[0] * q_corr[0] + q_corr[3] * q_corr[3]);
    float twist_corr_inv[4] = { 1.0f, 0, 0, 0 };
    if (norm_corr > 1e-6f) {
        twist_corr_inv[0] = q_corr[0] / norm_corr;
        twist_corr_inv[3] = -q_corr[3] / norm_corr;
    }

    float swing_corr[4];
    quat_mult(q_corr, twist_corr_inv, swing_corr);

    float norm_pred = sqrtf(q_pred[0] * q_pred[0] + q_pred[3] * q_pred[3]);
    float twist_pred[4] = { 1.0f, 0, 0, 0 };
    if (norm_pred > 1e-6f) {
        twist_pred[0] = q_pred[0] / norm_pred;
        twist_pred[3] = q_pred[3] / norm_pred;
    }

    quat_mult(swing_corr, twist_pred, q_corr);
    normalize(q_corr);
}

/* ========================================================================== */
/* Covariance prediction                                                      */
/* ========================================================================== */

/**
 * Orientation covariance prediction: P = F*P*F' + Q_d
 * Q_d is computed from gyro noise density scaled by dt:
 *   Q_d_diag = (dt/2)^2 * sigma^2_gyro
 */
static void predict_covar_orientation(float jacobian[4][4],
                                      float dt,
                                      float predicted_covar[4][4])
{
    float m1[4][4];
    float jacobian_transposed[4][4];

    transpose4x4(jacobian, jacobian_transposed);
    MAT_MUL(jacobian, ekf.quaternion.covar, m1, 4, 4, 4);
    MAT_MUL(m1, jacobian_transposed, predicted_covar, 4, 4, 4);

    /* Q_d = (dt/2)^2 * sigma^2_gyro  (diagonal) */
    const float q_d = (dt * 0.5f) * (dt * 0.5f) * GYRO_NOISE_VAR;
    for (int i = 0; i < 4; i++)
        predicted_covar[i][i] += q_d;
}

/**
 * Sparse covariance prediction exploiting F = [[I, dt*I], [0, I]].
 *
 * Process noise Q_d is derived from accelerometer noise density:
 *   G = [[dt^2/2 * I], [dt * I]]
 *   Q_d = G * sigma^2_accel * G'
 *     = [[dt^4/4 * s2, dt^3/2 * s2],
 *        [dt^3/2 * s2, dt^2   * s2]]   (diagonal blocks)
 */
static void predict_covar_body_sparse(float dt, float predicted_covar[6][6])
{
    const float dt2 = dt * dt;
    const float dt3 = dt2 * dt;
    const float dt4 = dt2 * dt2;
    const float s2 = 2;
    const float (*P)[6] = (const float (*)[6])ekf.body.covar;

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            const float Ppp = P[i][j];
            const float Ppv = P[i][j + 3];
            const float Pvp = P[i + 3][j];
            const float Pvv = P[i + 3][j + 3];

            /* Q_d blocks are diagonal, so only add on i==j */
            const float q_pp = (i == j) ? (dt4 * 0.25f * s2) : 0.0f;
            const float q_pv = (i == j) ? (dt3 * 0.5f  * s2) : 0.0f;
            const float q_vv = (i == j) ? (dt2 * s2)         : 0.0f;

            predicted_covar[i][j]         = Ppp + dt * (Ppv + Pvp) + dt2 * Pvv + q_pp;
            predicted_covar[i][j + 3]     = Ppv + dt * Pvv + q_pv;
            predicted_covar[i + 3][j]     = Pvp + dt * Pvv + q_pv;
            predicted_covar[i + 3][j + 3] = Pvv + q_vv;
        }
    }
}

/* ========================================================================== */
/* Initialization                                                             */
/* ========================================================================== */

void init_ekf(const float expected_g[3])
{
    /* Orientation: quaternion to identity, covariance to identity */
    ekf.quaternion.vals[0] = 1.0f;
    ekf.quaternion.vals[1] = 0.0f;
    ekf.quaternion.vals[2] = 0.0f;
    ekf.quaternion.vals[3] = 0.0f;
    ekf.quaternion.index = 0;

    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            ekf.quaternion.covar[i][j] = (i == j) ? 1.0f : 0.0f;

    for (int i = 0; i < 3; i++)
        ekf.expected_g[i] = expected_g[i];

    /* Body: state to zero, covariance to identity */
    for (int i = 0; i < 3; i++) {
        ekf.body.position[i] = 0.0f;
        ekf.body.velocity[i] = 0.0f;
    }
    ekf.body.index = 0;

    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            ekf.body.covar[i][j] = (i == j) ? 1.0f : 0.0f;
}

/* ========================================================================== */
/* Orientation EKF tick                                                       */
/* ========================================================================== */

void tick_ekf_orientation(float deltaTime, float gyro[3], float accel[3])
{
    ekf.quaternion.index += 1;
    s_orientation_tick_count++;

    /* --- 1. PREDICTION --- */
    static float processing_quaternion[4];
    state_transition_orientation(&ekf.quaternion, deltaTime, gyro, processing_quaternion);

    static float state_jacobian_quaternion[4][4];
    get_state_jacobian_orientation(gyro, deltaTime, state_jacobian_quaternion);

    static float predicted_covar_quaternion[4][4];
    predict_covar_orientation(state_jacobian_quaternion, deltaTime, predicted_covar_quaternion);

    /* --- 2. UPDATE (Correction) --- */
    static float innovation_quaternion[3][1];
    static float predicted_accel[3];

    float a_norm = sqrtf(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
    float g_norm = sqrtf(gyro[0]*gyro[0] + gyro[1]*gyro[1] + gyro[2]*gyro[2]);

    /* Skip measurement correction during strong linear accel or fast rotation */
     if (!ekf_accel_update_valid(a_norm) || !ekf_gyro_update_valid(g_norm)) {
         memcpy(ekf.quaternion.vals, processing_quaternion, sizeof(processing_quaternion));
         memcpy(ekf.quaternion.covar, predicted_covar_quaternion, sizeof(predicted_covar_quaternion));
         return;
    }

    predict_accel_from_quat(processing_quaternion, predicted_accel, ekf.expected_g);

    static float a_normalized[3];
    if (a_norm > 0.1f) {
        for (int i = 0; i < 3; i++) a_normalized[i] = accel[i] / a_norm;
    } else {
        for (int i = 0; i < 3; i++) a_normalized[i] = ekf.expected_g[i];
    }

    for (int i = 0; i < 3; i++)
        innovation_quaternion[i][0] = a_normalized[i] - predicted_accel[i];

    /* H Jacobian */
    static float h_jacobian_quaternion[3][4];
    get_h_jacobian_quaternion(processing_quaternion, ekf.expected_g, h_jacobian_quaternion);

    static float h_jacobian_quaternion_t[4][3];
    transpose3x4_to_4x3(h_jacobian_quaternion, h_jacobian_quaternion_t);

    /* S = H * P * H' + R */
    static float mat1_q[4][3];
    MAT_MUL(predicted_covar_quaternion, h_jacobian_quaternion_t, mat1_q, 4, 4, 3);

    static float mat3_q[3][3];
    MAT_MUL(h_jacobian_quaternion, mat1_q, mat3_q, 3, 4, 3);

    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            mat3_q[i][j] += R_orientation[i][j];

    static float inv_mat3_q[3][3];
    bool inverse_success = inverse(mat3_q, inv_mat3_q);
    if (!inverse(mat3_q, inv_mat3_q)) {
        return;
    }

    /* K = P * H' * S^-1 */
    static float kalman_gain_quaternion[4][3];
    MAT_MUL(mat1_q, inv_mat3_q, kalman_gain_quaternion, 4, 3, 3);

    /* State update */
    static float adjustment_quaternion[4][1];
    MAT_MUL(kalman_gain_quaternion, innovation_quaternion, adjustment_quaternion, 4, 3, 1);

    for (int i = 0; i < 4; i++)
        ekf.quaternion.vals[i] = processing_quaternion[i] + adjustment_quaternion[i][0];

    normalize(ekf.quaternion.vals);
    ekf_preserve_predicted_yaw(processing_quaternion, ekf.quaternion.vals);

    /* Covariance update: Joseph form P = (I-KH)*P*(I-KH)' + K*R*K' */
    if (ekf.quaternion.index <= UPDATE_COVAR) {
        return;
    }
    ekf.quaternion.index = 0;

    static float IKH[4][4];
    MAT_MUL(kalman_gain_quaternion, h_jacobian_quaternion, IKH, 4, 3, 4);
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            IKH[i][j] = (i == j) ? (1.0f - IKH[i][j]) : (-IKH[i][j]);

    static float IKH_P[4][4];
    MAT_MUL(IKH, predicted_covar_quaternion, IKH_P, 4, 4, 4);

    static float IKH_t[4][4];
    transpose4x4(IKH, IKH_t);
    static float new_covar_quaternion[4][4];
    MAT_MUL(IKH_P, IKH_t, new_covar_quaternion, 4, 4, 4);

    static float KR[4][3];
    float R_ori_copy[3][3];
    memcpy(R_ori_copy, R_orientation, sizeof(R_orientation));
    MAT_MUL(kalman_gain_quaternion, R_ori_copy, KR, 4, 3, 3);
    static float KR_Kt[4][4];
    static float K_t[3][4];
    transpose4x3_to_3x4(kalman_gain_quaternion, K_t);
    MAT_MUL(KR, K_t, KR_Kt, 4, 3, 4);

    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            new_covar_quaternion[i][j] += KR_Kt[i][j];

    memcpy(ekf.quaternion.covar, new_covar_quaternion, sizeof(new_covar_quaternion));

    for (int i = 0; i < 4; i++) {
        if (ekf.quaternion.covar[i][i] > MAX_QUAT_VARIANCE) {
            ekf.quaternion.covar[i][i] = MAX_QUAT_VARIANCE;
        }
        for (int j = i + 1; j < 4; j++) {
            float avg = (ekf.quaternion.covar[i][j] + ekf.quaternion.covar[j][i]) * 0.5f;
            ekf.quaternion.covar[i][j] = ekf.quaternion.covar[j][i] = avg;
        }
    }
}

/* ========================================================================== */
/* Body EKF: predict (IMU-driven) and update (GPS measurement) split          */
/* ========================================================================== */

/* Predicted state stored between predict and update calls */
static float s_body_predicted_pos[3];
static float s_body_predicted_vel[3];
static float s_body_predicted_covar[6][6];

/**
 * Body EKF prediction step. Run every IMU cycle.
 * Uses nav-frame acceleration to propagate position and velocity.
 */
void predict_ekf_body(float dt, float accel_nav[3])
{
    s_body_tick_count++;

    state_transition_body(&ekf.body, dt, accel_nav,
                          s_body_predicted_pos, s_body_predicted_vel);
    predict_covar_body_sparse(dt, s_body_predicted_covar);

    /* Commit predicted state (will be corrected by update if GPS available) */
    for (int i = 0; i < 3; i++) {
        ekf.body.position[i] = s_body_predicted_pos[i];
        ekf.body.velocity[i] = s_body_predicted_vel[i];
    }
    memcpy(ekf.body.covar, s_body_predicted_covar, sizeof(s_body_predicted_covar));
}

/**
 * Body EKF update step. Run only when GPS measurement is available.
 * Exploits H = [I_3, 0_3] sparsity:
 *   S = P[0:3][0:3] + R
 *   K = P[:,0:3] * S^-1
 */
void update_ekf_body(float gps_pos[3])
{
    ekf.body.index += 1;

    const float (*P)[6] = (const float (*)[6])ekf.body.covar;

    /* Innovation: y = z - H*x = gps - position */
    float innovation[3];
    for (int i = 0; i < 3; i++)
        innovation[i] = gps_pos[i] - ekf.body.position[i];

    /* S = P[0:3][0:3] + R */
    float S[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            S[i][j] = P[i][j] + R_body[i][j];

    float S_inv[3][3];
    if (!inverse(S, S_inv)) {
        return;
    }

    /* K = P[:,0:3] * S^-1 */
    float K[6][3];
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 3; j++) {
            float sum = 0.0f;
            for (int k = 0; k < 3; k++)
                sum += P[i][k] * S_inv[k][j];
            K[i][j] = sum;
        }

    /* State update: x += K * y */
    for (int i = 0; i < 3; i++) {
        float adj = K[i][0]*innovation[0] + K[i][1]*innovation[1] + K[i][2]*innovation[2];
        ekf.body.position[i] += adj;
    }
    for (int i = 0; i < 3; i++) {
        float adj = K[i+3][0]*innovation[0] + K[i+3][1]*innovation[1] + K[i+3][2]*innovation[2];
        ekf.body.velocity[i] += adj;
    }

    /* Covariance update: P = (I-KH)*P = P - K * P[0:3,:] */
    ekf.body.index = 0;

    float new_covar[6][6];
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++) {
            float kp = K[i][0]*P[0][j] + K[i][1]*P[1][j] + K[i][2]*P[2][j];
            new_covar[i][j] = P[i][j] - kp;
        }

    memcpy(ekf.body.covar, new_covar, sizeof(new_covar));
}

/**
 * Body EKF barometer update. Run when baro measurement is available.
 *
 * H = [0, 0, 1, 0, 0, 0] (scalar observation of z-position only).
 * All operations reduce to scalar math — no matrix inversion needed.
 *
 * @param altitude_m  Barometric altitude in meters, relative to launch site.
 */
void update_ekf_body_baro(float altitude_m)
{
    float (*P)[6] = ekf.body.covar;

    /* Innovation: y = z_baro - position_z */
    const float innovation = altitude_m - ekf.body.position[2];

    /* S = H*P*H' + R = P[2][2] + R_baro  (scalar) */
    const float S = P[2][2] + BARO_NOISE_VAR_M2;
    if (S < 1e-12f)
        return;
    const float S_inv = 1.0f / S;

    /* K = P * H' * S^-1 = P[:,2] / S  (6x1 vector) */
    float K[6];
    for (int i = 0; i < 6; i++)
        K[i] = P[i][2] * S_inv;

    /* State update: x += K * y */
    for (int i = 0; i < 3; i++)
        ekf.body.position[i] += K[i] * innovation;
    for (int i = 0; i < 3; i++)
        ekf.body.velocity[i] += K[i + 3] * innovation;

    /* Covariance update: P = P - K * P[2,:]  (rank-1) */
    float P_row2[6];
    for (int j = 0; j < 6; j++)
        P_row2[j] = P[2][j];

    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            P[i][j] -= K[i] * P_row2[j];
}

/* ========================================================================== */
/* Getters                                                                    */
/* ========================================================================== */

void get_state(float quaternion[4], float position[3], float velocity[3])
{
    for (int i = 0; i < 4; i++)
        quaternion[i] = ekf.quaternion.vals[i];

    for (int i = 0; i < 3; i++) {
        position[i] = ekf.body.position[i];
        velocity[i] = ekf.body.velocity[i];
    }
}
