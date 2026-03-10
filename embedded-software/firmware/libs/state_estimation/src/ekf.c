#include "state_estimation/ekf.h"
#include <math.h>
#include <string.h>
#include "state_estimation/matrix.h"
#include "state_estimation/quaternion.h"
#include "state_estimation/body.h"
#include "state_estimation/state.h"

static EKF ekf;

// ==========================================
// HELPER FUNCTIONS
// ==========================================

void predict_covar_orientation(float jacobian[4][4], float predicted_covar[4][4]) {
    float m1[4][4];
    float jacobian_transposed[4][4];
    
    transpose4x4(jacobian, jacobian_transposed);
    mat_mul((const float *)jacobian, (const float *)ekf.quaternion.covar, (float *)m1, 4, 4, 4);
    mat_mul((const float *)m1, (const float *)jacobian_transposed, (float *)predicted_covar, 4, 4, 4);

    for (int i = 0; i < 4; i++) 
        for (int j = 0; j < 4; j++) 
            predicted_covar[i][j] += ekf.quaternion.process[i][j];
}

/**
 * Sparse covariance prediction exploiting F = [[I, dt*I], [0, I]].
 *
 * P (6x6) is partitioned into 3x3 blocks:
 *   P = [[Ppp, Ppv],
 *        [Pvp, Pvv]]
 *
 * F*P*F' = [[Ppp + dt*(Ppv+Pvp) + dt²*Pvv,  Ppv + dt*Pvv],
 *           [Pvp + dt*Pvv,                    Pvv         ]]
 */
void predict_covar_body_sparse(float dt, float predicted_covar[6][6]) {
    const float dt2 = dt * dt;
    const float (*P)[6] = (const float (*)[6])ekf.body.covar;

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            float Ppp = P[i][j];
            float Ppv = P[i][j + 3];
            float Pvp = P[i + 3][j];
            float Pvv = P[i + 3][j + 3];

            /* Top-left: Ppp + dt*(Ppv+Pvp) + dt²*Pvv */
            predicted_covar[i][j] = Ppp + dt * (Ppv + Pvp) + dt2 * Pvv
                                    + ekf.body.process[i][j];

            /* Top-right: Ppv + dt*Pvv */
            predicted_covar[i][j + 3] = Ppv + dt * Pvv
                                         + ekf.body.process[i][j + 3];

            /* Bottom-left: Pvp + dt*Pvv */
            predicted_covar[i + 3][j] = Pvp + dt * Pvv
                                         + ekf.body.process[i + 3][j];

            /* Bottom-right: Pvv (unchanged) */
            predicted_covar[i + 3][j + 3] = Pvv
                                             + ekf.body.process[i + 3][j + 3];
        }
    }
}

// ==========================================
// INITIALIZATION
// ==========================================

void init_ekf_orientation(
    float process_noise[4][4],
    float measurement_noise[3][3],
    float expected_g[3]
) {
    // Initialize quaternion to identity
    ekf.quaternion.vals[0] = 1;
    ekf.quaternion.vals[1] = 0;
    ekf.quaternion.vals[2] = 0;
    ekf.quaternion.vals[3] = 0;
    ekf.quaternion.index = 0;

    // Set Covariance to Identity
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            ekf.quaternion.covar[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }

    // Copy Noise and Gravity
    for (int i = 0; i < 4; i++) 
        for (int j = 0; j < 4; j++) 
            ekf.quaternion.process[i][j] = process_noise[i][j];
            
    for (int i = 0; i < 3; i++) 
        for (int j = 0; j < 3; j++) 
            ekf.quaternion.measurement[i][j] = measurement_noise[i][j];

    for (int i = 0; i < 3; i++) 
        ekf.expected_g[i] = expected_g[i];
}

void init_ekf_body(
    float process_noise[6][6],
    float measurement_noise[3][3]
) {
    // Initialize state to 0
    for (int i = 0; i < 3; i++) ekf.body.position[i] = 0;
    for (int i = 0; i < 3; i++) ekf.body.velocity[i] = 0;
    ekf.body.index = 0;

    // Set Covariance to Identity
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            ekf.body.covar[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }

    // Copy Noise
    for (int i = 0; i < 6; i++) 
        for (int j = 0; j < 6; j++) 
            ekf.body.process[i][j] = process_noise[i][j];

    for (int i = 0; i < 3; i++) 
        for (int j = 0; j < 3; j++) 
            ekf.body.measurement[i][j] = measurement_noise[i][j];
}

// Wrapper for legacy initialization if needed
void init_ekf(
    float process_noise_quaternion[4][4],
    float measurement_noise_quaternion[3][3],
    float process_noise_body[6][6],
    float measurement_noise_body[3][3],
    float expected_g[3]
) {
    init_ekf_orientation(process_noise_quaternion, measurement_noise_quaternion, expected_g);
    init_ekf_body(process_noise_body, measurement_noise_body);
}

// ==========================================
// UPDATE LOOPS
// ==========================================

void tick_ekf_orientation(float deltaTime, float gyro[3], float accel[3]) {
    ekf.quaternion.index += 1;

    // --- 1. PREDICTION ---
    float processing_quaternion[4];
    state_transition_orientation(&ekf.quaternion, deltaTime, gyro, processing_quaternion);

    float state_jacobian_quaternion[4][4];
    get_state_jacobian_orientation(gyro, deltaTime, state_jacobian_quaternion);

    float predicted_covar_quaternion[4][4];
    predict_covar_orientation(state_jacobian_quaternion, predicted_covar_quaternion);

    // --- 2. UPDATE (Correction) ---
    float innovation_quaternion[3][1];
    float predicted_accel[3];
    
    // Project current quaternion to expected gravity vector
    predict_accel_from_quat(processing_quaternion, predicted_accel, ekf.expected_g);

    // Normalize accel for innovation
    float a_norm = sqrtf(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
    float a_normalized[3];
    if (a_norm > 0.1f) {
        for(int i=0; i<3; i++) a_normalized[i] = accel[i] / a_norm;
    } else {
        for(int i=0; i<3; i++) a_normalized[i] = ekf.expected_g[i];
    }

    // Calculate Innovation
    for (int i = 0; i < 3; i++) 
        innovation_quaternion[i][0] = a_normalized[i] - predicted_accel[i];

    // H Jacobian
    float h_jacobian_quaternion[3][4];
    get_h_jacobian_quaternion(processing_quaternion, ekf.expected_g, h_jacobian_quaternion);

    float h_jacobian_quaternion_t[4][3];
    transpose3x4_to_4x3(h_jacobian_quaternion, h_jacobian_quaternion_t);

    // Calculate S = H * (P * H') + R  (reuse P*H' for Kalman gain)
    float mat1_q[4][3];
    mat_mul((const float *)predicted_covar_quaternion, (const float *)h_jacobian_quaternion_t, (float *)mat1_q, 4, 4, 3); // P * H'

    float mat3_q[3][3]; // S = H * (P * H')
    mat_mul((const float *)h_jacobian_quaternion, (const float *)mat1_q, (float *)mat3_q, 3, 4, 3);

    // Add Measurement Noise (R)
    for (int i = 0; i < 3; i++) 
        for (int j = 0; j < 3; j++) 
            mat3_q[i][j] += ekf.quaternion.measurement[i][j];

    // Invert S
    float inv_mat3_q[3][3]; 
    if (!inverse(mat3_q, inv_mat3_q)) return; // Failed to invert

    // Calculate Kalman Gain: K = P * H' * S^-1
    float kalman_gain_quaternion[4][3];
    mat_mul((const float *)mat1_q, (const float *)inv_mat3_q, (float *)kalman_gain_quaternion, 4, 3, 3);

    // Update State
    float adjustment_quaternion[4][1];
    mat_mul((const float *)kalman_gain_quaternion, (const float *)innovation_quaternion, (float *)adjustment_quaternion, 4, 3, 1);

    for (int i = 0; i < 4; i++) 
        ekf.quaternion.vals[i] = processing_quaternion[i] + adjustment_quaternion[i][0];
    
    normalize(ekf.quaternion.vals);

    // Update Covariance: P = (I - K * H) * P
    if (ekf.quaternion.index <= UPDATE_COVAR) {
        return;
    }

    ekf.quaternion.index = 0;

    float KH_q[4][4];
    mat_mul((const float *)kalman_gain_quaternion, (const float *)h_jacobian_quaternion, (float *)KH_q, 4, 3, 4);

    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            KH_q[i][j] = (i == j) ? (1.0f - KH_q[i][j]) : (-KH_q[i][j]);

    float new_covar_quaternion[4][4];
    mat_mul((const float *)KH_q, (const float *)predicted_covar_quaternion, (float *)new_covar_quaternion, 4, 4, 4);
    
    memcpy(ekf.quaternion.covar, new_covar_quaternion, sizeof(new_covar_quaternion));
}

/**
 * Body EKF update exploiting H = [I₃, 0₃] sparsity throughout.
 *
 * With H = [I, 0]:
 *   H*P*H' = P[0:3][0:3]  (top-left 3x3 block, zero multiplies)
 *   P*H'   = P[:,0:3]     (first 3 columns of P, zero multiplies)
 *   K      = P[:,0:3] * S⁻¹
 *   P_new  = P - K * P[0:3,:]  (since (I-KH)*P simplifies)
 */
void tick_ekf_body(float deltaTime, float accel[3], float gps_pos[3]) {
    ekf.body.index += 1;

    /* --- 1. PREDICTION --- */
    float processing_position[3];
    float processing_velocity[3];
    state_transition_body(&ekf.body, deltaTime, accel,
                          processing_position, processing_velocity);

    float predicted_covar[6][6];
    predict_covar_body_sparse(deltaTime, predicted_covar);

    /* --- 2. UPDATE (Correction) --- */

    /* Innovation: y = z - H*x_pred = gps - position */
    float innovation[3];
    for (int i = 0; i < 3; i++)
        innovation[i] = gps_pos[i] - processing_position[i];

    /* S = H*P*H' + R = P[0:3][0:3] + R   (3x3, zero multiplies) */
    float S[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            S[i][j] = predicted_covar[i][j] + ekf.body.measurement[i][j];

    float S_inv[3][3];
    if (!inverse(S, S_inv)) return;

    /* K = P*H' * S⁻¹ = P[:,0:3] * S⁻¹   (6x3 * 3x3 = 54 MACs) */
    float K[6][3];
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 3; j++) {
            float sum = 0.0f;
            for (int k = 0; k < 3; k++)
                sum += predicted_covar[i][k] * S_inv[k][j];
            K[i][j] = sum;
        }

    /* State update: x += K * innovation */
    for (int i = 0; i < 3; i++) {
        float adj = K[i][0]*innovation[0] + K[i][1]*innovation[1] + K[i][2]*innovation[2];
        ekf.body.position[i] = processing_position[i] + adj;
    }
    for (int i = 0; i < 3; i++) {
        float adj = K[i+3][0]*innovation[0] + K[i+3][1]*innovation[1] + K[i+3][2]*innovation[2];
        ekf.body.velocity[i] = processing_velocity[i] + adj;
    }

    /* Covariance update: P = (I - K*H) * P = P - K * P[0:3,:]
     * Since H=[I,0], K*H has non-zero only in first 3 columns,
     * so (I-KH)*P[i][j] = P[i][j] - sum_{k=0..2} K[i][k]*P[k][j] */
    if (ekf.body.index <= UPDATE_COVAR) {
        return;
    }
    ekf.body.index = 0;

    float new_covar[6][6];
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++) {
            float kp = K[i][0]*predicted_covar[0][j]
                     + K[i][1]*predicted_covar[1][j]
                     + K[i][2]*predicted_covar[2][j];
            new_covar[i][j] = predicted_covar[i][j] - kp;
        }

    memcpy(ekf.body.covar, new_covar, sizeof(new_covar));
}

// ==========================================
// GETTERS
// ==========================================

void get_state(float quaternion[4], float position[3], float velocity[3])
{
    for (int i = 0; i < 4; i++)
        quaternion[i] = ekf.quaternion.vals[i];

    for (int i = 0; i < 3; i++) {
        position[i] = ekf.body.position[i];
        velocity[i] = ekf.body.velocity[i];
    }
}