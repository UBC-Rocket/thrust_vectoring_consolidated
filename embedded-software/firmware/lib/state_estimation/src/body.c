/**
 * @file    body.c
 * @brief   Body-frame state transitions and measurement models (position/velocity).
 */
#include "state_estimation/body.h"
#include "state_estimation/state.h"
#include "state_estimation/matrix.h"
#include <math.h>
#include <string.h>

#define GRAV_MPS2 9.807f

void transform_accel_data(const float a_body[3], const float q[4],
                          float a_nav[3])
{
    quaternion_t qt;
    qt.w = q[0]; qt.x = q[1]; qt.y = q[2]; qt.z = q[3];

    rotation_matrix_t R;
    quaternion_to_rotation_matrix(&qt, &R);

    /* Convert body-frame specific force [g] to [m/s²] and rotate to nav */
    float a_body_ms2[3] = { a_body[0] * GRAV_MPS2,
                            a_body[1] * GRAV_MPS2,
                            a_body[2] * GRAV_MPS2 };

    rotation_matrix_vector_mul(&R, a_body_ms2, a_nav);

    /* Subtract gravity (z-up convention: g_nav = [0, 0, +GRAV]) */
    a_nav[2] -= GRAV_MPS2;
}

void state_transition_body(const body_state_t *state, float dt,
                           const float a_nav[3],
                           float out_p[3], float out_v[3])
{
    for (int i = 0; i < 3; i++) {
        out_p[i] = state->position[i]
                 + dt * state->velocity[i]
                 + 0.5f * dt * dt * a_nav[i];
        out_v[i] = state->velocity[i] + dt * a_nav[i];
    }
}

void get_state_jacobian_body(float dt, float F[6][6])
{
    memset(F, 0, sizeof(float) * 6 * 6);
    for (int i = 0; i < 6; i++) F[i][i] = 1.0f;
    F[0][3] = dt;
    F[1][4] = dt;
    F[2][5] = dt;
}

void predict_covar_body_sparse(const float P[6][6], const float Q[6][6],
                               float dt, float P_pred[6][6])
{
    const float dt2 = dt * dt;

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            float Ppp = P[i][j];
            float Ppv = P[i][j + 3];
            float Pvp = P[i + 3][j];
            float Pvv = P[i + 3][j + 3];

            P_pred[i][j]         = Ppp + dt * (Ppv + Pvp) + dt2 * Pvv + Q[i][j] * dt;
            P_pred[i][j + 3]     = Ppv + dt * Pvv + Q[i][j + 3] * dt;
            P_pred[i + 3][j]     = Pvp + dt * Pvv + Q[i + 3][j] * dt;
            P_pred[i + 3][j + 3] = Pvv + Q[i + 3][j + 3] * dt;
        }
    }
}

float pressure_to_height(int32_t pressure_centi)
{
    return 44307.69f * (1.0f - powf((float)pressure_centi / 101325.0f, 0.190284f));
}
