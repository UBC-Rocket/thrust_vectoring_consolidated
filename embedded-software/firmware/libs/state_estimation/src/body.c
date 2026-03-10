#include <math.h>
#include "state_estimation/matrix.h"
#include "state_estimation/ekf.h"
#include <string.h>
#include "state_estimation/quaternion.h"
#include "state_estimation/state.h"

#define GRAV_MPS2 9.807f

/**
 * Rotate body-frame acceleration to nav frame and subtract gravity.
 * a is in g (specific force); new_a is linear acceleration in nav frame [m/s^2].
 */
void transform_accel_data(
    float a[3], // body-frame accel in g
    float q[4], // orientation in quat (body-to-nav, w,x,y,z)
    float new_a[3]
)
{
    quaternion_t quat;
    quat.w = q[0];
    quat.x = q[1];
    quat.y = q[2];
    quat.z = q[3];
    rotation_matrix_t R;
    quaternion_to_rotation_matrix(&quat, &R);

    float a_body_ms2[3] = { a[0] * GRAV_MPS2, a[1] * GRAV_MPS2, a[2] * GRAV_MPS2 };
    rotation_matrix_vector_mul(&R, a_body_ms2, new_a);
    /* Linear accel in nav: a_linear = f_nav - g_nav. Z-up => g_nav = [0,0,-GRAV]. With f = a_data*GRAV (at rest f_nav = [0,0,+GRAV]), subtract [0,0,GRAV] to get a_linear = 0. */
    new_a[0] += 0.0f;
    new_a[1] += 0.0f;
    new_a[2] -= GRAV_MPS2;
}

void state_transition_body(
    body_state *state,
    float time_step,
    float a[3], // accel data
    float out_p[3],
    float out_v[3]
)
{
    for (int i = 0; i < 3; i++) {
        out_p[i] = state->position[i] + time_step * state->velocity[i]
                              + 0.5f * time_step * time_step * a[i];
    }

    for (int i = 0; i < 3; i++) {
        out_v[i] = state->velocity[i] + time_step * a[i];
    }
}


void get_state_jacobian_body(
    float dT,
    float j[6][6]
)
{
    memcpy(j,
        (float[6][6]) {
            {1, 0, 0, dT, 0, 0},
            {0, 1, 0, 0, dT, 0},
            {0, 0, 1, 0, 0, dT},
            {0, 0, 0, 1, 0, 0},
            {0, 0, 0, 0,1, 0},
            {0, 0, 0, 0, 0, 1}
        },
        (sizeof (float[6][6])));
}

void get_h_jacobian_body(
    float h_jacobian[3][6]
)
{
    memcpy(h_jacobian,
    (float[3][6]) {
        {1, 0, 0, 0, 0, 0},
        {0, 1, 0, 0, 0, 0},
        {0, 0, 1, 0, 0, 0},
    },
    (sizeof (float[3][6])));
}

float pressure_to_height(uint32_t pressure_centi) {
    return 44307.69f * (1.0f - powf((float)pressure_centi / 101325.0f, 0.190284f));
}