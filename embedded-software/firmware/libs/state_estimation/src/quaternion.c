#include "state_estimation/state.h"
#include "state_estimation/ekf.h"
#include "state_estimation/matrix.h"
#include <math.h>
#include <string.h>

void state_transition_orientation(
    quaternion_state *state,
    float time_step,
    float g[3], // gyro data
    float out_q[4]
)
{
    float P[4][1];
    for (int i = 0; i < 4; i++) P[i][0] = state->vals[i];

    /*
    | 0   -gx   -gy  -gz  |
    | gx   0     gz  -gy  |
    | gy  -gz    0    gx  |
    | gz   gy   -gx   0   |
    */

    float B[4][4] = {{0, -g[0], -g[1], -g[2]},
                    {g[0], 0, g[2], -g[1]},
                    {g[1], -g[2], 0, g[0]},
                    {g[2], g[1], -g[0], 0}};

    float C[4][1];

    MAT_MUL(B, P, C, 4, 4, 1);

    for (int i = 0; i < 4; i++) C[i][0] = C[i][0] * (time_step * 0.5f);

    for (int i = 0; i < 4; i++) {
        out_q[i] = state->vals[i] + C[i][0];
    }

    normalize(out_q);
}

void get_state_jacobian_orientation(
    float g[3],
    float dT,
    float j[4][4]
)
{
    /*
    |  1    -gxdt   -gydt   -gzdt |
    | gxdt    1      gzdt   -gydt |
    | gydt  -gzdt     1      gxdt |
    | gzdt   gydt   -gxdt     1   |
    */
    float xdt = 0.5f * g[0] * dT;
    float ydt = 0.5f * g[1] * dT;
    float zdt = 0.5f * g[2] * dT;

    memcpy(j,
        (float [4][4]){
            {1,   -xdt, -ydt, -zdt},
            {xdt,  1,    zdt, -ydt},
            {ydt, -zdt,  1,    xdt},
            {zdt,  ydt, -xdt,  1  }
        },
        sizeof(float[4][4]));

    // j[0][0]=1;   j[0][1]=-xdt; j[0][2]=-ydt; j[0][3]=-zdt;
    // j[1][0]=xdt; j[1][1]=1;    j[1][2]=zdt;  j[1][3]=-ydt;
    // j[2][0]=ydt; j[2][1]=-zdt; j[2][2]=1;    j[2][3]=xdt;
    // j[3][0]=zdt; j[3][1]=ydt;  j[3][2]=-xdt; j[3][3]=1;

}

void predict_accel_from_quat(const float q[4], float accel_pred[3], float expected_g[3])
{
    rotation_matrix_t r;
    quaternion_t quat;
    quat.w = q[0]; quat.x = q[1]; quat.y = q[2]; quat.z = q[3];

    quaternion_to_rotation_matrix(&quat, &r);

    for (int i = 0; i < 3; i++) accel_pred[i] = 0;

    for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) accel_pred[i] += r.R[j][i] * expected_g[j];
}

void get_h_jacobian_quaternion(float q[4], float eg[3], float H[3][4]) {
    float w = q[0];
    float x = q[1];
    float y = q[2];
    float z = q[3];
    
    float gx = eg[0];
    float gy = eg[1];
    float gz = eg[2];

    // Derivatives of R^T * g
    // Where R^T is the rotation from World to Body
    
    // Row 0 (Gradient of a_x)
    H[0][0] = -2.0f * y;
    H[0][1] =  2.0f * z;
    H[0][2] = -2.0f * w;
    H[0][3] =  2.0f * x;

    // Row 1 (ay)
    H[1][0] =  2.0f * x;
    H[1][1] =  2.0f * w;
    H[1][2] =  2.0f * z;
    H[1][3] =  2.0f * y;

    // Row 2 (az)
    H[2][0] =  2.0f * w;
    H[2][1] = -2.0f * x;
    H[2][2] = -2.0f * y;
    H[2][3] =  2.0f * z;

    // for(int i=0; i<3; i++) {
    //     for(int j=0; j<4; j++) {
    //         H[i][j] = -H[i][j];
    //     }
    // }
}

void quat_to_euler(float q[4], float e[3]) {
    float w = q[0], x = q[1], y = q[2], z = q[3];

    float sinr = 2.0f * (w*x + y*z);
    float cosr = 1.0f - 2.0f * (x*x + y*y);
    e[0] = atan2f(sinr, cosr) * 180.0f / (float)M_PI;

    float sinp = 2.0f * (w*y - z*x);
    if (fabsf(sinp) >= 1.0f)
        e[1] = copysignf((float)M_PI / 2.0f, sinp) * 180.0f / (float)M_PI;
    else
        e[1] = asinf(sinp) * 180.0f / (float)M_PI;

    float siny = 2.0f * (w*z + x*y);
    float cosy = 1.0f - 2.0f * (y*y + z*z);
    e[2] = atan2f(siny, cosy) * 180.0f / (float)M_PI;
}

/**
 * Quaternions in format [w, x, y, z]
 * out = q1 * q2
 */
void quat_mult(const float q1[4], const float q2[4], float out[4])
{
    float w1 = q1[0], x1 = q1[1], y1 = q1[2], z1 = q1[3];
    float w2 = q2[0], x2 = q2[1], y2 = q2[2], z2 = q2[3];

    out[0] = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2; // w
    out[1] = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2; // x
    out[2] = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2; // y
    out[3] = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2; // z
}