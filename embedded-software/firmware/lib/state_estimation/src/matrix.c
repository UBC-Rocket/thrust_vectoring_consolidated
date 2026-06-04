#include "state_estimation/matrix.h"
#include <math.h>

/* CMSIS-DSP is only linked in for the firmware (cross-compile) builds — see
 * state_estimation/CMakeLists.txt. On host (state_estimation_tests), the
 * naive impls below stay in play; the firmware path delegates the two
 * functions that actually move the EKF needle (mat_mul + mat_transpose on
 * the 15×15 covariance) to arm_mat_mult_f32 / arm_mat_trans_f32.
 *
 * Skipped from the CMSIS-DSP swap:
 *   - inverse() — 3×3 closed-form cofactor expansion beats CMSIS-DSP's
 *     general Gauss-Jordan at this size, and arm_mat_inverse_f32 clobbers
 *     its input (we'd have to copy first).
 *   - transpose3x3 / 6x6 / 3x6_to_6x3 — function-call overhead dominates
 *     at these sizes; the compiler unrolls the inline loops fine.
 *   - normalize — 4 mults + sqrt, nothing to gain.
 */
#ifdef STATE_EST_USE_CMSIS_DSP
#include "arm_math.h"

void mat_mul(const float *A, const float *B, float *C, int r1, int c1, int c2)
{
    /* arm_matrix_instance_f32 holds pData as non-const, but the routine
     * only reads pSrcA/pSrcB. Casting away const is the documented usage. */
    const arm_matrix_instance_f32 mA = { (uint16_t)r1, (uint16_t)c1, (float *)A };
    const arm_matrix_instance_f32 mB = { (uint16_t)c1, (uint16_t)c2, (float *)B };
    arm_matrix_instance_f32       mC = { (uint16_t)r1, (uint16_t)c2, C };
    (void)arm_mat_mult_f32(&mA, &mB, &mC);
}

void mat_transpose(const float *A, float *AT, int rows, int cols)
{
    const arm_matrix_instance_f32 mA  = { (uint16_t)rows, (uint16_t)cols, (float *)A };
    arm_matrix_instance_f32       mAT = { (uint16_t)cols, (uint16_t)rows, AT };
    (void)arm_mat_trans_f32(&mA, &mAT);
}

#else  /* host build — naive impls */

void mat_mul(const float *A, const float *B, float *C, int r1, int c1, int c2)
{
    for (int i = 0; i < r1; ++i) {
        for (int j = 0; j < c2; ++j) {
            float sum = 0.0f;
            for (int k = 0; k < c1; ++k) {
                sum += A[i * c1 + k] * B[k * c2 + j];
            }
            C[i * c2 + j] = sum;
        }
    }
}

void mat_transpose(const float *A, float *AT, int rows, int cols)
{
    for (int i = 0; i < rows; ++i)
        for (int j = 0; j < cols; ++j)
            AT[j * rows + i] = A[i * cols + j];
}

#endif  /* STATE_EST_USE_CMSIS_DSP */

void normalize(float q[4])
{
    float norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (norm > 1e-10f) {
        for (int i = 0; i < 4; i++) q[i] /= norm;
    }
}

int inverse(float a[3][3], float inv[3][3])
{
    float c00 = a[1][1]*a[2][2] - a[1][2]*a[2][1];
    float c01 = a[1][2]*a[2][0] - a[1][0]*a[2][2];
    float c02 = a[1][0]*a[2][1] - a[1][1]*a[2][0];

    float det = a[0][0]*c00 + a[0][1]*c01 + a[0][2]*c02;
    if (fabsf(det) < 1e-10f) return 0;

    float inv_det = 1.0f / det;

    inv[0][0] = c00 * inv_det;
    inv[0][1] = (a[0][2]*a[2][1] - a[0][1]*a[2][2]) * inv_det;
    inv[0][2] = (a[0][1]*a[1][2] - a[0][2]*a[1][1]) * inv_det;
    inv[1][0] = c01 * inv_det;
    inv[1][1] = (a[0][0]*a[2][2] - a[0][2]*a[2][0]) * inv_det;
    inv[1][2] = (a[0][2]*a[1][0] - a[0][0]*a[1][2]) * inv_det;
    inv[2][0] = c02 * inv_det;
    inv[2][1] = (a[0][1]*a[2][0] - a[0][0]*a[2][1]) * inv_det;
    inv[2][2] = (a[0][0]*a[1][1] - a[0][1]*a[1][0]) * inv_det;

    return 1;
}

void transpose3x3(const float A[3][3], float AT[3][3])
{
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            AT[j][i] = A[i][j];
}

void transpose6x6(const float A[6][6], float AT[6][6])
{
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j)
            AT[j][i] = A[i][j];
}

void transpose3x6_to_6x3(const float A[3][6], float AT[6][3])
{
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 6; ++j)
            AT[j][i] = A[i][j];
}
