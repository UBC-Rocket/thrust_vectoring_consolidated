#include "state_estimation/matrix.h"
#include <math.h>

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
