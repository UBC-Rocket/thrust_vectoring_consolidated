/**
 * @file    matrix.h
 * @brief   Matrix operations for the ESKF.
 */
#ifndef MATRIX_H
#define MATRIX_H

#include <string.h>

/**
 * @brief Generic matrix multiply: C = A * B.
 * A is (r1 x c1), B is (c1 x c2), C is (r1 x c2).
 * All matrices are flat row-major arrays.
 */
void mat_mul(const float *A, const float *B, float *C, int r1, int c1, int c2);

/**
 * @brief Generic in-place matrix transpose.
 * A is (rows x cols), AT is (cols x rows). Row-major.
 */
void mat_transpose(const float *A, float *AT, int rows, int cols);

/**
 * @brief Normalize a quaternion to unit length.
 */
void normalize(float q[4]);

/**
 * @brief Invert a 3x3 matrix via cofactor/determinant.
 * @return 1 on success, 0 if singular.
 */
int inverse(float a[3][3], float inv[3][3]);

/**
 * @brief Transpose a 3x3 matrix.
 */
void transpose3x3(const float A[3][3], float AT[3][3]);

/**
 * @brief Transpose a 6x6 matrix.
 */
void transpose6x6(const float A[6][6], float AT[6][6]);

/**
 * @brief Transpose a 3x6 to 6x3 matrix.
 */
void transpose3x6_to_6x3(const float A[3][6], float AT[6][3]);

/* ========================================================================
 * Fixed-dimension inline matrix operations.
 *
 * Compile-time-known dimensions let the compiler fully unroll loops,
 * schedule FMA instructions, and eliminate function-call overhead.
 * ====================================================================== */

static inline void mat3_mul(const float A[3][3], const float B[3][3],
                            float C[3][3])
{
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            float s = 0.0f;
            for (int k = 0; k < 3; k++)
                s += A[i][k] * B[k][j];
            C[i][j] = s;
        }
}

static inline void mat3_mul_add(const float A[3][3], const float B[3][3],
                                float C[3][3])
{
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            for (int k = 0; k < 3; k++)
                C[i][j] += A[i][k] * B[k][j];
}

static inline void mat3_transpose(const float A[3][3], float AT[3][3])
{
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            AT[j][i] = A[i][j];
}

/** C = A * B^T  (avoids forming the explicit transpose) */
static inline void mat3_mul_BT(const float A[3][3], const float B[3][3],
                                float C[3][3])
{
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            float s = 0.0f;
            for (int k = 0; k < 3; k++)
                s += A[i][k] * B[j][k];  /* B transposed: row j = column j */
            C[i][j] = s;
        }
}

/** C += A * B^T */
static inline void mat3_mul_BT_add(const float A[3][3], const float B[3][3],
                                    float C[3][3])
{
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            for (int k = 0; k < 3; k++)
                C[i][j] += A[i][k] * B[j][k];
}

static inline void mat3_copy(const float src[3][3], float dst[3][3])
{
    memcpy(dst, src, sizeof(float) * 9);
}

static inline void mat3_zero(float M[3][3])
{
    memset(M, 0, sizeof(float) * 9);
}

static inline void mat3_add_scaled(float dst[3][3], const float src[3][3],
                                   float scale)
{
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            dst[i][j] += src[i][j] * scale;
}

/* ========================================================================
 * NxN and Nx3 operations sized to ESKF_ERR_DIM.
 *
 * These require ekf.h to be included first (for ESKF_ERR_DIM).
 * Providing the dimension as a compile-time constant lets the compiler
 * unroll and schedule optimally.
 * ====================================================================== */

#ifdef ESKF_ERR_DIM

#define _N ESKF_ERR_DIM

static inline void matN_transpose_3xN(const float A[3][_N], float AT[_N][3])
{
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < _N; j++)
            AT[j][i] = A[i][j];
}

/** C(NxN) = A(NxN) * B(NxN) */
static inline void matN_mul_NxN(const float *A, const float *B, float *C)
{
    for (int i = 0; i < _N; i++)
        for (int j = 0; j < _N; j++) {
            float s = 0.0f;
            for (int k = 0; k < _N; k++)
                s += A[i * _N + k] * B[k * _N + j];
            C[i * _N + j] = s;
        }
}

/** C(Nx3) = A(NxN) * B(Nx3) */
static inline void matN_mul_Nx3(const float A[_N][_N], const float B[_N][3],
                                float C[_N][3])
{
    for (int i = 0; i < _N; i++)
        for (int j = 0; j < 3; j++) {
            float s = 0.0f;
            for (int k = 0; k < _N; k++)
                s += A[i][k] * B[k][j];
            C[i][j] = s;
        }
}

/** C(3x3) = A(3xN) * B(Nx3) */
static inline void matN_mul_3x3(const float A[3][_N], const float B[_N][3],
                                float C[3][3])
{
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            float s = 0.0f;
            for (int k = 0; k < _N; k++)
                s += A[i][k] * B[k][j];
            C[i][j] = s;
        }
}

/** C(Nx3) = A(Nx3) * B(3x3) */
static inline void matN_mul_Nx3_by_3x3(const float A[_N][3],
                                        const float B[3][3],
                                        float C[_N][3])
{
    for (int i = 0; i < _N; i++)
        for (int j = 0; j < 3; j++) {
            float s = 0.0f;
            for (int k = 0; k < 3; k++)
                s += A[i][k] * B[k][j];
            C[i][j] = s;
        }
}

/** C(NxN) = A(Nx3) * B(3xN) */
static inline void matN_mul_Nx3_by_3xN(const float A[_N][3],
                                        const float B[3][_N],
                                        float C[_N][_N])
{
    for (int i = 0; i < _N; i++)
        for (int j = 0; j < _N; j++) {
            float s = 0.0f;
            for (int k = 0; k < 3; k++)
                s += A[i][k] * B[k][j];
            C[i][j] = s;
        }
}

#undef _N

#endif /* ESKF_ERR_DIM */

#endif /* MATRIX_H */
