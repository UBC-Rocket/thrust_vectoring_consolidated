/**
 * @file    matrix.h
 * @brief   Matrix operations for the ESKF.
 */
#ifndef MATRIX_H
#define MATRIX_H

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

#endif /* MATRIX_H */
