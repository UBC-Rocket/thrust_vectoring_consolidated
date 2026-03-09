#ifndef MATRIX_H
#define MATRIX_H

/**
 * @brief Generic matrix multiply: C = A * B.
 * A is (r1 x c1), B is (c1 x c2), C is (r1 x c2).
 * All matrices are flat row-major arrays.
 */
void mat_mul(const float *A, const float *B, float *C, int r1, int c1, int c2);

void normalize(float q[4]);

int inverse(float a[3][3], float inv[3][3]);

void transpose3x4_to_4x3(const float A[3][4], float AT[4][3]);

void transpose4x4(const float A[4][4], float AT[4][4]);

void transpose6x6(const float A[6][6], float AT[6][6]);

void transpose3x3(const float A[3][3], float AT[3][3]);

void transpose3x6_to_6x3(const float A[3][6], float AT[6][3]);

#endif