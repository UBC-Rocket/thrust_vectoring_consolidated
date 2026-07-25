#include "unity.h"
#include "state_estimation/matrix.h"
#include "state_estimation/state.h"
#include <math.h>

#define TOL 1e-5f

/* ---------- inverse ---------- */

void test_inverse_identity(void)
{
    float I[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
    float inv[3][3];
    int ok = inverse(I, inv);
    TEST_ASSERT_EQUAL_INT(1, ok);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            TEST_ASSERT_FLOAT_WITHIN(TOL, (i == j) ? 1.0f : 0.0f, inv[i][j]);
}

void test_inverse_known_matrix(void)
{
    float A[3][3] = {{2, 1, 0}, {0, 3, 0}, {0, 0, 4}};
    float inv[3][3];
    int ok = inverse(A, inv);
    TEST_ASSERT_EQUAL_INT(1, ok);

    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            float sum = 0;
            for (int k = 0; k < 3; k++)
                sum += A[i][k] * inv[k][j];
            TEST_ASSERT_FLOAT_WITHIN(TOL, (i == j) ? 1.0f : 0.0f, sum);
        }
}

void test_inverse_singular_returns_zero(void)
{
    float S[3][3] = {{1,2,3},{4,5,6},{7,8,9}};
    float inv[3][3];
    int ok = inverse(S, inv);
    TEST_ASSERT_EQUAL_INT(0, ok);
}

/* ---------- transpose3x3 ---------- */

void test_transpose3x3(void)
{
    float A[3][3] = {{1,2,3},{4,5,6},{7,8,9}};
    float AT[3][3];
    transpose3x3(A, AT);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            TEST_ASSERT_FLOAT_WITHIN(TOL, A[j][i], AT[i][j]);
}

/* ---------- mat_transpose (generic) ---------- */

void test_mat_transpose_3x4(void)
{
    float A[12]; /* 3x4 */
    float AT[12]; /* 4x3 */
    for (int i = 0; i < 12; i++) A[i] = (float)i;
    mat_transpose(A, AT, 3, 4);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 4; j++)
            TEST_ASSERT_FLOAT_WITHIN(TOL, A[i * 4 + j], AT[j * 3 + i]);
}

/* ---------- matrix33_vec3_mul ---------- */

void test_matrix33_vec3_mul(void)
{
    float M[3][3] = {{1,0,0},{0,2,0},{0,0,3}};
    float v[3] = {1, 2, 3};
    float out[3];
    matrix33_vec3_mul(M, v, out);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 1.0f, out[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 4.0f, out[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 9.0f, out[2]);
}

/* ---------- mat_mul ---------- */

void test_mat_mul_identity(void)
{
    float I[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
    float A[3][3] = {{1,2,3},{4,5,6},{7,8,9}};
    float C[3][3];
    mat_mul((const float *)I, (const float *)A, (float *)C, 3, 3, 3);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            TEST_ASSERT_FLOAT_WITHIN(TOL, A[i][j], C[i][j]);
}
