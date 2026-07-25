#include "unity.h"
#include "state_estimation/state.h"
#include "state_estimation/ekf.h"
#include "state_estimation/quaternion.h"
#include "state_estimation/matrix.h"
#include <math.h>
#include <string.h>

#define TOL 1e-5f

/* ---------- quaternion_conjugate ---------- */

void test_quaternion_conjugate_negates_vector(void)
{
    quaternion_t q = {1.0f, 2.0f, 3.0f, 4.0f};
    quaternion_t out;
    quaternion_conjugate(&q, &out);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 1.0f, out.w);
    TEST_ASSERT_FLOAT_WITHIN(TOL, -2.0f, out.x);
    TEST_ASSERT_FLOAT_WITHIN(TOL, -3.0f, out.y);
    TEST_ASSERT_FLOAT_WITHIN(TOL, -4.0f, out.z);
}

/* ---------- quaternion_multiply ---------- */

void test_quaternion_multiply_identity(void)
{
    quaternion_t id = {1, 0, 0, 0};
    quaternion_t q  = {0.5f, 0.5f, 0.5f, 0.5f};
    quaternion_t out;
    quaternion_multiply(&id, &q, &out);
    TEST_ASSERT_FLOAT_WITHIN(TOL, q.w, out.w);
    TEST_ASSERT_FLOAT_WITHIN(TOL, q.x, out.x);
    TEST_ASSERT_FLOAT_WITHIN(TOL, q.y, out.y);
    TEST_ASSERT_FLOAT_WITHIN(TOL, q.z, out.z);
}

void test_quaternion_multiply_inverse_gives_identity(void)
{
    quaternion_t q = {0.5f, 0.5f, 0.5f, 0.5f};
    quaternion_t qc, out;
    quaternion_conjugate(&q, &qc);
    quaternion_multiply(&q, &qc, &out);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 1.0f, out.w);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.x);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.y);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.z);
}

/* ---------- quaternion_to_rotation_matrix ---------- */

void test_identity_quaternion_gives_identity_matrix(void)
{
    quaternion_t id = {1, 0, 0, 0};
    rotation_matrix_t R;
    quaternion_to_rotation_matrix(&id, &R);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            TEST_ASSERT_FLOAT_WITHIN(TOL, (i == j) ? 1.0f : 0.0f, R.R[i][j]);
}

void test_90deg_z_rotation_matrix(void)
{
    float s = sinf((float)M_PI / 4.0f);
    float c = cosf((float)M_PI / 4.0f);
    quaternion_t q = {c, 0, 0, s};
    rotation_matrix_t R;
    quaternion_to_rotation_matrix(&q, &R);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, R.R[0][0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, -1.0f, R.R[0][1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 1.0f, R.R[1][0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, R.R[1][1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 1.0f, R.R[2][2]);
}

/* ---------- rotation_matrix_vector_mul ---------- */

void test_rotation_matrix_vector_mul_identity(void)
{
    rotation_matrix_t R = {{{1,0,0},{0,1,0},{0,0,1}}};
    float v[3] = {1, 2, 3};
    float out[3];
    rotation_matrix_vector_mul(&R, v, out);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 1.0f, out[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 2.0f, out[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 3.0f, out[2]);
}

/* ---------- vec3_cross ---------- */

void test_vec3_cross_basic(void)
{
    float a[3] = {1, 0, 0};
    float b[3] = {0, 1, 0};
    float out[3];
    vec3_cross(a, b, out);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 1.0f, out[2]);
}

void test_vec3_cross_anticommutative(void)
{
    float a[3] = {1, 2, 3};
    float b[3] = {4, 5, 6};
    float ab[3], ba[3];
    vec3_cross(a, b, ab);
    vec3_cross(b, a, ba);
    for (int i = 0; i < 3; i++)
        TEST_ASSERT_FLOAT_WITHIN(TOL, -ab[i], ba[i]);
}

/* ---------- vec3_dot ---------- */

void test_vec3_dot(void)
{
    float a[3] = {1, 2, 3};
    float b[3] = {4, 5, 6};
    TEST_ASSERT_FLOAT_WITHIN(TOL, 32.0f, vec3_dot(a, b));
}

/* ---------- normalize ---------- */

void test_normalize_unit_length(void)
{
    float q[4] = {1, 1, 1, 1};
    normalize(q);
    float norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 1.0f, norm);
    float expected = 0.5f;
    for (int i = 0; i < 4; i++)
        TEST_ASSERT_FLOAT_WITHIN(TOL, expected, q[i]);
}

/* ---------- quat_to_euler ---------- */

void test_quat_to_euler_identity(void)
{
    float q[4] = {1, 0, 0, 0};
    float e[3];
    quat_to_euler(q, e);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, e[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, e[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, e[2]);
}

/* ---------- quaternion_from_rotation_vector ---------- */

void test_quaternion_from_rotation_vector_zero(void)
{
    float rv[3] = {0, 0, 0};
    quaternion_t out;
    quaternion_from_rotation_vector(rv, &out);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 1.0f, out.w);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.x);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.y);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.z);
}

void test_quaternion_from_rotation_vector_90deg_z(void)
{
    float angle = (float)M_PI / 2.0f;
    float rv[3] = {0, 0, angle};
    quaternion_t out;
    quaternion_from_rotation_vector(rv, &out);
    float expected_w = cosf(angle / 2.0f);
    float expected_z = sinf(angle / 2.0f);
    TEST_ASSERT_FLOAT_WITHIN(TOL, expected_w, out.w);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.x);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.y);
    TEST_ASSERT_FLOAT_WITHIN(TOL, expected_z, out.z);
}

/* ---------- eskf_propagate_nominal ---------- */

void test_eskf_propagate_nominal_zero_gyro(void)
{
    orientation_eskf_state_t state;
    memset(&state, 0, sizeof(state));
    state.q_nom[0] = 1.0f; /* identity */

    float gyro[3] = {0, 0, 0};
    eskf_propagate_nominal(&state, gyro, 0.01f);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 1.0f, state.q_nom[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, state.q_nom[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, state.q_nom[2]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, state.q_nom[3]);
}

/* ---------- skew_symmetric_3x3 ---------- */

void test_skew_symmetric_3x3(void)
{
    float v[3] = {1, 2, 3};
    float S[3][3];
    skew_symmetric_3x3(v, S);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, S[0][0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, -3.0f, S[0][1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 2.0f, S[0][2]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 3.0f, S[1][0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, S[1][1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, -1.0f, S[1][2]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, -2.0f, S[2][0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 1.0f, S[2][1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, S[2][2]);
}
