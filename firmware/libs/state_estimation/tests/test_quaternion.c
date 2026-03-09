#include "unity.h"
#include "state_estimation/state.h"
#include "state_estimation/ekf.h"
#include "state_estimation/quaternion.h"
#include "state_estimation/matrix.h"
#include <math.h>

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
    /* q * conj(q) should give identity for unit quaternion */
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
    /* 90 degrees around z: q = cos(45) + sin(45)*k */
    float s = sinf((float)M_PI / 4.0f);
    float c = cosf((float)M_PI / 4.0f);
    quaternion_t q = {c, 0, 0, s};
    rotation_matrix_t R;
    quaternion_to_rotation_matrix(&q, &R);
    /* R should be [[0,-1,0],[1,0,0],[0,0,1]] */
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
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, e[0]); /* roll */
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, e[1]); /* pitch */
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, e[2]); /* yaw */
}

/* ---------- state_transition_orientation ---------- */

void test_state_transition_orientation_zero_gyro(void)
{
    /* With zero gyro, quaternion should remain identity */
    quaternion_state qs;
    qs.vals[0] = 1; qs.vals[1] = 0; qs.vals[2] = 0; qs.vals[3] = 0;
    float gyro[3] = {0, 0, 0};
    float out[4];
    state_transition_orientation(&qs, 0.01f, gyro, out);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 1.0f, out[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out[2]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out[3]);
}
