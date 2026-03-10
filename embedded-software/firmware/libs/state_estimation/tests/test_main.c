#include "unity.h"

void setUp(void) {}
void tearDown(void) {}

/* test_quaternion.c */
void test_quaternion_conjugate_negates_vector(void);
void test_quaternion_multiply_identity(void);
void test_quaternion_multiply_inverse_gives_identity(void);
void test_identity_quaternion_gives_identity_matrix(void);
void test_90deg_z_rotation_matrix(void);
void test_rotation_matrix_vector_mul_identity(void);
void test_vec3_cross_basic(void);
void test_vec3_cross_anticommutative(void);
void test_vec3_dot(void);
void test_normalize_unit_length(void);
void test_quat_to_euler_identity(void);
void test_state_transition_orientation_zero_gyro(void);

/* test_matrix.c */
void test_inverse_identity(void);
void test_inverse_known_matrix(void);
void test_inverse_singular_returns_zero(void);
void test_transpose3x3(void);
void test_transpose4x4(void);
void test_matrix33_vec3_mul(void);
void test_mat_mul_identity(void);

/* test_ekf.c */
void test_ekf_init_gives_identity_quaternion(void);
void test_ekf_orientation_tick_stable_at_rest(void);
void test_ekf_body_tick_zero_accel(void);

int main(void)
{
    UNITY_BEGIN();

    /* quaternion */
    RUN_TEST(test_quaternion_conjugate_negates_vector);
    RUN_TEST(test_quaternion_multiply_identity);
    RUN_TEST(test_quaternion_multiply_inverse_gives_identity);
    RUN_TEST(test_identity_quaternion_gives_identity_matrix);
    RUN_TEST(test_90deg_z_rotation_matrix);
    RUN_TEST(test_rotation_matrix_vector_mul_identity);
    RUN_TEST(test_vec3_cross_basic);
    RUN_TEST(test_vec3_cross_anticommutative);
    RUN_TEST(test_vec3_dot);
    RUN_TEST(test_normalize_unit_length);
    RUN_TEST(test_quat_to_euler_identity);
    RUN_TEST(test_state_transition_orientation_zero_gyro);

    /* matrix */
    RUN_TEST(test_inverse_identity);
    RUN_TEST(test_inverse_known_matrix);
    RUN_TEST(test_inverse_singular_returns_zero);
    RUN_TEST(test_transpose3x3);
    RUN_TEST(test_transpose4x4);
    RUN_TEST(test_matrix33_vec3_mul);
    RUN_TEST(test_mat_mul_identity);

    /* ekf */
    RUN_TEST(test_ekf_init_gives_identity_quaternion);
    RUN_TEST(test_ekf_orientation_tick_stable_at_rest);
    RUN_TEST(test_ekf_body_tick_zero_accel);

    return UNITY_END();
}
