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
void test_quaternion_from_rotation_vector_zero(void);
void test_quaternion_from_rotation_vector_90deg_z(void);
void test_eskf_propagate_nominal_zero_gyro(void);
void test_skew_symmetric_3x3(void);

/* test_matrix.c */
void test_inverse_identity(void);
void test_inverse_known_matrix(void);
void test_inverse_singular_returns_zero(void);
void test_transpose3x3(void);
void test_mat_transpose_3x4(void);
void test_matrix33_vec3_mul(void);

/* test_ekf.c */
void test_eskf_init_gives_identity_quaternion(void);
void test_eskf_orientation_stable_at_rest(void);
void test_eskf_body_zero_accel_gps_origin(void);
void test_eskf_calibration_seeds_biases(void);

/* test_eskf_trajectory.c */
void test_trajectory_static(void);
void test_trajectory_pitch_maneuver(void);
void test_trajectory_vertical_ascent(void);
void test_trajectory_dual_imu_static(void);
void test_trajectory_dual_baro_ascent(void);

int main(void)
{
    UNITY_BEGIN();

    /* quaternion / state math */
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
    RUN_TEST(test_quaternion_from_rotation_vector_zero);
    RUN_TEST(test_quaternion_from_rotation_vector_90deg_z);
    RUN_TEST(test_eskf_propagate_nominal_zero_gyro);
    RUN_TEST(test_skew_symmetric_3x3);

    /* matrix */
    RUN_TEST(test_inverse_identity);
    RUN_TEST(test_inverse_known_matrix);
    RUN_TEST(test_inverse_singular_returns_zero);
    RUN_TEST(test_transpose3x3);
    RUN_TEST(test_mat_transpose_3x4);
    RUN_TEST(test_matrix33_vec3_mul);

    /* eskf */
    RUN_TEST(test_eskf_init_gives_identity_quaternion);
    RUN_TEST(test_eskf_orientation_stable_at_rest);
    RUN_TEST(test_eskf_body_zero_accel_gps_origin);
    RUN_TEST(test_eskf_calibration_seeds_biases);

    /* trajectory integration tests */
    RUN_TEST(test_trajectory_static);
    RUN_TEST(test_trajectory_pitch_maneuver);
    RUN_TEST(test_trajectory_vertical_ascent);
    RUN_TEST(test_trajectory_dual_imu_static);
    RUN_TEST(test_trajectory_dual_baro_ascent);

    return UNITY_END();
}
