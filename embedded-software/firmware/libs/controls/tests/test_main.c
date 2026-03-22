#include "unity.h"

void setUp(void) {}
void tearDown(void) {}

/* test_pid.c */
void test_pid_init_zeros_state(void);
void test_pid_proportional_only(void);
void test_pid_integral_accumulates(void);
void test_pid_anti_windup(void);
void test_pid_output_clamping(void);
void test_pid_reset_clears_state(void);
void test_pid_set_gains(void);
void test_pid_zero_error(void);

/* test_flight_controller.c — Eq 1 */
void test_eq1_identity_des_90z_meas(void);
void test_eq1_90x_des_identity_meas(void);
void test_eq1_zero_error(void);
void test_eq1_90deg_pitch(void);
void test_eq1_opposite_rolls(void);
/* Eq 2 */
void test_eq2_90deg_about_x(void);
void test_eq2_mixed_xy(void);
void test_eq2_identity(void);
/* Eq 3 */
void test_eq3_simple_cross(void);
void test_eq3_general(void);
void test_eq3_zero_torque(void);
void test_eq3_pitch_torque(void);
void test_eq3_yaw_torque(void);
/* Eq 4 */
void test_eq4_along_z(void);
void test_eq4_along_y(void);
void test_eq4_purely_vertical(void);
void test_eq4_345_triangle(void);
void test_eq4_51213_triangle(void);
/* Eq 5 */
void test_eq5_case1(void);
void test_eq5_case2(void);
void test_eq5_vertical(void);
void test_eq5_pitch_correction(void);
void test_eq5_yaw_correction(void);
/* Eq 6-7 */
void test_eq6_zero_angle(void);
void test_eq6_positive_theta_x(void);
void test_eq7_zero_angle(void);
void test_eq7_positive_theta_y(void);
void test_eq67_vertical(void);
void test_eq67_y_deflection(void);
void test_eq67_x_deflection(void);
/* Eq 8 */
void test_eq8_zero_angles(void);
void test_eq8_mixed_angles(void);
void test_eq8_neg_pitch(void);
void test_eq8_pos_yaw(void);
/* Eq 9 */
void test_eq9_hover(void);
void test_eq9_general(void);
void test_eq9_vertical_ascent(void);
void test_eq9_lateral_ignored(void);
void test_eq9_angled_thrust(void);
/* Eq 10 */
void test_eq10_single_axis(void);
void test_eq10_multi_axis(void);
void test_eq10_zero_rotation(void);
void test_eq10_scalar_mmoi_pitch(void);
void test_eq10_scalar_mmoi_complex(void);
/* Eq 11 */
void test_eq11_zero_gyro(void);
void test_eq11_general(void);
void test_eq11_all_zero(void);
void test_eq11_correction(void);
void test_eq11_offsetting(void);
/* Eq 12 */
void test_eq12_along_z(void);
void test_eq12_diagonal(void);
void test_eq12_no_command(void);
void test_eq12_z_isolation(void);
void test_eq12_x_isolation(void);
/* Eq 13 */
void test_eq13_along_z(void);
void test_eq13_diagonal(void);
void test_eq13_all_zero(void);
void test_eq13_remove_roll(void);

int main(void)
{
    UNITY_BEGIN();

    /* PID */
    RUN_TEST(test_pid_init_zeros_state);
    RUN_TEST(test_pid_proportional_only);
    RUN_TEST(test_pid_integral_accumulates);
    RUN_TEST(test_pid_anti_windup);
    RUN_TEST(test_pid_output_clamping);
    RUN_TEST(test_pid_reset_clears_state);
    RUN_TEST(test_pid_set_gains);
    RUN_TEST(test_pid_zero_error);

    /* Controls Equations*/
    /* Eq 1 */
    RUN_TEST(test_eq1_identity_des_90z_meas);
    RUN_TEST(test_eq1_90x_des_identity_meas);
    RUN_TEST(test_eq1_zero_error);
    RUN_TEST(test_eq1_90deg_pitch);
    RUN_TEST(test_eq1_opposite_rolls);

    /* Eq 2 */
    RUN_TEST(test_eq2_90deg_about_x);
    RUN_TEST(test_eq2_mixed_xy);
    RUN_TEST(test_eq2_identity);

    /* Eq 3 */
    RUN_TEST(test_eq3_simple_cross);
    RUN_TEST(test_eq3_general);
    RUN_TEST(test_eq3_zero_torque);
    RUN_TEST(test_eq3_pitch_torque);
    RUN_TEST(test_eq3_yaw_torque);

    /* Eq 4 */
    RUN_TEST(test_eq4_along_z);
    RUN_TEST(test_eq4_along_y);
    RUN_TEST(test_eq4_purely_vertical);
    RUN_TEST(test_eq4_345_triangle);
    RUN_TEST(test_eq4_51213_triangle);

    /* Eq 5 */
    RUN_TEST(test_eq5_case1);
    RUN_TEST(test_eq5_case2);
    RUN_TEST(test_eq5_vertical);
    RUN_TEST(test_eq5_pitch_correction);
    RUN_TEST(test_eq5_yaw_correction);

    /* Eq 6-7 */
    RUN_TEST(test_eq6_zero_angle);
    RUN_TEST(test_eq6_positive_theta_x);
    RUN_TEST(test_eq7_zero_angle);
    RUN_TEST(test_eq7_positive_theta_y);
    RUN_TEST(test_eq67_vertical);
    RUN_TEST(test_eq67_y_deflection);
    RUN_TEST(test_eq67_x_deflection);

    /* Eq 8 */
    RUN_TEST(test_eq8_zero_angles);
    RUN_TEST(test_eq8_mixed_angles);
    RUN_TEST(test_eq8_neg_pitch);
    RUN_TEST(test_eq8_pos_yaw);

    /* Eq 9 */
    RUN_TEST(test_eq9_hover);
    RUN_TEST(test_eq9_general);
    RUN_TEST(test_eq9_vertical_ascent);
    RUN_TEST(test_eq9_lateral_ignored);
    RUN_TEST(test_eq9_angled_thrust);

    /* Eq 10 */
    RUN_TEST(test_eq10_single_axis);
    RUN_TEST(test_eq10_multi_axis);
    RUN_TEST(test_eq10_zero_rotation);
    RUN_TEST(test_eq10_scalar_mmoi_pitch);
    RUN_TEST(test_eq10_scalar_mmoi_complex);

    /* Eq 11 */
    RUN_TEST(test_eq11_zero_gyro);
    RUN_TEST(test_eq11_general);
    RUN_TEST(test_eq11_all_zero);
    RUN_TEST(test_eq11_correction);
    RUN_TEST(test_eq11_offsetting);

    /* Eq 12 */
    RUN_TEST(test_eq12_along_z);
    RUN_TEST(test_eq12_diagonal);
    RUN_TEST(test_eq12_no_command);
    RUN_TEST(test_eq12_z_isolation);
    RUN_TEST(test_eq12_x_isolation);

    /* Eq 13 */
    RUN_TEST(test_eq13_along_z);
    RUN_TEST(test_eq13_diagonal);
    RUN_TEST(test_eq13_all_zero);
    RUN_TEST(test_eq13_remove_roll);

    return UNITY_END();
}
