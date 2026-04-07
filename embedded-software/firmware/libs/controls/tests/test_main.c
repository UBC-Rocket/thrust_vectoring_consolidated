#include "unity.h"

void setUp(void)
{
}
void tearDown(void)
{
}

/* test_pid.c */
void test_pid_init_zeros_state(void);
void test_pid_proportional_only(void);
void test_pid_integral_accumulates(void);
void test_pid_anti_windup(void);
void test_pid_output_clamping(void);
void test_pid_reset_clears_state(void);
void test_pid_set_gains(void);
void test_pid_zero_error(void);
void test_pid_ref_deriv_zero(void);
void test_pid_ref_deriv_kd_only(void);
void test_pid_ref_deriv_anti_windup(void);
void test_pid_ref_deriv_output_clamping(void);

/* test_flight_controller.c — black-box scenarios */
void test_fc_hover_at_reference(void);
void test_fc_pitch_attitude_error_drives_x_torque(void);
void test_fc_yaw_error_routes_to_differential_props(void);
void test_fc_rate_damping(void);
void test_fc_altitude_climb(void);
void test_fc_altitude_descent(void);
void test_fc_velocity_damping(void);
void test_fc_thrust_saturation_upper(void);
void test_fc_gimbal_saturation(void);
void test_fc_integral_windup_bound(void);
void test_fc_shortest_path_quaternion(void);
void test_fc_gyroscopic_feedforward(void);

/* test_pwm.c */
void test_pwm_zeroes(void);
void test_pwm_midpoint(void);
void test_pwm_max_thrust(void);
void test_pwm_max_torque_upper(void);
void test_pwm_max_torque_lower(void);

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
    RUN_TEST(test_pid_ref_deriv_zero);
    RUN_TEST(test_pid_ref_deriv_kd_only);
    RUN_TEST(test_pid_ref_deriv_anti_windup);
    RUN_TEST(test_pid_ref_deriv_output_clamping);

    /* Flight controller — black-box */
    RUN_TEST(test_fc_hover_at_reference);
    RUN_TEST(test_fc_pitch_attitude_error_drives_x_torque);
    RUN_TEST(test_fc_yaw_error_routes_to_differential_props);
    RUN_TEST(test_fc_rate_damping);
    RUN_TEST(test_fc_altitude_climb);
    RUN_TEST(test_fc_altitude_descent);
    RUN_TEST(test_fc_velocity_damping);
    RUN_TEST(test_fc_thrust_saturation_upper);
    RUN_TEST(test_fc_gimbal_saturation);
    RUN_TEST(test_fc_integral_windup_bound);
    RUN_TEST(test_fc_shortest_path_quaternion);
    RUN_TEST(test_fc_gyroscopic_feedforward);

    /* PWM */
    RUN_TEST(test_pwm_zeroes);
    RUN_TEST(test_pwm_midpoint);
    RUN_TEST(test_pwm_max_thrust);
    RUN_TEST(test_pwm_max_torque_upper);
    RUN_TEST(test_pwm_max_torque_lower);

    return UNITY_END();
}
