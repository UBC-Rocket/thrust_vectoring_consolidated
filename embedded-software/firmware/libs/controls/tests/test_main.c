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

/* test_flight_controller.c */
void test_flight_controller_at_rest(void);
void test_flight_controller_attitude_error(void);
void test_fc_null_inputs(void);
void test_fc_x_axis_tilt_corrective_torque(void);
void test_fc_y_axis_tilt_corrective_torque(void);
void test_fc_angular_rate_damping(void);
void test_fc_yaw_error_allocates_to_thrust(void);
void test_fc_thrust_below_ref(void);
void test_fc_thrust_above_ref(void);
void test_fc_thrust_clamped_to_tmax(void);
void test_fc_thrust_clamped_to_tmin(void);
void test_fc_gimbal_angle_clamped(void);
void test_fc_dt_too_small(void);
void test_fc_zero_pid(void);

/* test_pid.c (new) */
void test_pid_derivative_on_measurement(void);
void test_pid_set_limits(void);

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
    RUN_TEST(test_pid_derivative_on_measurement);
    RUN_TEST(test_pid_set_limits);

    /* Flight Controller */
    RUN_TEST(test_flight_controller_at_rest);
    RUN_TEST(test_flight_controller_attitude_error);
    RUN_TEST(test_fc_null_inputs);
    RUN_TEST(test_fc_x_axis_tilt_corrective_torque);
    RUN_TEST(test_fc_y_axis_tilt_corrective_torque);
    RUN_TEST(test_fc_angular_rate_damping);
    RUN_TEST(test_fc_yaw_error_allocates_to_thrust);
    RUN_TEST(test_fc_thrust_below_ref);
    RUN_TEST(test_fc_thrust_above_ref);
    RUN_TEST(test_fc_thrust_clamped_to_tmax);
    RUN_TEST(test_fc_thrust_clamped_to_tmin);
    RUN_TEST(test_fc_gimbal_angle_clamped);
    RUN_TEST(test_fc_dt_too_small);
    RUN_TEST(test_fc_zero_pid);

    return UNITY_END();
}
