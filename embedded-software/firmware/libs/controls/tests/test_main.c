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

/* test_flight_controller.c */
void test_flight_controller_at_rest(void);
void test_flight_controller_attitude_error(void);

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

    /* Flight Controller */
    RUN_TEST(test_flight_controller_at_rest);
    RUN_TEST(test_flight_controller_attitude_error);

    /* PWM */
    RUN_TEST(test_pwm_zeroes);
    RUN_TEST(test_pwm_midpoint);
    RUN_TEST(test_pwm_max_thrust);
    RUN_TEST(test_pwm_max_torque_upper);
    RUN_TEST(test_pwm_max_torque_lower);

    return UNITY_END();
}
