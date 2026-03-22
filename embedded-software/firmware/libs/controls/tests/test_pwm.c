#include "unity.h"

#include "controls/pwm.h"

#include <math.h>

#define TOL (20) /**< Tolerance for PWM period comparisons */

void test_pwm_zeroes(void)
{
    pwm_setpoint_t setpoint = pwm_setpoint_from_forces(0, 0);
    TEST_ASSERT_UINT16_WITHIN(TOL, 1000, setpoint.lower_motor_us);
    TEST_ASSERT_UINT16_WITHIN(TOL, 1000, setpoint.upper_motor_us);
}

void test_pwm_midpoint(void)
{
    pwm_setpoint_t setpoint = pwm_setpoint_from_forces(500, 0);
    TEST_ASSERT_UINT16_WITHIN(TOL, 1532, setpoint.lower_motor_us);
    TEST_ASSERT_UINT16_WITHIN(TOL, 1559, setpoint.upper_motor_us);
}

void test_pwm_max_thrust(void)
{
    pwm_setpoint_t setpoint = pwm_setpoint_from_forces(1594.56, 0);
    TEST_ASSERT_UINT16_WITHIN(TOL, 1950, setpoint.lower_motor_us);
    TEST_ASSERT_UINT16_WITHIN(TOL, 1999, setpoint.upper_motor_us);
}

void test_pwm_max_torque_upper(void)
{
    pwm_setpoint_t setpoint = pwm_setpoint_from_forces(782.16, 0.1952);
    TEST_ASSERT_UINT16_WITHIN(TOL, 1999, setpoint.lower_motor_us);
    TEST_ASSERT_UINT16_WITHIN(TOL, 1000, setpoint.upper_motor_us);
}

void test_pwm_max_torque_lower(void)
{
    pwm_setpoint_t setpoint = pwm_setpoint_from_forces(887.44, -0.1765);
    TEST_ASSERT_UINT16_WITHIN(TOL, 1000, setpoint.lower_motor_us);
    TEST_ASSERT_UINT16_WITHIN(TOL, 1999, setpoint.upper_motor_us);
}
