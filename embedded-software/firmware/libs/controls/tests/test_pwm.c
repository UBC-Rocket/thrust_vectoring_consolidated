#include "unity.h"
#include "controls/pwm.h"
#include <math.h>

#define TOL 20 /**< Tolerance for floating-point comparisons */

void test_pwm_zeroes(void)
{
    pwm_setpoint_t setpoint_pwm = pwm_from_setpoint(0,0);
    TEST_ASSERT_EQUAL_UINT16(1000, setpoint_pwm.upper_motor_us);
    TEST_ASSERT_EQUAL_UINT16(1000, setpoint_pwm.lower_motor_us);
}

void test_pwm_midpoint(void)
{
    pwm_setpoint_t setpoint_pwm = pwm_from_setpoint(500,0);
    TEST_ASSERT_EQUAL_UINT16(1531.9, setpoint_pwm.upper_motor_us);
    TEST_ASSERT_EQUAL_UINT16(1559.4, setpoint_pwm.lower_motor_us);
}

void test_pwm_max_thrust(void)
{
    pwm_setpoint_t setpoint_pwm = pwm_from_setpoint(1594.56,0);
    TEST_ASSERT_EQUAL_UINT16(1949.9, setpoint_pwm.upper_motor_us);
    TEST_ASSERT_EQUAL_UINT16(1999.0, setpoint_pwm.lower_motor_us);
}

void test_pwm_max_torque_upper(void)
{
    pwm_setpoint_t setpoint_pwm = pwm_from_setpoint(782.16, 0.1952);
    TEST_ASSERT_EQUAL_UINT16(1999.0, setpoint_pwm.upper_motor_us);
    TEST_ASSERT_EQUAL_UINT16(1000.0, setpoint_pwm.lower_motor_us);
}

void test_pwm_max_torque_lower(void)
{
    pwm_setpoint_t setpoint_pwm = pwm_from_setpoint(887.44, -0.1765);
    TEST_ASSERT_EQUAL_UINT16(1000.0, setpoint_pwm.upper_motor_us);
    TEST_ASSERT_EQUAL_UINT16(1999.0, setpoint_pwm.lower_motor_us);
}