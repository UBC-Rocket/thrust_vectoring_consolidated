#include "unity.h"
#include "controls/pid.h"
#include <math.h>

#define TOL 1e-5f

/* ---------- pid_init ---------- */

void test_pid_init_zeros_state(void)
{
    pid_controller_t pid;
    pid_init(&pid, 1.0f, 0.5f, 0.1f, 100.0f, -500.0f, 500.0f);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, pid.integral_sum);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, pid.prev_measurement);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 1.0f, pid.kp);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.5f, pid.ki);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.1f, pid.kd);
}

/* ---------- pid_compute proportional only ---------- */

void test_pid_proportional_only(void)
{
    pid_controller_t pid;
    pid_init(&pid, 2.0f, 0.0f, 0.0f, 100.0f, -1000.0f, 1000.0f);
    float out = pid_compute(&pid, 10.0f, 0.0f, 0.01f);
    /* error = 10, kp = 2 => P = 20 */
    TEST_ASSERT_FLOAT_WITHIN(TOL, 20.0f, out);
}

/* ---------- pid_compute integral accumulation ---------- */

void test_pid_integral_accumulates(void)
{
    pid_controller_t pid;
    pid_init(&pid, 0.0f, 1.0f, 0.0f, 1000.0f, -1000.0f, 1000.0f);
    float dt = 0.01f;
    /* Apply constant error of 10 for 10 steps */
    for (int i = 0; i < 10; i++) {
        pid_compute(&pid, 10.0f, 0.0f, dt);
    }
    /* integral_sum should be 10 * 0.01 * 10 = 1.0 */
    TEST_ASSERT_FLOAT_WITHIN(TOL, 1.0f, pid_get_integral(&pid));
}

/* ---------- pid anti-windup ---------- */

void test_pid_anti_windup(void)
{
    pid_controller_t pid;
    pid_init(&pid, 0.0f, 1.0f, 0.0f, 5.0f, -1000.0f, 1000.0f);
    /* Large error * many steps should be clamped to integral_limit */
    for (int i = 0; i < 1000; i++) {
        pid_compute(&pid, 100.0f, 0.0f, 0.1f);
    }
    TEST_ASSERT_FLOAT_WITHIN(TOL, 5.0f, pid_get_integral(&pid));
}

/* ---------- pid output clamping ---------- */

void test_pid_output_clamping(void)
{
    pid_controller_t pid;
    pid_init(&pid, 10.0f, 0.0f, 0.0f, 100.0f, -50.0f, 50.0f);
    float out = pid_compute(&pid, 100.0f, 0.0f, 0.01f);
    /* kp*error = 1000, should clamp to 50 */
    TEST_ASSERT_FLOAT_WITHIN(TOL, 50.0f, out);
}

/* ---------- pid_reset ---------- */

void test_pid_reset_clears_state(void)
{
    pid_controller_t pid;
    pid_init(&pid, 1.0f, 1.0f, 1.0f, 100.0f, -500.0f, 500.0f);
    pid_compute(&pid, 10.0f, 5.0f, 0.01f);
    pid_reset(&pid);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, pid.integral_sum);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, pid.prev_measurement);
}

/* ---------- pid_set_gains ---------- */

void test_pid_set_gains(void)
{
    pid_controller_t pid;
    pid_init(&pid, 1.0f, 1.0f, 1.0f, 100.0f, -500.0f, 500.0f);
    pid_set_gains(&pid, 2.0f, 3.0f, 4.0f);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 2.0f, pid.kp);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 3.0f, pid.ki);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 4.0f, pid.kd);
}

/* ---------- pid zero error gives zero output ---------- */

void test_pid_zero_error(void)
{
    pid_controller_t pid;
    pid_init(&pid, 1.0f, 1.0f, 0.0f, 100.0f, -500.0f, 500.0f);
    float out = pid_compute(&pid, 5.0f, 5.0f, 0.01f);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out);
}

/* ---------- derivative on measurement ---------- */

void test_pid_derivative_on_measurement(void)
{
    pid_controller_t pid;
    /* kp=1, ki=0, kd=1: output = error + derivative-on-measurement */
    pid_init(&pid, 1.0f, 0.0f, 1.0f, 100.0f, -1000.0f, 1000.0f);

    /* First call: measurement=0->0, derivative = -(0-0)/1 = 0, error=0, output=0 */
    float out1 = pid_compute(&pid, 0.0f, 0.0f, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out1);

    /* Second call: measurement jumps to 5, setpoint still 0
     * error = -5, derivative = -(5-0)/1 = -5
     * output = kp*(-5) + kd*(-5) = -5 + (-5) = -10 */
    float out2 = pid_compute(&pid, 0.0f, 5.0f, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(TOL, -10.0f, out2);
}

/* ---------- pid_set_limits clamps existing integral ---------- */

void test_pid_set_limits(void)
{
    pid_controller_t pid;
    /* ki=1 to accumulate integral; wide limits so nothing else clips */
    pid_init(&pid, 0.0f, 1.0f, 0.0f, 1000.0f, -1000.0f, 1000.0f);

    /* 8 steps: error=100, dt=0.1 -> integral += 100*0.1=10 per step -> integral=80 */
    for (int i = 0; i < 8; i++) {
        pid_compute(&pid, 100.0f, 0.0f, 0.1f);
    }
    TEST_ASSERT_FLOAT_WITHIN(TOL, 80.0f, pid_get_integral(&pid));

    /* Tighten limit to 50: integral should be clamped */
    pid_set_limits(&pid, 50.0f, -200.0f, 200.0f);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 50.0f, pid_get_integral(&pid));

    /* Tighten further to 30 */
    pid_set_limits(&pid, 30.0f, -100.0f, 100.0f);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 30.0f, pid_get_integral(&pid));
}
