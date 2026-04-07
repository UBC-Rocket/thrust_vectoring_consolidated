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

/* ---------- pid_compute_with_ref_deriv: zero error and zero rate ---------- */

void test_pid_ref_deriv_zero(void)
{
    pid_controller_t pid;
    pid_init(&pid, 2.0f, 1.0f, 3.0f, 100.0f, -500.0f, 500.0f);
    float out = pid_compute_with_ref_deriv(&pid, 5.0f, 0.0f, 5.0f, 0.0f, 0.01f);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out);
}

/* ---------- pid_compute_with_ref_deriv: derivative-on-error term ---------- */

void test_pid_ref_deriv_kd_only(void)
{
    pid_controller_t pid;
    pid_init(&pid, 0.0f, 0.0f, 4.0f, 100.0f, -500.0f, 500.0f);
    /* error = 0, error_dot = 1 - (-2) = 3, kd*error_dot = 12 */
    float out = pid_compute_with_ref_deriv(&pid, 0.0f, 1.0f, 0.0f, -2.0f, 0.01f);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 12.0f, out);
}

/* ---------- pid_compute_with_ref_deriv: anti-windup ---------- */

void test_pid_ref_deriv_anti_windup(void)
{
    pid_controller_t pid;
    pid_init(&pid, 0.0f, 1.0f, 0.0f, 5.0f, -1000.0f, 1000.0f);
    for (int i = 0; i < 1000; ++i)
    {
        pid_compute_with_ref_deriv(&pid, 100.0f, 0.0f, 0.0f, 0.0f, 0.1f);
    }
    TEST_ASSERT_FLOAT_WITHIN(TOL, 5.0f, pid_get_integral(&pid));
}

/* ---------- pid_compute_with_ref_deriv: output clamping ---------- */

void test_pid_ref_deriv_output_clamping(void)
{
    pid_controller_t pid;
    pid_init(&pid, 100.0f, 0.0f, 0.0f, 100.0f, -50.0f, 50.0f);
    /* kp*error = 100*10 = 1000, should clamp to 50 */
    float out = pid_compute_with_ref_deriv(&pid, 10.0f, 0.0f, 0.0f, 0.0f, 0.01f);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 50.0f, out);
}
