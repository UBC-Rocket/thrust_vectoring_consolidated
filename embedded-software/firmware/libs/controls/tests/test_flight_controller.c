#include "unity.h"
#include "controls/flight_controller.h"
#include "state_estimation/state.h"
#include <math.h>
#include <string.h>

#define TOL 1e-3f

static flight_controller_config_t make_default_config(void)
{
    flight_controller_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));

    /* Diagonal Kp, Kd, I */
    for (int i = 0; i < 3; i++) {
        cfg.attitude.Kp[i][i] = 10.0f;
        cfg.attitude.Kd[i][i] = 1.0f;
        cfg.attitude.I[i][i]  = 0.01f;
    }
    cfg.allocation.t_hat[0] = 0;
    cfg.allocation.t_hat[1] = 0;
    cfg.allocation.t_hat[2] = -1;

    cfg.gimbal.L = 0.15f;
    cfg.gimbal.theta_min = -0.5f;
    cfg.gimbal.theta_max = 0.5f;

    cfg.thrust.m = 1.0f;
    cfg.thrust.g = 9.81f;
    cfg.thrust.T_min = 0.0f;
    cfg.thrust.T_max = 20.0f;
    cfg.thrust.kp = 5.0f;
    cfg.thrust.ki = 0.1f;
    cfg.thrust.kd = 1.0f;
    cfg.thrust.integral_limit = 10.0f;
    cfg.thrust.a_z_min = -5.0f;
    cfg.thrust.a_z_max = 5.0f;

    return cfg;
}

/* ---------- flight_controller_init + run at rest ---------- */

void test_flight_controller_at_rest(void)
{
    flight_controller_config_t cfg = make_default_config();
    flight_controller_init(&cfg);

    state_t state;
    memset(&state, 0, sizeof(state));
    state.q_bn.w = 1.0f; /* identity quaternion */

    flight_controller_ref_t ref;
    memset(&ref, 0, sizeof(ref));
    ref.q_ref.w = 1.0f; /* same as current */
    ref.z_ref = 0.0f;
    ref.vz_ref = 0.0f;

    control_output_t out;
    flight_controller_run(&state, &ref, &cfg, &out, 0.01f);

    /* At rest with matching attitude, gimbal torques should be near zero */
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_gim[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_gim[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_gim[2]);

    /* Gimbal angles should be within configured limits */
    TEST_ASSERT(out.theta_x_cmd >= cfg.gimbal.theta_min - TOL);
    TEST_ASSERT(out.theta_x_cmd <= cfg.gimbal.theta_max + TOL);
    TEST_ASSERT(out.theta_y_cmd >= cfg.gimbal.theta_min - TOL);
    TEST_ASSERT(out.theta_y_cmd <= cfg.gimbal.theta_max + TOL);

    /* Thrust should be near hover (m*g = 9.81) */
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 9.81f, out.T_cmd);
}

/* ---------- flight_controller produces nonzero torque with attitude error ---------- */

void test_flight_controller_attitude_error(void)
{
    flight_controller_config_t cfg = make_default_config();
    flight_controller_init(&cfg);

    state_t state;
    memset(&state, 0, sizeof(state));
    /* Tilt 10 degrees about x: q = cos(5deg) + sin(5deg)*i */
    float angle = 10.0f * (float)M_PI / 180.0f;
    state.q_bn.w = cosf(angle / 2.0f);
    state.q_bn.x = sinf(angle / 2.0f);

    flight_controller_ref_t ref;
    memset(&ref, 0, sizeof(ref));
    ref.q_ref.w = 1.0f; /* want identity */

    control_output_t out;
    flight_controller_run(&state, &ref, &cfg, &out, 0.01f);

    /* There should be a corrective torque */
    float torque_mag = sqrtf(out.tau_gim[0]*out.tau_gim[0] +
                             out.tau_gim[1]*out.tau_gim[1] +
                             out.tau_gim[2]*out.tau_gim[2]);
    TEST_ASSERT_GREATER_THAN(0.01f, torque_mag);
}
