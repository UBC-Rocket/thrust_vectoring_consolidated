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
// controller should be detect a tilt and procude a corrective torque.

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

/* ---------- null inputs do not crash ---------- */

void test_fc_null_inputs(void)
{
    flight_controller_config_t cfg = make_default_config();
    flight_controller_init(&cfg);

    state_t state;
    memset(&state, 0, sizeof(state));
    state.q_bn.w = 1.0f;

    flight_controller_ref_t ref;
    memset(&ref, 0, sizeof(ref));
    ref.q_ref.w = 1.0f;

    control_output_t out;
    memset(&out, 0, sizeof(out));

    /* Each of these should silently return without crashing */
    flight_controller_run(NULL, &ref, &cfg, &out, 0.01f);
    flight_controller_run(&state, NULL, &cfg, &out, 0.01f);
    flight_controller_run(&state, &ref, NULL, &out, 0.01f);
    flight_controller_run(&state, &ref, &cfg, NULL, 0.01f);
    /* No assertions needed: test passes if it does not crash */
}

/* ---------- x-axis tilt produces corrective torque in x ---------- */

void test_fc_x_axis_tilt_corrective_torque(void)
{
    flight_controller_config_t cfg = make_default_config();
    flight_controller_init(&cfg);

    state_t state;
    memset(&state, 0, sizeof(state));
    /* 2-degree rotation about x */
    float half = 1.0f * (float)M_PI / 180.0f; /* half-angle = 1 deg */
    state.q_bn.w = cosf(half);
    state.q_bn.x = sinf(half);

    flight_controller_ref_t ref;
    memset(&ref, 0, sizeof(ref));
    ref.q_ref.w = 1.0f;

    control_output_t out;
    flight_controller_run(&state, &ref, &cfg, &out, 0.01f);

    /* phi[0] = 2*sin(1deg), tau_cmd[0] = -Kp*phi[0] = -10*2*sin(pi/180) ≈ -0.3490
     * with t_hat=[0,0,-1] and tau_cmd[2]=0 -> tau_gim[0] = tau_cmd[0] */
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, -0.3490f, out.tau_gim[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_gim[1]);
}

/* ---------- y-axis tilt produces corrective torque in y ---------- */

void test_fc_y_axis_tilt_corrective_torque(void)
{
    flight_controller_config_t cfg = make_default_config();
    flight_controller_init(&cfg);

    state_t state;
    memset(&state, 0, sizeof(state));
    /* 2-degree rotation about y */
    float half = 1.0f * (float)M_PI / 180.0f;
    state.q_bn.w = cosf(half);
    state.q_bn.y = sinf(half);

    flight_controller_ref_t ref;
    memset(&ref, 0, sizeof(ref));
    ref.q_ref.w = 1.0f;

    control_output_t out;
    flight_controller_run(&state, &ref, &cfg, &out, 0.01f);

    /* phi[1] = 2*sin(1deg), tau_cmd[1] = -Kp*phi[1] ≈ -0.3490 */
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, -0.3490f, out.tau_gim[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_gim[0]);
}

/* ---------- angular rate damping: Kd opposes rotation ---------- */

void test_fc_angular_rate_damping(void)
{
    flight_controller_config_t cfg = make_default_config();
    flight_controller_init(&cfg);

    state_t state;
    memset(&state, 0, sizeof(state));
    state.q_bn.w = 1.0f;         /* identity: no attitude error */
    state.omega_b[0] = 1.0f;     /* 1 rad/s about x */

    flight_controller_ref_t ref;
    memset(&ref, 0, sizeof(ref));
    ref.q_ref.w = 1.0f;

    control_output_t out;
    flight_controller_run(&state, &ref, &cfg, &out, 0.01f);

    /* phi=0, omega×(I*omega) = [1,0,0]×[0.01,0,0] = [0,0,0] (parallel)
     * tau_cmd[0] = -Kd[0][0]*1.0 = -1.0 -> tau_gim[0] = -1.0 */
    TEST_ASSERT_FLOAT_WITHIN(TOL, -1.0f, out.tau_gim[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f,  out.tau_gim[1]);
}

/* ---------- yaw error is allocated to tau_thrust, not tau_gim ---------- */

void test_fc_yaw_error_allocates_to_thrust(void)
{
    flight_controller_config_t cfg = make_default_config();
    flight_controller_init(&cfg);

    state_t state;
    memset(&state, 0, sizeof(state));
    /* 5-degree rotation about z */
    float half = 2.5f * (float)M_PI / 180.0f;
    state.q_bn.w = cosf(half);
    state.q_bn.z = sinf(half);

    flight_controller_ref_t ref;
    memset(&ref, 0, sizeof(ref));
    ref.q_ref.w = 1.0f;

    control_output_t out;
    flight_controller_run(&state, &ref, &cfg, &out, 0.01f);

    /* tau_cmd[2] = -Kp*2*sin(2.5deg) ≈ -0.8724, t_hat[2]=-1:
     * tau_thrust = -0.8724/(-1) = 0.8724, tau_gim[2] = 0 */
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_gim[2]);
    TEST_ASSERT_FLOAT_WITHIN(1e-2f, 0.8724f, out.tau_thrust);
}

/* ---------- z below reference: thrust increases above hover ---------- */

void test_fc_thrust_below_ref(void)
{
    flight_controller_config_t cfg = make_default_config();
    flight_controller_init(&cfg);

    state_t state;
    memset(&state, 0, sizeof(state));
    state.q_bn.w = 1.0f;
    state.pos[2] = 0.0f;

    flight_controller_ref_t ref;
    memset(&ref, 0, sizeof(ref));
    ref.q_ref.w = 1.0f;
    ref.z_ref = 1.0f; /* want to be 1 m higher */

    control_output_t out;
    flight_controller_run(&state, &ref, &cfg, &out, 0.01f);

    /* kp=5, error=1 -> a_z_cmd >= 4 -> T_cmd > m*g = 9.81 */
    TEST_ASSERT_GREATER_THAN(cfg.thrust.m * cfg.thrust.g + 1.0f, out.T_cmd);
}

/* ---------- z above reference: thrust decreases below hover ---------- */

void test_fc_thrust_above_ref(void)
{
    flight_controller_config_t cfg = make_default_config();
    flight_controller_init(&cfg);

    state_t state;
    memset(&state, 0, sizeof(state));
    state.q_bn.w = 1.0f;
    state.pos[2] = 2.0f; /* 2 m above desired */

    flight_controller_ref_t ref;
    memset(&ref, 0, sizeof(ref));
    ref.q_ref.w = 1.0f;
    ref.z_ref = 0.0f;

    control_output_t out;
    flight_controller_run(&state, &ref, &cfg, &out, 0.01f);

    /* error=-2, a_z_cmd clamped to a_z_min=-5 -> T_cmd = 1*(9.81-5) = 4.81 */
    TEST_ASSERT_LESS_THAN(cfg.thrust.m * cfg.thrust.g - 1.0f, out.T_cmd);
}

/* ---------- T_max clamping ---------- */

void test_fc_thrust_clamped_to_tmax(void)
{
    flight_controller_config_t cfg = make_default_config();
    cfg.thrust.T_max = 12.0f; /* lower ceiling: 12 < m*(g+a_z_max) = 14.81 */
    flight_controller_init(&cfg);

    state_t state;
    memset(&state, 0, sizeof(state));
    state.q_bn.w = 1.0f;
    state.pos[2] = 0.0f;

    flight_controller_ref_t ref;
    memset(&ref, 0, sizeof(ref));
    ref.q_ref.w = 1.0f;
    ref.z_ref = 100.0f; /* huge error forces a_z_cmd to a_z_max */

    control_output_t out;
    flight_controller_run(&state, &ref, &cfg, &out, 0.01f);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 12.0f, out.T_cmd);
}

/* ---------- T_min clamping ---------- */

void test_fc_thrust_clamped_to_tmin(void)
{
    flight_controller_config_t cfg = make_default_config();
    cfg.thrust.T_min = 5.0f; /* floor: 5 > m*(g+a_z_min) = 4.81 */
    flight_controller_init(&cfg);

    state_t state;
    memset(&state, 0, sizeof(state));
    state.q_bn.w = 1.0f;
    state.pos[2] = 2.0f;

    flight_controller_ref_t ref;
    memset(&ref, 0, sizeof(ref));
    ref.q_ref.w = 1.0f;
    ref.z_ref = 0.0f;

    control_output_t out;
    flight_controller_run(&state, &ref, &cfg, &out, 0.01f);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 5.0f, out.T_cmd);
}

/* ---------- large attitude error saturates gimbal angles ---------- */

void test_fc_gimbal_angle_clamped(void)
{
    flight_controller_config_t cfg = make_default_config();
    flight_controller_init(&cfg);

    state_t state;
    memset(&state, 0, sizeof(state));
    /* 45-degree rotation about x: tau_gim[0] will be huge -> T_y >> T_cmd */
    float half = 22.5f * (float)M_PI / 180.0f;
    state.q_bn.w = cosf(half);
    state.q_bn.x = sinf(half);

    flight_controller_ref_t ref;
    memset(&ref, 0, sizeof(ref));
    ref.q_ref.w = 1.0f;

    control_output_t out;
    flight_controller_run(&state, &ref, &cfg, &out, 0.01f);

    /* Infeasible torque demand -> angles saturate at theta_max */
    TEST_ASSERT_FLOAT_WITHIN(TOL, cfg.gimbal.theta_max, out.theta_x_cmd);
    TEST_ASSERT(out.theta_y_cmd >= cfg.gimbal.theta_min - TOL);
    TEST_ASSERT(out.theta_y_cmd <= cfg.gimbal.theta_max + TOL);
}

/* ---------- tiny dt bypasses z-PID and gives hover thrust ---------- */

void test_fc_dt_too_small(void)
{
    flight_controller_config_t cfg = make_default_config();
    flight_controller_init(&cfg);

    state_t state;
    memset(&state, 0, sizeof(state));
    state.q_bn.w = 1.0f;

    flight_controller_ref_t ref;
    memset(&ref, 0, sizeof(ref));
    ref.q_ref.w = 1.0f;
    ref.z_ref = 5.0f; /* large z error, but dt too small to act on it */

    control_output_t out;
    /* dt < MIN_DT_S (1e-6): a_z_cmd = 0 -> T_cmd = m*(g+0) */
    flight_controller_run(&state, &ref, &cfg, &out, 1e-7f);

    TEST_ASSERT_FLOAT_WITHIN(0.01f, cfg.thrust.m * cfg.thrust.g, out.T_cmd);
}

/* ---------- all pid values set to zero ---------- */

void test_fc_zero_pid(void)
{
    flight_controller_config_t cfg = make_default_config();
    /* Zero out all PID gains */
    memset(&cfg.attitude.Kp, 0, sizeof(cfg.attitude.Kp));
    memset(&cfg.attitude.Kd, 0, sizeof(cfg.attitude.Kd));
    memset(&cfg.thrust.kp, 0, sizeof(cfg.thrust.kp) + sizeof(cfg.thrust.ki) + sizeof(cfg.thrust.kd));
    flight_controller_init(&cfg);

    state_t state;
    memset(&state, 0, sizeof(state));
    /* 10-degree rotation about x */
    float half = 5.0f * (float)M_PI / 180.0f;
    state.q_bn.w = cosf(half);
    state.q_bn.x = sinf(half);

    flight_controller_ref_t ref;
    memset(&ref, 0, sizeof(ref));
    ref.q_ref.w = 1.0f;

    control_output_t out;
    flight_controller_run(&state, &ref, &cfg, &out, 0.01f);

    /* With zero gains there should be no corrective torque or change in thrust */
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_gim[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_gim[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_gim[2]);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, cfg.thrust.m * cfg.thrust.g, out.T_cmd);
}
