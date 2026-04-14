/*
 * Black-box tests for the Ulysses Phase 1 v2 flight controller.
 *
 * Only the public API (flight_controller_init, flight_controller_run) is
 * exercised. All assertions are on fields of control_output_t.
 *
 * Sign convention used by the implementation (matches PDF Eq 5.5 with the +z
 * branch chosen): hover thrust direction is +body-z, t_hat = [0, 0, 1].
 */
#include "unity.h"
#include "controls/flight_controller.h"
#include "state_estimation/state.h"

#include <math.h>
#include <string.h>

#define TOL        1e-4f
#define TOL_COARSE 1e-3f

/* ── Test fixtures ─────────────────────────────────────────────────────── */

static void make_default_config(flight_controller_config_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));

    /* Diagonal Kp = 4, Kd = 1 (units N·m/rad and N·m·s/rad) */
    cfg->attitude.Kp[0][0] = 4.0f;
    cfg->attitude.Kp[1][1] = 4.0f;
    cfg->attitude.Kp[2][2] = 4.0f;
    cfg->attitude.Kd[0][0] = 1.0f;
    cfg->attitude.Kd[1][1] = 1.0f;
    cfg->attitude.Kd[2][2] = 1.0f;

    /* Diagonal inertia */
    cfg->attitude.I[0][0] = 0.05f;
    cfg->attitude.I[1][1] = 0.05f;
    cfg->attitude.I[2][2] = 0.02f;

    /* Initial thrust direction = +z body */
    cfg->allocation.t_hat[0] = 0.0f;
    cfg->allocation.t_hat[1] = 0.0f;
    cfg->allocation.t_hat[2] = 1.0f;

    /* Gimbal geometry: arm length 0.2 m, ±20° travel */
    cfg->gimbal.L         = 0.2f;
    cfg->gimbal.theta_min = -0.349f; /* ≈ -20° */
    cfg->gimbal.theta_max =  0.349f;

    /* Thrust loop: 1 kg vehicle, generous thrust limits, modest PID */
    cfg->thrust.m              = 1.0f;
    cfg->thrust.g              = 9.8067f;
    cfg->thrust.T_min          = 0.0f;
    cfg->thrust.T_max          = 50.0f;
    cfg->thrust.kp             = 8.0f;
    cfg->thrust.ki             = 2.0f;
    cfg->thrust.kd             = 4.0f;
    cfg->thrust.integral_limit = 5.0f;
    cfg->thrust.a_z_min        = -10.0f;
    cfg->thrust.a_z_max        =  20.0f;
}

static void make_state_at_rest(state_t *state)
{
    memset(state, 0, sizeof(*state));
    state->q_bn.w   = 1.0f;
    state->q_bn.x   = 0.0f;
    state->q_bn.y   = 0.0f;
    state->q_bn.z   = 0.0f;
    state->omega_b[0] = 0.0f;
    state->omega_b[1] = 0.0f;
    state->omega_b[2] = 0.0f;
    state->pos[0] = state->pos[1] = state->pos[2] = 0.0f;
    state->vel[0] = state->vel[1] = state->vel[2] = 0.0f;
}

static void make_default_ref(flight_controller_ref_t *ref)
{
    memset(ref, 0, sizeof(*ref));
    ref->q_ref.w = 1.0f;
    ref->z_ref   = 0.0f;
    ref->vz_ref  = 0.0f;
}

/* ── 1. Hover at reference: zero error → only gravity-compensating thrust ── */

void test_fc_hover_at_reference(void)
{
    flight_controller_config_t cfg;
    flight_controller_ref_t    ref;
    state_t                    state;
    control_output_t           out;

    make_default_config(&cfg);
    make_default_ref(&ref);
    make_state_at_rest(&state);

    flight_controller_init(&cfg);
    flight_controller_run(&state, &ref, &cfg, &out, 0.01f);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_cmd[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_cmd[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_cmd[2]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_gim[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_gim[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_gim[2]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_thrust);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.theta_x_cmd);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.theta_y_cmd);
    TEST_ASSERT_FLOAT_WITHIN(TOL_COARSE, cfg.thrust.m * cfg.thrust.g, out.T_cmd);
}

/* ── 2. Pure pitch attitude error (rotation about body x) ─────────────── */

void test_fc_pitch_attitude_error_drives_x_torque(void)
{
    flight_controller_config_t cfg;
    flight_controller_ref_t    ref;
    state_t                    state;
    control_output_t           out;

    make_default_config(&cfg);
    make_default_ref(&ref);
    make_state_at_rest(&state);

    /* Measured attitude: small rotation about body x (≈ 10°) */
    float half = 0.5f * 0.1745f;  /* ≈ 5° */
    state.q_bn.w = cosf(half);
    state.q_bn.x = sinf(half);
    state.q_bn.y = 0.0f;
    state.q_bn.z = 0.0f;

    flight_controller_init(&cfg);
    flight_controller_run(&state, &ref, &cfg, &out, 0.01f);

    /* The PD law produces a non-zero torque in the x channel only. */
    TEST_ASSERT_TRUE(fabsf(out.tau_cmd[0]) > 1e-3f);
    TEST_ASSERT_FLOAT_WITHIN(TOL_COARSE, 0.0f, out.tau_cmd[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL_COARSE, 0.0f, out.tau_cmd[2]);

    /* phi[0] is non-zero (axis-angle error along body x). */
    TEST_ASSERT_TRUE(fabsf(out.phi[0]) > 1e-3f);

    /* The pure-x torque has no z-component, so tau_thrust must be zero. */
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_thrust);

    /* Gimbal deflects in the x channel; y stays put. */
    TEST_ASSERT_TRUE(fabsf(out.theta_x_cmd) > 1e-4f);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.theta_y_cmd);
}

/* ── 3. Pure yaw error (rotation about body z): routes torque to props ── */

void test_fc_yaw_error_routes_to_differential_props(void)
{
    flight_controller_config_t cfg;
    flight_controller_ref_t    ref;
    state_t                    state;
    control_output_t           out;

    make_default_config(&cfg);
    make_default_ref(&ref);
    make_state_at_rest(&state);

    /* Measured attitude: small rotation about body z */
    float half = 0.5f * 0.1745f;
    state.q_bn.w = cosf(half);
    state.q_bn.z = sinf(half);

    flight_controller_init(&cfg);
    flight_controller_run(&state, &ref, &cfg, &out, 0.01f);

    /* tau_cmd has a non-zero z component */
    TEST_ASSERT_TRUE(fabsf(out.tau_cmd[2]) > 1e-3f);

    /* tau_thrust (scalar axial command) is non-zero. */
    TEST_ASSERT_TRUE(fabsf(out.tau_thrust) > 1e-3f);

    /* tau_gim z-component is zero — at hover t_hat = [0,0,1], so the entire
       z component of tau_cmd is subtracted off. */
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_gim[2]);
}

/* ── 4. Rate damping: zero attitude error, non-zero ω → tau_cmd opposes ω ─ */

void test_fc_rate_damping(void)
{
    flight_controller_config_t cfg;
    flight_controller_ref_t    ref;
    state_t                    state;
    control_output_t           out;

    make_default_config(&cfg);
    make_default_ref(&ref);
    make_state_at_rest(&state);

    state.omega_b[0] = 1.0f;  /* 1 rad/s about body x */

    flight_controller_init(&cfg);
    flight_controller_run(&state, &ref, &cfg, &out, 0.01f);

    /* phi = 0 → only -Kd*omega + ω×(Iω). With diagonal I and ω along x,
       ω × (Iω) = 0, so tau_cmd[0] = -Kd*ω = -1*1 = -1. */
    TEST_ASSERT_FLOAT_WITHIN(TOL_COARSE, -1.0f, out.tau_cmd[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL_COARSE,  0.0f, out.tau_cmd[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL_COARSE,  0.0f, out.tau_cmd[2]);
}

/* ── 5. Altitude climb: z below z_ref → T_cmd > m·g ───────────────────── */

void test_fc_altitude_climb(void)
{
    flight_controller_config_t cfg;
    flight_controller_ref_t    ref;
    state_t                    state;
    control_output_t           out;

    make_default_config(&cfg);
    make_default_ref(&ref);
    make_state_at_rest(&state);

    ref.z_ref     = 5.0f;   /* setpoint above current */
    state.pos[2]  = 0.0f;

    flight_controller_init(&cfg);
    flight_controller_run(&state, &ref, &cfg, &out, 0.01f);

    TEST_ASSERT_TRUE(out.T_cmd > cfg.thrust.m * cfg.thrust.g);
}

/* ── 6. Altitude descent: z above z_ref → T_cmd < m·g ─────────────────── */

void test_fc_altitude_descent(void)
{
    flight_controller_config_t cfg;
    flight_controller_ref_t    ref;
    state_t                    state;
    control_output_t           out;

    make_default_config(&cfg);
    make_default_ref(&ref);
    make_state_at_rest(&state);

    ref.z_ref    = 0.0f;
    state.pos[2] = 5.0f;

    flight_controller_init(&cfg);
    flight_controller_run(&state, &ref, &cfg, &out, 0.01f);

    TEST_ASSERT_TRUE(out.T_cmd < cfg.thrust.m * cfg.thrust.g);
}

/* ── 7. Velocity damping: at z_ref but climbing too fast → T < m·g ────── */

void test_fc_velocity_damping(void)
{
    flight_controller_config_t cfg;
    flight_controller_ref_t    ref;
    state_t                    state;
    control_output_t           out;

    make_default_config(&cfg);
    make_default_ref(&ref);
    make_state_at_rest(&state);

    ref.z_ref    = 0.0f;
    ref.vz_ref   = 0.0f;
    state.pos[2] = 0.0f;
    state.vel[2] = 1.0f;   /* climbing while we want to be still */

    flight_controller_init(&cfg);
    flight_controller_run(&state, &ref, &cfg, &out, 0.01f);

    /* error = 0, error_dot = 0 - 1 = -1 → kd*error_dot = -4 → a_z_cmd = -4
       → T_cmd = m*(g-4) < m*g. */
    TEST_ASSERT_TRUE(out.T_cmd < cfg.thrust.m * cfg.thrust.g);
}

/* ── 8. Thrust saturation at upper limit ──────────────────────────────── */

void test_fc_thrust_saturation_upper(void)
{
    flight_controller_config_t cfg;
    flight_controller_ref_t    ref;
    state_t                    state;
    control_output_t           out;

    make_default_config(&cfg);
    /* Tighten T_max so the m*(g + a_z_max) ceiling exceeds it. */
    cfg.thrust.T_max = 15.0f;

    make_default_ref(&ref);
    make_state_at_rest(&state);

    ref.z_ref    = 1000.0f;
    state.pos[2] = 0.0f;

    flight_controller_init(&cfg);
    /* Pump the loop so the integral and proportional terms saturate the
       output. */
    for (int i = 0; i < 10; ++i)
    {
        flight_controller_run(&state, &ref, &cfg, &out, 0.01f);
    }

    TEST_ASSERT_FLOAT_WITHIN(TOL, cfg.thrust.T_max, out.T_cmd);
}

/* ── 9. Gimbal saturation: huge attitude error → angles clamped to limit ─ */

void test_fc_gimbal_saturation(void)
{
    flight_controller_config_t cfg;
    flight_controller_ref_t    ref;
    state_t                    state;
    control_output_t           out;

    make_default_config(&cfg);
    /* Crank up Kp so the gimbal angles want to exceed the limit. */
    cfg.attitude.Kp[0][0] = 1000.0f;
    cfg.attitude.Kp[1][1] = 1000.0f;

    make_default_ref(&ref);
    make_state_at_rest(&state);

    /* 30° pitch about body y */
    float half = 0.5f * 0.5236f;
    state.q_bn.w = cosf(half);
    state.q_bn.y = sinf(half);

    flight_controller_init(&cfg);
    flight_controller_run(&state, &ref, &cfg, &out, 0.01f);

    /* theta_y_cmd should be clamped to a limit. */
    int saturated = (fabsf(out.theta_y_cmd - cfg.gimbal.theta_max) < TOL ||
                     fabsf(out.theta_y_cmd - cfg.gimbal.theta_min) < TOL);
    TEST_ASSERT_TRUE(saturated);
}

/* ── 10. Integral windup: sustained error → integral bounded ──────────── */

void test_fc_integral_windup_bound(void)
{
    flight_controller_config_t cfg;
    flight_controller_ref_t    ref;
    state_t                    state;
    control_output_t           out;

    make_default_config(&cfg);
    make_default_ref(&ref);
    make_state_at_rest(&state);

    ref.z_ref    = 100.0f;
    state.pos[2] = 0.0f;

    flight_controller_init(&cfg);
    for (int i = 0; i < 10000; ++i)
    {
        flight_controller_run(&state, &ref, &cfg, &out, 0.01f);
    }

    TEST_ASSERT_TRUE(fabsf(out.z_pid_integral) <= cfg.thrust.integral_limit + TOL);
}

/* ── 11. Shortest-path quaternion: q_err with w<0 produces same control ── */

void test_fc_shortest_path_quaternion(void)
{
    flight_controller_config_t cfg;
    flight_controller_ref_t    ref;
    state_t                    state_pos, state_neg;
    control_output_t           out_pos, out_neg;

    make_default_config(&cfg);
    make_default_ref(&ref);
    make_state_at_rest(&state_pos);
    make_state_at_rest(&state_neg);

    /* Same physical orientation expressed as q and -q. */
    float half = 0.5f * 0.3491f;  /* 20° about body y */
    state_pos.q_bn.w = cosf(half);
    state_pos.q_bn.y = sinf(half);

    state_neg.q_bn.w = -cosf(half);
    state_neg.q_bn.y = -sinf(half);

    flight_controller_init(&cfg);
    flight_controller_run(&state_pos, &ref, &cfg, &out_pos, 0.01f);

    flight_controller_init(&cfg);
    flight_controller_run(&state_neg, &ref, &cfg, &out_neg, 0.01f);

    TEST_ASSERT_FLOAT_WITHIN(TOL_COARSE, out_pos.tau_cmd[0], out_neg.tau_cmd[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL_COARSE, out_pos.tau_cmd[1], out_neg.tau_cmd[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL_COARSE, out_pos.tau_cmd[2], out_neg.tau_cmd[2]);
}

/* ── 12. Gyroscopic feedforward: ω with non-diagonal I produces ω×(Iω) ── */

void test_fc_gyroscopic_feedforward(void)
{
    flight_controller_config_t cfg;
    flight_controller_ref_t    ref;
    state_t                    state;
    control_output_t           out;

    make_default_config(&cfg);
    /* Zero out gain matrices so the only contribution is the gyro term. */
    memset(cfg.attitude.Kp, 0, sizeof(cfg.attitude.Kp));
    memset(cfg.attitude.Kd, 0, sizeof(cfg.attitude.Kd));

    /* Distinct principal inertias to make ω×(Iω) non-zero for ω across two
       axes. */
    memset(cfg.attitude.I, 0, sizeof(cfg.attitude.I));
    cfg.attitude.I[0][0] = 0.10f;
    cfg.attitude.I[1][1] = 0.05f;
    cfg.attitude.I[2][2] = 0.02f;

    make_default_ref(&ref);
    make_state_at_rest(&state);

    state.omega_b[0] = 1.0f;
    state.omega_b[1] = 2.0f;
    state.omega_b[2] = 0.0f;

    flight_controller_init(&cfg);
    flight_controller_run(&state, &ref, &cfg, &out, 0.01f);

    /* Hand-computed: I*ω = [0.10, 0.10, 0]; ω × (I*ω) =
         [ωy*(Iω)z - ωz*(Iω)y, ωz*(Iω)x - ωx*(Iω)z, ωx*(Iω)y - ωy*(Iω)x]
       = [2*0 - 0*0.10, 0*0.10 - 1*0, 1*0.10 - 2*0.10]
       = [0, 0, -0.10]. */
    TEST_ASSERT_FLOAT_WITHIN(TOL_COARSE,  0.0f, out.tau_cmd[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL_COARSE,  0.0f, out.tau_cmd[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL_COARSE, -0.10f, out.tau_cmd[2]);
}
