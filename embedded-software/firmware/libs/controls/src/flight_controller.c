/**
 * @file    flight_controller.c
 * @brief   Flight control body: torque, allocation, thrust PID, gimbal angles.
 *
 * Pipeline: torque_module -> control_allocation -> thrust_controller -> gimbal_angles.
 * Quaternion error uses conj(q_ref) * q_bn per proposal (Δq = q_ref ⊗ q* with world-to-body q). Z-PID uses
 * derivative-on-measurement (vz_ref not in derivative term).
 */

#include "controls/flight_controller.h"
#include "controls/pid.h"
#include "state_estimation/state.h"
#include <math.h>
#include <string.h>

#define MIN_DT_S        1.0e-6f   /**< Min dt [s] to avoid div-by-zero in PID */
#define MIN_T_CMD_GIMBAL 0.1f     /**< Min T_cmd [N] for gimbal angle computation */

static pid_controller_t z_pid;
static uint8_t z_pid_initialized = 0;

/** Clamp value to [min_val, max_val]. */
static float clampf(float value, float min_val, float max_val)
{
    if (value > max_val) return max_val;
    if (value < min_val) return min_val;
    return value;
}

/**
 * @brief Body torque control: Δq = conj(q_ref)*q_bn, φ = 2*vec(Δq), τ = -Kp*φ - Kd*ω + ω×(I*ω).
 * @param state Current state (q_bn, omega_b).
 * @param ref   Desired q_ref (body-to-nav).
 * @param cfg   Attitude gains and inertia.
 * @param tau_cmd Output torque command [3] [N·m].
 */
static void torque_module(const state_t *state,
                          const flight_controller_ref_t *ref,
                          const flight_controller_attitude_config_t *cfg,
                          float tau_cmd[3])
{
    quaternion_t q_ref_conj;
    quaternion_t dq;
    float phi[3];
    float Kp_phi[3];
    float Kd_omega[3];
    float I_omega[3];
    float omega_cross_I_omega[3];

    /* Δq = conj(q_ref) ⊗ q_bn per proposal (world-to-body q: Δq = q_ref ⊗ q* with q = conj(q_bn)); shortest path: if dq.w < 0 then negate */
    quaternion_conjugate(&ref->q_ref, &q_ref_conj);
    quaternion_multiply(&q_ref_conj, &state->q_bn, &dq);
    if (dq.w < 0.0f) {
        dq.w = -dq.w;
        dq.x = -dq.x;
        dq.y = -dq.y;
        dq.z = -dq.z;
    }
    /* φ ≈ 2 * vector_part(Δq) */
    phi[0] = 2.0f * dq.x;
    phi[1] = 2.0f * dq.y;
    phi[2] = 2.0f * dq.z;

    /* τ_cmd = -Kp φ - Kd ω + ω×(I ω) */
    matrix33_vec3_mul(cfg->Kp, phi, Kp_phi);
    matrix33_vec3_mul(cfg->Kd, state->omega_b, Kd_omega);
    matrix33_vec3_mul(cfg->I, state->omega_b, I_omega);
    vec3_cross(state->omega_b, I_omega, omega_cross_I_omega);

    tau_cmd[0] = -Kp_phi[0] - Kd_omega[0] + omega_cross_I_omega[0];
    tau_cmd[1] = -Kp_phi[1] - Kd_omega[1] + omega_cross_I_omega[1];
    tau_cmd[2] = -Kp_phi[2] - Kd_omega[2] + omega_cross_I_omega[2];
}

/**
 * @brief Split τ_cmd into τ_gim and τ_thrust: τ_thrust = τ_z/t_z, τ_gim = τ_cmd - τ_thrust*t̂.
 * @param tau_cmd   Total torque command [3].
 * @param t_hat     Unit thrust direction [3].
 * @param tau_gim   Output gimbal torque [3].
 * @param tau_thrust Output scalar differential torque from props.
 */
static void control_allocation(const float tau_cmd[3],
                               const float t_hat[3],
                               float tau_gim[3],
                               float *tau_thrust)
{
    float t_z = t_hat[2];
    if (fabsf(t_z) < 1.0e-6f) {
        t_z = (t_z >= 0.0f) ? 1.0e-6f : -1.0e-6f;
    }
    float tau_z = tau_cmd[2];
    *tau_thrust = tau_z / t_z;
    /* τ_gim = τ_cmd - τ_thrust * t̂ */
    tau_gim[0] = tau_cmd[0] - (*tau_thrust) * t_hat[0];
    tau_gim[1] = tau_cmd[1] - (*tau_thrust) * t_hat[1];
    tau_gim[2] = tau_cmd[2] - (*tau_thrust) * t_hat[2];
}

/**
 * @brief Z PID -> a_z_cmd, then T_cmd = m*(g + a_z_cmd) saturated to [T_min, T_max].
 * @param z_ref     Desired z [m].
 * @param vz_ref    Desired ż [m/s] (not used in derivative; PID is derivative-on-measurement).
 * @param z         Current z [m].
 * @param vz        Current ż [m/s].
 * @param cfg       Thrust config.
 * @param dt_s      Time step [s].
 * @param T_cmd_out Output thrust magnitude [N].
 * @return a_z_cmd [m/s²] (for reference).
 */
static float thrust_controller(float z_ref, float vz_ref, float z, float vz,
                               const flight_controller_thrust_config_t *cfg,
                               float dt_s,
                               float *T_cmd_out)
{
    float a_z_cmd;
    if (dt_s < MIN_DT_S) {
        a_z_cmd = 0.0f;
    } else {
        a_z_cmd = pid_compute(&z_pid, z_ref, z, dt_s);
    }
    a_z_cmd = clampf(a_z_cmd, cfg->a_z_min, cfg->a_z_max);
    float T_cmd = cfg->m * (cfg->g + a_z_cmd);
    *T_cmd_out = clampf(T_cmd, cfg->T_min, cfg->T_max);
    return a_z_cmd;
}

/**
 * @brief From τ_gim and T_cmd compute T_x,T_y,T_z -> t̂ -> θ_x = atan2(t_y,-t_z), θ_y = -asin(t_x), saturated.
 * @param tau_gim     Gimbal torque [3] (τ_x, τ_y, 0).
 * @param T_cmd       Thrust magnitude [N].
 * @param cfg         Gimbal L and angle limits.
 * @param theta_x_cmd Output gimbal angle x [rad].
 * @param theta_y_cmd Output gimbal angle y [rad].
 */
static void gimbal_angles(const float tau_gim[3],
                          float T_cmd,
                          const flight_controller_gimbal_config_t *cfg,
                          float *theta_x_cmd,
                          float *theta_y_cmd)
{
    if (T_cmd < MIN_T_CMD_GIMBAL) {
        *theta_x_cmd = 0.0f;
        *theta_y_cmd = 0.0f;
        return;
    }
    float L = cfg->L;
    if (L < 1.0e-6f) {
        *theta_x_cmd = 0.0f;
        *theta_y_cmd = 0.0f;
        return;
    }
    /* T_x = τ_y/L, T_y = -τ_x/L (from τ_gim = [-L*T_y, L*T_x, 0]) */
    float T_x = tau_gim[1] / L;
    float T_y = -tau_gim[0] / L;
    float T_x2_y2 = T_x * T_x + T_y * T_y;
    float T_sq = T_cmd * T_cmd;
    float T_z;
    if (T_x2_y2 >= T_sq) {
        /* Infeasible: clamp lateral to max and set T_z to small positive */
        float scale = sqrtf(T_sq / (T_x2_y2 + 1.0e-12f));
        T_x *= scale;
        T_y *= scale;
        T_z = 0.01f * T_cmd;
    } else {
        T_z = sqrtf(T_sq - T_x2_y2);
    }
    /* Unit thrust t̂ */
    float tx = T_x / T_cmd;
    float ty = T_y / T_cmd;
    float tz = T_z / T_cmd;
    /* θ_x = atan2(t_y, -t_z), θ_y = -asin(t_x) */
    float theta_x = atan2f(ty, -tz);
    float theta_y = -asinf(clampf(tx, -1.0f, 1.0f));
    *theta_x_cmd = clampf(theta_x, cfg->theta_min, cfg->theta_max);
    *theta_y_cmd = clampf(theta_y, cfg->theta_min, cfg->theta_max);
}

/**
 * @brief Initialize z-PID state. Call once before first run or when re-arming.
 * @param config Controller config (thrust gains used for z-PID).
 */
void flight_controller_init(const flight_controller_config_t *config)
{
    if (!config) return;
    const flight_controller_thrust_config_t *t = &config->thrust;
    pid_init(&z_pid,
             t->kp, t->ki, t->kd,
             t->integral_limit,
             t->a_z_min, t->a_z_max);
    z_pid_initialized = 1;
}

/**
 * @brief Run one control step: torque -> allocation -> thrust PID -> gimbal angles.
 * @param state  Estimated state.
 * @param ref    Desired attitude and z.
 * @param config Controller parameters.
 * @param out    Output tau_gim, tau_thrust, T_cmd, theta_x/y.
 * @param dt_s   Time step [s].
 */
void flight_controller_run(const state_t *state,
                           const flight_controller_ref_t *ref,
                           const flight_controller_config_t *config,
                           control_output_t *out,
                           float dt_s)
{
    if (!state || !ref || !config || !out) return;

    if (!z_pid_initialized) {
        flight_controller_init(config);
    }

    float tau_cmd[3];
    torque_module(state, ref, &config->attitude, tau_cmd);

    control_allocation(tau_cmd,
                      config->allocation.t_hat,
                      out->tau_gim,
                      &out->tau_thrust);

    float z = state->pos[2];
    float vz = state->vel[2];
    float T_cmd;
    (void)thrust_controller(ref->z_ref, ref->vz_ref, z, vz,
                           &config->thrust, dt_s, &T_cmd);
    out->T_cmd = T_cmd;

    gimbal_angles(out->tau_gim, out->T_cmd, &config->gimbal,
                  &out->theta_x_cmd, &out->theta_y_cmd);
}
