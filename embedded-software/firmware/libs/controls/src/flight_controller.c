/**
 * @file    flight_controller.c
 * @brief   13-equation TVC algorithm from Control System Architecture PDF.
 *
 * Pipeline implements PDF equations in execution order:
 *   Eq 1:    Quaternion error
 *   Eq 2:    Axis-angle error extraction
 *   Eq 10:   Gyroscopic torque
 *   Eq 11:   Command torque
 *   Eq 12:   Roll torque (projection onto thrust direction)
 *   Eq 13:   Gimbal torque
 *   Eq 3:    Perpendicular thrust
 *   Eq 4:    Parallel thrust
 *   Eq 5:    Desired thrust vector (inline)
 *   Eq 6-7:  Gimbal angles
 *   Eq 8:    Thrust direction update
 *   z-PID:   Altitude control (not part of PDF)
 *   Eq 9:    Thrust magnitude
 */

#include "controls/flight_controller.h"
#include "controls/pid.h"
#include "state_estimation/state.h"
#include <math.h>
#include <string.h>

#define MIN_DT_S 1.0e-6f /**< Min dt [s] to avoid div-by-zero in PID */

/* ── Static state persisting between calls ─────────────────────────────── */

static pid_controller_t z_pid;
static uint8_t z_pid_initialized = 0;
static float s_t_hat[3]; /**< Thrust direction (Eq 8) */
static float s_t_mag;    /**< Thrust magnitude (Eq 9) */


/* ── Utility ───────────────────────────────────────────────────────────── */

float clampf(float value, float min_val, float max_val)
{
    if (value > max_val)
        return max_val;
    if (value < min_val)
        return min_val;
    return value;
}

/* ── Eq 1: Quaternion error ────────────────────────────────────────────── */

/**
 * @brief q_err = q_des * conj(q_meas); negate if w < 0 for shortest path.
 */
void compute_quaternion_error(const quaternion_t *q_des,
                              const quaternion_t *q_meas,
                              quaternion_t *q_err)
{
    /* Both q_des and q_meas are body-to-nav quaternions.
     * Error = q_des * conj(q_meas): the rotation FROM current TO desired.
     * This matches the header comment and the spec when both quaternions
     * use the same convention. */
    quaternion_t q_meas_conj;
    quaternion_conjugate(q_meas, &q_meas_conj);
    quaternion_multiply(q_des, &q_meas_conj, q_err);

    // flip is w < 0 to ensure shortest rotation
    if (q_err->w < 0.0f)
    {
        q_err->w = -q_err->w;
        q_err->x = -q_err->x;
        q_err->y = -q_err->y;
        q_err->z = -q_err->z;
    }
}

/* ── Eq 2: Axis-angle error ────────────────────────────────────────────── */

/**
 * @brief phi = 2 * atan2(||v||, q0) * v / ||v||; exact axis-angle extraction.
 */
void compute_axis_angle_error(const quaternion_t *q_err, float phi[3])
{
    float vx = q_err->x;
    float vy = q_err->y;
    float vz = q_err->z;
    float v_norm = sqrtf(vx * vx + vy * vy + vz * vz);

    if (v_norm < 1.0e-9f)
    {
        phi[0] = 0.0f;
        phi[1] = 0.0f;
        phi[2] = 0.0f;
        return;
    }

    float angle = 2.0f * atan2f(v_norm, q_err->w);
    float scale = angle / v_norm;
    phi[0] = scale * vx;
    phi[1] = scale * vy;
    phi[2] = scale * vz;
}

/* ── Eq 10: Gyroscopic torque ──────────────────────────────────────────── */

/**
 * @brief Spec eq 3.6 feedforward term: ω × (I·ω).
 */
void compute_gyroscopic_torque(const float I[3][3],
                               const float omega[3],
                               float tau_gyro[3])
{
    float I_omega[3];
    matrix33_vec3_mul(I, omega, I_omega);
    vec3_cross(omega, I_omega, tau_gyro);
}

/* ── Eq 11: Command torque ─────────────────────────────────────────────── */

/**
 * @brief Spec eq 3.8: τ_cmd = -Kp·φ - Kd·ω + ω×(I·ω).
 */
void compute_command_torque(const float Kp[3][3],
                            const float Kd[3][3],
                            const float phi[3],
                            const float omega[3],
                            const float tau_gyro[3],
                            float tau_cmd[3])
{
    float Kp_phi[3], Kd_omega[3];
    matrix33_vec3_mul(Kp, phi, Kp_phi);
    matrix33_vec3_mul(Kd, omega, Kd_omega);

    tau_cmd[0] = -Kp_phi[0] - Kd_omega[0] + tau_gyro[0];
    tau_cmd[1] = -Kp_phi[1] - Kd_omega[1] + tau_gyro[1];
    tau_cmd[2] = -Kp_phi[2] - Kd_omega[2] + tau_gyro[2];
}

/* ── Eq 12: Roll torque (projection onto thrust direction) ─────────────── */

/**
 * @brief tau_roll = (tau_cmd . t_hat) * t_hat.
 */
void compute_roll_torque(const float tau_cmd[3],
                         const float t_hat[3],
                         float tau_roll[3])
{
    float proj = vec3_dot(tau_cmd, t_hat);
    tau_roll[0] = proj * t_hat[0];
    tau_roll[1] = proj * t_hat[1];
    tau_roll[2] = proj * t_hat[2];
}

/* ── Eq 13: Gimbal torque ──────────────────────────────────────────────── */

/**
 * @brief tau_gim = tau_cmd - tau_roll.
 */
void compute_gimbal_torque(const float tau_cmd[3],
                           const float tau_roll[3],
                           float tau_gim[3])
{
    tau_gim[0] = tau_cmd[0] - tau_roll[0];
    tau_gim[1] = tau_cmd[1] - tau_roll[1];
    tau_gim[2] = tau_cmd[2] - tau_roll[2];
}

/* ── Eq 3: Perpendicular thrust ────────────────────────────────────────── */

/**
 * @brief t_perp = (tau_gim x r_gim) / ||r_gim||^2.
 */
void compute_perpendicular_thrust(const float tau_gim[3],
                                  const float r_gim[3],
                                  float t_perp[3])
{
    float r_norm_sq = vec3_dot(r_gim, r_gim);
    if (r_norm_sq < 1.0e-12f)
    {
        t_perp[0] = 0.0f;
        t_perp[1] = 0.0f;
        t_perp[2] = 0.0f;
        return;
    }
    float cross[3];
    vec3_cross(tau_gim, r_gim, cross);
    float inv = 1.0f / r_norm_sq;
    t_perp[0] = cross[0] * inv;
    t_perp[1] = cross[1] * inv;
    t_perp[2] = cross[2] * inv;
}

/* ── Eq 4: Parallel thrust ─────────────────────────────────────────────── */

/**
 * @brief t_par = sqrt(||t||^2 - ||t_perp||^2) * r_hat_gim.
 *        Uses s_t_mag from previous iteration for ||t||.
 */
void compute_parallel_thrust(float t_mag,
                             const float t_perp[3],
                             const float r_gim[3],
                             float t_par[3])
{
    float r_norm_sq = vec3_dot(r_gim, r_gim);
    if (r_norm_sq < 1.0e-12f)
    {
        t_par[0] = 0.0f;
        t_par[1] = 0.0f;
        t_par[2] = 0.0f;
        return;
    }

    float t_perp_sq = vec3_dot(t_perp, t_perp);
    float diff = t_mag * t_mag - t_perp_sq;
    if (diff < 0.0f)
        diff = 0.0f;

    float par_mag = sqrtf(diff);
    float inv_r = 1.0f / sqrtf(r_norm_sq);

    t_par[0] = par_mag * r_gim[0] * inv_r;
    t_par[1] = par_mag * r_gim[1] * inv_r;
    t_par[2] = par_mag * r_gim[2] * inv_r;
}

/* ── Eq 6-7: Gimbal angles ─────────────────────────────────────────────── */

/**
 * @brief theta_x = -atan2(t_hat_des[1], t_hat_des[2]),
 *        theta_y = asin(t_hat_des[0]).
 *        Clamp to [theta_min, theta_max].
 */
void compute_gimbal_angles(const float t_des[3],
                           const flight_controller_gimbal_config_t *cfg,
                           float *theta_x_cmd,
                           float *theta_y_cmd)
{
    float t_norm = sqrtf(vec3_dot(t_des, t_des));
    if (t_norm < 1.0e-9f)
    {
        *theta_x_cmd = 0.0f;
        *theta_y_cmd = 0.0f;
        return;
    }

    float inv = 1.0f / t_norm;
    float tx = t_des[0] * inv;
    float ty = t_des[1] * inv;
    float tz = t_des[2] * inv;

    /* Spec eq 5.7: θx = atan2(ty, -tz), θy = -asin(tx)
     * with t̂ = [-sin(θy), sin(θx)cos(θy), -cos(θx)cos(θy)] (thrust in -z) */
    float theta_x = atan2f(ty, -tz);
    float theta_y = -asinf(clampf(tx, -1.0f, 1.0f));

    *theta_x_cmd = clampf(theta_x, cfg->theta_min, cfg->theta_max);
    *theta_y_cmd = clampf(theta_y, cfg->theta_min, cfg->theta_max);
}

/* ── Eq 8: Thrust direction update ─────────────────────────────────────── */

/**
 * @brief Spec eq 5.6: t̂ = Rx(θx)Ry(θy)(-k̂)
 *        = [-sin(θy), sin(θx)cos(θy), -cos(θx)cos(θy)].
 *        Thrust in -z body direction (z-up convention).
 */
void update_thrust_direction(float theta_x, float theta_y,
                             float t_hat[3])
{
    float sx = sinf(theta_x);
    float cx = cosf(theta_x);
    float sy = sinf(theta_y);
    float cy = cosf(theta_y);

    t_hat[0] = -sy;
    t_hat[1] = sx * cy;
    t_hat[2] = -cx * cy;
}

/* ── Eq 9: Thrust magnitude ────────────────────────────────────────────── */

/**
 * @brief T = -m * dot(a_des, t_hat); clamp >= 0.
 *
 * t_hat points in -z body (thrust downward), a_des points in +z nav
 * (upward for hover).  The dot product is negative when they oppose,
 * so we negate to obtain a positive thrust magnitude.
 */
float compute_thrust_magnitude(float m, const float a_des[3],
                               const float t_hat[3])
{
    float T = -m * vec3_dot(a_des, t_hat);
    if (T < 0.0f)
        T = 0.0f;
    return T;
}

/* ── Public API ────────────────────────────────────────────────────────── */

void flight_controller_init(const flight_controller_config_t *config)
{
    if (!config)
        return;

    const flight_controller_thrust_config_t *t = &config->thrust;
    pid_init(&z_pid,
             t->kp, t->ki, t->kd,
             t->integral_limit,
             t->a_z_min, t->a_z_max);
    z_pid_initialized = 1;

    s_t_hat[0] = config->allocation.t_hat[0];
    s_t_hat[1] = config->allocation.t_hat[1];
    s_t_hat[2] = config->allocation.t_hat[2];
    /* Bootstrap to hover thrust so the first compute_parallel_thrust call
     * produces a dominant z-component in t_des.  With s_t_mag=0, t_par=0
     * and t_des is purely perpendicular → gimbal saturates at ±90° →
     * thrust direction goes horizontal → T_cmd=0 → s_t_mag stays 0.
     * This vicious cycle never recovers. */
    s_t_mag = config->thrust.m * config->thrust.g;
}

void flight_controller_run(const state_t *state,
                           const flight_controller_ref_t *ref,
                           const flight_controller_config_t *config,
                           control_output_t *out,
                           float dt_s)
{
    if (!state || !ref || !config || !out)
        return;

    if (!z_pid_initialized)
    {
        flight_controller_init(config);
    }

    const flight_controller_thrust_config_t *tcfg = &config->thrust;
    const flight_controller_gimbal_config_t *gcfg = &config->gimbal;

    /* Construct r_gim pointing in -z (thrust direction) so that
     * compute_parallel_thrust produces t_par in -z, consistent with
     * the -z thrust convention used by update_thrust_direction. */
    float r_gim[3] = {0.0f, 0.0f, -gcfg->L};

    /* Eq 1: Quaternion error */
    quaternion_t q_err;
    compute_quaternion_error(&ref->q_ref, &state->q_bn, &q_err);

    /* Eq 2: Axis-angle error */
    float phi[3];
    compute_axis_angle_error(&q_err, phi);
    out->phi_x = phi[0];
    out->phi_y = phi[1];
    out->phi_z = phi[2];

    /* Eq 10: Gyroscopic torque */
    float tau_gyro[3];
    compute_gyroscopic_torque(config->attitude.I, state->omega_b, tau_gyro);

    /* Eq 11: Command torque (spec eq 3.8) */
    float tau_cmd[3];
    compute_command_torque(config->attitude.Kp, config->attitude.Kd,
                           phi, state->omega_b, tau_gyro, tau_cmd);

    /* Eq 12: Roll torque */
    float tau_roll[3];
    compute_roll_torque(tau_cmd, s_t_hat, tau_roll);

    /* Eq 13: Gimbal torque */
    compute_gimbal_torque(tau_cmd, tau_roll, out->tau_gim);

    /* Eq 3: Perpendicular thrust */
    float t_perp[3];
    compute_perpendicular_thrust(out->tau_gim, r_gim, t_perp);

    /* Eq 4: Parallel thrust */
    float t_par[3];
    compute_parallel_thrust(s_t_mag, t_perp, r_gim, t_par);

    /* Eq 5: Desired thrust vector */
    float t_des[3];
    t_des[0] = t_perp[0] + t_par[0];
    t_des[1] = t_perp[1] + t_par[1];
    t_des[2] = t_perp[2] + t_par[2];

    /* Eq 6-7: Gimbal angles */
    compute_gimbal_angles(t_des, gcfg, &out->theta_x_cmd, &out->theta_y_cmd);

    /* Eq 8: Update thrust direction */
    update_thrust_direction(out->theta_x_cmd, out->theta_y_cmd, s_t_hat);

    /* Z-PID: compute a_z_cmd */
    float a_z_cmd;
    if (dt_s < MIN_DT_S)
    {
        a_z_cmd = 0.0f;
    }
    else
    {
        a_z_cmd = pid_compute(&z_pid, ref->z_ref, state->pos[2], dt_s);
    }
    a_z_cmd = clampf(a_z_cmd, tcfg->a_z_min, tcfg->a_z_max);

    /* Desired acceleration vector */
    float a_des[3] = {0.0f, 0.0f, tcfg->g + a_z_cmd};

    /* Eq 9: Thrust magnitude */
    float T = compute_thrust_magnitude(tcfg->m, a_des, s_t_hat);
    T = clampf(T, tcfg->T_min, tcfg->T_max);
    out->T_cmd = T;
    s_t_mag = T;
    out->z_pid_integral = pid_get_integral(&z_pid);

    /* Roll torque scalar: projection of tau_cmd onto thrust direction */
    out->tau_thrust = vec3_dot(tau_cmd, s_t_hat);
}
