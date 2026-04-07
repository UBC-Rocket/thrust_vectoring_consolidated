/**
 * @file    flight_controller.c
 * @brief   Ulysses Phase 1 Proposal v2 — four-module flight control pipeline.
 *
 * Pipeline (per K. Lew, Jan 2026):
 *   §3 Torque Module          : tau_cmd = -Kp*phi - Kd*omega + omega x (I omega)
 *   §4 Torque Decomposition   : tau_z = tau_cmd . k_hat
 *                               tau_thrust = tau_z / t_z
 *                               tau_gim = tau_cmd - tau_thrust * t_hat
 *   §5 Gimbal Control         : Tx = tau_y/L, Ty = -tau_x/L
 *                               Tz = +sqrt(T^2 - Tx^2 - Ty^2)  (positive root,
 *                                 thrust along body +z at hover; matches Eq 5.5)
 *                               theta_x = -atan2(ty, tz), theta_y = arcsin(tx)
 *                                 (signs derived from Eq 5.6 with +k̂ instead of
 *                                 -k̂; Eq 5.6/5.7 in the PDF are inconsistent
 *                                 with Eq 5.5 — we take Eq 5.5 as the source
 *                                 of truth and re-derive the angle extraction.)
 *                               saturate to [theta_min, theta_max]
 *   §6 Thrust Control         : a_z_cmd = Kp*e + Ki*∫e + Kd*(vz_ref - vz)
 *                               T_cmd = m*(g + a_z_cmd), saturate
 *
 * All internal helpers are static — only flight_controller_init() and
 * flight_controller_run() have external linkage.
 */

#include "controls/flight_controller.h"
#include "controls/pid.h"
#include "state_estimation/state.h"

#include <math.h>
#include <string.h>

/** Min dt [s] to skip the z-PID step (avoids division by zero). */
#define MIN_DT_S 1.0e-6f

/** Tolerance below which a vector norm is considered zero. */
#define EPS_NORM 1.0e-9f

/* ── Static state persisting between calls ─────────────────────────────── */

static pid_controller_t s_z_pid;
static uint8_t s_z_pid_initialized = 0;

/** Current thrust direction in body frame (updated after gimbal control). */
static float s_t_hat[3] = {0.0f, 0.0f, 1.0f};

/* ── Utilities ─────────────────────────────────────────────────────────── */

static float clampf(float v, float lo, float hi)
{
    if (v > hi) return hi;
    if (v < lo) return lo;
    return v;
}

/* ── Module 1: Torque Module (PDF §3, Eq 3.8) ──────────────────────────── */

/**
 * @brief Compute the body torque command implementing the PD attitude law.
 *
 * Steps:
 *   1. Δq = q_ref ⊗ q_meas* ; flip if w<0 for shortest-path rotation.
 *   2. phi = exact axis-angle: 2*atan2(||v||, w) * v/||v||.
 *   3. tau_cmd = -Kp*phi - Kd*omega + omega x (I*omega).
 */
static void torque_module(const flight_controller_attitude_config_t *cfg,
                          const quaternion_t *q_ref,
                          const quaternion_t *q_meas,
                          const float omega[3],
                          float phi_out[3],
                          float tau_cmd_out[3])
{
    /* Eq 3.1: quaternion error */
    quaternion_t q_meas_conj;
    quaternion_conjugate(q_meas, &q_meas_conj);
    quaternion_t q_err;
    quaternion_multiply(q_ref, &q_meas_conj, &q_err);

    /* Shortest-path: if w<0, the rotation is >pi, flip the sign. */
    if (q_err.w < 0.0f)
    {
        q_err.w = -q_err.w;
        q_err.x = -q_err.x;
        q_err.y = -q_err.y;
        q_err.z = -q_err.z;
    }

    /* Eq 3.2 / 3.5: axis-angle error (exact form) */
    float vx = q_err.x, vy = q_err.y, vz = q_err.z;
    float v_norm = sqrtf(vx * vx + vy * vy + vz * vz);
    if (v_norm < EPS_NORM)
    {
        phi_out[0] = 0.0f;
        phi_out[1] = 0.0f;
        phi_out[2] = 0.0f;
    }
    else
    {
        float angle = 2.0f * atan2f(v_norm, q_err.w);
        float scale = angle / v_norm;
        phi_out[0] = scale * vx;
        phi_out[1] = scale * vy;
        phi_out[2] = scale * vz;
    }

    /* Kp * phi (full 3x3 matrix multiply) */
    float Kp_phi[3];
    matrix33_vec3_mul(cfg->Kp, phi_out, Kp_phi);

    /* Kd * omega */
    float Kd_w[3];
    matrix33_vec3_mul(cfg->Kd, omega, Kd_w);

    /* I * omega, then omega x (I*omega) — gyroscopic feedforward */
    float I_w[3];
    matrix33_vec3_mul(cfg->I, omega, I_w);
    float gyro[3];
    vec3_cross(omega, I_w, gyro);

    /* Eq 3.8: tau_cmd = -Kp*phi - Kd*omega + omega x (I*omega) */
    tau_cmd_out[0] = -Kp_phi[0] - Kd_w[0] + gyro[0];
    tau_cmd_out[1] = -Kp_phi[1] - Kd_w[1] + gyro[1];
    tau_cmd_out[2] = -Kp_phi[2] - Kd_w[2] + gyro[2];
}

/* ── Module 2: Torque Decomposition (PDF §4) ───────────────────────────── */

/**
 * @brief Split tau_cmd into a gimbal portion and a scalar differential-prop
 *        torque.
 *
 * Eq 4.2: tau_z = tau_cmd . k_hat (= tau_cmd[2])
 * Eq 4.3: tau_thrust = tau_z / t_z
 * Eq 4.4: tau_gim    = tau_cmd - tau_thrust * t_hat
 *
 * If t_z is too small (thrust direction nearly horizontal — degenerate), the
 * differential-prop branch can't authoritatively realize the axial torque, so
 * tau_thrust collapses to zero and the full tau_cmd is left to the gimbal.
 */
static void torque_decomposition(const float tau_cmd[3],
                                 const float t_hat[3],
                                 float tau_gim_out[3],
                                 float *tau_thrust_out)
{
    float t_z = t_hat[2];
    float tau_z = tau_cmd[2];

    float tau_thrust;
    if (fabsf(t_z) < EPS_NORM)
    {
        tau_thrust = 0.0f;
    }
    else
    {
        tau_thrust = tau_z / t_z;
    }

    tau_gim_out[0] = tau_cmd[0] - tau_thrust * t_hat[0];
    tau_gim_out[1] = tau_cmd[1] - tau_thrust * t_hat[1];
    tau_gim_out[2] = tau_cmd[2] - tau_thrust * t_hat[2];

    *tau_thrust_out = tau_thrust;
}

/* ── Module 3: Gimbal Control (PDF §5) ─────────────────────────────────── */

/**
 * @brief Map (tau_gim, T) to gimbal angles (theta_x, theta_y) and update the
 *        thrust direction t_hat.
 *
 * Eq 5.5: t_hat = [tau_y/(LT), -tau_x/(LT), sqrt(1 - tx^2 - ty^2)]
 *         (positive root, +body-z convention)
 * Eq 5.7 (re-derived for +k̂): theta_x = -atan2(ty, tz), theta_y = arcsin(tx)
 * Eq 5.10: clip(theta, theta_min, theta_max)
 *
 * The recomputed t_hat (from the saturated angles via Eq 5.6 with +k̂) is
 * written back to t_hat_out so the next iteration's torque decomposition uses
 * a consistent direction.
 */
static void gimbal_control(const flight_controller_gimbal_config_t *cfg,
                           const float tau_gim[3],
                           float T,
                           float *theta_x_out,
                           float *theta_y_out,
                           float t_hat_out[3])
{
    float L = cfg->L;

    /* Eq 5.5: build t_hat from gimbal torques. Both L and T must be > 0. */
    float tx, ty, tz;
    if (L > EPS_NORM && T > EPS_NORM)
    {
        float inv_LT = 1.0f / (L * T);
        tx =  tau_gim[1] * inv_LT;
        ty = -tau_gim[0] * inv_LT;
    }
    else
    {
        tx = 0.0f;
        ty = 0.0f;
    }

    /* Saturate the lateral components so 1 - tx^2 - ty^2 stays >= 0
       (corresponds to T_x^2 + T_y^2 <= T^2 in the PDF). */
    tx = clampf(tx, -1.0f, 1.0f);
    ty = clampf(ty, -1.0f, 1.0f);
    float lat_sq = tx * tx + ty * ty;
    if (lat_sq > 1.0f)
    {
        float scale = 1.0f / sqrtf(lat_sq);
        tx *= scale;
        ty *= scale;
        lat_sq = 1.0f;
    }
    tz = sqrtf(1.0f - lat_sq);  /* +z convention: t_hat ≈ +k̂ when ungimballed */

    /* Eq 5.7 (re-derived for +k̂): extract angles from t_hat (exact). */
    float theta_x = -atan2f(ty, tz);
    float theta_y = asinf(clampf(tx, -1.0f, 1.0f));

    /* Eq 5.10: saturate angles. */
    theta_x = clampf(theta_x, cfg->theta_min, cfg->theta_max);
    theta_y = clampf(theta_y, cfg->theta_min, cfg->theta_max);

    *theta_x_out = theta_x;
    *theta_y_out = theta_y;

    /* Eq 5.6: rebuild t_hat from the saturated angles so subsequent
       iterations decompose torque against the actual achievable direction. */
    float sx = sinf(theta_x), cx = cosf(theta_x);
    float sy = sinf(theta_y), cy = cosf(theta_y);
    t_hat_out[0] =  sy;
    t_hat_out[1] = -sx * cy;
    t_hat_out[2] =  cx * cy;
}

/* ── Module 4: Thrust Control (PDF §6) ─────────────────────────────────── */

/**
 * @brief 1D PID on z position -> thrust magnitude command (Eq 6.1–6.4).
 *
 *   e        = z_ref - z
 *   e_dot    = vz_ref - vz
 *   a_z_cmd  = Kp*e + Ki*∫e + Kd*e_dot   (clamped to [a_z_min, a_z_max])
 *   T_cmd    = m*(g + a_z_cmd)            (clamped to [T_min, T_max])
 */
static float thrust_control(const flight_controller_thrust_config_t *cfg,
                            float z_ref,
                            float vz_ref,
                            float z,
                            float vz,
                            float dt_s)
{
    float a_z_cmd;
    if (dt_s < MIN_DT_S)
    {
        a_z_cmd = 0.0f;
    }
    else
    {
        a_z_cmd = pid_compute_with_ref_deriv(&s_z_pid,
                                             z_ref, vz_ref,
                                             z, vz,
                                             dt_s);
    }
    a_z_cmd = clampf(a_z_cmd, cfg->a_z_min, cfg->a_z_max);

    float T_cmd = cfg->m * (cfg->g + a_z_cmd);
    T_cmd = clampf(T_cmd, cfg->T_min, cfg->T_max);
    return T_cmd;
}

/* ── Public API ────────────────────────────────────────────────────────── */

void flight_controller_init(const flight_controller_config_t *config)
{
    if (!config) return;

    const flight_controller_thrust_config_t *t = &config->thrust;
    pid_init(&s_z_pid,
             t->kp, t->ki, t->kd,
             t->integral_limit,
             t->a_z_min, t->a_z_max);
    s_z_pid_initialized = 1;

    s_t_hat[0] = config->allocation.t_hat[0];
    s_t_hat[1] = config->allocation.t_hat[1];
    s_t_hat[2] = config->allocation.t_hat[2];
}

void flight_controller_run(const state_t *state,
                           const flight_controller_ref_t *ref,
                           const flight_controller_config_t *config,
                           control_output_t *out,
                           float dt_s)
{
    if (!state || !ref || !config || !out) return;

    if (!s_z_pid_initialized)
    {
        flight_controller_init(config);
    }

    /* §3 Torque Module */
    torque_module(&config->attitude,
                  &ref->q_ref, &state->q_bn,
                  state->omega_b,
                  out->phi,
                  out->tau_cmd);

    /* §6 Thrust Control — needed before §5 to know T (actuator magnitude)
       used by gimbal control to convert torque to thrust direction. */
    out->T_cmd = thrust_control(&config->thrust,
                                ref->z_ref, ref->vz_ref,
                                state->pos[2], state->vel[2],
                                dt_s);
    out->z_pid_integral = pid_get_integral(&s_z_pid);

    /* §4 Torque Decomposition — uses the current (pre-update) thrust
       direction so the split is consistent with what the actuators are
       physically producing this cycle. */
    torque_decomposition(out->tau_cmd, s_t_hat,
                         out->tau_gim, &out->tau_thrust);

    /* §5 Gimbal Control — produces angles and updates s_t_hat for next
       iteration. */
    gimbal_control(&config->gimbal,
                   out->tau_gim, out->T_cmd,
                   &out->theta_x_cmd, &out->theta_y_cmd,
                   s_t_hat);
}
