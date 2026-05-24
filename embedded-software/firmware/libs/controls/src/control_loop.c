/**
 * @file    control_loop.c
 * @brief   Simple two-axis tilt-balancing controller for thrust-vector control.
 *
 * Algorithm (per axis, x and y independently):
 *   1. Extract tilt angle phi from body-to-nav quaternion q_bn using the
 *      axis-angle decomposition: phi = 2 * atan2(||v||, q.w) * v / ||v||.
 *      phi[0] is tilt about body-x, phi[1] about body-y.
 *      phi[2] (yaw / spin about thrust axis) is discarded — unactuatable.
 *   2. Run a PID with setpoint = 0 and measurement = phi[axis].
 *   3. The PID output is the gimbal angle command [rad] for that axis,
 *      clamped to [theta_min_rad, theta_max_rad] by the PID output limits.
 */

#include "controls/control_loop.h"
#include "controls/pid.h"
#include "state_estimation/state.h"

#include <math.h>
#include <stddef.h>

#define MIN_DT_S 1.0e-6f  /**< Minimum dt [s] to guard against div-by-zero */

static pid_controller_t s_pid_x;
static pid_controller_t s_pid_y;

void control_loop_init(const control_loop_config_t *cfg)
{
    if (!cfg)
        return;

    pid_init(&s_pid_x,
             cfg->x.kp, cfg->x.ki, cfg->x.kd,
             cfg->x.integral_limit,
             cfg->x.theta_min_rad, cfg->x.theta_max_rad);

    pid_init(&s_pid_y,
             cfg->y.kp, cfg->y.ki, cfg->y.kd,
             cfg->y.integral_limit,
             cfg->y.theta_min_rad, cfg->y.theta_max_rad);
}

void control_loop_run(const state_t *state,
                      const control_loop_config_t *cfg,
                      control_loop_output_t *out,
                      float dt_s)
{
    if (!state || !cfg || !out)
        return;

    /* --- Step 1: extract tilt angles from q_bn via axis-angle ---
     *
     * q_bn rotates vectors from body frame to nav frame.
     * The "tilt error" that needs to be corrected is the rotation from nav
     * (upright reference) to body, which is conj(q_bn).  Taking the
     * axis-angle of conj(q_bn) gives the rotation needed to return the
     * body to upright:
     *   q_err = q_identity * conj(q_bn) = conj(q_bn)
     *
     * axis-angle: angle = 2 * atan2(||v||, w),  axis = v / ||v||
     * phi = angle * axis
     */
    quaternion_t q_err;
    quaternion_conjugate(&state->q_bn, &q_err);

    float vx = q_err.x;
    float vy = q_err.y;
    float v_norm = sqrtf(vx * vx + vy * vy + q_err.z * q_err.z);

    float phi_x, phi_y;
    if (v_norm < 1.0e-9f) {
        phi_x = 0.0f;
        phi_y = 0.0f;
    } else {
        float angle = 2.0f * atan2f(v_norm, q_err.w);
        float scale = angle / v_norm;
        phi_x = scale * vx;
        phi_y = scale * vy;
    }

    out->phi_x = phi_x;
    out->phi_y = phi_y;

    /* --- Step 2: PID — setpoint = 0 (upright), measurement = tilt --- */
    if (dt_s < MIN_DT_S) {
        out->theta_x_cmd = 0.0f;
        out->theta_y_cmd = 0.0f;
        return;
    }

    out->theta_x_cmd = pid_compute(&s_pid_x, 0.0f, cfg->x.sign * phi_x, dt_s);
    out->theta_y_cmd = pid_compute(&s_pid_y, 0.0f, cfg->y.sign * phi_y, dt_s);
}
