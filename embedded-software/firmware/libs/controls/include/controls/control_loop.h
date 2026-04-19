/**
 * @file    control_loop.h
 * @brief   Simple tilt-balancing control loop for thrust-vector control.
 *
 * Takes body tilt (phi_x, phi_y) derived from the body-to-nav quaternion
 * and drives each axis through an independent PID, outputting gimbal angle
 * commands directly.  The z-axis (yaw / roll about thrust) is ignored because
 * the gimbal cannot actuate it.
 */

#ifndef CONTROL_LOOP_H
#define CONTROL_LOOP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "state_estimation/state.h"

/** PID gains and output limits for a single tilt axis. */
typedef struct {
    float kp;
    float ki;
    float kd;
    float integral_limit;   /**< Anti-windup limit [rad] */
    float theta_min_rad;    /**< Min gimbal angle output [rad] */
    float theta_max_rad;    /**< Max gimbal angle output [rad] */
    float sign;             /**< Measurement sign: +1.0f normal, -1.0f to flip axis */
} control_loop_axis_cfg_t;

/** Configuration for both tilt axes. */
typedef struct {
    control_loop_axis_cfg_t x;  /**< Body-x tilt (pitch) */
    control_loop_axis_cfg_t y;  /**< Body-y tilt (roll) */
} control_loop_config_t;

/** Outputs produced by one control loop step. */
typedef struct {
    float theta_x_cmd;  /**< Gimbal angle command about x [rad] */
    float theta_y_cmd;  /**< Gimbal angle command about y [rad] */
    float phi_x;        /**< Measured tilt error about x [rad] */
    float phi_y;        /**< Measured tilt error about y [rad] */
} control_loop_output_t;

/**
 * @brief Initialize (or re-initialize) the control loop PIDs.
 *
 * Must be called once before the first control_loop_run, and again on each
 * arm event so that integral state is cleared.
 *
 * @param cfg  Pointer to axis configuration (both x and y).
 */
void control_loop_init(const control_loop_config_t *cfg);

/**
 * @brief Run one control loop step.
 *
 * Extracts body tilt angles from state->q_bn via axis-angle decomposition,
 * runs two independent PID controllers (setpoint = 0), and writes gimbal
 * angle commands to out.
 *
 * @param state  Current estimated state (q_bn used; other fields ignored).
 * @param cfg    Axis configuration (gains, limits).
 * @param out    Output: gimbal angle commands and tilt error values.
 * @param dt_s   Time step since last call [s].
 */
void control_loop_run(const state_t *state,
                      const control_loop_config_t *cfg,
                      control_loop_output_t *out,
                      float dt_s);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_LOOP_H */
