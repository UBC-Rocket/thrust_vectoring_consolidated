/**
 * @file    flight_controller.h
 * @brief   Flight control body: torque, decomposition, gimbal control, thrust.
 *
 * Implements the four-module pipeline from Ulysses Phase 1 Proposal v2
 * (Kenneth Lew, Jan 2026):
 *   §3 Torque Module          : tau_cmd = -Kp*phi - Kd*omega + omega x (I omega)
 *   §4 Torque Decomposition   : split tau_cmd into gimbal vs differential prop
 *   §5 Gimbal Control         : map (tau_gim, T) to servo angles (theta_x, theta_y)
 *   §6 Thrust Control         : 1D PID on z position -> thrust magnitude command
 *
 * Convention: z-axis up. All quaternions unit. q_bn maps body -> nav.
 */

#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "state_estimation/state.h"

/** Attitude gains and inertia (Section 3). */
typedef struct {
    float Kp[3][3];  /**< Proportional gain: attitude error -> torque [N·m/rad] */
    float Kd[3][3];  /**< Derivative gain: angular rate -> torque [N·m·s/rad] */
    float I[3][3];   /**< Inertia matrix body frame [kg·m²] */
} flight_controller_attitude_config_t;

/** Allocation: unit thrust direction in body frame (Section 4). */
typedef struct {
    float t_hat[3];  /**< Unit thrust direction (e.g. [0,0,-1] for z-up) */
} flight_controller_allocation_config_t;

/** Gimbal geometry and angle limits (Section 5). */
typedef struct {
    float L;          /**< Arm length [m] */
    float theta_min;  /**< Min gimbal angle [rad] */
    float theta_max;  /**< Max gimbal angle [rad] */
} flight_controller_gimbal_config_t;

/** Thrust PID and limits (Section 6). */
typedef struct {
    float m;               /**< Vehicle mass [kg] */
    float g;               /**< Gravity [m/s²] */
    float T_min;           /**< Min thrust [N] */
    float T_max;           /**< Max thrust [N] */
    float kp;              /**< Z position PID Kp */
    float ki;              /**< Z position PID Ki */
    float kd;              /**< Z position PID Kd */
    float integral_limit;  /**< Anti-windup limit for z integral */
    float a_z_min;         /**< Min vertical accel cmd [m/s²] */
    float a_z_max;         /**< Max vertical accel cmd [m/s²] */
} flight_controller_thrust_config_t;

/** Full controller configuration. */
typedef struct {
    flight_controller_attitude_config_t attitude;
    flight_controller_allocation_config_t allocation;
    flight_controller_gimbal_config_t gimbal;
    flight_controller_thrust_config_t thrust;
} flight_controller_config_t;

/** Reference: desired attitude (body-to-nav) and z setpoints. */
typedef struct {
    quaternion_t q_ref;  /**< Desired body-to-nav (unit); error = q_ref * conj(q_bn) */
    float z_ref;         /**< Desired z [m] */
    float vz_ref;       /**< Desired z velocity [m/s] */
} flight_controller_ref_t;

/** Controller output for downstream ESC/gimbal actuators and logging. */
typedef struct {
    float tau_cmd[3];      /**< Total commanded body torque (post PD law) [N·m] */
    float tau_gim[3];      /**< Torque routed to gimbal [N·m] */
    float tau_thrust;      /**< Scalar differential-prop torque (axial) [N·m] */
    float T_cmd;           /**< Thrust magnitude command [N] */
    float theta_x_cmd;     /**< Gimbal angle about body x [rad] */
    float theta_y_cmd;     /**< Gimbal angle about body y [rad] */
    float phi[3];          /**< Axis-angle attitude error [rad] (debug) */
    float z_pid_integral;  /**< Z-axis PID integral accumulator (debug) */
} control_output_t;

/**
 * @brief Initialize flight controller internal state (e.g. z-PID).
 * Call once before first flight_controller_run, or when re-arming.
 * @param config Controller config (thrust gains used for z-PID).
 */
void flight_controller_init(const flight_controller_config_t *config);

/**
 * @brief Run one step: torque → allocation → thrust PID → gimbal angles.
 * @param state  Current estimated state (from state_exchange).
 * @param ref    Desired attitude and z setpoints.
 * @param config Controller parameters.
 * @param out    Output: tau_gim, tau_thrust, T_cmd, theta_x_cmd, theta_y_cmd.
 * @param dt_s   Time step since last call [s].
 */
void flight_controller_run(const state_t *state,
                           const flight_controller_ref_t *ref,
                           const flight_controller_config_t *config,
                           control_output_t *out,
                           float dt_s);

#ifdef __cplusplus
}
#endif

#endif /* FLIGHT_CONTROLLER_H */
