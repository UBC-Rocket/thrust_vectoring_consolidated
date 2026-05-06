/**
 * @file    flight_controller.h
 * @brief   Flight control body: torque, allocation, thrust PID, gimbal angles.
 *
 * Implements (main.tex): body torque control, control allocation, thrust
 * control (1D PID on z), gimbal angle computation. No Kalman or ESC/servo I/O.
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
    float Ki[3][3];
    float Kd[3][3];  /**< Derivative gain: angular rate -> torque [N·m·s/rad] */
    float I[3][3];   /**< Inertia matrix body frame [kg·m²] */
    // float x_integral_limit;
    // float y_integral_limit;
    // float z_integral_limit;
    // float x_torque_min;
    // float x_torque_max;
    // float y_torque_min;
    // float y_torque_max;
    // float z_torque_min;
    // float z_torque_max;
} flight_controller_attitude_config_t;

/** Allocation: unit thrust direction in body frame (Section 4). */
typedef struct {
    float thrust_dir[3];  /**< Unit thrust direction (e.g. [0,0,-1] for z-up) */
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
    float tau_gim[3];      /**< Torque for gimbal [N·m] */
    float tau_thrust;      /**< Differential torque from props [N·m] */
    float T_cmd;           /**< Thrust magnitude [N] */
    float theta_x_cmd;     /**< Gimbal angle x [rad] */
    float theta_y_cmd;     /**< Gimbal angle y [rad] */
    float phi_x;           /**< Attitude error vector x = 2*dq.x [rad] */
    float phi_y;           /**< Attitude error vector y = 2*dq.y [rad] */
    float phi_z;           /**< Attitude error vector z = 2*dq.z [rad] */
    float z_pid_integral;  /**< Z-axis PID integral accumulator [m] */
} control_output_t;

/**
 * @brief Initialize flight controller internal state (e.g. z-PID).
 * Call once before first flight_controller_run, or when re-arming.
 * @param config Controller config (thrust gains used for z-PID).
 */
void flight_controller_init(const flight_controller_config_t *config);

// /**
//  * @brief Set flight controller internal state. 
//  * Call to update config (e.g. thrust gains) 
//  * @param config Controller config (thrust gains used for z-PID).
//  */
// void flight_controller_set_config(const flight_controller_config_t *config);

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

/**
 * @brief Reset flight controller internal state (e.g. z-PID integral).
 * 
 */
void flight_controller_reset(void)

#ifdef __cplusplus
}
#endif

#endif /* FLIGHT_CONTROLLER_H */
