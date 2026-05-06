/**
 ******************************************************************************
 * @file    pid.h
 * @brief   PID Controller Library for Drone Flight Control
 ******************************************************************************
 * @attention
 *
 * This PID controller is designed for drone rate control (roll, pitch, yaw).
 * It implements:
 *   - Derivative on measurement (prevents derivative kick on setpoint changes)
 *   - Integral anti-windup (prevents integral term overflow)
 *   - Output limiting (constrains control output)
 *
 * Usage Example:
 * @code
 * // Initialize PID controller for roll axis
 * pid_controller_t roll_pid;
 * pid_init(&roll_pid, 1.5f, 0.5f, 0.05f, 100.0f, -500.0f, 500.0f);
 *
 * // In your main flight control loop:
 * float dt = 0.002f;  // 2ms loop time (500Hz)
 * float desired_roll_rate = 45.0f;  // deg/s from pilot stick input
 * float current_gyro_rate = gyro.x;  // deg/s from IMU
 *
 * float motor_correction = pid_compute(&roll_pid, desired_roll_rate,
 *                                      current_gyro_rate, dt);
 * @endcode
 *
 ******************************************************************************
 */

#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/**
 * @brief PID Controller structure
 *
 * This structure holds all state and configuration for a single PID controller.
 * For drone control, you'll typically need three instances: roll, pitch, yaw.
 */
typedef struct {
  /* Tuning gains */
  float kp; /**< Proportional gain */
  float ki; /**< Integral gain */
  float kd; /**< Derivative gain */

  /* Internal state (do not modify directly) */
  float integral_sum;      /**< Accumulated integral term */
  float prev_measurement; /**< Previous measurement for derivative calculation */

  /* Limits */
  float integral_limit; /**< Maximum absolute value for integral term (anti-windup) */
  float output_min;    /**< Minimum output value */
  float output_max;    /**< Maximum output value */

  /* Configuration */
  float dt; /**< Last time step used (for reference) */

} pid_controller_t;

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  Initialize PID controller with gains and limits
 * @param  pid: Pointer to PID controller structure
 * @param  kp: Proportional gain
 * @param  ki: Integral gain
 * @param  kd: Derivative gain
 * @param  integral_limit: Maximum absolute value for integral term (anti-windup)
 * @param  output_min: Minimum output limit
 * @param  output_max: Maximum output limit
 * @retval None
 *
 * @note   After initialization, all internal state is zeroed. Call this once
 *         during setup for each PID controller instance.
 */
void pid_init(pid_controller_t *pid,
              float kp,
              float ki,
              float kd,
              float integral_limit,
              float output_min,
              float output_max);

/**
 * @brief  Compute PID output
 * @param  pid: Pointer to PID controller structure
 * @param  setpoint: Desired value (e.g., desired roll rate in deg/s)
 * @param  measurement: Current measured value (e.g., current gyro rate in deg/s)
 * @param  dt: Time step since last call in seconds (e.g., 0.002 for 500Hz)
 * @retval PID output value (constrained to output_min/output_max)
 *
 * @note   This function should be called at a consistent rate in your control loop.
 *         The dt parameter accounts for timing variations if needed.
 *
 * Algorithm:
 *   - error = setpoint - measurement
 *   - integral += error * dt (with anti-windup limiting)
 *   - derivative = -(measurement - prev_measurement) / dt  (derivative on measurement)
 *   - output = kp*error + ki*integral + kd*derivative
 *   - output is clamped to [output_min, output_max]
 */
float pid_compute(pid_controller_t *pid,
                  float setpoint,
                  float measurement,
                  float dt);

float pid_compute_pv(pid_controller_t *pid,
    float x_ref,
    float v_ref,
    float x,
    float v,
    float dt
);

/**
 * @brief  Reset PID controller state
 * @param  pid: Pointer to PID controller structure
 * @retval None
 *
 * @note   Clears integral accumulator and previous measurement.
 *         Use this when:
 *         - Switching flight modes
 *         - After a crash/disarm
 *         - When PID has been inactive
 */
void pid_reset(pid_controller_t *pid);

/**
 * @brief  Update PID gains at runtime
 * @param  pid: Pointer to PID controller structure
 * @param  kp: New proportional gain
 * @param  ki: New integral gain
 * @param  kd: New derivative gain
 * @retval None
 *
 * @note   Useful for in-flight tuning or adaptive control.
 *         Does not reset integral or other state.
 */
void pid_set_gains(pid_controller_t *pid, float kp, float ki, float kd);

/**
 * @brief  Update output and integral limits at runtime
 * @param  pid: Pointer to PID controller structure
 * @param  integral_limit: New maximum absolute value for integral term
 * @param  output_min: New minimum output limit
 * @param  output_max: New maximum output limit
 * @retval None
 *
 * @note   Immediately clamps existing integral sum if it exceeds new limits.
 */
void pid_set_limits(pid_controller_t *pid,
                    float integral_limit,
                    float output_min,
                    float output_max);

/**
 * @brief  Get current integral term value
 * @param  pid: Pointer to PID controller structure
 * @retval Current integral sum
 *
 * @note   Useful for debugging and monitoring controller state.
 */
float pid_get_integral(pid_controller_t *pid);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */
