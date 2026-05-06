/**
 ******************************************************************************
 * @file    pid.c
 * @brief   PID Controller Implementation for Drone Flight Control
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "controls/pid.h"
#include "clamp.h"

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  Initialize PID controller with gains and limits
 */
void pid_init(pid_controller_t *pid,
              float kp,
              float ki,
              float kd,
              float integral_limit,
              float output_min,
              float output_max) {
  /* Set tuning gains */
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;

  /* Initialize state to zero */
  pid->integral_sum = 0.0f;
  pid->prev_measurement = 0.0f;
  pid->dt = 0.0f;

  /* Set limits */
  pid->integral_limit = integral_limit;
  pid->output_min = output_min;
  pid->output_max = output_max;
}

/**
 * @brief  Compute PID output using derivative-on-measurement algorithm
 */
float pid_compute(pid_controller_t *pid,
                  float setpoint,
                  float measurement,
                  float dt) {
  float error;
  float proportional;
  float integral;
  float derivative;
  float output;

  /* Store dt for reference */
  pid->dt = dt;

  /* Calculate error: how far we are from desired setpoint */
  error = setpoint - measurement;

  /* --- PROPORTIONAL TERM --- */
  /* Responds immediately to current error */
  proportional = pid->kp * error;

  /* --- INTEGRAL TERM --- */
  /* Accumulates error over time to eliminate steady-state error */
  /* Integrate: sum up error over time */
  pid->integral_sum += error * dt;

  /* Apply anti-windup: clamp integral to prevent overflow */
  /* This prevents the integral from growing unbounded when output is saturated */
  pid->integral_sum =
      clamp_float(pid->integral_sum, -pid->integral_limit, pid->integral_limit);

  /* Calculate integral contribution */
  integral = pid->ki * pid->integral_sum;

  /* --- DERIVATIVE TERM --- */
  derivative = pid->kd * v_err;

  /* Store current measurement for next iteration */
  pid->prev_measurement = measurement;

  /* --- COMPUTE OUTPUT --- */
  /* Combine all three terms */
  output = proportional + integral + derivative;

  /* Apply output limits to ensure actuators stay within safe range */
  output = clamp_float(output, pid->output_min, pid->output_max);

  return output;
}

float pid_compute(pid_controller_t *pid,
    float x_ref,
    float v_ref,
    float x,
    float v,
    float dt) 
{
  float x_err, v_err;
  float proportional;
  float integral;
  float derivative;
  float output;

  /* Store dt for reference */
  pid->dt = dt;

  /* Calculate error: how far we are from desired setpoint */
  x_err = x_ref - x;
  v_err = v_ref - v;

  /* --- PROPORTIONAL TERM --- */
  /* Responds immediately to current error */
  proportional = pid->kp * x_err;

  /* --- INTEGRAL TERM --- */
  /* Accumulates error over time to eliminate steady-state error */
  /* Integrate: sum up error over time */
  pid->integral_sum += x_err * dt;

  /* Apply anti-windup: clamp integral to prevent overflow */
  /* This prevents the integral from growing unbounded when output is saturated */
  pid->integral_sum =
      clamp_float(pid->integral_sum, -pid->integral_limit, pid->integral_limit);

  /* Calculate integral contribution */
  integral = pid->ki * pid->integral_sum;

  /* --- DERIVATIVE TERM --- */
  /* Derivative on measurement (not error) to prevent derivative kick */
  /* When setpoint changes suddenly, derivative of error would spike */
  /* Using derivative of measurement avoids this problem */
  /* Note: Negative sign because we want to oppose rapid changes in measurement */
  derivative = -pid->kd * (measurement - pid->prev_measurement) / dt;

  /* Store current measurement for next iteration */
  pid->prev_measurement = measurement;

  /* --- COMPUTE OUTPUT --- */
  /* Combine all three terms */
  output = proportional + integral + derivative;

  /* Apply output limits to ensure actuators stay within safe range */
  output = clamp_float(output, pid->output_min, pid->output_max);

  return output;
}

/**
 * @brief  Reset PID controller internal state
 */
void pid_reset(pid_controller_t *pid) {
  /* Clear integral accumulator */
  pid->integral_sum = 0.0f;

  /* Reset previous measurement to avoid derivative spike */
  pid->prev_measurement = 0.0f;

  /* Reset dt */
  pid->dt = 0.0f;
}

/**
 * @brief  Update PID gains at runtime
 */
void pid_set_gains(pid_controller_t *pid, float kp, float ki, float kd) {
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
}

/**
 * @brief  Update output and integral limits at runtime
 */
void pid_set_limits(pid_controller_t *pid,
                    float integral_limit,
                    float output_min,
                    float output_max) {
  /* Update limits */
  pid->integral_limit = integral_limit;
  pid->output_min = output_min;
  pid->output_max = output_max;

  /* Clamp existing integral to new limits */
  pid->integral_sum = clamp_float(pid->integral_sum, -integral_limit, integral_limit);
}

/**
 * @brief  Get current integral term value
 */
float pid_get_integral(pid_controller_t *pid) {
  return pid->integral_sum;
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Clamp a value between min and max
 * @param  value: Value to clamp
 * @param  min_val: Minimum allowed value
 * @param  max_val: Maximum allowed value
 * @retval Clamped value
 */
static float clamp_float(float value, float min_val, float max_val) {
  if (value > max_val) {
    return max_val;
  } else if (value < min_val) {
    return min_val;
  }
  return value;
}
