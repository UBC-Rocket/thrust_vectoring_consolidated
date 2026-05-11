#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "motor_drivers/pwm_output.h"

typedef struct {
    pwm_output_t pwm;

    /* Calibration parameters */
    uint16_t us_min;
    uint16_t us_mid;
    uint16_t us_max;

    float deg_range;
    float mid_pt;

    volatile uint32_t compare_val;

    uint16_t us_last;
    bool enabled;
} servo_t;


/* ── Per-servo pulse width calibration (microseconds) ───────────────── */
/*
 * Axis assignment (verified against schematic and physical labelling):
 *   servo1  — TIM1 CH2 / PE11  — controls gimbal Y-axis (pitch)
 *   servo2  — TIM3 CH3 / PB0   — controls gimbal X-axis (roll)
 *
 * Use set_gimbal_degrees(x_deg, y_deg) rather than set_servo_pair_degrees
 * so that axis intent is explicit at the call site.
 */

/* Servo 1 (Y-axis / pitch): TIM1 CH2 / PE11 */
#define SERVO1_US_MIN   900
#define SERVO1_US_MID  1125
#define SERVO1_US_MAX  1450

/* Servo 2 (X-axis / roll): TIM3 CH3 / PB0 */
#define SERVO2_US_MIN  1575
#define SERVO2_US_MID  1860
#define SERVO2_US_MAX  2200

/* ── Gimbal angular limits ───────────────────────────────────────────── */
#define SERVO_GIMBAL_RANGE_DEG      60.0f  /**< Full range: ±30 ° each side */
#define SERVO_GIMBAL_HALF_RANGE_DEG 30.0f  /**< Clamp limit applied to both axes */

typedef struct {
    servo_t servo1;  /**< Y-axis (pitch) servo */
    servo_t servo2;  /**< X-axis (roll) servo */
} servo_pair_t;

void servo_init(servo_t *servo, const pwm_output_t *pwm);
void servo_init_with_deg_range(servo_t *servo, const pwm_output_t *pwm, float deg_range, float mid_pt);
void servo_enable(servo_t *servo, bool enable);
void servo_set_deg_range(servo_t *servo, float deg_range, float mid_pt);

/* Task-level: store desired angle (called from controls task). */
void set_servo_degree(servo_t *servo, float degree);
void set_servo_pair_degrees(float s1, float s2);

/**
 * @brief Command gimbal angles by physical axis.
 * Handles servo axis mapping and sign convention internally.
 * @param x_deg  Desired X-axis (roll) gimbal angle in degrees, clamped to ±SERVO_GIMBAL_HALF_RANGE_DEG.
 * @param y_deg  Desired Y-axis (pitch) gimbal angle in degrees, clamped to ±SERVO_GIMBAL_HALF_RANGE_DEG.
 */
void set_gimbal_degrees(float x_deg, float y_deg);

/* ISR-level: latch stored compare_val to hardware (called from TIM4 CH4). */
void apply_servo_pair_degrees(void);

void servo_pair_init(const pwm_output_t *pwm1, const pwm_output_t *pwm2);
void servo_pair_enable(bool enable);
