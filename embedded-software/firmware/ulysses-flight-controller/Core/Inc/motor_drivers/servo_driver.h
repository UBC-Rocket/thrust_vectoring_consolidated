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
/* Servo 1: TIM1 CH2 / PE11 (physical "Servo 2") */
#define SERVO1_US_MIN   900+710
#define SERVO1_US_MID  1125+710
#define SERVO1_US_MAX  1450+710
#define SERVO1_DEGREE_RANGE (55)

/* Servo 2: TIM3 CH3 / PB0 (physical "Servo 1") */
#define SERVO2_US_MIN  1575-600
#define SERVO2_US_MID  1860-600
#define SERVO2_US_MAX  2200-600
#define SERVO2_DEGREE_RANGE (52.5)

typedef struct {
    servo_t servo1;
    servo_t servo2;
} servo_pair_t;

void servo_init(servo_t *servo, const pwm_output_t *pwm);
void servo_init_with_deg_range(servo_t *servo, const pwm_output_t *pwm, float deg_range, float mid_pt);
void servo_enable(servo_t *servo, bool enable);
void servo_set_deg_range(servo_t *servo, float deg_range, float mid_pt);

/* Task-level: store desired angle (called from controls task). */
void set_servo_degree(servo_t *servo, float degree);
void set_servo_pair_degrees(float s1, float s2);

/* ISR-level: latch stored compare_val to hardware (called from TIM4 CH4). */
void apply_servo_pair_degrees(void);

void servo_pair_init(const pwm_output_t *pwm1, const pwm_output_t *pwm2);
void servo_pair_enable(bool enable);
