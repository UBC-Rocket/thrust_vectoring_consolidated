#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "motor_drivers/pwm_output.h"

#define ESC_PWM_MIN_US 1000U
#define ESC_PWM_MAX_US 2000U
#define ESC_PWM_FREQ_HZ 400U
#define ESC_ISR_RATE_HZ 800U
#define ESC_ISR_TO_PWM_DIVIDER (ESC_ISR_RATE_HZ / ESC_PWM_FREQ_HZ)

typedef struct {
    pwm_output_t pwm;

    uint16_t desired_pulse_us;
    volatile uint32_t desired_pulse_ticks;

    uint8_t update_divider_counter;
    bool initialized;
    bool armed;
} esc_t;

typedef struct {
    bool initialized;

    esc_t esc_upper;
    esc_t esc_lower;

    float thrust;
    float torque;
} esc_pair_t;

void esc_pair_init(const pwm_output_t *pwm_upper, const pwm_output_t *pwm_lower);
void esc_pair_arm(void);
void esc_pair_disarm(void);
void esc_pair_set_force(float thrust, float torque);
void esc_pair_apply(void);
