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

    float desired_thrust;
    uint16_t desired_pulse_us;
    volatile uint32_t desired_pulse_ticks;

    uint8_t update_divider_counter;
    bool initialized;
    bool armed;
} esc_t;

typedef struct {
    esc_t esc1;
    esc_t esc2;
} esc_pair_t;

void ESC_init(esc_t *esc, const pwm_output_t *pwm);
void ESC_arm(esc_t *esc);
void ESC_disarm(esc_t *esc);
void ESC_set_thrust(esc_t *esc, float thrust);
void ESC_apply(esc_t *esc);

void ESC_pair_init(const pwm_output_t *pwm1, const pwm_output_t *pwm2);
void ESC_pair_arm(void);
void ESC_pair_disarm(void);
void ESC_set_pair_thrust(float thrust1, float thrust2);
void ESC_apply_pair(void);
