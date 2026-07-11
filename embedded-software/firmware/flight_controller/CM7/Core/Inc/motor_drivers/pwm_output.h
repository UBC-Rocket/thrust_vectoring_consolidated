/**
 * @file    pwm_output.h
 * @brief   Generic HAL-timer PWM output helper. Ported from
 *          depracated/ulysses-flight-controller/Core/Inc/motor_drivers/
 *          pwm_output.h (STM32H5 -> STM32H7; logic unchanged).
 *
 * UBC Rocket, 2026
 */
#pragma once

#include "stm32h7xx_hal.h"
#include <stdint.h>

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    uint32_t timer_hz;
    uint32_t period_ticks;
} pwm_output_t;

uint32_t pwm_us_to_ticks(const pwm_output_t *pwm, uint16_t us);
uint32_t pwm_clamp_ticks(const pwm_output_t *pwm, uint32_t ticks);
void     pwm_set_compare(const pwm_output_t *pwm, uint32_t ticks);
