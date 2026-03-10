#include "motor_drivers/esc_driver.h"

#include <stddef.h>

static float clamp_f32(float x, float lo, float hi);
static uint16_t thrust_to_us(float thrust);

static esc_pair_t g_esc_pair;
static volatile bool g_esc_pair_ready = false;

/* ---- Init / arm / disarm ----------------------------------------------- */

void ESC_init(esc_t *esc, const pwm_output_t *pwm) {
    if (esc == NULL || pwm == NULL || pwm->htim == NULL) {
        return;
    }

    esc->pwm = *pwm;
    esc->desired_thrust = 0.0f;
    esc->desired_pulse_us = ESC_PWM_MIN_US;
    esc->desired_pulse_ticks = pwm_clamp_ticks(&esc->pwm, pwm_us_to_ticks(&esc->pwm, ESC_PWM_MIN_US));
    esc->update_divider_counter = 0U;
    esc->initialized = true;
    esc->armed = false;

    /* Set compare to min but do NOT start PWM — wait for arm. */
    pwm_set_compare(&esc->pwm, esc->desired_pulse_ticks);
}

void ESC_arm(esc_t *esc) {
    if (esc == NULL || !esc->initialized) {
        return;
    }
    (void)HAL_TIM_PWM_Start(esc->pwm.htim, esc->pwm.channel);
    esc->armed = true;
}

void ESC_disarm(esc_t *esc) {
    if (esc == NULL || !esc->initialized) {
        return;
    }
    esc->armed = false;
    esc->desired_thrust = 0.0f;
    esc->desired_pulse_us = ESC_PWM_MIN_US;
    esc->desired_pulse_ticks = pwm_clamp_ticks(&esc->pwm, pwm_us_to_ticks(&esc->pwm, ESC_PWM_MIN_US));
    pwm_set_compare(&esc->pwm, esc->desired_pulse_ticks);
    (void)HAL_TIM_PWM_Stop(esc->pwm.htim, esc->pwm.channel);
}

/* ---- Task-level API ---------------------------------------------------- */

void ESC_set_thrust(esc_t *esc, float thrust) {
    if (esc == NULL || !esc->initialized || !esc->armed) {
        return;
    }

    float clamped = clamp_f32(thrust, 0.0f, 1.0f);
    uint16_t pulse_us = thrust_to_us(clamped);
    uint32_t pulse_ticks = pwm_clamp_ticks(&esc->pwm, pwm_us_to_ticks(&esc->pwm, pulse_us));

    esc->desired_thrust = clamped;
    esc->desired_pulse_us = pulse_us;
    esc->desired_pulse_ticks = pulse_ticks;
}

/* ---- ISR-level API ----------------------------------------------------- */

void ESC_apply(esc_t *esc) {
    if (esc == NULL || !esc->initialized || !esc->armed) {
        return;
    }

    esc->update_divider_counter++;
    if (esc->update_divider_counter < ESC_ISR_TO_PWM_DIVIDER) {
        return;
    }
    esc->update_divider_counter = 0U;

    pwm_set_compare(&esc->pwm, esc->desired_pulse_ticks);
}

/* ---- Pair wrappers ----------------------------------------------------- */

void ESC_pair_init(const pwm_output_t *pwm1, const pwm_output_t *pwm2) {
    ESC_init(&g_esc_pair.esc1, pwm1);
    ESC_init(&g_esc_pair.esc2, pwm2);
    g_esc_pair_ready = true;
}

void ESC_pair_arm(void) {
    if (!g_esc_pair_ready) {
        return;
    }
    ESC_arm(&g_esc_pair.esc1);
    ESC_arm(&g_esc_pair.esc2);
}

void ESC_pair_disarm(void) {
    if (!g_esc_pair_ready) {
        return;
    }
    ESC_disarm(&g_esc_pair.esc1);
    ESC_disarm(&g_esc_pair.esc2);
}

void ESC_set_pair_thrust(float thrust1, float thrust2) {
    if (!g_esc_pair_ready) {
        return;
    }
    ESC_set_thrust(&g_esc_pair.esc1, thrust1);
    ESC_set_thrust(&g_esc_pair.esc2, thrust2);
}

void ESC_apply_pair(void) {
    if (!g_esc_pair_ready) {
        return;
    }
    ESC_apply(&g_esc_pair.esc1);
    ESC_apply(&g_esc_pair.esc2);
}

/* ---- Static helpers ---------------------------------------------------- */

static float clamp_f32(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static uint16_t thrust_to_us(float thrust) {
    float clamped = clamp_f32(thrust, 0.0f, 1.0f);
    float us = (float)ESC_PWM_MIN_US + clamped * (float)(ESC_PWM_MAX_US - ESC_PWM_MIN_US);
    return (uint16_t)(us + 0.5f);
}
