#include "motor_drivers/esc_driver.h"
#include "controls/pwm.h"
#include <stddef.h>

static float clamp_f32(float x, float lo, float hi);

static esc_pair_t g_esc_pair;
static volatile bool g_esc_pair_ready = false;

/* ---- Init / arm / disarm ----------------------------------------------- */

static void esc_init(esc_t *esc, const pwm_output_t *pwm) {
    if (esc == NULL || pwm == NULL || pwm->htim == NULL) {
        return;
    }

    esc->pwm = *pwm;
    esc->desired_pulse_us = ESC_PWM_MIN_US;
    esc->desired_pulse_ticks = pwm_clamp_ticks(&esc->pwm, pwm_us_to_ticks(&esc->pwm, ESC_PWM_MIN_US));
    esc->update_divider_counter = 0U;
    esc->initialized = true;
    esc->armed = false;

    /* Set compare to min but do NOT start PWM — wait for arm. */
    pwm_set_compare(&esc->pwm, esc->desired_pulse_ticks);
}

void esc_pair_init(const pwm_output_t *pwm1, const pwm_output_t *pwm2) {
    esc_init(&g_esc_pair.esc1, pwm1);
    esc_init(&g_esc_pair.esc2, pwm2);
    g_esc_pair.thrust = 0.0f;
    g_esc_pair.torque = 0.0f;
}

void esc_arm(esc_t *esc) {
    if (esc == NULL || !esc->initialized) {
        return;
    }
    (void)HAL_TIM_PWM_Start(esc->pwm.htim, esc->pwm.channel);
    esc->armed = true;
}

void esc_disarm(esc_t *esc) {
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

void esc_set_thrust_torque(esc_t *esc, float thrust) {
    if (esc == NULL || !esc->initialized || !esc->armed) {
        return;
    }

    uint16_t pulse_us = thrust_to_us(clamped);
    uint32_t pulse_ticks = pwm_clamp_ticks(&esc->pwm, pwm_us_to_ticks(&esc->pwm, pulse_us));

    esc->desired_thrust = clamped;
    esc->desired_pulse_us = pulse_us;
    esc->desired_pulse_ticks = pulse_ticks;
}

/* ---- ISR-level API ----------------------------------------------------- */

void esc_apply(esc_t *esc) {
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

void esc_pair_arm(void) {
    if (!g_esc_pair_ready) {
        return;
    }
    esc_arm(&g_esc_pair.esc1);
    esc_arm(&g_esc_pair.esc2);
}

void esc_pair_disarm(void) {
    if (!g_esc_pair_ready) {
        return;
    }
    esc_disarm(&g_esc_pair.esc1);
    esc_disarm(&g_esc_pair.esc2);
}

void esc_set_pair_thrust(float thrust1, float thrust2) {
    if (!g_esc_pair_ready) {
        return;
    }
    esc_set_thrust(&g_esc_pair.esc1, thrust1);
    esc_set_thrust(&g_esc_pair.esc2, thrust2);
}

void esc_apply_pair(void) {
    if (!g_esc_pair_ready) {
        return;
    }
    esc_apply(&g_esc_pair.esc1);
    esc_apply(&g_esc_pair.esc2);
}

/* ---- Static helpers ---------------------------------------------------- */

static float clamp_f32(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}
