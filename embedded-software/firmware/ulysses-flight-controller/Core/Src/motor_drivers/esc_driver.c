#include "motor_drivers/esc_driver.h"

#include <stddef.h>
#include <stdint.h>

#include "controls/pwm.h"
#include "motor_drivers/pwm_output.h"

static esc_pair_t g_esc_pair;

static void esc_init(esc_t *esc, const pwm_output_t *pwm);
static void esc_arm(esc_t *esc);
static void esc_disarm(esc_t *esc);
static void esc_set_pwm_pulse_us(esc_t *esc, uint16_t pulse_us);
static uint32_t esc_pwm_us_to_ticks(esc_t *esc, uint16_t pulse_us);
static void esc_apply(esc_t *esc);

/* ---- Single ESC API ----------------------------------------------- */

static void esc_init(esc_t *esc, const pwm_output_t *pwm)
{
    if (esc == NULL || pwm == NULL || pwm->htim == NULL) {
        return;
    }

    esc->pwm = *pwm;
    esc->desired_pulse_us = ESC_PWM_MIN_US;
    esc->desired_pulse_ticks = esc_pwm_us_to_ticks(esc, esc->desired_pulse_us);
    esc->update_divider_counter = 0U;
    esc->initialized = true;
    esc->armed = false;

    /* Set compare to min but do NOT start PWM — wait for arm. */
    pwm_set_compare(&esc->pwm, esc->desired_pulse_ticks);
}

static void esc_arm(esc_t *esc)
{
    if (esc == NULL || !esc->initialized) {
        return;
    }

    esc->armed = true;

    (void)HAL_TIM_PWM_Start(esc->pwm.htim, esc->pwm.channel);
}

static void esc_disarm(esc_t *esc)
{
    if (esc == NULL || !esc->initialized) {
        return;
    }

    esc->armed = false;
    esc->desired_pulse_us = ESC_PWM_MIN_US;
    esc->desired_pulse_ticks = esc_pwm_us_to_ticks(esc, esc->desired_pulse_us);

    pwm_set_compare(&esc->pwm, esc->desired_pulse_ticks);

    (void)HAL_TIM_PWM_Stop(esc->pwm.htim, esc->pwm.channel);
}

static void esc_set_pwm_pulse_us(esc_t *esc, uint16_t pulse_us)
{
    if (esc == NULL || !esc->initialized || !esc->armed) {
        return;
    }

    uint32_t pulse_ticks = esc_pwm_us_to_ticks(esc, pulse_us);

    esc->desired_pulse_us = pulse_us;
    esc->desired_pulse_ticks = pulse_ticks;
}

static uint32_t esc_pwm_us_to_ticks(esc_t *esc, uint16_t pulse_us)
{
    return pwm_clamp_ticks(&esc->pwm, pwm_us_to_ticks(&esc->pwm, pulse_us));
}

/* ---- ISR-level API ----------------------------------------------------- */

static void esc_apply(esc_t *esc)
{
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

/* ---- ESC Pair API ----------------------------------------------------- */

void esc_pair_init(const pwm_output_t *pwm_upper, const pwm_output_t *pwm_lower)
{
    esc_init(&g_esc_pair.esc_upper, pwm_upper);
    esc_init(&g_esc_pair.esc_lower, pwm_lower);

    g_esc_pair.thrust = 0.0f;
    g_esc_pair.torque = 0.0f;

    g_esc_pair.initialized = true;
}

void esc_pair_arm(void)
{
    if (!g_esc_pair.initialized) {
        return;
    }

    esc_arm(&g_esc_pair.esc_upper);
    esc_arm(&g_esc_pair.esc_lower);
}

void esc_pair_disarm(void)
{
    if (!g_esc_pair.initialized) {
        return;
    }

    esc_disarm(&g_esc_pair.esc_upper);
    esc_disarm(&g_esc_pair.esc_lower);
}

void esc_pair_set_force(float thrust, float torque)
{
    if (!g_esc_pair.initialized) {
        return;
    }

    g_esc_pair.thrust = thrust;
    g_esc_pair.torque = torque;

    pwm_setpoint_t pulses = pwm_setpoint_from_forces(thrust, torque);

    esc_set_pwm_pulse_us(&g_esc_pair.esc_upper, pulses.upper_motor_us);
    esc_set_pwm_pulse_us(&g_esc_pair.esc_lower, pulses.lower_motor_us);
}

void esc_pair_apply(void)
{
    if (!g_esc_pair.initialized) {
        return;
    }

    esc_apply(&g_esc_pair.esc_upper);
    esc_apply(&g_esc_pair.esc_lower);
}
