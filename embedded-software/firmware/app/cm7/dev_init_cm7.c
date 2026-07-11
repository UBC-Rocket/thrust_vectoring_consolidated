/**
 * @file    dev_init_cm7.c
 * @brief   DEV-layer phase for CM7: plain PWM ESC output. Servos live on CM4.
 *
 * Wired to the same TIM4_CH1 (PD12, lower) / TIM4_CH2 (PD13, upper) pins
 * DShot used to drive — see motor_drivers/pwm_output.h and tim.c's
 * MX_TIM4_Init (now 1 us/tick, 400 Hz frame instead of DShot bit timing).
 *
 * UBC Rocket, 2026
 */
#include "app/app_init.h"
#include "app/esc_init.h"
#include "io_sys/io_test_hooks.h"

#include "tim.h"

#include <stdbool.h>
#include <string.h>

/* Standard RC ESC PWM: 1000-2000 us pulse, 400 Hz frame (matches the
 * deprecated esc_driver.h's ESC_PWM_FREQ_HZ, and divides evenly off the
 * existing 800 Hz control ISR). Timer runs at 1 MHz (see tim.c), so ticks
 * are microseconds 1:1; period_ticks = ARR + 1. */
#define ESC_PWM_TIMER_HZ     1000000U
#define ESC_PWM_PERIOD_TICKS 2500U   /* ARR = 2499 in MX_TIM4_Init */
#define ESC_PWM_MIN_US       1000U

static escs_t s_escs;
static bool   s_init_done;

IO_TEST_HOOK_RW(s_escs,      escs_t, dev_init_cm7_escs)
IO_TEST_HOOK_RW(s_init_done, bool,   dev_init_cm7_init_done)

void dev_init_cm7(void)
{
    if (s_init_done) return;

    memset(&s_escs, 0, sizeof(s_escs));

    s_escs.lower = (pwm_output_t){
        .htim = &htim4, .channel = TIM_CHANNEL_1,
        .timer_hz = ESC_PWM_TIMER_HZ, .period_ticks = ESC_PWM_PERIOD_TICKS,
    };
    s_escs.upper = (pwm_output_t){
        .htim = &htim4, .channel = TIM_CHANNEL_2,
        .timer_hz = ESC_PWM_TIMER_HZ, .period_ticks = ESC_PWM_PERIOD_TICKS,
    };

    /* Set compare to min but do NOT start PWM — wait for arm (controls_task,
     * gated on BENCH_NO_MOTORS), same as the deprecated esc_driver's
     * esc_init(). */
    pwm_set_compare(&s_escs.lower, pwm_us_to_ticks(&s_escs.lower, ESC_PWM_MIN_US));
    pwm_set_compare(&s_escs.upper, pwm_us_to_ticks(&s_escs.upper, ESC_PWM_MIN_US));

    s_init_done = true;
}

const escs_t *esc_handles(void)
{
    return s_init_done ? &s_escs : NULL;
}
