/**
 * @file    esc_init.h
 * @brief   View-struct for CM7 ESC PWM output wiring (TIM4_CH1/PD12 lower,
 *          TIM4_CH2/PD13 upper — same pins DShot used).
 *
 * The wiring itself is instantiated by @ref dev_init_cm7 (declared in
 * app_init.h).
 *
 * UBC Rocket, 2026
 */
#ifndef APP_ESC_INIT_H
#define APP_ESC_INIT_H

#include "motor_drivers/pwm_output.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    pwm_output_t lower;
    pwm_output_t upper;
} escs_t;

/**
 * @brief Return the ESC PWM wiring (NULL if dev_init_cm7 wasn't run).
 */
const escs_t *esc_handles(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_ESC_INIT_H */
