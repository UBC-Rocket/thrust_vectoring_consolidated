#include "motor_drivers/servo_driver.h"
#include "utilities/clamp.h"

#include <stddef.h>

// Helper functions
static uint16_t degree_to_us(const servo_t *servo, float degree);
static void servo_home(servo_t *servo);

// -------- Global state --------

static volatile bool s_servo_pair_ready = false;

static servo_pair_t s_servos;

static servo_config_t s_servo1_config = {
    .us_min = SERVO1_US_MIN,
    .us_max = SERVO1_US_MAX,
    .deg_range_max = SERVO1_DEG_RANGE_MAX,
    .deg_min = SERVO_GIMBAL_DEG_MIN,
    .deg_max = SERVO_GIMBAL_DEG_MAX,
    .deg_bias = SERVO1_DEG_BIAS,
    .reversed = SERVO1_REVERSED,
};

static servo_config_t s_servo2_config = {
    .us_min = SERVO2_US_MIN,
    .us_max = SERVO2_US_MAX,
    .deg_range_max = SERVO2_DEG_RANGE_MAX,
    .deg_min = SERVO_GIMBAL_DEG_MIN,
    .deg_max = SERVO_GIMBAL_DEG_MAX,
    .deg_bias = SERVO2_DEG_BIAS,
    .reversed = SERVO2_REVERSED,
};


/* ---- Task-level API ---------------------------------------------------- */

void set_servo_degree(servo_t *servo, float degree) {
    if (servo == NULL) {
        return;
    }

    uint16_t us = degree_to_us(servo, degree);
    servo->us_last = us;
    uint32_t ticks = pwm_us_to_ticks(&servo->pwm, us);
    servo->compare_val = pwm_clamp_ticks(&servo->pwm, ticks);
}

void set_gimbal_degrees(float x_deg, float y_deg) {
    if (!s_servo_pair_ready) {
        return;
    }
    /* servo1 = Y-axis (pitch); servo2 = X-axis (roll).
     * Negate both: positive command tilts toward positive axis per right-hand convention,
     * but servo mechanical direction is inverted relative to body frame. */
    set_servo_degree(&s_servos.servo1, y_deg);
    set_servo_degree(&s_servos.servo2, x_deg);
}

/* ---- ISR-level API ----------------------------------------------------- */

void apply_servo_pair_degrees(void) {
    if (!s_servo_pair_ready) {
        return;
    }
    pwm_set_compare(&s_servos.servo1.pwm, s_servos.servo1.compare_val);
    pwm_set_compare(&s_servos.servo2.pwm, s_servos.servo2.compare_val);
}

/* ---- Init / enable ----------------------------------------------------- */

static void servo_home(servo_t *servo) 
{
    if (servo == NULL) return;

    pwm_output_t *pwm = &servo->pwm;

    if (pwm == NULL) return; // check pwm channel is defined

    uint16_t home_us = degree_to_us(servo, servo->config.deg_bias);
    servo->us_last = home_us;  /* Start at mid position. */

    /* Start PWM output, set to mid position, then stop until enabled. */
    (void)HAL_TIM_PWM_Start(pwm->htim, pwm->channel);
    {
        uint32_t ticks = pwm_us_to_ticks(pwm, home_us);
        ticks = pwm_clamp_ticks(pwm, ticks);
        servo->compare_val = ticks;
        pwm_set_compare(pwm, ticks);
    }
    (void)HAL_TIM_PWM_Stop(pwm->htim, pwm->channel);
}

void servo_init(servo_t *servo, const pwm_output_t *pwm) {
    if (servo == NULL || pwm == NULL) {
        return;
    }
    
    servo->pwm = *pwm;
    servo->config = s_servo1_config; // default config; can be overridden after init
    servo->enabled = false;
    servo_home(servo);
}

void servo_enable(servo_t *servo, bool enable) {
    if (servo == NULL) {
        return;
    }

    servo->enabled = enable;

    if (enable) {
        uint32_t ticks = pwm_us_to_ticks(&servo->pwm, servo->us_last);
        ticks = pwm_clamp_ticks(&servo->pwm, ticks);
        pwm_set_compare(&servo->pwm, ticks);
        (void)HAL_TIM_PWM_Start(servo->pwm.htim, servo->pwm.channel);
    } else {
        uint16_t home_us = degree_to_us(servo, servo->config.deg_bias);

        uint32_t ticks = pwm_us_to_ticks(&servo->pwm, home_us);
        ticks = pwm_clamp_ticks(&servo->pwm, ticks);
        pwm_set_compare(&servo->pwm, ticks);
        (void)HAL_TIM_PWM_Stop(servo->pwm.htim, servo->pwm.channel);
    }
}

void servo_pair_init(const pwm_output_t *pwm1, const pwm_output_t *pwm2) {
    if (pwm1 == NULL || pwm2 == NULL) {
        return;
    }

    s_servos.servo1.pwm = *pwm1;
    s_servos.servo2.pwm = *pwm2;

    s_servos.servo1.config = s_servo1_config;
    s_servos.servo2.config = s_servo2_config;
    
    servo_home(&s_servos.servo1);
    servo_home(&s_servos.servo2);

    s_servo_pair_ready = true;
}

void servo_pair_enable(bool enable) {
    if (!s_servo_pair_ready) {
        return;
    }
    servo_enable(&s_servos.servo1, enable);
    servo_enable(&s_servos.servo2, enable);
}

/* ---- Static helpers ---------------------------------------------------- */

static uint16_t degree_to_us(const servo_t *servo, float degree) {
    const servo_config_t *cfg = &servo->config;

    float scale = (float)(cfg->us_max - cfg->us_min) / cfg->deg_range_max;

    float d = (cfg->reversed ? -degree : degree) + cfg->deg_bias;
    float d_clamped = clamp_float(d, cfg->deg_min + cfg->deg_bias, cfg->deg_max + cfg->deg_bias);

    uint16_t us = cfg->us_min + d_clamped * scale;
    return clamp_u16(us, cfg->us_min, cfg->us_max);
}
