#include "motor_drivers/servo_driver.h"
#include "utilities/clamp.h"

#include <stddef.h>

// Helper functions
static uint16_t degree_to_us(const servo_t *servo, float degree);
static void servo_calibrate(servo_t *servo);

// -------- Global state --------

static volatile bool g_servo_pair_ready = false;

static servo_pair_t servos = {
    .servo1 = {
        .us_min = SERVO1_US_MIN,
        .us_max = SERVO1_US_MAX,

        .deg_range_max = SERVO1_DEG_RANGE_MAX,
        .deg_min = SERVO_GIMBAL_DEG_MIN,
        .deg_max = SERVO_GIMBAL_DEG_MAX,
        .deg_bias = SERVO1_DEG_BIAS,

        .reversed = SERVO1_REVERSED,
        .enabled = false
    },
    .servo2 = {
        .us_min = SERVO2_US_MIN,
        .us_max = SERVO2_US_MAX,

        .deg_range_max = SERVO2_DEG_RANGE_MAX,
        .deg_min = SERVO_GIMBAL_DEG_MIN,
        .deg_max = SERVO_GIMBAL_DEG_MAX,
        .deg_bias = SERVO2_DEG_BIAS,
        
        .reversed = SERVO2_REVERSED,
        .enabled = false
    }
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
    if (!g_servo_pair_ready) {
        return;
    }
    /* servo1 = Y-axis (pitch); servo2 = X-axis (roll).
     * Negate both: positive command tilts toward positive axis per right-hand convention,
     * but servo mechanical direction is inverted relative to body frame. */
    set_servo_degree(&servos.servo1, y_deg);
    set_servo_degree(&servos.servo2, x_deg);
}

/* ---- ISR-level API ----------------------------------------------------- */

void apply_servo_pair_degrees(void) {
    if (!g_servo_pair_ready) {
        return;
    }
    pwm_set_compare(&servos.servo1.pwm, servos.servo1.compare_val);
    pwm_set_compare(&servos.servo2.pwm, servos.servo2.compare_val);
}

/* ---- Init / enable ----------------------------------------------------- */

static void servo_calibrate(servo_t *servo) 
{
    if (servo == NULL) return;

    float home = degree_to_us(servo, servo->deg_bias);

    servo->us_last = home;  /* Start at mid position. */

    pwm_output_t *pwm = &servo->pwm;

    /* Start PWM output, set to mid position, then stop until enabled. */
    (void)HAL_TIM_PWM_Start(pwm->htim, pwm->channel);
    {
        uint32_t ticks = pwm_us_to_ticks(pwm, home);
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

    servo->us_min = SERVO1_US_MIN;
    servo->us_max = SERVO1_US_MAX;
    servo->deg_range_max = SERVO1_DEG_RANGE_MAX;

    servo->deg_min = SERVO_GIMBAL_DEG_MIN;
    servo->deg_max = SERVO_GIMBAL_DEG_MAX;
    servo->deg_bias = SERVO1_DEG_BIAS;
    servo->reversed = SERVO1_REVERSED;

    servo->enabled = false;

    servo_calibrate(servo);
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
        float home = degree_to_us(servo, servo->deg_bias);

        uint32_t ticks = pwm_us_to_ticks(&servo->pwm, home);
        ticks = pwm_clamp_ticks(&servo->pwm, ticks);
        pwm_set_compare(&servo->pwm, ticks);
        (void)HAL_TIM_PWM_Stop(servo->pwm.htim, servo->pwm.channel);
    }
}

void servo_pair_init(const pwm_output_t *pwm1, const pwm_output_t *pwm2) {
    if (pwm1 == NULL || pwm2 == NULL) {
        return;
    }
    
    servo_calibrate(&servos.servo1);
    servo_calibrate(&servos.servo2);
    g_servo_pair_ready = true;
}

void servo_pair_enable(bool enable) {
    if (!g_servo_pair_ready) {
        return;
    }
    servo_enable(&servos.servo1, enable);
    servo_enable(&servos.servo2, enable);
}

/* ---- Static helpers ---------------------------------------------------- */

static uint16_t degree_to_us(const servo_t *servo, float degree) {
    float scale = (servo->us_max - servo->us_min) / servo->deg_range_max;

    float d = (servo->reversed ? -degree : degree) + servo->deg_bias;
    float d_clamped = clamp_float(d, servo->deg_min, servo->deg_max);

    uint16_t us = servo->us_min + d_clamped * scale;
    return clamp_u16(us, servo->us_min, servo->us_max);
}
