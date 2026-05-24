#include "motor_drivers/servo_driver.h"

#include <stddef.h>

static float clamp_f(float x, float lo, float hi);
static uint16_t clamp_u16(uint16_t x, uint16_t lo, uint16_t hi);
static uint16_t degree_to_us(const servo_t *servo, float degree);
static void servo_init_with_cal(servo_t *servo, const pwm_output_t *pwm,
                                uint16_t us_min, uint16_t us_mid, uint16_t us_max);

/* Written by task (set_servo_pair_degrees); read by ISR (apply_servo_pair_degrees). */
static servo_pair_t servos;
static volatile bool g_servo_pair_ready = false;

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

void set_servo_pair_degrees(float degree1, float degree2) {
    if (!g_servo_pair_ready) {
        return;
    }
    set_servo_degree(&servos.servo1, degree1);
    set_servo_degree(&servos.servo2, degree2);
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

static void servo_init_with_cal(servo_t *servo, const pwm_output_t *pwm,
                                uint16_t us_min, uint16_t us_mid, uint16_t us_max) {
    if (servo == NULL || pwm == NULL || pwm->htim == NULL) {
        return;
    }

    servo->pwm = *pwm;
    servo->us_min = us_min;
    servo->us_mid = us_mid;
    servo->us_max = us_max;
    servo->deg_range = 180.0f;
    servo->mid_pt = 0.0f;   /* 0° command = center PWM (straight down) */
    servo->enabled = false;
    servo->us_last = us_mid;

    /* Start PWM output, set to mid position, then stop until enabled. */
    (void)HAL_TIM_PWM_Start(pwm->htim, pwm->channel);
    {
        uint32_t ticks = pwm_us_to_ticks(&servo->pwm, us_mid);
        ticks = pwm_clamp_ticks(&servo->pwm, ticks);
        servo->compare_val = ticks;
        pwm_set_compare(&servo->pwm, ticks);
    }
    (void)HAL_TIM_PWM_Stop(pwm->htim, pwm->channel);
}

void servo_init(servo_t *servo, const pwm_output_t *pwm) {
    servo_init_with_cal(servo, pwm, SERVO1_US_MIN, SERVO1_US_MID, SERVO1_US_MAX);
}

void servo_init_with_deg_range(servo_t *servo, const pwm_output_t *pwm, float deg_range, float mid_pt) {
    servo_init(servo, pwm);
    servo_set_deg_range(servo, deg_range, mid_pt);
}

void servo_enable(servo_t *servo, bool enable) {
    if (servo == NULL) {
        return;
    }

    servo->enabled = enable;

    if (enable) {
        servo->us_last = clamp_u16(servo->us_last, servo->us_min, servo->us_max);
        uint32_t ticks = pwm_us_to_ticks(&servo->pwm, servo->us_last);
        ticks = pwm_clamp_ticks(&servo->pwm, ticks);
        pwm_set_compare(&servo->pwm, ticks);
        (void)HAL_TIM_PWM_Start(servo->pwm.htim, servo->pwm.channel);
    } else {
        uint32_t ticks = pwm_us_to_ticks(&servo->pwm, servo->us_mid);
        ticks = pwm_clamp_ticks(&servo->pwm, ticks);
        pwm_set_compare(&servo->pwm, ticks);
        (void)HAL_TIM_PWM_Stop(servo->pwm.htim, servo->pwm.channel);
    }
}

void servo_set_deg_range(servo_t *servo, float deg_range, float mid_pt) {
    if (servo == NULL) {
        return;
    }
    servo->deg_range = deg_range;
    servo->mid_pt = mid_pt;
}

void servo_pair_init(const pwm_output_t *pwm1, const pwm_output_t *pwm2) {
    if (pwm1 == NULL || pwm2 == NULL) {
        return;
    }
    servo_init_with_cal(&servos.servo1, pwm1, SERVO1_US_MIN, SERVO1_US_MID, SERVO1_US_MAX);
    servo_init_with_cal(&servos.servo2, pwm2, SERVO2_US_MIN, SERVO2_US_MID, SERVO2_US_MAX);
    servo_set_deg_range(&servos.servo1, SERVO1_DEGREE_RANGE, 0.0);
    servo_set_deg_range(&servos.servo2, SERVO2_DEGREE_RANGE, 0.0);
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

static float clamp_f(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static uint16_t clamp_u16(uint16_t x, uint16_t lo, uint16_t hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static uint16_t degree_to_us(const servo_t *servo, float degree) {
    float half_range = servo->deg_range * 0.5f;
    float min_deg = servo->mid_pt - half_range;
    float max_deg = servo->mid_pt + half_range;
    float d = clamp_f(degree, min_deg, max_deg);

    float us_f;
    if (d < servo->mid_pt) {
        /* Negative side: [min_deg .. mid_pt] → [us_min .. us_mid] */
        float t = (half_range > 0.0f) ? (d - min_deg) / half_range : 0.0f;
        us_f = (float)servo->us_min + t * (float)(servo->us_mid - servo->us_min);
    } else {
        /* Positive side: [mid_pt .. max_deg] → [us_mid .. us_max] */
        float t = (half_range > 0.0f) ? (d - servo->mid_pt) / half_range : 0.0f;
        us_f = (float)servo->us_mid + t * (float)(servo->us_max - servo->us_mid);
    }

    return clamp_u16((uint16_t)(us_f + 0.5f), servo->us_min, servo->us_max);
}
