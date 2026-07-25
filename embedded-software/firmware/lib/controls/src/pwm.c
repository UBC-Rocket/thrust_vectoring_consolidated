#include "controls/pwm.h"

#include <math.h>

/**
 * Inverse matrix to project a `[thrust, torque]` vector into
 * a `[lower_pwm_normalized, upper_pwm_normalized]`
 * vector.
 *
 * To ensure the regression model is linear and correctly intercepts
 * at 0, we will use a normalized PWM period:
 * pwm_normalized = (pwm_period - PWM_MIN_PERIOD_US)^2
 *
 * The fitted regression model is of form `Ax = b`, where `A` is the
 * regression coefficient matrix, `x` is the vector
 * `[lower_pwm_normalized, upper_pwm_normalized]`, and `b` is the
 * vector `[thrust, torque]`.
 *
 * The matrix below is `A_inv` in the equation `x = (A_inv)b`.
 */
static const float inverse_mtx[2][2] = {
    {57871.61952430651, 2859225.723821483},
    {63669.6665675616, -2520051.1606805213},
};

static inline float clampf(float value, float minimum, float maximum)
{
    return fminf(fmaxf(value, minimum), maximum);
}

/**
 * Computes the PWM setpoint necessary to generate the thrust and torque
 * setpoints desired.
 *
 * @param thrust Thrust setpoint in newtons
 * @param torque Torque setpoint in newton-meter
 * @return pwm_setpoint_t
 */
pwm_setpoint_t pwm_setpoint_from_forces(float thrust, float torque)
{
    float lower_normalized = (inverse_mtx[0][0] * thrust) + (inverse_mtx[0][1] * torque);
    float upper_normalized = (inverse_mtx[1][0] * thrust) + (inverse_mtx[1][1] * torque);

    float lower_period = sqrtf(lower_normalized) + PWM_MIN_PERIOD_US;
    float upper_period = sqrtf(upper_normalized) + PWM_MIN_PERIOD_US;

    pwm_setpoint_t setpoint = {
        .lower_motor_us = clampf(roundf(lower_period), PWM_MIN_PERIOD_US, PWM_MAX_PERIOD_US),
        .upper_motor_us = clampf(roundf(upper_period), PWM_MIN_PERIOD_US, PWM_MAX_PERIOD_US),
    };

    return setpoint;
}
