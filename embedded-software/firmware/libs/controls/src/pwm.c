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
static const double inverse_mtx[2][2] = {
    {565.861815543094, 2845722.037662509},
    {625.8535393360058, -2508149.344152053},
};

static inline double clamp(double value, double minimum, double maximum)
{
    return fmin(fmax(value, minimum), maximum);
}

/**
 * Computes the PWM setpoint necessary to generate the thrust and torque
 * setpoints desired.
 *
 * @param thrust Thrust setpoint
 * @param torque Torque setpoint
 * @return pwm_setpoint_t
 */
pwm_setpoint_t pwm_setpoint_from_forces(double thrust, double torque)
{
    double lower_normalized = (inverse_mtx[0][0] * thrust) + (inverse_mtx[0][1] * torque);
    double upper_normalized = (inverse_mtx[1][0] * thrust) + (inverse_mtx[1][1] * torque);

    double lower_period = sqrt(lower_normalized) + PWM_MIN_PERIOD_US;
    double upper_period = sqrt(upper_normalized) + PWM_MIN_PERIOD_US;

    // Model could extrapolate to values outside the valid range of PWM periods
    lower_period = clamp(round(lower_period), PWM_MIN_PERIOD_US, PWM_MAX_PERIOD_US);
    upper_period = clamp(round(upper_period), PWM_MIN_PERIOD_US, PWM_MAX_PERIOD_US);

    pwm_setpoint_t setpoint = {
        .lower_motor_us = lower_period,
        .upper_motor_us = upper_period,
    };

    return setpoint;
}
