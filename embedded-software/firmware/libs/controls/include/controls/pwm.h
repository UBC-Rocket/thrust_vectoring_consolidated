#ifndef LIB_CONTROLS_PWM_H
#define LIB_CONTROLS_PWM_H

#include <stdint.h>

#define PWM_MIN_PERIOD_US (1000)
#define PWM_MAX_PERIOD_US (2000)

typedef struct {
    uint16_t upper_motor_us; /**< PWM for the upper motor in microseconds */
    uint16_t lower_motor_us;
} pwm_setpoint_t;

pwm_setpoint_t pwm_setpoint_from_forces(double thrust, double torque);

#endif // LIB_CONTROLS_PWM_H
