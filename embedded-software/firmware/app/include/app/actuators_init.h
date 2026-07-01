/**
 * @file    actuators_init.h
 * @brief   View-struct for CM7 actuator driver singletons.
 *
 * The drivers themselves are instantiated by @ref dev_init_cm7 (declared in
 * app_init.h).
 *
 * UBC Rocket, 2026
 */
#ifndef APP_ACTUATORS_INIT_H
#define APP_ACTUATORS_INIT_H

#ifdef USE_DYNAMIXEL_SERVO
#include "dev_servo_dynamixel.h"
#else
#include "dev_servo_feetech.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
#ifdef USE_DYNAMIXEL_SERVO
    servo_dynamixel_t *servos;
#else
    servo_feetech_t   *servos;
#endif
} actuators_t;

/**
 * @brief Return pointers to the singletons (NULL if dev_init_cm7 wasn't run).
 */
const actuators_t *actuators_handles(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_ACTUATORS_INIT_H */
