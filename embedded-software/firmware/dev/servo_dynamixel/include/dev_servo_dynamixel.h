/**
 * @file    dev_servo_dynamixel.h
 * @brief   Robotis Dynamixel XM430 driver. Opaque singleton.
 *
 * UBC Rocket, 2026
 */
#ifndef DEV_SERVO_DYNAMIXEL_H
#define DEV_SERVO_DYNAMIXEL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct servo_dynamixel servo_dynamixel_t;

bool              servo_dynamixel_init(void);
void              servo_dynamixel_log_status(void);
servo_dynamixel_t *servo_dynamixel_get(void);
bool              servo_dynamixel_ping(servo_dynamixel_t *d);
bool              servo_dynamixel_set_goal_position(servo_dynamixel_t *d, uint8_t id, int32_t pos);
bool              servo_dynamixel_set_pair_goal_position(servo_dynamixel_t *d, int32_t pos);
bool              servo_dynamixel_set_pair_degrees(servo_dynamixel_t *d, float deg_x, float deg_y);
bool              servo_dynamixel_get_present_position(servo_dynamixel_t *d, uint8_t id, int32_t *pos);
void              servo_dynamixel_run_position_test(servo_dynamixel_t *d);

#ifdef __cplusplus
}
#endif

#endif /* DEV_SERVO_DYNAMIXEL_H */
