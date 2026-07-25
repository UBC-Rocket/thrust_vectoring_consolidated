/**
 * @file    dev_servo_feetech.h
 * @brief   Feetech STS/SCS servo driver. Opaque singleton.
 *
 * UBC Rocket, 2026
 */
#ifndef DEV_SERVO_FEETECH_H
#define DEV_SERVO_FEETECH_H

#include "io_sys/io_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct servo_feetech servo_feetech_t;

bool             servo_feetech_init             (void);
servo_feetech_t *servo_feetech_get              (void);
void             servo_feetech_set_pair_degrees (servo_feetech_t *d, float deg_x, float deg_y);
void             servo_feetech_enable           (servo_feetech_t *d, bool enable);
bool             servo_feetech_get_pair_degrees (servo_feetech_t *d, float *deg_x_out, float *deg_y_out);

#ifdef __cplusplus
}
#endif

#endif /* DEV_SERVO_FEETECH_H */
