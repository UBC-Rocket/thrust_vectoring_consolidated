/**
 * @file config.h
 * @brief Configuration for the ESC driver.
 *
 * UBC Rocket, 2026
 */

#ifndef DEV_ESC_DSHOT_CONFIG_H
#define DEV_ESC_DSHOT_CONFIG_H

#include "esc_dshot/limits.h"

#include <assert.h>

#define ESC_MIN_THROTTLE (0)
#define ESC_MAX_THROTTLE (1999)

#define ESC_PERCENTAGE_TO_THROTTLE(percentage) ((uint16_t)(((float)percentage) * ESC_MAX_THROTTLE))

typedef enum esc_motor_id {
    ESC_MOTOR_ID_LOWER = 0,
    ESC_MOTOR_ID_UPPER,
    ESC_MOTOR_ID_COUNT
} esc_motor_id_t;

static_assert(ESC_MOTOR_ID_COUNT < ESC_MAX_MOTOR_COUNT,
              "Exceeded maximum number of supports motors");

#endif // DEV_ESC_DSHOT_CONFIG_H
