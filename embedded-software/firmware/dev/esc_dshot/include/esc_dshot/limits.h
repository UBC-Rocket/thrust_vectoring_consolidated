/**
 * @file limits.h
 * @brief Configuration limits for the ESC driver.
 *
 * UBC Rocket, 2026
 */

#ifndef DEV_ESC_DSHOT_LIMITS_H
#define DEV_ESC_DSHOT_LIMITS_H

#include <stdint.h>

// Reserve highest motor index value to denote an invalid motor
#define ESC_MAX_MOTOR_COUNT (UINT32_MAX - 1)

#endif // DEV_ESC_DSHOT_LIMITS_H
