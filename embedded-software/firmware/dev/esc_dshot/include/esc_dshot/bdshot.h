/**
 * @file bdshot.h
 * @brief Driver for interacting with ESCs over bidirectional DShot.
 *
 * UBC Rocket, 2026
 */

#ifndef DEV_ESC_DSHOT_BDSHOT_H
#define DEV_ESC_DSHOT_BDSHOT_H

#include "esc_dshot/config.h"
#include "esc_dshot/io.h"

#include <stdbool.h>

typedef struct esc_motor_config {
    /// The number of magnetic poles in the motor.
    uint8_t pole_count;
} esc_motor_config_t;

typedef struct esc_motor_telemetry {
    float rpm;
} esc_motor_telemetry_t;

void esc_dshot_init();
bool esc_dshot_update();
bool esc_dshot_set_armed(bool is_armed);

bool esc_dshot_motor_init(esc_motor_id_t motor_id, const esc_motor_config_t *config,
                          const io_bdshot_esc_t *io_handle);
bool esc_dshot_motor_set_armed(esc_motor_id_t motor_id, bool is_armed);
bool esc_dshot_motor_set_throttle(esc_motor_id_t motor_id, uint16_t throttle);
bool esc_dshot_motor_get_telemetry(esc_motor_id_t motor_id, esc_motor_telemetry_t *telemetry);

#endif // DEV_ESC_DSHOT_BDSHOT_H
