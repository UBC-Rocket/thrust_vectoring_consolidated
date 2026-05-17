/**
 * @file    dev_esc_dshot.h
 * @brief   DShot ESC pair driver. Opaque singleton.
 *
 * UBC Rocket, 2026
 */
#ifndef DEV_ESC_DSHOT_H
#define DEV_ESC_DSHOT_H

#include "io_sys/io_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct esc_dshot esc_dshot_t;

bool         esc_dshot_init        (void);
esc_dshot_t *esc_dshot_get         (void);
void         esc_dshot_set_throttle(esc_dshot_t *d, float throttle_unit);
void         esc_dshot_send_command(esc_dshot_t *d, uint16_t cmd);
void         esc_dshot_arm         (esc_dshot_t *d, bool armed);

#ifdef __cplusplus
}
#endif

#endif /* DEV_ESC_DSHOT_H */
