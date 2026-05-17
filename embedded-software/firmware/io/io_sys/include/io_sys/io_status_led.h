/**
 * @file io_status_led.h
 * @brief Status LED interface.
 *
 * The H747 board has three status LEDs (yellow, red, green). They are
 * partitioned between the cores by the per-target IO impl; calls from the
 * "wrong" core become no-ops.
 *
 * UBC Rocket, 2026
 */

#ifndef IO_STATUS_LED_H
#define IO_STATUS_LED_H

#include "io_sys/io_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    IO_LED_GREEN = 0,
    IO_LED_YELLOW = 1,
    IO_LED_RED = 2,
    IO_LED_COUNT
} io_led_id_t;

void io_status_led_init(void);

void io_status_led_set(io_led_id_t id, bool on);
void io_status_led_toggle(io_led_id_t id);

#ifdef __cplusplus
}
#endif

#endif /* IO_STATUS_LED_H */
