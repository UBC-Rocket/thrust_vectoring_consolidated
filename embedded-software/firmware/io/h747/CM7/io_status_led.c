/**
 * @file    io_status_led.c
 * @brief   CM7 does not own any status LEDs on this board.
 *
 * The three STAT_LED pins are on GPIOD/G which live in the D2/D3 domains and
 * are driven by CM4. Requests from CM7 are no-ops; they can be forwarded via
 * an intercore event if needed.
 *
 * UBC Rocket, 2026
 */

#include "io_sys/io_status_led.h"

void io_status_led_init(void) {}
void io_status_led_set(io_led_id_t id, bool on) { (void)id; (void)on; }
void io_status_led_toggle(io_led_id_t id)        { (void)id; }
