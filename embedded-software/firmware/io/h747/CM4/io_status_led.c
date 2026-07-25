/**
 * @file    io_status_led.c
 * @brief   CM4 owns the RGB status LEDs and the dedicated STAT_LEDR on PB7.
 *
 * UBC Rocket, 2026
 */

#include "io_common.h"
#include "io_sys/io_status_led.h"

typedef struct { GPIO_TypeDef *port; uint16_t pin; } led_pin_t;

static const led_pin_t s_leds[IO_LED_COUNT] = {
    [IO_LED_GREEN]  = { RGB_G_GPIO_Port, RGB_G_Pin },
    [IO_LED_YELLOW] = { RGB_B_GPIO_Port, RGB_B_Pin },
    [IO_LED_RED]    = { STAT_LEDR_GPIO_Port, STAT_LEDR_Pin },
};

void io_status_led_init(void) {
    for (unsigned i = 0; i < IO_LED_COUNT; ++i) {
        HAL_GPIO_WritePin(s_leds[i].port, s_leds[i].pin, GPIO_PIN_RESET);
    }
}

void io_status_led_set(io_led_id_t id, bool on) {
    if (id >= IO_LED_COUNT) return;
    HAL_GPIO_WritePin(s_leds[id].port, s_leds[id].pin,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void io_status_led_toggle(io_led_id_t id) {
    if (id >= IO_LED_COUNT) return;
    HAL_GPIO_TogglePin(s_leds[id].port, s_leds[id].pin);
}
