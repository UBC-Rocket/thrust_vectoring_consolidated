/**
 * @file    servo_bus_gpio.c
 * @brief   UART8 servo bus GPIO: 74LVC2G241 OE (PD3).
 *
 * UBC Rocket, 2026
 */
#include "servo_bus_gpio.h"

#include "stm32h7xx_hal.h"

#define LVL_SHIFT_OE_Pin        GPIO_PIN_3
#define LVL_SHIFT_OE_GPIO_Port  GPIOD

void servo_bus_gpio_init(void) {
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOD_CLK_ENABLE();

    HAL_GPIO_WritePin(LVL_SHIFT_OE_GPIO_Port, LVL_SHIFT_OE_Pin, GPIO_PIN_SET);

    gpio.Pin   = LVL_SHIFT_OE_Pin;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LVL_SHIFT_OE_GPIO_Port, &gpio);
}

void servo_bus_gpio_enable(void) {
    HAL_GPIO_WritePin(LVL_SHIFT_OE_GPIO_Port, LVL_SHIFT_OE_Pin, GPIO_PIN_SET);
}
