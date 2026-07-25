#ifndef DEV_ESC_DSHOT_HAL_TO_LL_TRANSLATOR_H
#define DEV_ESC_DSHOT_HAL_TO_LL_TRANSLATOR_H

#include "stm32h7xx.h"
#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_tim.h"

#include <stdint.h>

#define HAL_TO_LL_CONVERSION_INVALID (UINT32_MAX)

static inline uint32_t hal_to_ll_convert_gpio_pin(uint32_t hal_gpio_pin)
{
    switch (hal_gpio_pin) {
    case GPIO_PIN_0: {
        return LL_GPIO_PIN_0;
    }
    case GPIO_PIN_1: {
        return LL_GPIO_PIN_1;
    }
    case GPIO_PIN_2: {
        return LL_GPIO_PIN_2;
    }
    case GPIO_PIN_3: {
        return LL_GPIO_PIN_3;
    }
    case GPIO_PIN_4: {
        return LL_GPIO_PIN_4;
    }
    case GPIO_PIN_5: {
        return LL_GPIO_PIN_5;
    }
    case GPIO_PIN_6: {
        return LL_GPIO_PIN_6;
    }
    case GPIO_PIN_7: {
        return LL_GPIO_PIN_7;
    }
    case GPIO_PIN_8: {
        return LL_GPIO_PIN_8;
    }
    case GPIO_PIN_9: {
        return LL_GPIO_PIN_9;
    }
    case GPIO_PIN_10: {
        return LL_GPIO_PIN_10;
    }
    case GPIO_PIN_11: {
        return LL_GPIO_PIN_11;
    }
    case GPIO_PIN_12: {
        return LL_GPIO_PIN_12;
    }
    case GPIO_PIN_13: {
        return LL_GPIO_PIN_13;
    }
    case GPIO_PIN_14: {
        return LL_GPIO_PIN_14;
    }
    case GPIO_PIN_15: {
        return LL_GPIO_PIN_15;
    }
    default: {
        return HAL_TO_LL_CONVERSION_INVALID;
    }
    }
}

static inline uint32_t hal_to_ll_convert_tim_channel(uint32_t hal_tim_channel)
{
    switch (hal_tim_channel) {
    case TIM_CHANNEL_1:
        return LL_TIM_CHANNEL_CH1;
    case TIM_CHANNEL_2:
        return LL_TIM_CHANNEL_CH2;
    case TIM_CHANNEL_3:
        return LL_TIM_CHANNEL_CH3;
    case TIM_CHANNEL_4:
        return LL_TIM_CHANNEL_CH4;
    default:
        return HAL_TO_LL_CONVERSION_INVALID;
    }
}

#endif // DEV_ESC_DSHOT_HAL_TO_LL_TRANSLATOR_H
