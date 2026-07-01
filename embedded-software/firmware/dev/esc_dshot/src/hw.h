#ifndef DEV_DSHOT_HW_H
#define DEV_DSHOT_HW_H

#include "stm32h7xx.h"

typedef struct io_bdshot_esc {
    TIM_HandleTypeDef *tim;
    uint32_t tim_channel;

    DMA_HandleTypeDef *dma;

    GPIO_TypeDef *gpio;
    uint32_t gpio_pin;

    // TODO: AF is cleared during disarm as we configure GPIO
    // pin as OUTPUT to ensure the signal line is low. Current
    // way of restoring the correct AF to restore control by TIM
    // is to hardcode this. Dynamically saving and restoring it
    // was the old way. Is there a better third option? Or just
    // decide which of the two is better.
    uint32_t gpio_original_af;
} io_bdshot_esc_t;

#endif // DEV_DSHOT_HW_H
