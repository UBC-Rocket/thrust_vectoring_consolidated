#ifndef DEV_DSHOT_BDSHOT_RAL_H
#define DEV_DSHOT_BDSHOT_RAL_H

#include "hal_to_ll_translator.h"

#include "stm32h745xx.h"
#include "stm32h7xx.h"
#include "stm32h7xx_hal_dma.h"
#include "stm32h7xx_ll_tim.h"
#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_dma.h"

#include <stdint.h>
#include <stdbool.h>

#define RAL_VALUE_INVALID (UINT32_MAX)

typedef enum ral_dma_transfer_direction {
    RAL_DMA_TRANSFER_PERIPH_TO_MEMORY,
    RAL_DMA_TRANSFER_MEMORY_TO_PERIPH,
    RAL_DMA_TRANSFER_MEMORY_TO_MEMORY,
} ral_dma_transfer_direction_t;

static inline volatile uint32_t *ral_get_tim_channel_ccrx_reg(TIM_HandleTypeDef *tim,
                                                              uint32_t hal_tim_channel)
{
    switch (hal_tim_channel) {
    case TIM_CHANNEL_1: {
        return &tim->Instance->CCR1;
    }
    case TIM_CHANNEL_2: {
        return &tim->Instance->CCR2;
    }
    case TIM_CHANNEL_3: {
        return &tim->Instance->CCR3;
    }
    case TIM_CHANNEL_4: {
        return &tim->Instance->CCR4;
    }
    default: {
        return NULL;
    }
    }
}

static inline volatile uint32_t ral_get_tim_channel_dma_src(TIM_HandleTypeDef *tim,
                                                            uint32_t hal_tim_channel)
{
    switch (hal_tim_channel) {
    case TIM_CHANNEL_1: {
        return TIM_DMA_CC1;
    }
    case TIM_CHANNEL_2: {
        return TIM_DMA_CC2;
    }
    case TIM_CHANNEL_3: {
        return TIM_DMA_CC3;
    }
    case TIM_CHANNEL_4: {
        return TIM_DMA_CC4;
    }
    default: {
        return RAL_VALUE_INVALID;
    }
    }
}

static inline bool ral_tim_channel_dma_set_enabled(TIM_HandleTypeDef *tim, uint32_t hal_tim_channel,
                                                   bool is_enabled)
{
    switch (hal_tim_channel) {
    case TIM_CHANNEL_1: {
        if (is_enabled) {
            LL_TIM_EnableDMAReq_CC1(tim->Instance);
        } else {
            LL_TIM_DisableDMAReq_CC1(tim->Instance);
        }
        break;
    }
    case TIM_CHANNEL_2: {
        if (is_enabled) {
            LL_TIM_EnableDMAReq_CC2(tim->Instance);
        } else {
            LL_TIM_DisableDMAReq_CC2(tim->Instance);
        }
        break;
    }
    case TIM_CHANNEL_3: {
        if (is_enabled) {
            LL_TIM_EnableDMAReq_CC3(tim->Instance);
        } else {
            LL_TIM_DisableDMAReq_CC3(tim->Instance);
        }
        break;
    }
    case TIM_CHANNEL_4: {
        if (is_enabled) {
            LL_TIM_EnableDMAReq_CC4(tim->Instance);
        } else {
            LL_TIM_DisableDMAReq_CC4(tim->Instance);
        }
        break;
    }
    default: {
        return false;
    }
    }

    return true;
}

static inline uint32_t ral_gpio_get_af(GPIO_TypeDef *gpio, uint32_t hal_gpio_pin)
{
    uint32_t alternate_function;

    const uint32_t ll_gpio_pin = hal_to_ll_convert_gpio_pin(hal_gpio_pin);

    if (ll_gpio_pin <= LL_GPIO_PIN_7) {
        alternate_function = LL_GPIO_GetAFPin_0_7(gpio, ll_gpio_pin);
    } else {
        alternate_function = LL_GPIO_GetAFPin_8_15(gpio, ll_gpio_pin);
    }

    return alternate_function;
}

static inline void ral_gpio_set_af(GPIO_TypeDef *gpio, uint32_t hal_gpio_pin,
                                   uint32_t alternate_function)
{
    const uint32_t ll_gpio_pin = hal_to_ll_convert_gpio_pin(hal_gpio_pin);

    if (ll_gpio_pin <= LL_GPIO_PIN_7) {
        LL_GPIO_SetAFPin_0_7(gpio, ll_gpio_pin, alternate_function);
    } else {
        LL_GPIO_SetAFPin_8_15(gpio, ll_gpio_pin, alternate_function);
    }
}

static inline void ral_dma_peripheral_increment_set_enabled(DMA_HandleTypeDef *dma, bool is_enabled)
{
    MODIFY_REG(((DMA_Stream_TypeDef *)dma->Instance)->CR, DMA_SxCR_PINC,
               is_enabled ? LL_DMA_PERIPH_INCREMENT : LL_DMA_PERIPH_NOINCREMENT);
}

static inline void ral_dma_memory_increment_set_enabled(DMA_HandleTypeDef *dma, bool is_enabled)
{
    MODIFY_REG(((DMA_Stream_TypeDef *)dma->Instance)->CR, DMA_SxCR_MINC,
               is_enabled ? LL_DMA_MEMORY_INCREMENT : LL_DMA_MEMORY_NOINCREMENT);
}

static inline void ral_dma_data_transfer_set_direction(DMA_HandleTypeDef *dma,
                                                       ral_dma_transfer_direction_t direction)
{
    uint32_t ll_dma_transfer_direction;

    switch (direction) {
    case RAL_DMA_TRANSFER_PERIPH_TO_MEMORY: {
        ll_dma_transfer_direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
        break;
    }
    case RAL_DMA_TRANSFER_MEMORY_TO_PERIPH: {
        ll_dma_transfer_direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
        break;
    }
    case RAL_DMA_TRANSFER_MEMORY_TO_MEMORY: {
        ll_dma_transfer_direction = LL_DMA_DIRECTION_MEMORY_TO_MEMORY;
    }
    }

    MODIFY_REG(((DMA_Stream_TypeDef *)dma->Instance)->CR, DMA_SxCR_DIR, ll_dma_transfer_direction);
}

#endif // DEV_DSHOT_BDSHOT_RAL_H
