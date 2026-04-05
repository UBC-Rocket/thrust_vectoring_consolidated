#ifndef __MAIN_H
#define __MAIN_H

#include "stm32h5xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

void fake_delay_us(uint32_t us);

static inline void delay_us(uint32_t us)
{
    fake_delay_us(us);
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void Error_Handler(void);

extern bool g_sd_card_initialized;

#define ICM40609_CS_GPIO_Port GPIOD
#define ICM40609_CS_Pin GPIO_PIN_15
#define ICM40609_INT1_Pin GPIO_PIN_12

#endif
