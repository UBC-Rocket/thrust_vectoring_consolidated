#include "hw.h"
#include "esc_dshot/io.h"

#include "main.h"

#include "stm32h7xx.h"

extern TIM_HandleTypeDef htim4;
extern DMA_HandleTypeDef hdma_tim4_ch1;
extern DMA_HandleTypeDef hdma_tim4_ch2;

const io_bdshot_esc_t IO_BDSHOT_ESC_UPPER = {
    .tim = &htim4,
    .tim_channel = TIM_CHANNEL_1,
    .dma = &hdma_tim4_ch1,
    .gpio = ESC1_PWM_GPIO_Port,
    .gpio_pin = ESC1_PWM_Pin,
    .gpio_original_af = GPIO_AF2_TIM4,
};

const io_bdshot_esc_t IO_BDSHOT_ESC_LOWER = {
    .tim = &htim4,
    .tim_channel = TIM_CHANNEL_2,
    .dma = &hdma_tim4_ch2,
    .gpio = ESC2_PWM_GPIO_Port,
    .gpio_pin = ESC2_PWM_Pin,
    .gpio_original_af = GPIO_AF2_TIM4,
};
