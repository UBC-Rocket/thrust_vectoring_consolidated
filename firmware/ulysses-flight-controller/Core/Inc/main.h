/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h5xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

static inline void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000UL);
    while ((DWT->CYCCNT - start) < ticks) {
        __NOP();
    }
}

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern bool g_sd_card_initialized;

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BNO_RST_Pin GPIO_PIN_3
#define BNO_RST_GPIO_Port GPIOE
#define BNO_CS_Pin GPIO_PIN_4
#define BNO_CS_GPIO_Port GPIOE
#define ESC_1_PWM_Pin GPIO_PIN_1
#define ESC_1_PWM_GPIO_Port GPIOA
#define BNO_WAKE_Pin GPIO_PIN_2
#define BNO_WAKE_GPIO_Port GPIOA
#define ESC_2_PWM_Pin GPIO_PIN_3
#define ESC_2_PWM_GPIO_Port GPIOA
#define BNO_BOOTN_Pin GPIO_PIN_4
#define BNO_BOOTN_GPIO_Port GPIOA
#define EXT_INT_1_Pin GPIO_PIN_4
#define EXT_INT_1_GPIO_Port GPIOC
#define EXT_INT_1_EXTI_IRQn EXTI4_IRQn
#define EXT_CS_1_Pin GPIO_PIN_5
#define EXT_CS_1_GPIO_Port GPIOC
#define SERVO_1_PWM_Pin GPIO_PIN_0
#define SERVO_1_PWM_GPIO_Port GPIOB
#define EXT_INT_2_Pin GPIO_PIN_7
#define EXT_INT_2_GPIO_Port GPIOE
#define EXT_INT_2_EXTI_IRQn EXTI7_IRQn
#define EXT_CS_2_Pin GPIO_PIN_8
#define EXT_CS_2_GPIO_Port GPIOE
#define EXT_INT_3_Pin GPIO_PIN_9
#define EXT_INT_3_GPIO_Port GPIOE
#define EXT_INT_3_EXTI_IRQn EXTI9_IRQn
#define EXT_CS_3_Pin GPIO_PIN_10
#define EXT_CS_3_GPIO_Port GPIOE
#define SERVO_2_PWM_Pin GPIO_PIN_11
#define SERVO_2_PWM_GPIO_Port GPIOE
#define EXT_INT_4_Pin GPIO_PIN_13
#define EXT_INT_4_GPIO_Port GPIOE
#define EXT_INT_4_EXTI_IRQn EXTI13_IRQn
#define EXT_CS_4_Pin GPIO_PIN_14
#define EXT_CS_4_GPIO_Port GPIOE
#define BARO1_CS_Pin GPIO_PIN_8
#define BARO1_CS_GPIO_Port GPIOD
#define BMI_GYRO_INT_2_Pin GPIO_PIN_10
#define BMI_GYRO_INT_2_GPIO_Port GPIOD
#define BMI_GYRO_INT_2_EXTI_IRQn EXTI10_IRQn
#define BMI_GYRO_INT_1_Pin GPIO_PIN_11
#define BMI_GYRO_INT_1_GPIO_Port GPIOD
#define BMI_GYRO_INT_1_EXTI_IRQn EXTI11_IRQn
#define BMI_ACC_INT_1_Pin GPIO_PIN_12
#define BMI_ACC_INT_1_GPIO_Port GPIOD
#define BMI_ACC_INT_1_EXTI_IRQn EXTI12_IRQn
#define BMI_ACC_INT_2_Pin GPIO_PIN_14
#define BMI_ACC_INT_2_GPIO_Port GPIOD
#define BMI_ACC_INT_2_EXTI_IRQn EXTI14_IRQn
#define BMI_ACC_CS_Pin GPIO_PIN_15
#define BMI_ACC_CS_GPIO_Port GPIOD
#define BMI_GYRO_CS_Pin GPIO_PIN_7
#define BMI_GYRO_CS_GPIO_Port GPIOC
#define STAT_LED_R_Pin GPIO_PIN_9
#define STAT_LED_R_GPIO_Port GPIOA
#define STAT_LED_G_Pin GPIO_PIN_10
#define STAT_LED_G_GPIO_Port GPIOA
#define SD_CARD_DETECT_Pin GPIO_PIN_3
#define SD_CARD_DETECT_GPIO_Port GPIOD
#define BNO_INT_Pin GPIO_PIN_8
#define BNO_INT_GPIO_Port GPIOB
#define BNO_INT_EXTI_IRQn EXTI8_IRQn
#define BARO2_CS_Pin GPIO_PIN_0
#define BARO2_CS_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
