/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPI_MMC5983MA hspi1
#define SPI_MS5611 hspi2
#define SPI_ICM40609 hspi3
#define SPI_BMI088 hspi4
#define UART_mmWAVE huart4
#define UART_RFD900X huart7
#define UART_SAM_M10Q huart5
#define UART_MaUWB_A huart1
#define UART_MaUWB_B huart6
#define BMI_ACC_CS_Pin GPIO_PIN_3
#define BMI_ACC_CS_GPIO_Port GPIOE
#define BMI_GYRO_CS_Pin GPIO_PIN_4
#define BMI_GYRO_CS_GPIO_Port GPIOE
#define BMI_ACC_INT1_Pin GPIO_PIN_13
#define BMI_ACC_INT1_GPIO_Port GPIOC
#define BMI_ACC_INT2_Pin GPIO_PIN_14
#define BMI_ACC_INT2_GPIO_Port GPIOC
#define EXTRA_PC15_Pin GPIO_PIN_15
#define EXTRA_PC15_GPIO_Port GPIOC
#define BMI_GYRO_INT2_Pin GPIO_PIN_9
#define BMI_GYRO_INT2_GPIO_Port GPIOF
#define BMI_GYRO_INT1_Pin GPIO_PIN_10
#define BMI_GYRO_INT1_GPIO_Port GPIOF
#define GNSS_RST_Pin GPIO_PIN_0
#define GNSS_RST_GPIO_Port GPIOC
#define PHY_INT_Pin GPIO_PIN_3
#define PHY_INT_GPIO_Port GPIOA
#define EXTRA_PB0_Pin GPIO_PIN_0
#define EXTRA_PB0_GPIO_Port GPIOB
#define EXTRA_PE9_Pin GPIO_PIN_9
#define EXTRA_PE9_GPIO_Port GPIOE
#define EXTRA_PE10_Pin GPIO_PIN_10
#define EXTRA_PE10_GPIO_Port GPIOE
#define ICM_INT1_Pin GPIO_PIN_12
#define ICM_INT1_GPIO_Port GPIOE
#define EXTRA_PE13_Pin GPIO_PIN_13
#define EXTRA_PE13_GPIO_Port GPIOE
#define PHY_RST_Pin GPIO_PIN_14
#define PHY_RST_GPIO_Port GPIOE
#define ICM_INT2_Pin GPIO_PIN_15
#define ICM_INT2_GPIO_Port GPIOE
#define STAT_LEDY_Pin GPIO_PIN_10
#define STAT_LEDY_GPIO_Port GPIOD
#define EXTRA_PD11_Pin GPIO_PIN_11
#define EXTRA_PD11_GPIO_Port GPIOD
#define STAT_LEDR_Pin GPIO_PIN_14
#define STAT_LEDR_GPIO_Port GPIOD
#define MMC_INT_Pin GPIO_PIN_6
#define MMC_INT_GPIO_Port GPIOG
#define STAT_LEDG_Pin GPIO_PIN_7
#define STAT_LEDG_GPIO_Port GPIOG
#define SD_CARD_DETECT_Pin GPIO_PIN_4
#define SD_CARD_DETECT_GPIO_Port GPIOD
#define EXTRA_PG13_Pin GPIO_PIN_13
#define EXTRA_PG13_GPIO_Port GPIOG
#define EXTRA_PB7_Pin GPIO_PIN_7
#define EXTRA_PB7_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
