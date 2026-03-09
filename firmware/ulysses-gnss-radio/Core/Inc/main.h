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
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

typedef enum {
    BOARD_STATE_IDLE = 0xFA,            // listening for radio messages, low rate GPS data intake
    BOARD_STATE_RUNNING = 0xFB,         // full rate GPS data, radio active
    BOARD_STATE_GPS_ERROR = 0xFC,       // GPS UART not responding or invalid data
    BOARD_STATE_RADIO_ERROR = 0xFD,     // Radio UART not responding or invalid data
    BOARD_STATE_QUEUE_OVERFLOW = 0xFE,  // data coming in faster than FC is reading
    BOARD_STATE_ERROR = 0xFF            // unknown error occurred, may still be serving data
} board_state_t;

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
#define FC_INT_Pin GPIO_PIN_1
#define FC_INT_GPIO_Port GPIOB
#define STAT_LEDR_Pin GPIO_PIN_2
#define STAT_LEDR_GPIO_Port GPIOB
#define GPS_RST_Pin GPIO_PIN_0
#define GPS_RST_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
