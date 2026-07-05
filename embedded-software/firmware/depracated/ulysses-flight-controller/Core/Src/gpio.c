/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins
     PC14-OSC32_IN(OSC32_IN)   ------> RCC_OSC32_IN
     PC15-OSC32_OUT(OSC32_OUT)   ------> RCC_OSC32_OUT
     PH0-OSC_IN(PH0)   ------> RCC_OSC_IN
     PH1-OSC_OUT(PH1)   ------> RCC_OSC_OUT
     PA0   ------> PWR_WKUP1
     PA13(JTMS/SWDIO)   ------> DEBUG_JTMS-SWDIO
     PA14(JTCK/SWCLK)   ------> DEBUG_JTCK-SWCLK
     PB3(JTDO/TRACESWO)   ------> DEBUG_JTDO-SWO
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, BNO_RST_Pin|BNO_CS_Pin|EXT_CS_2_Pin|EXT_CS_3_Pin
                          |EXT_CS_4_Pin|BARO2_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, BNO_WAKE_Pin|BNO_BOOTN_Pin|STAT_LED_R_Pin|STAT_LED_G_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, EXT_CS_1_Pin|BMI_GYRO_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, BARO1_CS_Pin|BMI_ACC_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BNO_RST_Pin BNO_CS_Pin EXT_CS_2_Pin EXT_CS_3_Pin
                           EXT_CS_4_Pin BARO2_CS_Pin */
  GPIO_InitStruct.Pin = BNO_RST_Pin|BNO_CS_Pin|EXT_CS_2_Pin|EXT_CS_3_Pin
                          |EXT_CS_4_Pin|BARO2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : BNO_WAKE_Pin BNO_BOOTN_Pin STAT_LED_R_Pin STAT_LED_G_Pin */
  GPIO_InitStruct.Pin = BNO_WAKE_Pin|BNO_BOOTN_Pin|STAT_LED_R_Pin|STAT_LED_G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : EXT_INT_1_Pin */
  GPIO_InitStruct.Pin = EXT_INT_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EXT_INT_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EXT_CS_1_Pin BMI_GYRO_CS_Pin */
  GPIO_InitStruct.Pin = EXT_CS_1_Pin|BMI_GYRO_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : EXT_INT_2_Pin */
  GPIO_InitStruct.Pin = EXT_INT_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(EXT_INT_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EXT_INT_3_Pin EXT_INT_4_Pin */
  GPIO_InitStruct.Pin = EXT_INT_3_Pin|EXT_INT_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : BARO1_CS_Pin BMI_ACC_CS_Pin */
  GPIO_InitStruct.Pin = BARO1_CS_Pin|BMI_ACC_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : BMI_GYRO_INT_2_Pin BMI_GYRO_INT_1_Pin BMI_ACC_INT_1_Pin BMI_ACC_INT_2_Pin */
  GPIO_InitStruct.Pin = BMI_GYRO_INT_2_Pin|BMI_GYRO_INT_1_Pin|BMI_ACC_INT_1_Pin|BMI_ACC_INT_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CARD_DETECT_Pin */
  GPIO_InitStruct.Pin = SD_CARD_DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SD_CARD_DETECT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BNO_INT_Pin */
  GPIO_InitStruct.Pin = BNO_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BNO_INT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI7_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI7_IRQn);

  HAL_NVIC_SetPriority(EXTI8_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI8_IRQn);

  HAL_NVIC_SetPriority(EXTI9_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI9_IRQn);

  HAL_NVIC_SetPriority(EXTI10_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI10_IRQn);

  HAL_NVIC_SetPriority(EXTI11_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI11_IRQn);

  HAL_NVIC_SetPriority(EXTI12_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI12_IRQn);

  HAL_NVIC_SetPriority(EXTI13_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI13_IRQn);

  HAL_NVIC_SetPriority(EXTI14_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI14_IRQn);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
