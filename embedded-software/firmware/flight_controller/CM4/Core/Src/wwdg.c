/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    wwdg.c
  * @brief   This file provides code for the configuration
  *          of the WWDG instances.
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
#include "wwdg.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

WWDG_HandleTypeDef hwwdg2;

/* WWDG2 init function */
void MX_WWDG2_Init(void)
{

  /* USER CODE BEGIN WWDG2_Init 0 */

  /* USER CODE END WWDG2_Init 0 */

  /* USER CODE BEGIN WWDG2_Init 1 */

  /* USER CODE END WWDG2_Init 1 */
  hwwdg2.Instance = WWDG2;
  hwwdg2.Init.Prescaler = WWDG_PRESCALER_1;
  hwwdg2.Init.Window = 64;
  hwwdg2.Init.Counter = 64;
  hwwdg2.Init.EWIMode = WWDG_EWI_DISABLE;
  if (HAL_WWDG_Init(&hwwdg2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN WWDG2_Init 2 */

  /* USER CODE END WWDG2_Init 2 */

}

void HAL_WWDG_MspInit(WWDG_HandleTypeDef* wwdgHandle)
{

  if(wwdgHandle->Instance==WWDG2)
  {
  /* USER CODE BEGIN WWDG2_MspInit 0 */

  /* USER CODE END WWDG2_MspInit 0 */
    /* WWDG2 clock enable */
    HAL_RCCEx_WWDGxSysResetConfig(RCC_WWDG2);
    __HAL_RCC_WWDG2_CLK_ENABLE();
  /* USER CODE BEGIN WWDG2_MspInit 1 */

  /* USER CODE END WWDG2_MspInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

