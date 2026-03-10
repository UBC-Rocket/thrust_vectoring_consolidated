/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32h5xx_hal_msp.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
extern DMA_HandleTypeDef handle_GPDMA1_Channel5;

extern DMA_HandleTypeDef handle_GPDMA1_Channel4;

extern DMA_HandleTypeDef handle_GPDMA1_Channel1;

extern DMA_HandleTypeDef handle_GPDMA1_Channel0;

extern DMA_HandleTypeDef handle_GPDMA1_Channel3;

extern DMA_HandleTypeDef handle_GPDMA1_Channel2;

extern DMA_HandleTypeDef handle_GPDMA2_Channel5;

extern DMA_HandleTypeDef handle_GPDMA2_Channel4;

extern DMA_HandleTypeDef handle_GPDMA2_Channel0;

extern DMA_HandleTypeDef handle_GPDMA2_Channel3;

extern DMA_HandleTypeDef handle_GPDMA2_Channel2;

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                                            /**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{

  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  /* System interrupt init*/
  /* PendSV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

/**
  * @brief DCACHE MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hdcache: DCACHE handle pointer
  * @retval None
  */
void HAL_DCACHE_MspInit(DCACHE_HandleTypeDef* hdcache)
{
  if(hdcache->Instance==DCACHE1)
  {
    /* USER CODE BEGIN DCACHE1_MspInit 0 */

    /* USER CODE END DCACHE1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_DCACHE1_CLK_ENABLE();
    /* USER CODE BEGIN DCACHE1_MspInit 1 */

    /* USER CODE END DCACHE1_MspInit 1 */

  }

}

/**
  * @brief DCACHE MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param hdcache: DCACHE handle pointer
  * @retval None
  */
void HAL_DCACHE_MspDeInit(DCACHE_HandleTypeDef* hdcache)
{
  if(hdcache->Instance==DCACHE1)
  {
    /* USER CODE BEGIN DCACHE1_MspDeInit 0 */

    /* USER CODE END DCACHE1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_DCACHE1_CLK_DISABLE();
    /* USER CODE BEGIN DCACHE1_MspDeInit 1 */

    /* USER CODE END DCACHE1_MspDeInit 1 */
  }

}

/**
  * @brief SD MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hsd: SD handle pointer
  * @retval None
  */
void HAL_SD_MspInit(SD_HandleTypeDef* hsd)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(hsd->Instance==SDMMC1)
  {
    /* USER CODE BEGIN SDMMC1_MspInit 0 */

    /* USER CODE END SDMMC1_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SDMMC1;
    PeriphClkInitStruct.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_PLL1Q;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_SDMMC1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**SDMMC1 GPIO Configuration
    PB2     ------> SDMMC1_CMD
    PB13     ------> SDMMC1_D0
    PC9     ------> SDMMC1_D1
    PC10     ------> SDMMC1_D2
    PC11     ------> SDMMC1_D3
    PC12     ------> SDMMC1_CK
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* SDMMC1 interrupt Init */
    HAL_NVIC_SetPriority(SDMMC1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(SDMMC1_IRQn);
    /* USER CODE BEGIN SDMMC1_MspInit 1 */
    HAL_NVIC_SetPriority(SDMMC1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(SDMMC1_IRQn);

    /* USER CODE END SDMMC1_MspInit 1 */

  }

}

/**
  * @brief SD MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param hsd: SD handle pointer
  * @retval None
  */
void HAL_SD_MspDeInit(SD_HandleTypeDef* hsd)
{
  if(hsd->Instance==SDMMC1)
  {
    /* USER CODE BEGIN SDMMC1_MspDeInit 0 */

    /* USER CODE END SDMMC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SDMMC1_CLK_DISABLE();

    /**SDMMC1 GPIO Configuration
    PB2     ------> SDMMC1_CMD
    PB13     ------> SDMMC1_D0
    PC9     ------> SDMMC1_D1
    PC10     ------> SDMMC1_D2
    PC11     ------> SDMMC1_D3
    PC12     ------> SDMMC1_CK
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_2|GPIO_PIN_13);

    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12);

    /* SDMMC1 interrupt DeInit */
    HAL_NVIC_DisableIRQ(SDMMC1_IRQn);
    /* USER CODE BEGIN SDMMC1_MspDeInit 1 */

    /* USER CODE END SDMMC1_MspDeInit 1 */
  }

}

/**
  * @brief SPI MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hspi: SPI handle pointer
  * @retval None
  */
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(hspi->Instance==SPI1)
  {
    /* USER CODE BEGIN SPI1_MspInit 0 */

    /* USER CODE END SPI1_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI1;
    PeriphClkInitStruct.Spi1ClockSelection = RCC_SPI1CLKSOURCE_PLL1Q;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* SPI1 DMA Init */
    /* GPDMA1_REQUEST_SPI1_RX Init */
    handle_GPDMA1_Channel5.Instance = GPDMA1_Channel5;
    handle_GPDMA1_Channel5.Init.Request = GPDMA1_REQUEST_SPI1_RX;
    handle_GPDMA1_Channel5.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
    handle_GPDMA1_Channel5.Init.Direction = DMA_PERIPH_TO_MEMORY;
    handle_GPDMA1_Channel5.Init.SrcInc = DMA_SINC_FIXED;
    handle_GPDMA1_Channel5.Init.DestInc = DMA_DINC_INCREMENTED;
    handle_GPDMA1_Channel5.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_BYTE;
    handle_GPDMA1_Channel5.Init.DestDataWidth = DMA_DEST_DATAWIDTH_BYTE;
    handle_GPDMA1_Channel5.Init.Priority = DMA_LOW_PRIORITY_LOW_WEIGHT;
    handle_GPDMA1_Channel5.Init.SrcBurstLength = 1;
    handle_GPDMA1_Channel5.Init.DestBurstLength = 1;
    handle_GPDMA1_Channel5.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0|DMA_DEST_ALLOCATED_PORT0;
    handle_GPDMA1_Channel5.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
    handle_GPDMA1_Channel5.Init.Mode = DMA_NORMAL;
    if (HAL_DMA_Init(&handle_GPDMA1_Channel5) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hspi, hdmarx, handle_GPDMA1_Channel5);

    if (HAL_DMA_ConfigChannelAttributes(&handle_GPDMA1_Channel5, DMA_CHANNEL_NPRIV) != HAL_OK)
    {
      Error_Handler();
    }

    /* GPDMA1_REQUEST_SPI1_TX Init */
    handle_GPDMA1_Channel4.Instance = GPDMA1_Channel4;
    handle_GPDMA1_Channel4.Init.Request = GPDMA1_REQUEST_SPI1_TX;
    handle_GPDMA1_Channel4.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
    handle_GPDMA1_Channel4.Init.Direction = DMA_MEMORY_TO_PERIPH;
    handle_GPDMA1_Channel4.Init.SrcInc = DMA_SINC_INCREMENTED;
    handle_GPDMA1_Channel4.Init.DestInc = DMA_DINC_FIXED;
    handle_GPDMA1_Channel4.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_BYTE;
    handle_GPDMA1_Channel4.Init.DestDataWidth = DMA_DEST_DATAWIDTH_BYTE;
    handle_GPDMA1_Channel4.Init.Priority = DMA_LOW_PRIORITY_LOW_WEIGHT;
    handle_GPDMA1_Channel4.Init.SrcBurstLength = 1;
    handle_GPDMA1_Channel4.Init.DestBurstLength = 1;
    handle_GPDMA1_Channel4.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0|DMA_DEST_ALLOCATED_PORT0;
    handle_GPDMA1_Channel4.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
    handle_GPDMA1_Channel4.Init.Mode = DMA_NORMAL;
    if (HAL_DMA_Init(&handle_GPDMA1_Channel4) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hspi, hdmatx, handle_GPDMA1_Channel4);

    if (HAL_DMA_ConfigChannelAttributes(&handle_GPDMA1_Channel4, DMA_CHANNEL_NPRIV) != HAL_OK)
    {
      Error_Handler();
    }

    /* SPI1 interrupt Init */
    HAL_NVIC_SetPriority(SPI1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(SPI1_IRQn);
    /* USER CODE BEGIN SPI1_MspInit 1 */

    /* USER CODE END SPI1_MspInit 1 */
  }
  else if(hspi->Instance==SPI2)
  {
    /* USER CODE BEGIN SPI2_MspInit 0 */

    /* USER CODE END SPI2_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI2;
    PeriphClkInitStruct.Spi2ClockSelection = RCC_SPI2CLKSOURCE_PLL1Q;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_SPI2_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**SPI2 GPIO Configuration
    PC1     ------> SPI2_MOSI
    PC2     ------> SPI2_MISO
    PB10     ------> SPI2_SCK
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* SPI2 DMA Init */
    /* GPDMA1_REQUEST_SPI2_RX Init */
    handle_GPDMA1_Channel1.Instance = GPDMA1_Channel1;
    handle_GPDMA1_Channel1.Init.Request = GPDMA1_REQUEST_SPI2_RX;
    handle_GPDMA1_Channel1.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
    handle_GPDMA1_Channel1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    handle_GPDMA1_Channel1.Init.SrcInc = DMA_SINC_FIXED;
    handle_GPDMA1_Channel1.Init.DestInc = DMA_DINC_INCREMENTED;
    handle_GPDMA1_Channel1.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_BYTE;
    handle_GPDMA1_Channel1.Init.DestDataWidth = DMA_DEST_DATAWIDTH_BYTE;
    handle_GPDMA1_Channel1.Init.Priority = DMA_LOW_PRIORITY_LOW_WEIGHT;
    handle_GPDMA1_Channel1.Init.SrcBurstLength = 1;
    handle_GPDMA1_Channel1.Init.DestBurstLength = 1;
    handle_GPDMA1_Channel1.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0|DMA_DEST_ALLOCATED_PORT0;
    handle_GPDMA1_Channel1.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
    handle_GPDMA1_Channel1.Init.Mode = DMA_NORMAL;
    if (HAL_DMA_Init(&handle_GPDMA1_Channel1) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hspi, hdmarx, handle_GPDMA1_Channel1);

    if (HAL_DMA_ConfigChannelAttributes(&handle_GPDMA1_Channel1, DMA_CHANNEL_NPRIV) != HAL_OK)
    {
      Error_Handler();
    }

    /* GPDMA1_REQUEST_SPI2_TX Init */
    handle_GPDMA1_Channel0.Instance = GPDMA1_Channel0;
    handle_GPDMA1_Channel0.Init.Request = GPDMA1_REQUEST_SPI2_TX;
    handle_GPDMA1_Channel0.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
    handle_GPDMA1_Channel0.Init.Direction = DMA_MEMORY_TO_PERIPH;
    handle_GPDMA1_Channel0.Init.SrcInc = DMA_SINC_INCREMENTED;
    handle_GPDMA1_Channel0.Init.DestInc = DMA_DINC_FIXED;
    handle_GPDMA1_Channel0.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_BYTE;
    handle_GPDMA1_Channel0.Init.DestDataWidth = DMA_DEST_DATAWIDTH_BYTE;
    handle_GPDMA1_Channel0.Init.Priority = DMA_LOW_PRIORITY_LOW_WEIGHT;
    handle_GPDMA1_Channel0.Init.SrcBurstLength = 1;
    handle_GPDMA1_Channel0.Init.DestBurstLength = 1;
    handle_GPDMA1_Channel0.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0|DMA_DEST_ALLOCATED_PORT0;
    handle_GPDMA1_Channel0.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
    handle_GPDMA1_Channel0.Init.Mode = DMA_NORMAL;
    if (HAL_DMA_Init(&handle_GPDMA1_Channel0) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hspi, hdmatx, handle_GPDMA1_Channel0);

    if (HAL_DMA_ConfigChannelAttributes(&handle_GPDMA1_Channel0, DMA_CHANNEL_NPRIV) != HAL_OK)
    {
      Error_Handler();
    }

    /* SPI2 interrupt Init */
    HAL_NVIC_SetPriority(SPI2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(SPI2_IRQn);
    /* USER CODE BEGIN SPI2_MspInit 1 */

    /* USER CODE END SPI2_MspInit 1 */
  }
  else if(hspi->Instance==SPI4)
  {
    /* USER CODE BEGIN SPI4_MspInit 0 */

    /* USER CODE END SPI4_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI4;
    PeriphClkInitStruct.Spi4ClockSelection = RCC_SPI4CLKSOURCE_PCLK2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_SPI4_CLK_ENABLE();

    __HAL_RCC_GPIOE_CLK_ENABLE();
    /**SPI4 GPIO Configuration
    PE5     ------> SPI4_MISO
    PE6     ------> SPI4_MOSI
    PE12     ------> SPI4_SCK
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI4;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* SPI4 DMA Init */
    /* GPDMA1_REQUEST_SPI4_TX Init */
    handle_GPDMA1_Channel3.Instance = GPDMA1_Channel3;
    handle_GPDMA1_Channel3.Init.Request = GPDMA1_REQUEST_SPI4_TX;
    handle_GPDMA1_Channel3.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
    handle_GPDMA1_Channel3.Init.Direction = DMA_MEMORY_TO_PERIPH;
    handle_GPDMA1_Channel3.Init.SrcInc = DMA_SINC_INCREMENTED;
    handle_GPDMA1_Channel3.Init.DestInc = DMA_DINC_FIXED;
    handle_GPDMA1_Channel3.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_BYTE;
    handle_GPDMA1_Channel3.Init.DestDataWidth = DMA_DEST_DATAWIDTH_BYTE;
    handle_GPDMA1_Channel3.Init.Priority = DMA_LOW_PRIORITY_LOW_WEIGHT;
    handle_GPDMA1_Channel3.Init.SrcBurstLength = 1;
    handle_GPDMA1_Channel3.Init.DestBurstLength = 1;
    handle_GPDMA1_Channel3.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0|DMA_DEST_ALLOCATED_PORT0;
    handle_GPDMA1_Channel3.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
    handle_GPDMA1_Channel3.Init.Mode = DMA_NORMAL;
    if (HAL_DMA_Init(&handle_GPDMA1_Channel3) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hspi, hdmatx, handle_GPDMA1_Channel3);

    if (HAL_DMA_ConfigChannelAttributes(&handle_GPDMA1_Channel3, DMA_CHANNEL_NPRIV) != HAL_OK)
    {
      Error_Handler();
    }

    /* GPDMA1_REQUEST_SPI4_RX Init */
    handle_GPDMA1_Channel2.Instance = GPDMA1_Channel2;
    handle_GPDMA1_Channel2.Init.Request = GPDMA1_REQUEST_SPI4_RX;
    handle_GPDMA1_Channel2.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
    handle_GPDMA1_Channel2.Init.Direction = DMA_PERIPH_TO_MEMORY;
    handle_GPDMA1_Channel2.Init.SrcInc = DMA_SINC_FIXED;
    handle_GPDMA1_Channel2.Init.DestInc = DMA_DINC_INCREMENTED;
    handle_GPDMA1_Channel2.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_BYTE;
    handle_GPDMA1_Channel2.Init.DestDataWidth = DMA_DEST_DATAWIDTH_BYTE;
    handle_GPDMA1_Channel2.Init.Priority = DMA_LOW_PRIORITY_LOW_WEIGHT;
    handle_GPDMA1_Channel2.Init.SrcBurstLength = 1;
    handle_GPDMA1_Channel2.Init.DestBurstLength = 1;
    handle_GPDMA1_Channel2.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0|DMA_DEST_ALLOCATED_PORT0;
    handle_GPDMA1_Channel2.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
    handle_GPDMA1_Channel2.Init.Mode = DMA_NORMAL;
    if (HAL_DMA_Init(&handle_GPDMA1_Channel2) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hspi, hdmarx, handle_GPDMA1_Channel2);

    if (HAL_DMA_ConfigChannelAttributes(&handle_GPDMA1_Channel2, DMA_CHANNEL_NPRIV) != HAL_OK)
    {
      Error_Handler();
    }

    /* SPI4 interrupt Init */
    HAL_NVIC_SetPriority(SPI4_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(SPI4_IRQn);
    /* USER CODE BEGIN SPI4_MspInit 1 */

    /* USER CODE END SPI4_MspInit 1 */
  }

}

/**
  * @brief SPI MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param hspi: SPI handle pointer
  * @retval None
  */
void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi)
{
  if(hspi->Instance==SPI1)
  {
    /* USER CODE BEGIN SPI1_MspDeInit 0 */

    /* USER CODE END SPI1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI1_CLK_DISABLE();

    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);

    /* SPI1 DMA DeInit */
    HAL_DMA_DeInit(hspi->hdmarx);
    HAL_DMA_DeInit(hspi->hdmatx);

    /* SPI1 interrupt DeInit */
    HAL_NVIC_DisableIRQ(SPI1_IRQn);
    /* USER CODE BEGIN SPI1_MspDeInit 1 */

    /* USER CODE END SPI1_MspDeInit 1 */
  }
  else if(hspi->Instance==SPI2)
  {
    /* USER CODE BEGIN SPI2_MspDeInit 0 */

    /* USER CODE END SPI2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI2_CLK_DISABLE();

    /**SPI2 GPIO Configuration
    PC1     ------> SPI2_MOSI
    PC2     ------> SPI2_MISO
    PB10     ------> SPI2_SCK
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_1|GPIO_PIN_2);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10);

    /* SPI2 DMA DeInit */
    HAL_DMA_DeInit(hspi->hdmarx);
    HAL_DMA_DeInit(hspi->hdmatx);

    /* SPI2 interrupt DeInit */
    HAL_NVIC_DisableIRQ(SPI2_IRQn);
    /* USER CODE BEGIN SPI2_MspDeInit 1 */

    /* USER CODE END SPI2_MspDeInit 1 */
  }
  else if(hspi->Instance==SPI4)
  {
    /* USER CODE BEGIN SPI4_MspDeInit 0 */

    /* USER CODE END SPI4_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI4_CLK_DISABLE();

    /**SPI4 GPIO Configuration
    PE5     ------> SPI4_MISO
    PE6     ------> SPI4_MOSI
    PE12     ------> SPI4_SCK
    */
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_12);

    /* SPI4 DMA DeInit */
    HAL_DMA_DeInit(hspi->hdmatx);
    HAL_DMA_DeInit(hspi->hdmarx);

    /* SPI4 interrupt DeInit */
    HAL_NVIC_DisableIRQ(SPI4_IRQn);
    /* USER CODE BEGIN SPI4_MspDeInit 1 */

    /* USER CODE END SPI4_MspDeInit 1 */
  }

}

/**
  * @brief TIM_Base MSP Initialization
  * This function configures the hardware resources used in this example
  * @param htim_base: TIM_Base handle pointer
  * @retval None
  */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base->Instance==TIM1)
  {
    /* USER CODE BEGIN TIM1_MspInit 0 */

    /* USER CODE END TIM1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM1_CLK_ENABLE();
    /* USER CODE BEGIN TIM1_MspInit 1 */

    /* USER CODE END TIM1_MspInit 1 */
  }
  else if(htim_base->Instance==TIM2)
  {
    /* USER CODE BEGIN TIM2_MspInit 0 */

    /* USER CODE END TIM2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();
    /* USER CODE BEGIN TIM2_MspInit 1 */

    /* USER CODE END TIM2_MspInit 1 */
  }
  else if(htim_base->Instance==TIM3)
  {
    /* USER CODE BEGIN TIM3_MspInit 0 */

    /* USER CODE END TIM3_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();
    /* USER CODE BEGIN TIM3_MspInit 1 */

    /* USER CODE END TIM3_MspInit 1 */
  }
  else if(htim_base->Instance==TIM4)
  {
    /* USER CODE BEGIN TIM4_MspInit 0 */

    /* USER CODE END TIM4_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM4_CLK_ENABLE();
    /* TIM4 interrupt Init */
    HAL_NVIC_SetPriority(TIM4_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
    /* USER CODE BEGIN TIM4_MspInit 1 */

    /* USER CODE END TIM4_MspInit 1 */
  }

}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htim->Instance==TIM1)
  {
    /* USER CODE BEGIN TIM1_MspPostInit 0 */

    /* USER CODE END TIM1_MspPostInit 0 */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    /**TIM1 GPIO Configuration
    PE11     ------> TIM1_CH2
    */
    GPIO_InitStruct.Pin = SERVO_2_PWM_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(SERVO_2_PWM_GPIO_Port, &GPIO_InitStruct);

    /* USER CODE BEGIN TIM1_MspPostInit 1 */

    /* USER CODE END TIM1_MspPostInit 1 */
  }
  else if(htim->Instance==TIM2)
  {
    /* USER CODE BEGIN TIM2_MspPostInit 0 */

    /* USER CODE END TIM2_MspPostInit 0 */

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM2 GPIO Configuration
    PA1     ------> TIM2_CH2
    PA3     ------> TIM2_CH4
    */
    GPIO_InitStruct.Pin = ESC_1_PWM_Pin|ESC_2_PWM_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USER CODE BEGIN TIM2_MspPostInit 1 */

    /* USER CODE END TIM2_MspPostInit 1 */
  }
  else if(htim->Instance==TIM3)
  {
    /* USER CODE BEGIN TIM3_MspPostInit 0 */

    /* USER CODE END TIM3_MspPostInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**TIM3 GPIO Configuration
    PB0     ------> TIM3_CH3
    */
    GPIO_InitStruct.Pin = SERVO_1_PWM_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(SERVO_1_PWM_GPIO_Port, &GPIO_InitStruct);

    /* USER CODE BEGIN TIM3_MspPostInit 1 */

    /* USER CODE END TIM3_MspPostInit 1 */
  }

}
/**
  * @brief TIM_Base MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param htim_base: TIM_Base handle pointer
  * @retval None
  */
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base->Instance==TIM1)
  {
    /* USER CODE BEGIN TIM1_MspDeInit 0 */

    /* USER CODE END TIM1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM1_CLK_DISABLE();
    /* USER CODE BEGIN TIM1_MspDeInit 1 */

    /* USER CODE END TIM1_MspDeInit 1 */
  }
  else if(htim_base->Instance==TIM2)
  {
    /* USER CODE BEGIN TIM2_MspDeInit 0 */

    /* USER CODE END TIM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();
    /* USER CODE BEGIN TIM2_MspDeInit 1 */

    /* USER CODE END TIM2_MspDeInit 1 */
  }
  else if(htim_base->Instance==TIM3)
  {
    /* USER CODE BEGIN TIM3_MspDeInit 0 */

    /* USER CODE END TIM3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM3_CLK_DISABLE();
    /* USER CODE BEGIN TIM3_MspDeInit 1 */

    /* USER CODE END TIM3_MspDeInit 1 */
  }
  else if(htim_base->Instance==TIM4)
  {
    /* USER CODE BEGIN TIM4_MspDeInit 0 */

    /* USER CODE END TIM4_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM4_CLK_DISABLE();

    /* TIM4 interrupt DeInit */
    HAL_NVIC_DisableIRQ(TIM4_IRQn);
    /* USER CODE BEGIN TIM4_MspDeInit 1 */

    /* USER CODE END TIM4_MspDeInit 1 */
  }

}

/**
  * @brief UART MSP Initialization
  * This function configures the hardware resources used in this example
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(huart->Instance==UART4)
  {
    /* USER CODE BEGIN UART4_MspInit 0 */

    /* USER CODE END UART4_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART4;
    PeriphClkInitStruct.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_UART4_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**UART4 GPIO Configuration
    PD0     ------> UART4_RX
    PD1     ------> UART4_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* UART4 DMA Init */
    /* GPDMA2_REQUEST_UART4_RX Init */
    handle_GPDMA2_Channel5.Instance = GPDMA2_Channel5;
    handle_GPDMA2_Channel5.Init.Request = GPDMA2_REQUEST_UART4_RX;
    handle_GPDMA2_Channel5.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
    handle_GPDMA2_Channel5.Init.Direction = DMA_PERIPH_TO_MEMORY;
    handle_GPDMA2_Channel5.Init.SrcInc = DMA_SINC_FIXED;
    handle_GPDMA2_Channel5.Init.DestInc = DMA_DINC_INCREMENTED;
    handle_GPDMA2_Channel5.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_BYTE;
    handle_GPDMA2_Channel5.Init.DestDataWidth = DMA_DEST_DATAWIDTH_BYTE;
    handle_GPDMA2_Channel5.Init.Priority = DMA_LOW_PRIORITY_LOW_WEIGHT;
    handle_GPDMA2_Channel5.Init.SrcBurstLength = 1;
    handle_GPDMA2_Channel5.Init.DestBurstLength = 1;
    handle_GPDMA2_Channel5.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0|DMA_DEST_ALLOCATED_PORT0;
    handle_GPDMA2_Channel5.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
    handle_GPDMA2_Channel5.Init.Mode = DMA_NORMAL;
    if (HAL_DMA_Init(&handle_GPDMA2_Channel5) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(huart, hdmarx, handle_GPDMA2_Channel5);

    if (HAL_DMA_ConfigChannelAttributes(&handle_GPDMA2_Channel5, DMA_CHANNEL_NPRIV) != HAL_OK)
    {
      Error_Handler();
    }

    /* GPDMA2_REQUEST_UART4_TX Init */
    handle_GPDMA2_Channel4.Instance = GPDMA2_Channel4;
    handle_GPDMA2_Channel4.Init.Request = GPDMA2_REQUEST_UART4_TX;
    handle_GPDMA2_Channel4.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
    handle_GPDMA2_Channel4.Init.Direction = DMA_MEMORY_TO_PERIPH;
    handle_GPDMA2_Channel4.Init.SrcInc = DMA_SINC_INCREMENTED;
    handle_GPDMA2_Channel4.Init.DestInc = DMA_DINC_FIXED;
    handle_GPDMA2_Channel4.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_BYTE;
    handle_GPDMA2_Channel4.Init.DestDataWidth = DMA_DEST_DATAWIDTH_BYTE;
    handle_GPDMA2_Channel4.Init.Priority = DMA_LOW_PRIORITY_LOW_WEIGHT;
    handle_GPDMA2_Channel4.Init.SrcBurstLength = 1;
    handle_GPDMA2_Channel4.Init.DestBurstLength = 1;
    handle_GPDMA2_Channel4.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0|DMA_DEST_ALLOCATED_PORT0;
    handle_GPDMA2_Channel4.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
    handle_GPDMA2_Channel4.Init.Mode = DMA_NORMAL;
    if (HAL_DMA_Init(&handle_GPDMA2_Channel4) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(huart, hdmatx, handle_GPDMA2_Channel4);

    if (HAL_DMA_ConfigChannelAttributes(&handle_GPDMA2_Channel4, DMA_CHANNEL_NPRIV) != HAL_OK)
    {
      Error_Handler();
    }

    /* UART4 interrupt Init */
    HAL_NVIC_SetPriority(UART4_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(UART4_IRQn);
    /* USER CODE BEGIN UART4_MspInit 1 */

    /* USER CODE END UART4_MspInit 1 */
  }
  else if(huart->Instance==USART1)
  {
    /* USER CODE BEGIN USART1_MspInit 0 */
#ifndef DEBUG
    return;
#endif
    /* USER CODE END USART1_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PB6     ------> USART1_TX
    PB7     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USART1 DMA Init */
    /* GPDMA2_REQUEST_USART1_TX Init */
    handle_GPDMA2_Channel0.Instance = GPDMA2_Channel0;
    handle_GPDMA2_Channel0.Init.Request = GPDMA2_REQUEST_USART1_TX;
    handle_GPDMA2_Channel0.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
    handle_GPDMA2_Channel0.Init.Direction = DMA_MEMORY_TO_PERIPH;
    handle_GPDMA2_Channel0.Init.SrcInc = DMA_SINC_INCREMENTED;
    handle_GPDMA2_Channel0.Init.DestInc = DMA_DINC_FIXED;
    handle_GPDMA2_Channel0.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_BYTE;
    handle_GPDMA2_Channel0.Init.DestDataWidth = DMA_DEST_DATAWIDTH_BYTE;
    handle_GPDMA2_Channel0.Init.Priority = DMA_LOW_PRIORITY_LOW_WEIGHT;
    handle_GPDMA2_Channel0.Init.SrcBurstLength = 1;
    handle_GPDMA2_Channel0.Init.DestBurstLength = 1;
    handle_GPDMA2_Channel0.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0|DMA_DEST_ALLOCATED_PORT0;
    handle_GPDMA2_Channel0.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
    handle_GPDMA2_Channel0.Init.Mode = DMA_NORMAL;
    if (HAL_DMA_Init(&handle_GPDMA2_Channel0) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(huart, hdmatx, handle_GPDMA2_Channel0);

    if (HAL_DMA_ConfigChannelAttributes(&handle_GPDMA2_Channel0, DMA_CHANNEL_NPRIV) != HAL_OK)
    {
      Error_Handler();
    }

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    /* USER CODE BEGIN USART1_MspInit 1 */

    /* USER CODE END USART1_MspInit 1 */
  }
  else if(huart->Instance==USART2)
  {
    /* USER CODE BEGIN USART2_MspInit 0 */

    /* USER CODE END USART2_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART2;
    PeriphClkInitStruct.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PD5     ------> USART2_TX
    PD6     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* USART2 DMA Init */
    /* GPDMA2_REQUEST_USART2_RX Init */
    handle_GPDMA2_Channel3.Instance = GPDMA2_Channel3;
    handle_GPDMA2_Channel3.Init.Request = GPDMA2_REQUEST_USART2_RX;
    handle_GPDMA2_Channel3.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
    handle_GPDMA2_Channel3.Init.Direction = DMA_PERIPH_TO_MEMORY;
    handle_GPDMA2_Channel3.Init.SrcInc = DMA_SINC_FIXED;
    handle_GPDMA2_Channel3.Init.DestInc = DMA_DINC_INCREMENTED;
    handle_GPDMA2_Channel3.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_BYTE;
    handle_GPDMA2_Channel3.Init.DestDataWidth = DMA_DEST_DATAWIDTH_BYTE;
    handle_GPDMA2_Channel3.Init.Priority = DMA_LOW_PRIORITY_LOW_WEIGHT;
    handle_GPDMA2_Channel3.Init.SrcBurstLength = 1;
    handle_GPDMA2_Channel3.Init.DestBurstLength = 1;
    handle_GPDMA2_Channel3.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0|DMA_DEST_ALLOCATED_PORT0;
    handle_GPDMA2_Channel3.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
    handle_GPDMA2_Channel3.Init.Mode = DMA_NORMAL;
    if (HAL_DMA_Init(&handle_GPDMA2_Channel3) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(huart, hdmarx, handle_GPDMA2_Channel3);

    if (HAL_DMA_ConfigChannelAttributes(&handle_GPDMA2_Channel3, DMA_CHANNEL_NPRIV) != HAL_OK)
    {
      Error_Handler();
    }

    /* GPDMA2_REQUEST_USART2_TX Init */
    handle_GPDMA2_Channel2.Instance = GPDMA2_Channel2;
    handle_GPDMA2_Channel2.Init.Request = GPDMA2_REQUEST_USART2_TX;
    handle_GPDMA2_Channel2.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
    handle_GPDMA2_Channel2.Init.Direction = DMA_MEMORY_TO_PERIPH;
    handle_GPDMA2_Channel2.Init.SrcInc = DMA_SINC_INCREMENTED;
    handle_GPDMA2_Channel2.Init.DestInc = DMA_DINC_FIXED;
    handle_GPDMA2_Channel2.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_BYTE;
    handle_GPDMA2_Channel2.Init.DestDataWidth = DMA_DEST_DATAWIDTH_BYTE;
    handle_GPDMA2_Channel2.Init.Priority = DMA_LOW_PRIORITY_LOW_WEIGHT;
    handle_GPDMA2_Channel2.Init.SrcBurstLength = 1;
    handle_GPDMA2_Channel2.Init.DestBurstLength = 1;
    handle_GPDMA2_Channel2.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0|DMA_DEST_ALLOCATED_PORT0;
    handle_GPDMA2_Channel2.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
    handle_GPDMA2_Channel2.Init.Mode = DMA_NORMAL;
    if (HAL_DMA_Init(&handle_GPDMA2_Channel2) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(huart, hdmatx, handle_GPDMA2_Channel2);

    if (HAL_DMA_ConfigChannelAttributes(&handle_GPDMA2_Channel2, DMA_CHANNEL_NPRIV) != HAL_OK)
    {
      Error_Handler();
    }

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    /* USER CODE BEGIN USART2_MspInit 1 */

    /* USER CODE END USART2_MspInit 1 */
  }

}

/**
  * @brief UART MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
  if(huart->Instance==UART4)
  {
    /* USER CODE BEGIN UART4_MspDeInit 0 */

    /* USER CODE END UART4_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART4_CLK_DISABLE();

    /**UART4 GPIO Configuration
    PD0     ------> UART4_RX
    PD1     ------> UART4_TX
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0|GPIO_PIN_1);

    /* UART4 DMA DeInit */
    HAL_DMA_DeInit(huart->hdmarx);
    HAL_DMA_DeInit(huart->hdmatx);

    /* UART4 interrupt DeInit */
    HAL_NVIC_DisableIRQ(UART4_IRQn);
    /* USER CODE BEGIN UART4_MspDeInit 1 */

    /* USER CODE END UART4_MspDeInit 1 */
  }
  else if(huart->Instance==USART1)
  {
    /* USER CODE BEGIN USART1_MspDeInit 0 */
#ifndef DEBUG
    return;
#endif
    /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PB6     ------> USART1_TX
    PB7     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6|GPIO_PIN_7);

    /* USART1 DMA DeInit */
    HAL_DMA_DeInit(huart->hdmatx);

    /* USART1 interrupt DeInit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
    /* USER CODE BEGIN USART1_MspDeInit 1 */

    /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(huart->Instance==USART2)
  {
    /* USER CODE BEGIN USART2_MspDeInit 0 */

    /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PD5     ------> USART2_TX
    PD6     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_5|GPIO_PIN_6);

    /* USART2 DMA DeInit */
    HAL_DMA_DeInit(huart->hdmarx);
    HAL_DMA_DeInit(huart->hdmatx);

    /* USART2 interrupt DeInit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
    /* USER CODE BEGIN USART2_MspDeInit 1 */

    /* USER CODE END USART2_MspDeInit 1 */
  }

}

/**
  * @brief PCD MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hpcd: PCD handle pointer
  * @retval None
  */
void HAL_PCD_MspInit(PCD_HandleTypeDef* hpcd)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(hpcd->Instance==USB_DRD_FS)
  {
    /* USER CODE BEGIN USB_DRD_FS_MspInit 0 */

    /* USER CODE END USB_DRD_FS_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
    PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* Enable VDDUSB */
    HAL_PWREx_EnableVddUSB();
    /* Peripheral clock enable */
    __HAL_RCC_USB_CLK_ENABLE();
    /* USER CODE BEGIN USB_DRD_FS_MspInit 1 */

    /* USER CODE END USB_DRD_FS_MspInit 1 */

  }

}

/**
  * @brief PCD MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param hpcd: PCD handle pointer
  * @retval None
  */
void HAL_PCD_MspDeInit(PCD_HandleTypeDef* hpcd)
{
  if(hpcd->Instance==USB_DRD_FS)
  {
    /* USER CODE BEGIN USB_DRD_FS_MspDeInit 0 */

    /* USER CODE END USB_DRD_FS_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USB_CLK_DISABLE();
    /* USER CODE BEGIN USB_DRD_FS_MspDeInit 1 */

    /* USER CODE END USB_DRD_FS_MspDeInit 1 */
  }

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
