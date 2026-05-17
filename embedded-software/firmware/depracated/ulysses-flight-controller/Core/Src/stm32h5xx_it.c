/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h5xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32h5xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "crash/crash_dump.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
volatile uint32_t g_hardfault_cfsr;
volatile uint32_t g_hardfault_hfsr;
volatile uint32_t g_hardfault_bfar;
volatile uint32_t g_hardfault_shcsr;
volatile uint32_t g_hardfault_lr;
volatile uint32_t g_hardfault_sp;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern SD_HandleTypeDef hsd1;
extern DMA_HandleTypeDef handle_GPDMA1_Channel5;
extern DMA_HandleTypeDef handle_GPDMA1_Channel4;
extern DMA_HandleTypeDef handle_GPDMA1_Channel1;
extern DMA_HandleTypeDef handle_GPDMA1_Channel0;
extern DMA_HandleTypeDef handle_GPDMA1_Channel3;
extern DMA_HandleTypeDef handle_GPDMA1_Channel2;
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi4;
extern TIM_HandleTypeDef htim4;
extern DMA_HandleTypeDef handle_GPDMA2_Channel5;
extern DMA_HandleTypeDef handle_GPDMA2_Channel4;
extern DMA_HandleTypeDef handle_GPDMA2_Channel0;
extern DMA_HandleTypeDef handle_GPDMA2_Channel3;
extern DMA_HandleTypeDef handle_GPDMA2_Channel2;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim6;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
  __asm volatile (
      "tst lr, #4        \n"
      "ite eq             \n"
      "mrseq r0, msp     \n"
      "mrsne r0, psp     \n"
      "mov r1, lr        \n"
      "mov r2, #0        \n" /* CRASH_FAULT_HARD */
      "b crash_dump_handler \n"
  );
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */
  __asm volatile (
      "tst lr, #4        \n"
      "ite eq             \n"
      "mrseq r0, msp     \n"
      "mrsne r0, psp     \n"
      "mov r1, lr        \n"
      "mov r2, #1        \n" /* CRASH_FAULT_MEMMANAGE */
      "b crash_dump_handler \n"
  );
  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */
  __asm volatile (
      "tst lr, #4        \n"
      "ite eq             \n"
      "mrseq r0, msp     \n"
      "mrsne r0, psp     \n"
      "mov r1, lr        \n"
      "mov r2, #2        \n" /* CRASH_FAULT_BUS */
      "b crash_dump_handler \n"
  );
  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */
  __asm volatile (
      "tst lr, #4        \n"
      "ite eq             \n"
      "mrseq r0, msp     \n"
      "mrsne r0, psp     \n"
      "mov r1, lr        \n"
      "mov r2, #3        \n" /* CRASH_FAULT_USAGE */
      "b crash_dump_handler \n"
  );
  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32H5xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h5xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI Line4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */

  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(EXT_INT_1_Pin);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles EXTI Line7 interrupt.
  */
void EXTI7_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI7_IRQn 0 */

  /* USER CODE END EXTI7_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(EXT_INT_2_Pin);
  /* USER CODE BEGIN EXTI7_IRQn 1 */

  /* USER CODE END EXTI7_IRQn 1 */
}

/**
  * @brief This function handles EXTI Line8 interrupt.
  */
void EXTI8_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI8_IRQn 0 */

  /* USER CODE END EXTI8_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(BNO_INT_Pin);
  /* USER CODE BEGIN EXTI8_IRQn 1 */

  /* USER CODE END EXTI8_IRQn 1 */
}

/**
  * @brief This function handles EXTI Line9 interrupt.
  */
void EXTI9_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_IRQn 0 */

  /* USER CODE END EXTI9_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(EXT_INT_3_Pin);
  /* USER CODE BEGIN EXTI9_IRQn 1 */

  /* USER CODE END EXTI9_IRQn 1 */
}

/**
  * @brief This function handles EXTI Line10 interrupt.
  */
void EXTI10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI10_IRQn 0 */

  /* USER CODE END EXTI10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(BMI_GYRO_INT_2_Pin);
  /* USER CODE BEGIN EXTI10_IRQn 1 */

  /* USER CODE END EXTI10_IRQn 1 */
}

/**
  * @brief This function handles EXTI Line11 interrupt.
  */
void EXTI11_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI11_IRQn 0 */

  /* USER CODE END EXTI11_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(BMI_GYRO_INT_1_Pin);
  /* USER CODE BEGIN EXTI11_IRQn 1 */

  /* USER CODE END EXTI11_IRQn 1 */
}

/**
  * @brief This function handles EXTI Line12 interrupt.
  */
void EXTI12_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI12_IRQn 0 */

  /* USER CODE END EXTI12_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(BMI_ACC_INT_1_Pin);
  /* USER CODE BEGIN EXTI12_IRQn 1 */

  /* USER CODE END EXTI12_IRQn 1 */
}

/**
  * @brief This function handles EXTI Line13 interrupt.
  */
void EXTI13_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI13_IRQn 0 */

  /* USER CODE END EXTI13_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(EXT_INT_4_Pin);
  /* USER CODE BEGIN EXTI13_IRQn 1 */

  /* USER CODE END EXTI13_IRQn 1 */
}

/**
  * @brief This function handles EXTI Line14 interrupt.
  */
void EXTI14_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI14_IRQn 0 */

  /* USER CODE END EXTI14_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(BMI_ACC_INT_2_Pin);
  /* USER CODE BEGIN EXTI14_IRQn 1 */

  /* USER CODE END EXTI14_IRQn 1 */
}

/**
  * @brief This function handles GPDMA1 Channel 0 global interrupt.
  */
void GPDMA1_Channel0_IRQHandler(void)
{
  /* USER CODE BEGIN GPDMA1_Channel0_IRQn 0 */

  /* USER CODE END GPDMA1_Channel0_IRQn 0 */
  HAL_DMA_IRQHandler(&handle_GPDMA1_Channel0);
  /* USER CODE BEGIN GPDMA1_Channel0_IRQn 1 */

  /* USER CODE END GPDMA1_Channel0_IRQn 1 */
}

/**
  * @brief This function handles GPDMA1 Channel 1 global interrupt.
  */
void GPDMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN GPDMA1_Channel1_IRQn 0 */

  /* USER CODE END GPDMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&handle_GPDMA1_Channel1);
  /* USER CODE BEGIN GPDMA1_Channel1_IRQn 1 */

  /* USER CODE END GPDMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles GPDMA1 Channel 2 global interrupt.
  */
void GPDMA1_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN GPDMA1_Channel2_IRQn 0 */

  /* USER CODE END GPDMA1_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&handle_GPDMA1_Channel2);
  /* USER CODE BEGIN GPDMA1_Channel2_IRQn 1 */

  /* USER CODE END GPDMA1_Channel2_IRQn 1 */
}

/**
  * @brief This function handles GPDMA1 Channel 3 global interrupt.
  */
void GPDMA1_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN GPDMA1_Channel3_IRQn 0 */

  /* USER CODE END GPDMA1_Channel3_IRQn 0 */
  HAL_DMA_IRQHandler(&handle_GPDMA1_Channel3);
  /* USER CODE BEGIN GPDMA1_Channel3_IRQn 1 */

  /* USER CODE END GPDMA1_Channel3_IRQn 1 */
}

/**
  * @brief This function handles GPDMA1 Channel 4 global interrupt.
  */
void GPDMA1_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN GPDMA1_Channel4_IRQn 0 */

  /* USER CODE END GPDMA1_Channel4_IRQn 0 */
  HAL_DMA_IRQHandler(&handle_GPDMA1_Channel4);
  /* USER CODE BEGIN GPDMA1_Channel4_IRQn 1 */

  /* USER CODE END GPDMA1_Channel4_IRQn 1 */
}

/**
  * @brief This function handles GPDMA1 Channel 5 global interrupt.
  */
void GPDMA1_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN GPDMA1_Channel5_IRQn 0 */

  /* USER CODE END GPDMA1_Channel5_IRQn 0 */
  HAL_DMA_IRQHandler(&handle_GPDMA1_Channel5);
  /* USER CODE BEGIN GPDMA1_Channel5_IRQn 1 */

  /* USER CODE END GPDMA1_Channel5_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt.
  */
void TIM6_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_IRQn 0 */

  /* USER CODE END TIM6_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_IRQn 1 */

  /* USER CODE END TIM6_IRQn 1 */
}

/**
  * @brief This function handles SPI1 global interrupt.
  */
void SPI1_IRQHandler(void)
{
  /* USER CODE BEGIN SPI1_IRQn 0 */

  /* USER CODE END SPI1_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi1);
  /* USER CODE BEGIN SPI1_IRQn 1 */

  /* USER CODE END SPI1_IRQn 1 */
}

/**
  * @brief This function handles SPI2 global interrupt.
  */
void SPI2_IRQHandler(void)
{
  /* USER CODE BEGIN SPI2_IRQn 0 */

  /* USER CODE END SPI2_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi2);
  /* USER CODE BEGIN SPI2_IRQn 1 */

  /* USER CODE END SPI2_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles UART4 global interrupt.
  */
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */

  /* USER CODE END UART4_IRQn 0 */
  HAL_UART_IRQHandler(&huart4);
  /* USER CODE BEGIN UART4_IRQn 1 */

  /* USER CODE END UART4_IRQn 1 */
}

/**
  * @brief This function handles SDMMC1 global interrupt.
  */
void SDMMC1_IRQHandler(void)
{
  /* USER CODE BEGIN SDMMC1_IRQn 0 */

  /* USER CODE END SDMMC1_IRQn 0 */
  HAL_SD_IRQHandler(&hsd1);
  /* USER CODE BEGIN SDMMC1_IRQn 1 */

  /* USER CODE END SDMMC1_IRQn 1 */
}

/**
  * @brief This function handles SPI4 global interrupt.
  */
void SPI4_IRQHandler(void)
{
  /* USER CODE BEGIN SPI4_IRQn 0 */

  /* USER CODE END SPI4_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi4);
  /* USER CODE BEGIN SPI4_IRQn 1 */

  /* USER CODE END SPI4_IRQn 1 */
}

/**
  * @brief This function handles GPDMA2 Channel 0 global interrupt.
  */
void GPDMA2_Channel0_IRQHandler(void)
{
  /* USER CODE BEGIN GPDMA2_Channel0_IRQn 0 */

  /* USER CODE END GPDMA2_Channel0_IRQn 0 */
  HAL_DMA_IRQHandler(&handle_GPDMA2_Channel0);
  /* USER CODE BEGIN GPDMA2_Channel0_IRQn 1 */

  /* USER CODE END GPDMA2_Channel0_IRQn 1 */
}

/**
  * @brief This function handles GPDMA2 Channel 2 global interrupt.
  */
void GPDMA2_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN GPDMA2_Channel2_IRQn 0 */

  /* USER CODE END GPDMA2_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&handle_GPDMA2_Channel2);
  /* USER CODE BEGIN GPDMA2_Channel2_IRQn 1 */

  /* USER CODE END GPDMA2_Channel2_IRQn 1 */
}

/**
  * @brief This function handles GPDMA2 Channel 3 global interrupt.
  */
void GPDMA2_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN GPDMA2_Channel3_IRQn 0 */

  /* USER CODE END GPDMA2_Channel3_IRQn 0 */
  HAL_DMA_IRQHandler(&handle_GPDMA2_Channel3);
  /* USER CODE BEGIN GPDMA2_Channel3_IRQn 1 */

  /* USER CODE END GPDMA2_Channel3_IRQn 1 */
}

/**
  * @brief This function handles GPDMA2 Channel 4 global interrupt.
  */
void GPDMA2_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN GPDMA2_Channel4_IRQn 0 */

  /* USER CODE END GPDMA2_Channel4_IRQn 0 */
  HAL_DMA_IRQHandler(&handle_GPDMA2_Channel4);
  /* USER CODE BEGIN GPDMA2_Channel4_IRQn 1 */

  /* USER CODE END GPDMA2_Channel4_IRQn 1 */
}

/**
  * @brief This function handles GPDMA2 Channel 5 global interrupt.
  */
void GPDMA2_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN GPDMA2_Channel5_IRQn 0 */

  /* USER CODE END GPDMA2_Channel5_IRQn 0 */
  HAL_DMA_IRQHandler(&handle_GPDMA2_Channel5);
  /* USER CODE BEGIN GPDMA2_Channel5_IRQn 1 */

  /* USER CODE END GPDMA2_Channel5_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
