/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32g0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "spi_slave.h"
#include "uart_callbacks.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_spi1_tx;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart5_tx;
extern DMA_HandleTypeDef hdma_usart5_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0+ Processor Interruption and Exception Handlers          */
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

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel 1 interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */
  // SPI1 RX DMA handler (for SPI slave) - we handle DMA ourselves
  spi_slave_dma1_ch1_irq_handler();
  return;  // Skip HAL handler - we use register-level DMA
  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_tx);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel 2 and channel 3 interrupts.
  */
void DMA1_Channel2_3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_3_IRQn 0 */
  /* Channel 2 (SPI1_TX): spi_slave.c sets TCIE=0, no interrupt generated.
     Transaction end is detected via NSS rising edge (EXTI), not DMA TC. */

  /* Channel 3 (USART1_TX): handle DMA completion for debug UART TX.
     Must check GIF3 (not just TCIF3) because HAL_DMA_Start_IT enables
     HT interrupt — if we only matched TC|TE, HTIF3 would never clear
     and the ISR would re-enter infinitely. */
  if (DMA1->ISR & DMA_ISR_GIF3) {
    HAL_DMA_IRQHandler(&hdma_usart1_tx);
  }
  return;
  /* USER CODE END DMA1_Channel2_3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_rx);
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  /* USER CODE BEGIN DMA1_Channel2_3_IRQn 1 */

  /* USER CODE END DMA1_Channel2_3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 Ch4 to Ch7, DMA2 Ch1 to Ch5 and DMAMUX1 Overrun Interrupts.
  */
void DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX1_OVR_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX1_OVR_IRQn 0 */

  /* USER CODE END DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX1_OVR_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  HAL_DMA_IRQHandler(&hdma_usart5_tx);
  HAL_DMA_IRQHandler(&hdma_usart5_rx);
  HAL_DMA_IRQHandler(&hdma_usart6_tx);
  HAL_DMA_IRQHandler(&hdma_usart6_rx);
  /* USER CODE BEGIN DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX1_OVR_IRQn 1 */

  /* USER CODE END DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX1_OVR_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
  // Character Match on '\n' (HAL doesn't handle CM)
  if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_CMF) &&
      __HAL_UART_GET_IT_SOURCE(&huart1, UART_IT_CM))
  {
      __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_CMF);
      uart_cm_handler(&huart1);
  }
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART3, USART4, USART5, USART6, LPUART1 globlal Interrupts (combined with EXTI 28).
  */
void USART3_4_5_6_LPUART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_4_5_6_LPUART1_IRQn 0 */
  // Character Match on 0x00 for USART5 Radio (HAL doesn't handle CM)
  if (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_CMF) &&
      __HAL_UART_GET_IT_SOURCE(&huart5, UART_IT_CM))
  {
      __HAL_UART_CLEAR_FLAG(&huart5, UART_CLEAR_CMF);
      uart_cm_handler(&huart5);
  }
  // Character Match on '\n' for USART6 GPS (HAL doesn't handle CM)
  if (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_CMF) &&
      __HAL_UART_GET_IT_SOURCE(&huart6, UART_IT_CM))
  {
      __HAL_UART_CLEAR_FLAG(&huart6, UART_CLEAR_CMF);
      uart_cm_handler(&huart6);
  }
  /* USER CODE END USART3_4_5_6_LPUART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart5);
  HAL_UART_IRQHandler(&huart6);
  /* USER CODE BEGIN USART3_4_5_6_LPUART1_IRQn 1 */

  /* USER CODE END USART3_4_5_6_LPUART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/**
  * @brief This function handles SPI1 global interrupt.
  *
  * SPI1 is used for the SPI slave (GNSS/Radio board).
  * RXNE interrupt captures the command byte.
  */
void SPI1_IRQHandler(void)
{
  spi_slave_spi1_irq_handler();
}

/**
  * @brief This function handles EXTI line 4 to 15 interrupts.
  *
  * PA4 (SPI1_NSS) is configured as EXTI4 for transaction end detection.
  */
void EXTI4_15_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_15_IRQn 0 */
  // SPI1 NSS rising edge handler (for SPI slave transaction end)
  spi_slave_nss_exti_handler();
  return;
  /* USER CODE END EXTI4_15_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
  /* USER CODE BEGIN EXTI4_15_IRQn 1 */

  /* USER CODE END EXTI4_15_IRQn 1 */
}

/* USER CODE END 1 */
