/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "radio_queue.h"
#include "gps_nema_queue.h"
#include "gps_fix_queue.h"
#include "spi_slave.h"
#include "radio_driver.h"
#include "gps.h"
#include "uart_callbacks.h"
#ifdef DEBUG
#include "debug_uart.h"
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi1_rx;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart5_tx;
DMA_HandleTypeDef hdma_usart5_rx;
DMA_HandleTypeDef hdma_usart6_tx;
DMA_HandleTypeDef hdma_usart6_rx;

/* USER CODE BEGIN PV */
static radio_message_queue_t radio_rx_queue;
static radio_message_queue_t radio_tx_queue;
static gps_sample_queue_t gps_sample_queue;
static gps_fix_queue_t gps_fix_queue;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART5_UART_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART5_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  // Release GPS from reset BEFORE starting UART reception.
  // GPS TX line is LOW/floating while module is in reset, which causes
  // continuous UART framing errors that corrupt the DMA state machine.
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);
  HAL_Delay(10);  // Let GPS TX line stabilize to idle HIGH

  // Initialize GPS queues
  gps_sample_queue_init(&gps_sample_queue);  // Pull mode: raw NMEA
  gps_fix_queue_init(&gps_fix_queue);        // Push mode: parsed fixes

  // Initialize GPS driver with shared queues
  gps_set_queue(&gps_sample_queue);
  gps_set_fix_queue(&gps_fix_queue);
  gps_init(&huart6, NULL);  // GPS on UART6, no raw NMEA echo (frees UART1 for debug log)

  // Initialize radio driver (handles its own queue initialization)
  radio_init(&radio_rx_queue);

  // Initialize radio TX queue (SPI master → radio UART5)
  radio_message_queue_init(&radio_tx_queue);
  // Start USART1 DMA reception + Character Match on '\n'
#ifdef DEBUG
  // Initialize debug UART system (injection + logging)
  uart_callbacks_init(&huart1);
  debug_uart_init(&radio_rx_queue, &gps_sample_queue);
  HAL_UART_Transmit(&huart1, (uint8_t*)"System initialized - awaiting configuration from master\r\n", 57, 100);
#endif

  // Wait for configuration frame from master (blocking)
  config_frame_t config_frame;
  //HAL_StatusTypeDef status = HAL_SPI_Receive(&hspi1, (uint8_t*)&config_frame, sizeof(config_frame_t), HAL_MAX_DELAY);
  HAL_StatusTypeDef status = HAL_OK;
  config_frame.mode = SPI_MODE_PUSH;
  if (status == HAL_OK) {
      // Parse and validate the mode
      spi_protocol_mode_t mode;
      if (config_frame.mode == SPI_MODE_PULL) {
          mode = SPI_MODE_PULL;
      } else if (config_frame.mode == SPI_MODE_PUSH) {
          mode = SPI_MODE_PUSH;
      } else {
          // Invalid mode - default to PULL (safe fallback)
          mode = SPI_MODE_PULL;
      }

      // Set protocol mode for SPI slave and GPS
      spi_slave_set_protocol_mode(mode);
      gps_set_protocol_mode(mode);

#ifdef DEBUG
      const char* mode_str = (mode == SPI_MODE_PUSH) ? "PUSH" : "PULL";
      HAL_UART_Transmit(&huart1, (uint8_t*)"Configuration received - Mode: ", 31, 100);
      HAL_UART_Transmit(&huart1, (uint8_t*)mode_str, strlen(mode_str), 100);
      HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, 100);
#endif
  } else {
#ifdef DEBUG
      HAL_UART_Transmit(&huart1, (uint8_t*)"ERROR: Failed to receive configuration frame\r\n", 46, 100);
#endif
  }

  // Wait for master flight controller to finish booting and initialize SPI1 bus.
  // Without this delay, the slave may assert IRQ before the master is ready,
  // causing missed or corrupted transactions.
#ifdef DEBUG
  HAL_UART_Transmit(&huart1, (uint8_t*)"Waiting 3s for master to initialize...\r\n", 40, 100);
#endif
  {
      // Drain debug log queue during wait so GPS NMEA messages don't
      // overflow the 20-entry queue and cause SPI messages to be dropped.
      uint32_t wait_start = HAL_GetTick();
      while (HAL_GetTick() - wait_start < 3000) {
#ifdef DEBUG
          debug_uart_process_logs();
#endif
          HAL_Delay(1);
      }
  }

  // Initialize SPI slave (uses radio RX, radio TX, and GPS queues)
  spi_slave_init(&radio_rx_queue, &radio_tx_queue, &gps_sample_queue, &gps_fix_queue);

#ifdef DEBUG
  // One-shot SPI state dump after initialization
  {
      const spi_debug_capture_t *spi_dbg = spi_slave_get_debug_capture();
      debug_uart_log_spi_arm(spi_dbg);
  }
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

      // Process GPS UART data (builds NMEA sentences from ring buffer)
      gps_process();

      // Push mode tick - check for pending data and assert IRQ if needed
      spi_slave_tick();

      // Transmit radio messages from SPI master over UART5
      if (!radio_message_queue_empty(&radio_tx_queue)) {
          uint8_t tx_msg[RADIO_MESSAGE_MAX_LEN];
          radio_message_dequeue(&radio_tx_queue, tx_msg);

          // Find actual message length (null-padded to 256)
          uint16_t tx_len = 0;
          while (tx_len < RADIO_MESSAGE_MAX_LEN && tx_msg[tx_len] != 0) {
              tx_len++;
          }

          if (tx_len > 0) {
              radio_send(tx_msg, tx_len);
#ifdef DEBUG
              debug_uart_log_spi_radio_tx(tx_msg, tx_len);
#endif
          }
      }

#ifdef DEBUG
      // Log SPI transaction debug data (rate-limited to avoid flooding queue)
      {
          static uint32_t last_spi_txn_tick = 0;
          static uint32_t last_spi_exti = 0;
          const spi_debug_capture_t *spi_dbg = spi_slave_get_debug_capture();
          uint32_t now_spi = HAL_GetTick();
          if (spi_dbg->exti_count != last_spi_exti && (now_spi - last_spi_txn_tick >= 2000)) {
              last_spi_exti = spi_dbg->exti_count;
              last_spi_txn_tick = now_spi;
              debug_uart_log_spi_txn(spi_dbg);
          }
      }

      // Periodic radio UART5 health check (every 5 seconds)
      {
          static uint32_t last_radio_diag_tick = 0;
          uint32_t now = HAL_GetTick();
          if (now - last_radio_diag_tick >= 5000) {
              last_radio_diag_tick = now;
              radio_diag_t rd = radio_get_diag();
              debug_uart_log_radio_diag(
                  rd.cm_events, rd.bytes_fed, rd.msgs_enqueued, rd.uart_errors,
                  huart5.Instance->CR1, huart5.Instance->CR3,
                  huart5.Instance->ISR, (uint16_t)huart5.hdmarx->Instance->CNDTR);
          }
      }

      // Process and transmit pending debug log messages
      debug_uart_process_logs();
#endif
      
      HAL_Delay(1);  // Reduced delay for faster push response
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART5_UART_Init(void)
{

  /* USER CODE BEGIN USART5_Init 0 */

  /* USER CODE END USART5_Init 0 */

  /* USER CODE BEGIN USART5_Init 1 */

  /* USER CODE END USART5_Init 1 */
  huart5.Instance = USART5;
  huart5.Init.BaudRate = 57600;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART5_Init 2 */

  /* USER CODE END USART5_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  /* DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX1_OVR_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX1_OVR_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX1_OVR_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, FC_INT_Pin|STAT_LEDR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPS_RST_GPIO_Port, GPS_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : FC_INT_Pin STAT_LEDR_Pin */
  GPIO_InitStruct.Pin = FC_INT_Pin|STAT_LEDR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : GPS_RST_Pin */
  GPIO_InitStruct.Pin = GPS_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPS_RST_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
