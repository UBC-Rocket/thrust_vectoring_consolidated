/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "cmsis_os2.h"
#include "dcache.h"
#include "gpdma.h"
#include "icache.h"
#include "sdmmc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "debug/log.h"
#include "sensors_init.h"
#include "timestamp.h"
#include "spi1_bus.h"
#include "gnss_radio_master.h"
#include "motor_drivers/servo_driver.h"
#include "motor_drivers/esc_driver.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

static bool sd_card_is_inserted(void)
{
  GPIO_PinState state = HAL_GPIO_ReadPin(SD_CARD_DETECT_GPIO_Port, SD_CARD_DETECT_Pin);
  /* Assuming the detect pin is low when a card is present. Adjust if hardware differs. */
  return state == GPIO_PIN_RESET;
}

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
bool g_sd_card_initialized = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
static HAL_StatusTypeDef sd_enable_internal_dma(SD_HandleTypeDef *hsd);

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
  /* Initialize high-resolution timestamp (enables DWT cycle counter) */
  timestamp_init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_GPDMA2_Init();
  MX_GPDMA1_Init();
  MX_SDMMC1_SD_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI4_Init();
  MX_UART4_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_USB_PCD_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_DCACHE1_Init();
  MX_ICACHE_Init();
  /* USER CODE BEGIN 2 */

  /* --- TIM4 interrupt channels ------------------------------------ */
  HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_2);   /* 800 Hz -> controls notify */
  HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_4);   /* 800 Hz -> ESC/servo/baro */

  /* --- Servo pair: TIM1 CH2 (servo1), TIM3 CH3 (servo2) - 200Hz -- */
  {
    pwm_output_t pwm1 = {
      .htim        = &htim1,
      .channel     = TIM_CHANNEL_2,
      .timer_hz    = 1000000U,
      .period_ticks = 5000U,
    };
    pwm_output_t pwm2 = {
      .htim        = &htim3,
      .channel     = TIM_CHANNEL_3,
      .timer_hz    = 1000000U,
      .period_ticks = 5000U,
    };
    servo_pair_init(&pwm1, &pwm2);
    servo_pair_enable(true);
  }

  /* --- ESC pair: TIM2 CH2 (esc1), TIM2 CH4 (esc2) - 400Hz ------- */
  {
    pwm_output_t pwm1 = {
      .htim        = &htim2,
      .channel     = TIM_CHANNEL_2,
      .timer_hz    = 1000000U,
      .period_ticks = 2500U,
    };
    pwm_output_t pwm2 = {
      .htim        = &htim2,
      .channel     = TIM_CHANNEL_4,
      .timer_hz    = 1000000U,
      .period_ticks = 2500U,
    };
    esc_pair_init(&pwm1, &pwm2);
  }

#ifdef ULYSSES_ENABLE_DEBUG_LOGGING
  debug_log_init(&huart1);
#endif // ULYSSES_ENABLE_DEBUG_LOGGING

  if (g_sd_card_initialized) {
    if (sd_enable_internal_dma(&hsd1) != HAL_OK) {
      Error_Handler();
    }
  }

  /*
   * Initialize all SPI sensors before starting the scheduler.
   * This avoids long blocking operations inside tasks with interrupts disabled.
   * The sensor init takes ~700ms due to device reset delays.
   */
  sensors_init_status_t sensor_status = sensors_init();

  /* Optional: Check sensor status and handle failures */
  if (!sensors_init_critical_ok(&sensor_status)) {
    /* IMU initialization failed - system may not be flight-ready */
#ifdef ULYSSES_ENABLE_DEBUG_LOGGING
    /* Log the error codes for debugging */
    (void)sensor_status;  /* Avoid unused warning in release builds */
#endif
    /* Continue anyway - tasks will check ready flags */
  }

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Call init function for freertos objects (in app_freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLL1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1_VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the programming delay
  */
  __HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_2);
}

/* USER CODE BEGIN 4 */
static HAL_StatusTypeDef sd_enable_internal_dma(SD_HandleTypeDef *hsd)
{
#if defined(SDMMC_ENABLE_IDMA_SINGLE_BUFF)
  hsd->Instance->IDMACTRL = SDMMC_ENABLE_IDMA_SINGLE_BUFF;
  return HAL_OK;
#else
  (void)hsd;
  return HAL_ERROR;
#endif
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
