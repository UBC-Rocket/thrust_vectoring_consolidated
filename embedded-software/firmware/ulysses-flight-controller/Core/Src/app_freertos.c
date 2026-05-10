/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : FreeRTOS applicative file
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
#include "app_freertos.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "state_exchange.h"
#include "stm32l4xx_hal.h"
#include "task.h"

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
/* USER CODE BEGIN Variables */

#ifdef ULYSSES_ENABLE_DEBUG_LOGGING
osThreadId_t DebugLoggingTaskHandle;
const osThreadAttr_t DebugLoggingTask_attributes = {
  .name = "DebugLogging",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 128 * 4,
};
#endif // ULYSSES_ENABLE_DEBUG_LOGGING

/* USER CODE END Variables */
/* Definitions for MissionManager */
osThreadId_t MissionManagerHandle;
const osThreadAttr_t MissionManager_attributes = {
  .name = "MissionManager",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 512 * 4
};
/* Definitions for Controls */
osThreadId_t ControlsHandle;
const osThreadAttr_t Controls_attributes = {
  .name = "Controls",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 512 * 4
};
/* Definitions for StateEstimation */
osThreadId_t StateEstimationHandle;
const osThreadAttr_t StateEstimation_attributes = {
  .name = "StateEstimation",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 512 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  state_exchange_init();

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
  /* creation of MissionManager */
  MissionManagerHandle = osThreadNew(mission_manager_task_start, NULL, &MissionManager_attributes);

  /* Controls and StateEstimation disabled — SPI sensors not connected on test board */
  ControlsHandle = NULL;
  StateEstimationHandle = NULL;

  /* USER CODE BEGIN RTOS_THREADS */

#ifdef ULYSSES_ENABLE_DEBUG_LOGGING
  DebugLoggingTaskHandle = osThreadNew(debug_logging_task_start, NULL, &DebugLoggingTask_attributes);
#endif // ULYSSES_ENABLE_DEBUG_LOGGING

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

extern UART_HandleTypeDef huart2;

void vApplicationMallocFailedHook(void)
{
    static const char msg[] = "MALLOC_FAILED\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, sizeof(msg) - 1, 200);
    taskDISABLE_INTERRUPTS();
    for (;;) {}
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    HAL_UART_Transmit(&huart2, (uint8_t *)"STACK_OVF:", 10, 100);
    /* pcTaskName is a null-terminated string up to configMAX_TASK_NAME_LEN */
    int len = 0;
    while (pcTaskName[len] && len < 16) len++;
    HAL_UART_Transmit(&huart2, (uint8_t *)pcTaskName, len, 100);
    HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2, 100);
    taskDISABLE_INTERRUPTS();
    for (;;) {}
}

/* USER CODE END Application */

