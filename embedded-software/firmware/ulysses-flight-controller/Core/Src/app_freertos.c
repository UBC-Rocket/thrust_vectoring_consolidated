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
#include "crash/crash_dump.h"

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

osThreadId_t SdFlushTaskHandle;
const osThreadAttr_t SdFlushTask_attributes = {
  .name = "SdFlush",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 256 * 4,
};
extern void sd_flush_task_start(void *argument);

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
  crash_dump_register_task((TaskHandle_t)MissionManagerHandle);
  vTaskSetTaskNumber((TaskHandle_t)MissionManagerHandle, 1);

  /* creation of Controls */
  ControlsHandle = osThreadNew(controls_task_start, NULL, &Controls_attributes);
  crash_dump_register_task((TaskHandle_t)ControlsHandle);
  vTaskSetTaskNumber((TaskHandle_t)ControlsHandle, 2);

  /* creation of StateEstimation */
  StateEstimationHandle = osThreadNew(state_estimation_task_start, NULL, &StateEstimation_attributes);
  crash_dump_register_task((TaskHandle_t)StateEstimationHandle);
  vTaskSetTaskNumber((TaskHandle_t)StateEstimationHandle, 3);

  /* USER CODE BEGIN RTOS_THREADS */

#ifdef ULYSSES_ENABLE_DEBUG_LOGGING
  DebugLoggingTaskHandle = osThreadNew(debug_logging_task_start, NULL, &DebugLoggingTask_attributes);
  crash_dump_register_task((TaskHandle_t)DebugLoggingTaskHandle);
  vTaskSetTaskNumber((TaskHandle_t)DebugLoggingTaskHandle, 4);
#endif // ULYSSES_ENABLE_DEBUG_LOGGING

  SdFlushTaskHandle = osThreadNew(sd_flush_task_start, NULL, &SdFlushTask_attributes);
  crash_dump_register_task((TaskHandle_t)SdFlushTaskHandle);
  vTaskSetTaskNumber((TaskHandle_t)SdFlushTaskHandle, 5);

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

#include "timestamp.h"

void vApplicationTickHook(void)
{
    timestamp_update();
}

/* USER CODE END Application */

