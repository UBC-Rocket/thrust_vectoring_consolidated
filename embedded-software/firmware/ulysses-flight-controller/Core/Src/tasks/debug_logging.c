#include "debug/log.h"
#include "stm32l4xx_hal.h"

extern UART_HandleTypeDef huart2;

void debug_logging_task_start(void *argument)
{
    static const char alive[] = "DLOG_TASK_ALIVE\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)alive, sizeof(alive) - 1, 1000);
    debug_log_processing_start();
}