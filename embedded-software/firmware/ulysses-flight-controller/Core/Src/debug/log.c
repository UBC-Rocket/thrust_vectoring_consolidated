#include "debug/log.h"

#include <stdbool.h>
#include <stdarg.h>

#include "printf/printf.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32l4xx.h"
#include "cmsis_os2.h"
#include "stream_buffer.h"

struct logger
{
    UART_HandleTypeDef *output_uart;

    osSemaphoreId_t tx_busy_sem_id;
    StaticSemaphore_t tx_busy_sem;

    osMutexId_t message_buffer_mutex_id;
    StaticSemaphore_t message_buffer_mutex;

    StreamBufferHandle_t message_buffer_handle;
    StaticStreamBuffer_t message_buffer_struct;
    uint8_t message_buffer[DEBUG_LOG_MAX_MESSAGE_BUFFER_SIZE];

    uint8_t tx_buffer[DEBUG_LOG_MAX_TX_BUFFER_SIZE];
};

static struct logger logger;

static int get_formatted_message_length(const char *const format, va_list *const args);
static void signal_tx_complete(UART_HandleTypeDef *huart);

static int get_formatted_message_length(const char *const format, va_list *const args)
{
    va_list args_copy;

    va_copy(args_copy, *args);

    int size = vsnprintf_(NULL, 0, format, args_copy);

    va_end(args_copy);

    return size;
}

static void signal_tx_complete(UART_HandleTypeDef *huart)
{
    UNUSED(huart);

    (void)osSemaphoreRelease(logger.tx_busy_sem_id);
}

void debug_log_init(UART_HandleTypeDef *output_uart)
{
    osMutexAttr_t message_buffer_mutex_attr = {
        .attr_bits = osMutexPrioInherit,
        .cb_mem = &logger.message_buffer_mutex,
        .cb_size = sizeof(logger.message_buffer_mutex),
    };

    osSemaphoreAttr_t tx_busy_sem_attr = {
        .cb_mem = &logger.tx_busy_sem,
        .cb_size = sizeof(logger.tx_busy_sem),
    };

    logger.output_uart = output_uart;
    logger.message_buffer_mutex_id = osMutexNew(&message_buffer_mutex_attr);
    logger.tx_busy_sem_id = osSemaphoreNew(1, 1, &tx_busy_sem_attr);

    logger.message_buffer_handle = xStreamBufferCreateStatic(DEBUG_LOG_MAX_MESSAGE_BUFFER_SIZE - 1,
                                                             0,
                                                             logger.message_buffer,
                                                             &logger.message_buffer_struct);

    HAL_UART_RegisterCallback(output_uart,
                              HAL_UART_TX_COMPLETE_CB_ID,
                              signal_tx_complete);
}

void debug_log_print(const char *const message, ...)
{
    configASSERT(logger.output_uart != NULL);

    char formatted[DEBUG_LOG_MAX_MESSAGE_LENGTH];
    va_list args;

    va_start(args, message);

    int length = get_formatted_message_length(message, &args);

    // Format failed or the message is empty
    if (length <= 0)
    {
        va_end(args);
        return;
    }

    // Count the null terminator
    length += 1;

    if (length > DEBUG_LOG_MAX_MESSAGE_LENGTH)
    {
        length = DEBUG_LOG_MAX_MESSAGE_LENGTH;
    }

    int written = vsnprintf_(formatted, length, message, args);

    va_end(args);

    if (written <= 0)
    {
        return;
    }

    osStatus_t status = osMutexAcquire(logger.message_buffer_mutex_id,
                                       osWaitForever);

    if (status != osOK)
    {
        return;
    }

    (void)xStreamBufferSend(logger.message_buffer_handle, formatted, written, 0);
    (void)osMutexRelease(logger.message_buffer_mutex_id);
}

void debug_log_processing_start()
{
    configASSERT(logger.output_uart != NULL);

    while (true)
    {
        size_t bytes_available = xStreamBufferReceive(logger.message_buffer_handle,
                                                      logger.tx_buffer,
                                                      sizeof(logger.tx_buffer),
                                                      osWaitForever);

        if (bytes_available <= 0)
        {
            continue;
        }

        osStatus_t status = osSemaphoreAcquire(logger.tx_busy_sem_id, osWaitForever);

        if (status != osOK)
        {
            continue;
        }

        HAL_StatusTypeDef tx_status = HAL_UART_Transmit_DMA(logger.output_uart,
                                                            logger.tx_buffer,
                                                            bytes_available);

        if (tx_status != HAL_OK)
        {
            (void)osSemaphoreRelease(logger.tx_busy_sem_id);
        }
    }
}
