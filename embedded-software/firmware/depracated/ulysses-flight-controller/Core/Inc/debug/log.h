#ifndef DEBUG_LOG_H
#define DEBUG_LOG_H

#include "stm32h5xx_hal.h"

#ifndef DEBUG_LOG_MAX_MESSAGE_BUFFER_SIZE
    #define DEBUG_LOG_MAX_MESSAGE_BUFFER_SIZE (512)
#endif // DEBUG_LOG_MAX_MESSAGE_BUFFER_SIZE

#ifndef DEBUG_LOG_MAX_TX_BUFFER_SIZE
    #define DEBUG_LOG_MAX_TX_BUFFER_SIZE (256)
#endif // DEBUG_LOG_MAX_TX_BUFFER_SIZE

#ifndef DEBUG_LOG_MAX_MESSAGE_LENGTH
    #define DEBUG_LOG_MAX_MESSAGE_LENGTH (64)
#endif // DEBUG_LOG_MAX_MESSAGE_LENGTH

void debug_log_init(UART_HandleTypeDef *output_uart);

void debug_log_print(const char *const message, ...);

void debug_log_processing_start();

#ifdef DEBUG
    #define DLOG_PRINT(...) debug_log_print(__VA_ARGS__)
#else
    #define DLOG_PRINT(...) \
        do                  \
        {                   \
        } while (0)
#endif // DEBUG

#endif // DEBUG_LOG_H