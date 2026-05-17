#pragma once

/**
 * @file gps.h
 * @brief GPS Driver using DMA circular + Character Match on '\n'
 *
 * Efficient NMEA sentence reception with minimal CPU overhead.
 * Uses DMA circular buffer with Character Match interrupt on '\n'.
 * Matches the CM pattern used by USART1 and USART5.
 */

#include "stm32g0xx_hal.h"
#include "gps_nema_queue.h"
#include "gps_fix_queue.h"
#include "protocol_config.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Initialization
 * ============================================================================ */

/**
 * @brief Set the GPS sample queue for SPI push/pull mode (raw NMEA)
 * @param queue Pointer to shared GPS sample queue
 */
void gps_set_queue(gps_sample_queue_t *queue);

/**
 * @brief Set the GPS fix queue for SPI push mode (parsed fixes)
 * @param queue Pointer to shared GPS fix queue
 */
void gps_set_fix_queue(gps_fix_queue_t *queue);

/**
 * @brief Set the GPS protocol mode (PULL = raw NMEA, PUSH = parsed fix)
 * @param mode Protocol mode from protocol_config.h
 */
void gps_set_protocol_mode(spi_protocol_mode_t mode);

/**
 * @brief Initialize GPS driver with DMA circular + CM reception
 *
 * Configures Character Match on '\n' and starts DMA circular receive.
 * Call after HAL peripheral initialization.
 *
 * @param gps_uart UART handle for GPS module (e.g., UART6)
 * @param out_uart UART handle for debug output (e.g., UART1), can be NULL
 */
void gps_init(UART_HandleTypeDef *gps_uart, UART_HandleTypeDef *out_uart);

/**
 * @brief Process GPS data (main loop call)
 *
 * In DMA mode, most processing happens in callbacks.
 * This can be called from main loop for any deferred processing.
 */
void gps_process(void);

/* ============================================================================
 * UART Callbacks - Wire these from main.c or stm32g0xx_it.c
 * ============================================================================ */

/**
 * @brief Character Match RX callback (called on '\n')
 *
 * This is the main receive handler. Called from uart_cm_handler().
 *
 * @param huart UART handle
 * @param Size Current position in DMA buffer
 */
void gps_rx_event_callback(UART_HandleTypeDef *huart, uint16_t Size);

/**
 * @brief UART TX complete callback (for debug output)
 *
 * Call from HAL_UART_TxCpltCallback.
 *
 * @param huart UART handle
 */
void gps_uart_tx_cplt_callback(UART_HandleTypeDef *huart);

/**
 * @brief UART error callback
 *
 * Call from HAL_UART_ErrorCallback. Restarts DMA on error.
 *
 * @param huart UART handle
 */
void gps_uart_error_callback(UART_HandleTypeDef *huart);

/**
 * @brief Legacy RX complete callback (not used in DMA mode)
 *
 * Kept for compatibility. Does nothing in DMA mode.
 *
 * @param huart UART handle
 */
void gps_uart_rx_cplt_callback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif
