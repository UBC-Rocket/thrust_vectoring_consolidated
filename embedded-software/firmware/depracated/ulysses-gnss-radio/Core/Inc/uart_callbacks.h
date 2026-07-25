/**
 * @file uart_callbacks.h
 * @brief HAL UART callback declarations + Character Match handler
 *
 * Routes UART events to appropriate driver modules:
 * - USART1: Debug console (DMA circular + Character Match on '\n')
 * - USART5: Radio transceiver (DMA circular + Character Match on 0x00)
 * - USART6: GPS module (DMA circular + Character Match on '\n')
 */

#ifndef UART_CALLBACKS_H
#define UART_CALLBACKS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g0xx_hal.h"

/* ============================================================================
 * Initialization
 * ============================================================================ */

/**
 * @brief Initialize UART callback system
 *
 * Starts USART1 DMA circular reception and configures Character Match
 * on '\n' for line-based debug console input.
 * Must be called after UART peripheral initialization.
 *
 * @param huart1_handle Pointer to UART1 handle (debug console)
 */
void uart_callbacks_init(UART_HandleTypeDef *huart1_handle);

/* ============================================================================
 * Character Match Handler (called from stm32g0xx_it.c)
 * ============================================================================ */

/**
 * @brief Character Match interrupt handler
 *
 * Called from UART ISRs in stm32g0xx_it.c when CMF flag is set.
 * The HAL does not implement CM handling, so this is called manually
 * from the ISR before HAL_UART_IRQHandler().
 *
 * Routes CM events to appropriate processing:
 * - USART1: Extract line from circular DMA buffer, process complete line
 * - USART5: Call radio_rx_event_callback with current DMA position
 * - USART6: Call gps_rx_event_callback with current DMA position
 *
 * @param huart UART handle that triggered the CM event
 */
void uart_cm_handler(UART_HandleTypeDef *huart);

/* ============================================================================
 * HAL UART Callbacks (weak redefinitions)
 * ============================================================================ */

/**
 * @brief DMA RX Event callback for IDLE/HT/TC events
 *
 * All UARTs now use Character Match as their primary trigger.
 * This callback is kept for HAL compatibility but no longer routes events.
 * Called automatically by HAL from interrupt context.
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);

/**
 * @brief TX complete callback
 *
 * Routes to GPS driver for debug output TX completion.
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);

/**
 * @brief UART error callback
 *
 * Routes errors to GPS (UART6) and Radio (UART5) drivers for recovery.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* UART_CALLBACKS_H */
