/**
 * @file radio_driver.h
 * @brief Radio transceiver driver using DMA circular + Character Match
 *
 * Provides a simple API for sending and receiving radio messages
 * via UART5. Messages are null-terminated (0x00) on the wire.
 *
 * Uses DMA circular buffer with Character Match on 0x00 as the sole
 * interrupt trigger. DMA never stops — no restart gaps, no data loss.
 */

#ifndef RADIO_DRIVER_H
#define RADIO_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "radio_queue.h"
#include "stm32g0xx_hal.h"

/* ============================================================================
 * Configuration
 * ============================================================================ */

#define RADIO_MAX_MESSAGE_LEN   RADIO_MESSAGE_MAX_LEN  /* 256 bytes */

/* ============================================================================
 * Initialization
 * ============================================================================ */

/**
 * @brief Initialize radio driver with DMA reception
 *
 * Sets up UART5 for radio communication with DMA circular buffer
 * and IDLE line detection. Initializes the RX queue.
 *
 * @param rx_queue Pointer to queue for storing received messages
 */
void radio_init(radio_message_queue_t *rx_queue);

/* ============================================================================
 * TX API
 * ============================================================================ */

/**
 * @brief Send a message over the radio
 *
 * Transmits data followed by a null terminator (0x00).
 * Blocks until transmission is complete.
 *
 * @param data Pointer to data buffer
 * @param len Length of data (1-254 bytes)
 * @return true if sent successfully, false on error
 */
bool radio_send(const uint8_t *data, uint8_t len);

/* ============================================================================
 * RX API
 * ============================================================================ */

/**
 * @brief Check if a received message is available
 *
 * @return true if at least one message is waiting in the RX queue
 */
bool radio_available(void);

/**
 * @brief Read a received message from the queue
 *
 * Copies the oldest message from the RX queue into the provided buffer.
 * The message is removed from the queue.
 *
 * @param buffer Destination buffer (must be at least RADIO_MAX_MESSAGE_LEN bytes)
 * @return Length of the message (0 if queue was empty)
 */
uint8_t radio_read(uint8_t *buffer);

/**
 * @brief Get the number of messages in the RX queue
 *
 * @return Number of messages waiting (0-255)
 */
uint8_t radio_rx_count(void);

/**
 * @brief Get pointer to the radio RX queue
 *
 * Provides direct access for the SPI slave to read from.
 *
 * @return Pointer to the RX message queue
 */
radio_message_queue_t *radio_get_rx_queue(void);

/* ============================================================================
 * Diagnostics (DEBUG only)
 * ============================================================================ */

#ifdef DEBUG
/**
 * @brief Radio UART5 diagnostic counters
 */
typedef struct {
    uint32_t cm_events;       /**< Character Match ISR invocations */
    uint32_t bytes_fed;       /**< Total bytes processed through feed_byte */
    uint32_t msgs_enqueued;   /**< Messages successfully enqueued */
    uint32_t uart_errors;     /**< UART error callback invocations */
} radio_diag_t;

/**
 * @brief Get radio diagnostic counters
 * @return Snapshot of current counters
 */
radio_diag_t radio_get_diag(void);
#endif

/* ============================================================================
 * UART Callbacks - Wire these from stm32g0xx_it.c / uart_callbacks.c
 * ============================================================================ */

/**
 * @brief Character Match event callback
 *
 * Called from uart_cm_handler when 0x00 is received on USART5.
 * Processes new bytes in the circular DMA buffer.
 *
 * @param huart UART handle
 * @param Size Current position in DMA buffer
 */
void radio_rx_event_callback(UART_HandleTypeDef *huart, uint16_t Size);

/**
 * @brief UART error callback
 *
 * Call from HAL_UART_ErrorCallback. Restarts circular DMA on error.
 *
 * @param huart UART handle
 */
void radio_uart_error_callback(UART_HandleTypeDef *huart);

#endif /* RADIO_DRIVER_H */
