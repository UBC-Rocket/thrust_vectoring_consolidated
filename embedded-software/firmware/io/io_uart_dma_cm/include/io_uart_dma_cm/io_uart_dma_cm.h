/**
 * @file io_uart_dma_cm.h
 * @brief UART RX pattern: DMA-circular + character match for message framing.
 *
 * Configure UART RX as DMA circular into a caller-provided power-of-two buffer,
 * set the USART CR2 ADD register to the framing byte, disable HT/TC DMA
 * interrupts, and enable UART_IT_CM. On each character-match interrupt the
 * driver computes the segment written since the last match and hands it to
 * the caller's message callback.
 *
 * Typical framing bytes:
 *   - NMEA:      '\n'
 *   - RFD900x:    0x00
 *
 * Not safe to share one instance between multiple UARTs; declare one per UART.
 * The HAL UART IRQ handler must forward CM events to
 * @ref io_uart_dma_cm_on_match_isr.
 *
 * UBC Rocket, 2026
 */

#ifndef IO_UART_DMA_CM_H
#define IO_UART_DMA_CM_H

#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Callback invoked from the UART ISR when a complete message
 *        (up to and including the framing byte) has been received.
 *
 * The data pointer references a caller-owned scratch buffer that is reused
 * on the next match — copy out anything that must outlive this callback.
 */
typedef void (*io_uart_dma_cm_msg_cb_t)(const uint8_t *data, size_t len, void *user);

typedef struct uart_dma_cm {
    UART_HandleTypeDef   *huart;
    uint8_t              *circ_buf;      ///< caller-owned DMA RX buffer
    size_t                circ_size;
    uint8_t              *scratch;       ///< caller-owned message scratch
    size_t                scratch_size;
    io_uart_dma_cm_msg_cb_t  cb;
    void                 *user;
    volatile uint16_t     last_pos;      ///< consumed position in circ_buf
} io_uart_dma_cm_t;

/**
 * @brief Initialise an instance.
 *
 * @param u              zeroed out and populated
 * @param huart          UART peripheral
 * @param circ_buf       DMA RX buffer; must be >= 2 × the largest expected
 *                       frame so wrap-around is always catchable
 * @param circ_size      bytes in @p circ_buf
 * @param scratch        destination for reassembled messages
 * @param scratch_size   bytes in @p scratch (must be >= circ_size)
 * @param match_char     framing byte that fires CM
 * @param cb             per-message callback (may be NULL)
 * @param user           opaque pointer forwarded to @p cb
 */
void io_uart_dma_cm_init(io_uart_dma_cm_t *u,
                      UART_HandleTypeDef *huart,
                      uint8_t *circ_buf, size_t circ_size,
                      uint8_t *scratch,  size_t scratch_size,
                      uint8_t match_char,
                      io_uart_dma_cm_msg_cb_t cb, void *user);

/**
 * @brief Start DMA-circular reception and arm the CM interrupt.
 */
void io_uart_dma_cm_start(io_uart_dma_cm_t *u);

/**
 * @brief Call from the UART ISR when the CMF flag is set.
 *        Reads the DMA transfer counter, extracts the new bytes (handling
 *        wrap-around) and invokes the message callback.
 */
void io_uart_dma_cm_on_match_isr(io_uart_dma_cm_t *u);

/**
 * @brief Call from HAL_UART_ErrorCallback to re-arm DMA reception after an
 *        ORE/FE/NE/PE event.
 */
void io_uart_dma_cm_on_error_isr(io_uart_dma_cm_t *u);

/**
 * @brief Blocking transmit (convenience; used for low-rate config writes).
 */
bool io_uart_dma_cm_send(io_uart_dma_cm_t *u, const uint8_t *data, size_t len, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* IO_UART_DMA_CM_H */
