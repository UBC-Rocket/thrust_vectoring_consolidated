/**
 * @file    io_uart.h
 * @brief   IO-layer transport: UART against opaque port handles.
 *
 * UBC Rocket, 2026
 */
#ifndef IO_UART_H
#define IO_UART_H

#include "io_sys/io_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct io_uart io_uart_t;

typedef void (*io_uart_rx_cb_t)(const uint8_t *data, size_t len, void *user);

io_status_t io_uart_open(const io_uart_t *u, io_uart_rx_cb_t cb, void *user);
io_status_t io_uart_send(const io_uart_t *u, const uint8_t *data, size_t len, uint32_t timeout_ms);

io_status_t io_uart_xfer(const io_uart_t *u,
                         const uint8_t *tx, size_t tx_len,
                         uint8_t *rx, size_t rx_len,
                         uint32_t timeout_ms);

/**
 * @brief Fire-and-forget DMA transmit. Returns immediately after kicking
 *        the DMA stream; caller does NOT block waiting for the bytes to
 *        leave the wire. ISR-safe.
 *
 * Implementation note: caller owns @p data and MUST keep the buffer
 * valid + immutable until the DMA finishes. For ISR-driven control loops
 * the typical pattern is a small static double-buffer.
 *
 * Returns:
 *   IO_OK         the DMA stream accepted the job
 *   IO_ERR_BUSY   a previous async send hasn't completed yet
 *   IO_ERR_PARAM  invalid handle / zero length
 *   IO_ERR_BUS    HAL refused the kick (DMA stream config error)
 */
io_status_t io_uart_send_async(const io_uart_t *u, const uint8_t *data, size_t len);

/* Board-wired UART ports (defined in per-target io_uart.c). MISRA 8.4. */
extern const io_uart_t IO_UART_GPS;
extern const io_uart_t IO_UART_RADIO;
extern const io_uart_t IO_UART_MMWAVE;
extern const io_uart_t IO_UART_DEBUG;       /* CM4: hlpuart1 — console / messages CH_VCP */
extern const io_uart_t IO_UART_SERVO_BUS;

/**
 * @brief IRQ hooks. Called from stm32h7xx_it.c's UART IRQHandlers
 *        (USER CODE BEGIN UARTx_IRQn 0 / 1 blocks). The argument is a
 *        HAL `UART_HandleTypeDef *` — declared void* here so this
 *        header stays HAL-independent. The hook walks the streaming
 *        UART table and routes to io_uart_dma_cm_on_*_isr.
 */
void io_uart_on_match_isr(void *huart);
void io_uart_on_error_isr(void *huart);

/**
 * @brief Bring-up diagnostics: observe RAW bytes in a streaming UART's RX
 *        circular DMA buffer, independent of 0x00 frame completion. The
 *        character-match path only surfaces bytes once a 0x00 delimiter
 *        arrives, so un-framed traffic (e.g. someone typing in a terminal)
 *        is invisible to the normal RX path but visible here.
 *
 *  io_uart_diag_rx_index: current DMA write index in [0, buffer_size). It
 *                         advances whenever the UART physically receives a
 *                         byte. Frozen ⇒ nothing is arriving on the wire.
 *  io_uart_diag_rx_copy:  copy circular bytes [from, to) into @p out (handles
 *                         wrap). Returns bytes copied (<= max).
 */
uint32_t io_uart_diag_rx_index(const io_uart_t *u);
size_t   io_uart_diag_rx_copy(const io_uart_t *u, uint32_t from, uint32_t to,
                              uint8_t *out, size_t max);

#ifdef __cplusplus
}
#endif

#endif /* IO_UART_H */
