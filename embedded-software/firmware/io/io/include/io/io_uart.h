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

/* Board-wired UART ports (defined in per-target io_uart.c). MISRA 8.4. */
extern const io_uart_t IO_UART_GPS;
extern const io_uart_t IO_UART_RADIO;
extern const io_uart_t IO_UART_MMWAVE;
extern const io_uart_t IO_UART_SERVO_BUS;

#ifdef __cplusplus
}
#endif

#endif /* IO_UART_H */
