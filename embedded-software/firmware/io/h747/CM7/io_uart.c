/**
 * @file    io_uart.c (CM7)
 * @brief   CM7 UART transport. Will host IO_UART_SERVO_BUS (half-duplex
 *          Feetech bus, intended to live on huart8 per main.h's
 *          UART_SERVOS macro).
 *
 *  *** STUBBED — the IOC currently enables USART2 on CM7 but NOT USART8. ***
 *  Until the CubeMX project enables USART8 on CM7 (Mode = Half-duplex,
 *  baud 1 000 000 for Feetech), all bus operations report IO_ERR_NOT_READY.
 *  Re-wire to use huart8 once the handle is generated.
 *
 * UBC Rocket, 2026
 */
#include "io_common.h"
#include "io/io_uart.h"

#include "usart.h"

struct io_uart {
    UART_HandleTypeDef *huart;
    bool                streaming;
};

/* Stub: huart = NULL until the IOC adds USART8 to CM7. */
const io_uart_t IO_UART_SERVO_BUS = {
    .huart     = NULL,
    .streaming = false,
};

io_status_t io_uart_open(const io_uart_t *u, io_uart_rx_cb_t cb, void *user) {
    (void)u; (void)cb; (void)user;
    return IO_ERR_NOT_READY;
}

io_status_t io_uart_send(const io_uart_t *u, const uint8_t *data, size_t len, uint32_t timeout_ms) {
    (void)data; (void)len; (void)timeout_ms;
    if (!u || !u->huart) return IO_ERR_NOT_READY;
    return HAL_UART_Transmit(u->huart, (uint8_t *)data, (uint16_t)len, timeout_ms) == HAL_OK
               ? IO_OK : IO_ERR_TIMEOUT;
}

io_status_t io_uart_xfer(const io_uart_t *u,
                         const uint8_t *tx, size_t tx_len,
                         uint8_t *rx, size_t rx_len,
                         uint32_t timeout_ms) {
    (void)tx; (void)tx_len; (void)rx; (void)rx_len; (void)timeout_ms;
    if (!u || !u->huart) return IO_ERR_NOT_READY;

    if (HAL_HalfDuplex_EnableTransmitter(u->huart) != HAL_OK) return IO_ERR_BUS;
    if (HAL_UART_Transmit(u->huart, (uint8_t *)tx, (uint16_t)tx_len, timeout_ms) != HAL_OK)
        return IO_ERR_TIMEOUT;

    if (rx_len == 0) return IO_OK;

    if (HAL_HalfDuplex_EnableReceiver(u->huart) != HAL_OK) return IO_ERR_BUS;
    return HAL_UART_Receive(u->huart, rx, (uint16_t)rx_len, timeout_ms) == HAL_OK
               ? IO_OK : IO_ERR_TIMEOUT;
}
