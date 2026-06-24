/**
 * @file    dxl_hal.c
 * @brief   Dynamixel HAL: UART8 half-duplex at 57600 baud.
 *
 * UBC Rocket, 2026
 */
#include "dxl_hal.h"

#include "io/io_uart.h"

#include "usart.h"

#include <string.h>

#define DXL_BAUD_RATE  57600u

static bool s_ready;

bool dxl_hal_init(void) {
    if (HAL_UART_DeInit(&huart8) != HAL_OK) {
        return false;
    }

    huart8.Init.BaudRate = DXL_BAUD_RATE;
    if (HAL_HalfDuplex_Init(&huart8) != HAL_OK) {
        return false;
    }

    s_ready = true;
    return true;
}

static dxl_status_code_t recv_bytes(uint8_t *dst, size_t len, uint32_t timeout_ms) {
    if (HAL_HalfDuplex_EnableReceiver(&huart8) != HAL_OK) {
        return DXL_ERR_BUS;
    }
    if (HAL_UART_Receive(&huart8, dst, (uint16_t)len, timeout_ms) != HAL_OK) {
        return DXL_ERR_TIMEOUT;
    }
    return DXL_OK;
}

dxl_status_code_t dxl_hal_txrx(const uint8_t *tx, size_t tx_len,
                               uint8_t *rx, size_t rx_cap,
                               dxl_status_t *status,
                               uint32_t timeout_ms) {
    if (!s_ready || tx == NULL || rx == NULL || status == NULL || tx_len == 0u) {
        return DXL_ERR_PARAM;
    }

    if (io_uart_send(&IO_UART_SERVO_BUS, tx, tx_len, timeout_ms) != IO_OK) {
        return DXL_ERR_TIMEOUT;
    }

    if (rx_cap < DXL_PKT_MIN_SIZE) {
        return DXL_ERR_PARAM;
    }

    dxl_status_code_t st = recv_bytes(rx, DXL_PKT_MIN_SIZE, timeout_ms);
    if (st != DXL_OK) return st;

    if (rx[0] != DXL_HDR0 || rx[1] != DXL_HDR1 || rx[2] != DXL_HDR2) {
        return DXL_ERR_CRC;
    }

    const uint16_t wire_len = (uint16_t)rx[4] | ((uint16_t)rx[5] << 8);
    const size_t total = (size_t)DXL_PKT_HDR_SIZE + 1u + 2u + wire_len + 2u;
    if (total > rx_cap) {
        return DXL_ERR_PARAM;
    }

    if (total > DXL_PKT_MIN_SIZE) {
        const size_t remain = (size_t)wire_len - 1u; /* inst already in first chunk */
        st = recv_bytes(&rx[DXL_PKT_MIN_SIZE], remain, timeout_ms);
        if (st != DXL_OK) return st;
    }

    if (!dxl_pkt_parse_status(rx, total, status)) {
        return DXL_ERR_CRC;
    }
    if (status->error != 0u) {
        return DXL_ERR_STATUS;
    }
    return DXL_OK;
}
