/**
 * @file    dxl_hal.h
 * @brief   Dynamixel HAL: UART8 RS485 transport via 74LVC2G241 buffer.
 *
 * UBC Rocket, 2026
 */
#ifndef DXL_HAL_H
#define DXL_HAL_H

#include "dynamixel/dxl_packet.h"

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DXL_OK = 0,
    DXL_ERR_PARAM,
    DXL_ERR_TIMEOUT,
    DXL_ERR_CRC,
    DXL_ERR_STATUS,
    DXL_ERR_BUS,
} dxl_status_code_t;

bool dxl_hal_init(void);

bool dxl_hal_set_baud(uint32_t baud);
uint32_t dxl_hal_baud(void);

void dxl_hal_flush_rx(void);

dxl_status_code_t dxl_hal_txrx(const uint8_t *tx, size_t tx_len,
                               uint8_t *rx, size_t rx_cap,
                               size_t expected_rx_len,
                               dxl_status_t *status,
                               uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* DXL_HAL_H */
