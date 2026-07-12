/**
 * @file    dxl_hal.c
 * @brief   Dynamixel HAL: UART8 @ 57600 via SN74LVC2G241.
 *
 * PE0 = UART8_RX, PE1 = UART8_TX, PD3 = level-shifter OE,
 * PD15 = UART8_RTS (USART hardware DE).
 *
 * UBC Rocket, 2026
 */
#include "dxl_hal.h"

#include "servo_bus_gpio.h"

#include "usart.h"

#include <string.h>

#define DXL_BAUD_RATE          57600u
#define DXL_DE_ASSERT_BITS     0u
#define DXL_DE_DEASSERT_BITS   1u
#define DXL_DEFAULT_TIMEOUT_MS 20u

static bool s_ready;

static void flush_rx(void) {
    __HAL_UART_SEND_REQ(&huart8, UART_RXDATA_FLUSH_REQUEST);
    __HAL_UART_CLEAR_FLAG(&huart8, UART_CLEAR_OREF | UART_CLEAR_NEF |
                                   UART_CLEAR_PEF | UART_CLEAR_FEF);
    while (__HAL_UART_GET_FLAG(&huart8, UART_FLAG_RXNE)) {
        (void)huart8.Instance->RDR;
    }
}

static bool uart_rs485_init(uint32_t baud) {
    huart8.Init.BaudRate = baud;
    huart8.Init.WordLength = UART_WORDLENGTH_8B;
    huart8.Init.StopBits = UART_STOPBITS_1;
    huart8.Init.Parity = UART_PARITY_NONE;
    huart8.Init.Mode = UART_MODE_TX_RX;
    huart8.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart8.Init.OverSampling = UART_OVERSAMPLING_16;
    huart8.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart8.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart8.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    return HAL_RS485Ex_Init(&huart8,
                            UART_DE_POLARITY_HIGH,
                            DXL_DE_ASSERT_BITS,
                            DXL_DE_DEASSERT_BITS) == HAL_OK;
}

bool dxl_hal_init(void) {
    servo_bus_gpio_init();

    if (HAL_UART_DeInit(&huart8) != HAL_OK) {
        return false;
    }

    if (!uart_rs485_init(DXL_BAUD_RATE)) {
        return false;
    }

    flush_rx();
    s_ready = true;
    return true;
}

bool dxl_hal_set_baud(uint32_t baud) {
    if (!s_ready) {
        return false;
    }

    if (HAL_UART_DeInit(&huart8) != HAL_OK) {
        return false;
    }
    if (!uart_rs485_init(baud)) {
        return false;
    }

    flush_rx();
    return true;
}

uint32_t dxl_hal_baud(void) {
    return huart8.Init.BaudRate;
}

void dxl_hal_flush_rx(void) {
    if (s_ready) {
        flush_rx();
    }
}

void dxl_hal_recover(void)
{
    if (!s_ready) {
        return;
    }

    (void)HAL_UART_Abort(&huart8);
    flush_rx();
    huart8.ErrorCode = HAL_UART_ERROR_NONE;
    huart8.gState    = HAL_UART_STATE_READY;
    huart8.RxState   = HAL_UART_STATE_READY;
}

dxl_status_code_t dxl_hal_txrx(const uint8_t *tx, size_t tx_len,
                               uint8_t *rx, size_t rx_cap,
                               size_t expected_rx_len,
                               dxl_status_t *status,
                               uint32_t timeout_ms) {
    if (!s_ready || tx == NULL || rx == NULL || status == NULL ||
        tx_len == 0u || expected_rx_len == 0u) {
        return DXL_ERR_PARAM;
    }

    if (rx_cap < expected_rx_len) {
        return DXL_ERR_PARAM;
    }

    const uint32_t tmo =
        (timeout_ms == 0u) ? DXL_DEFAULT_TIMEOUT_MS : timeout_ms;

    servo_bus_gpio_enable();
    flush_rx();

    if (HAL_UART_Transmit(&huart8, (uint8_t *)tx, (uint16_t)tx_len, tmo) !=
        HAL_OK) {
        dxl_hal_recover();
        return DXL_ERR_TIMEOUT;
    }

    /* Do NOT flush RX here — the servo response begins arriving as soon as
     * TX completes; draining RX would discard the status packet. */

    HAL_StatusTypeDef hal = HAL_UART_Receive(&huart8, rx,
                                             (uint16_t)expected_rx_len, tmo);
    size_t actual = expected_rx_len;
    if (hal == HAL_TIMEOUT) {
        actual = expected_rx_len - (size_t)huart8.RxXferCount;
        if (actual == 0u) {
            dxl_hal_recover();
            return DXL_ERR_TIMEOUT;
        }
    } else if (hal != HAL_OK) {
        dxl_hal_recover();
        return DXL_ERR_BUS;
    }

    if (!dxl_pkt_parse_status(rx, actual, status)) {
        dxl_hal_recover();
        return DXL_ERR_CRC;
    }
    if (status->error != 0u) {
        return DXL_ERR_STATUS;
    }

    return DXL_OK;
}
