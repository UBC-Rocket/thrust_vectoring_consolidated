/**
 * @file    io_uart.c (CM7)
 * @brief   H747 CM7 IO impl: half-duplex UART for the Feetech servo bus.
 *
 * IO_UART_SERVO_BUS is hosted on UART8 (1 Mbps, half-duplex on PE1).
 * The control loop runs at 800 Hz directly in the TIM16 ISR; servo
 * commands MUST therefore use the fire-and-forget DMA path
 * (io_uart_send_async). The blocking io_uart_xfer + io_uart_send paths
 * are kept for offline diagnostics (servo identity reads, etc.) but
 * MUST NOT be called from the controls ISR.
 *
 * UBC Rocket, 2026
 */
#include "io_common.h"
#include "io/io_uart.h"

#include "usart.h"

#include <string.h>

struct io_uart {
    UART_HandleTypeDef *huart;
    bool                streaming;     /* false = half-duplex bus */
};

const io_uart_t IO_UART_SERVO_BUS = {
    .huart     = &huart8,
    .streaming = false,
};

/* Per-port async TX state. One entry today (only SERVO_BUS); generalises
 * trivially if a second half-duplex UART appears. */
typedef struct {
    UART_HandleTypeDef *huart;
    volatile bool       busy;
} async_tx_slot_t;

static async_tx_slot_t s_tx_slot = { .huart = &huart8, .busy = false };

/* -------------------------------------------------------------------------- */
/* Public API                                                                  */
/* -------------------------------------------------------------------------- */

io_status_t io_uart_open(const io_uart_t *u, io_uart_rx_cb_t cb, void *user) {
    (void)u; (void)cb; (void)user;
    /* Half-duplex bus has no streaming RX path. */
    return IO_ERR_NOT_READY;
}

io_status_t io_uart_send(const io_uart_t *u, const uint8_t *data, size_t len, uint32_t timeout_ms) {
    if (!u || !u->huart) return IO_ERR_NOT_READY;
    if (HAL_HalfDuplex_EnableTransmitter(u->huart) != HAL_OK) return IO_ERR_BUS;
    return HAL_UART_Transmit(u->huart, (uint8_t *)data, (uint16_t)len, timeout_ms) == HAL_OK
               ? IO_OK : IO_ERR_TIMEOUT;
}

io_status_t io_uart_xfer(const io_uart_t *u,
                         const uint8_t *tx, size_t tx_len,
                         uint8_t *rx, size_t rx_len,
                         uint32_t timeout_ms) {
    if (!u || !u->huart) return IO_ERR_NOT_READY;

    if (HAL_HalfDuplex_EnableTransmitter(u->huart) != HAL_OK) return IO_ERR_BUS;
    if (HAL_UART_Transmit(u->huart, (uint8_t *)tx, (uint16_t)tx_len, timeout_ms) != HAL_OK)
        return IO_ERR_TIMEOUT;

    if (rx_len == 0) return IO_OK;

    if (HAL_HalfDuplex_EnableReceiver(u->huart) != HAL_OK) return IO_ERR_BUS;
    return HAL_UART_Receive(u->huart, rx, (uint16_t)rx_len, timeout_ms) == HAL_OK
               ? IO_OK : IO_ERR_TIMEOUT;
}

io_status_t io_uart_send_async(const io_uart_t *u, const uint8_t *data, size_t len) {
    if (!u || !u->huart || data == NULL || len == 0U) return IO_ERR_PARAM;

    /* Atomic check-and-set on busy. Read-then-write is OK because we
     * only ever set busy here and clear it from the TxCplt ISR; if the
     * ISR is mid-clear during our read we get an IO_ERR_BUSY for one
     * iteration and the caller retries. */
    if (__atomic_exchange_n(&s_tx_slot.busy, true, __ATOMIC_ACQUIRE)) {
        return IO_ERR_BUSY;
    }

    /* Switch to transmitter direction. This pokes a CR1 bit; it does
     * NOT wait for anything. Safe in ISR. */
    if (HAL_HalfDuplex_EnableTransmitter(u->huart) != HAL_OK) {
        __atomic_store_n(&s_tx_slot.busy, false, __ATOMIC_RELEASE);
        return IO_ERR_BUS;
    }

    if (HAL_UART_Transmit_DMA(u->huart, (uint8_t *)data, (uint16_t)len) != HAL_OK) {
        __atomic_store_n(&s_tx_slot.busy, false, __ATOMIC_RELEASE);
        return IO_ERR_BUS;
    }

    return IO_OK;
}

/* -------------------------------------------------------------------------- */
/* HAL TX-complete callback                                                    */
/*                                                                             */
/* Fires when the LAST byte has shifted out of the UART (TC interrupt, set up  */
/* internally by HAL_UART_Transmit_DMA after the DMA-to-TDR pump completes).   */
/* Needs UART8_IRQHandler in stm32h7xx_it.c (CubeMX adds it when UART8 NVIC    */
/* is enabled — see io_uart.h for the IOC requirements).                       */
/* -------------------------------------------------------------------------- */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == s_tx_slot.huart) {
        __atomic_store_n(&s_tx_slot.busy, false, __ATOMIC_RELEASE);
    }
}

/* -------------------------------------------------------------------------- */
/* IRQ glue stubs — kept so stm32h7xx_it.c's includes don't break if/when      */
/* CM7 grows a streaming UART. Today there isn't one on this core.             */
/* -------------------------------------------------------------------------- */
void io_uart_on_match_isr(void *huart) { (void)huart; }
void io_uart_on_error_isr(void *huart) { (void)huart; }
