/**
 * @file    io_uart.c
 * @brief   H747 CM4 IO impl: streaming UARTs plus the full-duplex servo bus.
 *
 * UBC Rocket, 2026
 */
#include "io_common.h"
#include "io/io_uart.h"

#include "usart.h"
#include "io_uart_dma_cm/io_uart_dma_cm.h"

#include <string.h>

struct io_uart {
    UART_HandleTypeDef *huart;
    uint8_t             match_byte;
    bool                streaming;
    io_uart_dma_cm_t      *runtime;
    uint8_t            *dma_buf;
    size_t              dma_buf_size;
    uint8_t            *scratch;
    size_t              scratch_size;
};

static io_uart_dma_cm_t s_dma_gps, s_dma_radio, s_dma_mmwave, s_dma_debug;
static uint8_t       s_dma_buf_gps   [512];
static uint8_t       s_scratch_gps   [512];
static uint8_t       s_dma_buf_radio [1024];
static uint8_t       s_scratch_radio [256];
static uint8_t       s_dma_buf_mmwave[512];
static uint8_t       s_scratch_mmwave[256];
static uint8_t       s_dma_buf_debug [512];
static uint8_t       s_scratch_debug [512];

const io_uart_t IO_UART_GPS = {
    .huart        = &huart5, .match_byte = '\n', .streaming = true,
    .runtime      = &s_dma_gps,
    .dma_buf      = s_dma_buf_gps,   .dma_buf_size = sizeof(s_dma_buf_gps),
    .scratch      = s_scratch_gps,   .scratch_size = sizeof(s_scratch_gps),
};
const io_uart_t IO_UART_RADIO = {
    .huart        = &huart7, .match_byte = 0x00, .streaming = true,
    .runtime      = &s_dma_radio,
    .dma_buf      = s_dma_buf_radio, .dma_buf_size = sizeof(s_dma_buf_radio),
    .scratch      = s_scratch_radio, .scratch_size = sizeof(s_scratch_radio),
};
const io_uart_t IO_UART_MMWAVE = {
    .huart        = &huart4, .match_byte = 0x00, .streaming = true,
    .runtime      = &s_dma_mmwave,
    .dma_buf      = s_dma_buf_mmwave, .dma_buf_size = sizeof(s_dma_buf_mmwave),
    .scratch      = s_scratch_mmwave, .scratch_size = sizeof(s_scratch_mmwave),
};
const io_uart_t IO_UART_DEBUG = {
    .huart        = &hlpuart1, .match_byte = 0x00, .streaming = true,
    .runtime      = &s_dma_debug,
    .dma_buf      = s_dma_buf_debug, .dma_buf_size = sizeof(s_dma_buf_debug),
    .scratch      = s_scratch_debug, .scratch_size = sizeof(s_scratch_debug),
};

/* Servo bus: UART8 full-duplex on PE0/PE1 (1 Mbps at boot; Dynamixel driver
 * re-inits to 57600). Fire-and-forget TX uses DMA from the actuator task. */
const io_uart_t IO_UART_SERVO_BUS = {
    .huart     = &huart8,
    .streaming = false,
};

typedef struct {
    UART_HandleTypeDef *huart;
    volatile bool       busy;
} async_tx_slot_t;

static async_tx_slot_t s_servo_tx_slot = { .huart = &huart8, .busy = false };

io_status_t io_uart_open(const io_uart_t *u, io_uart_rx_cb_t cb, void *user) {
    if (!u) return IO_ERR_PARAM;
    if (!u->streaming) return IO_ERR_NOT_READY;
    io_uart_dma_cm_init(u->runtime, u->huart,
                     u->dma_buf, u->dma_buf_size,
                     u->scratch, u->scratch_size,
                     u->match_byte, cb, user);
    io_uart_dma_cm_start(u->runtime);
    return IO_OK;
}

io_status_t io_uart_send(const io_uart_t *u, const uint8_t *data, size_t len, uint32_t timeout_ms) {
    if (!u || !u->huart) return IO_ERR_NOT_READY;
    if (u->streaming) {
        return io_uart_dma_cm_send(u->runtime, data, len, timeout_ms) ? IO_OK : IO_ERR_TIMEOUT;
    }
    return HAL_UART_Transmit(u->huart, (uint8_t *)data, (uint16_t)len, timeout_ms) == HAL_OK
               ? IO_OK : IO_ERR_TIMEOUT;
}

io_status_t io_uart_xfer(const io_uart_t *u,
                         const uint8_t *tx, size_t tx_len,
                         uint8_t *rx, size_t rx_len,
                         uint32_t timeout_ms) {
    if (!u || !u->huart) return IO_ERR_NOT_READY;

    if (HAL_UART_Transmit(u->huart, (uint8_t *)tx, (uint16_t)tx_len, timeout_ms) != HAL_OK) {
        return IO_ERR_TIMEOUT;
    }
    if (rx_len == 0) return IO_OK;
    return HAL_UART_Receive(u->huart, rx, (uint16_t)rx_len, timeout_ms) == HAL_OK
               ? IO_OK : IO_ERR_TIMEOUT;
}

io_status_t io_uart_send_async(const io_uart_t *u, const uint8_t *data, size_t len) {
    if (!u || !u->huart || data == NULL || len == 0U) return IO_ERR_PARAM;

    if (__atomic_exchange_n(&s_servo_tx_slot.busy, true, __ATOMIC_ACQUIRE)) {
        return IO_ERR_BUSY;
    }

    if (HAL_UART_Transmit_DMA(u->huart, (uint8_t *)data, (uint16_t)len) != HAL_OK) {
        __atomic_store_n(&s_servo_tx_slot.busy, false, __ATOMIC_RELEASE);
        return IO_ERR_BUS;
    }

    return IO_OK;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == s_servo_tx_slot.huart) {
        __atomic_store_n(&s_servo_tx_slot.busy, false, __ATOMIC_RELEASE);
    }
}

static const io_uart_t * const s_streaming_uarts[] = {
    &IO_UART_GPS, &IO_UART_RADIO, &IO_UART_MMWAVE, &IO_UART_DEBUG,
};

void io_uart_on_match_isr(void *huart) {
    UART_HandleTypeDef *h = (UART_HandleTypeDef *)huart;
    for (unsigned i = 0; i < (sizeof(s_streaming_uarts)/sizeof(s_streaming_uarts[0])); ++i) {
        if (s_streaming_uarts[i]->huart == h) {
            io_uart_dma_cm_on_match_isr(s_streaming_uarts[i]->runtime);
            return;
        }
    }
}

void io_uart_on_error_isr(void *huart) {
    UART_HandleTypeDef *h = (UART_HandleTypeDef *)huart;
    for (unsigned i = 0; i < (sizeof(s_streaming_uarts)/sizeof(s_streaming_uarts[0])); ++i) {
        if (s_streaming_uarts[i]->huart == h) {
            io_uart_dma_cm_on_error_isr(s_streaming_uarts[i]->runtime);
            return;
        }
    }
}
