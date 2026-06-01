/**
 * @file    io_uart.c
 * @brief   H747 CM4 IO impl: UART transport. Defines struct io_uart plus
 *          the const IO_UART_<NAME> instances drivers reference.
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
    bool                streaming;        /* false = half-duplex bus */
    io_uart_dma_cm_t      *runtime;          /* DMA-circular state (NULL for bus mode) */
    uint8_t            *dma_buf;
    size_t              dma_buf_size;
    uint8_t            *scratch;
    size_t              scratch_size;
};

/* Per-port mutable storage. */
static io_uart_dma_cm_t s_dma_gps, s_dma_radio, s_dma_mmwave, s_dma_debug;
static uint8_t       s_dma_buf_gps   [512];
static uint8_t       s_scratch_gps   [512];
static uint8_t       s_dma_buf_radio [1024];
static uint8_t       s_scratch_radio [256];
static uint8_t       s_dma_buf_mmwave[512];
static uint8_t       s_scratch_mmwave[256];
/* Debug port carries COBS-framed messages records — 0x00 = frame
 * boundary. Sized for a state-estimate record (~88 B) with comfortable
 * headroom for back-to-back frames before the task drains. */
static uint8_t       s_dma_buf_debug [512];
static uint8_t       s_scratch_debug [512];

/* Public per-port symbols (referenced extern-const by drivers). */
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
/* IO_UART_SERVO_BUS lives in CM7's io_uart.c. */

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
    if (!u) return IO_ERR_PARAM;
    if (!u->streaming) return IO_ERR_NOT_READY;
    return io_uart_dma_cm_send(u->runtime, data, len, timeout_ms) ? IO_OK : IO_ERR_TIMEOUT;
}

io_status_t io_uart_xfer(const io_uart_t *u,
                         const uint8_t *tx, size_t tx_len,
                         uint8_t *rx, size_t rx_len,
                         uint32_t timeout_ms) {
    /* CM4 owns only streaming UARTs in this design. */
    (void)u; (void)tx; (void)tx_len; (void)rx; (void)rx_len; (void)timeout_ms;
    return IO_ERR_NOT_READY;
}

/* IRQ glue — call from HAL_UART_IRQHandler dispatch in stm32h7xx_it.c. */
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
