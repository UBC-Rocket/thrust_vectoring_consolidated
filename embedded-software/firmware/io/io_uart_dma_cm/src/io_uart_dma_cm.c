/**
 * @file io_uart_dma_cm.c
 * @brief UART DMA-circular + character-match — implementation.
 *
 * UBC Rocket, 2026
 */

#include "io_uart_dma_cm/io_uart_dma_cm.h"

#include <string.h>

void io_uart_dma_cm_init(io_uart_dma_cm_t *u,
                      UART_HandleTypeDef *huart,
                      uint8_t *circ_buf, size_t circ_size,
                      uint8_t *scratch,  size_t scratch_size,
                      uint8_t match_char,
                      io_uart_dma_cm_msg_cb_t cb, void *user) {
    memset(u, 0, sizeof(*u));
    u->huart        = huart;
    u->circ_buf     = circ_buf;
    u->circ_size    = circ_size;
    u->scratch      = scratch;
    u->scratch_size = scratch_size;
    u->cb           = cb;
    u->user         = user;
    u->last_pos     = 0;

    /* ADD[7:0] may only be written while UE=0 or RE=0 (H7 RM0399). */
    __HAL_UART_DISABLE(huart);
    MODIFY_REG(huart->Instance->CR2, USART_CR2_ADD,
               ((uint32_t)match_char << USART_CR2_ADD_Pos));
    __HAL_UART_ENABLE(huart);
}

void io_uart_dma_cm_start(io_uart_dma_cm_t *u) {
    HAL_UART_Receive_DMA(u->huart, u->circ_buf, (uint16_t)u->circ_size);

    /* We rely on CM as the sole trigger; silence HT/TC so they don't
     * interleave noisy HAL events. */
    if (u->huart->hdmarx) {
        __HAL_DMA_DISABLE_IT(u->huart->hdmarx, DMA_IT_HT | DMA_IT_TC);
    }
    __HAL_UART_ENABLE_IT(u->huart, UART_IT_CM);
}

void io_uart_dma_cm_on_match_isr(io_uart_dma_cm_t *u) {
    /* Clear CM flag up front so we can't miss a second match fired while
     * servicing this one. */
    __HAL_UART_CLEAR_FLAG(u->huart, UART_CLEAR_CMF);

    const uint16_t size = (uint16_t)u->circ_size;
    const uint16_t pos  = size - (uint16_t)__HAL_DMA_GET_COUNTER(u->huart->hdmarx);
    const uint16_t last = u->last_pos;

    size_t msg_len = 0;
    if (pos > last) {
        msg_len = (size_t)(pos - last);
        if (msg_len > u->scratch_size) msg_len = u->scratch_size;
        memcpy(u->scratch, &u->circ_buf[last], msg_len);
    } else if (pos < last) {
        const size_t tail = size - last;
        size_t copy_tail  = tail;
        if (copy_tail > u->scratch_size) copy_tail = u->scratch_size;
        memcpy(u->scratch, &u->circ_buf[last], copy_tail);
        size_t remaining = u->scratch_size - copy_tail;
        size_t copy_head = (pos < remaining) ? pos : remaining;
        memcpy(&u->scratch[copy_tail], u->circ_buf, copy_head);
        msg_len = copy_tail + copy_head;
    }
    u->last_pos = pos;

    if (msg_len > 0 && u->cb) {
        u->cb(u->scratch, msg_len, u->user);
    }
}

void io_uart_dma_cm_on_error_isr(io_uart_dma_cm_t *u) {
    /* Re-arm. HAL_UART_Abort + Receive_DMA is the cleanest recovery. */
    HAL_UART_AbortReceive(u->huart);
    u->last_pos = 0;
    io_uart_dma_cm_start(u);
}

bool io_uart_dma_cm_send(io_uart_dma_cm_t *u, const uint8_t *data, size_t len, uint32_t timeout_ms) {
    return HAL_UART_Transmit(u->huart, (uint8_t *)data, (uint16_t)len, timeout_ms) == HAL_OK;
}
