/**
 * @file sen0306_mmwave.c
 * @brief DFRobot SEN0306 mmWave radar driver implementation.
 *
 * Reception uses UART DMA (HAL_UARTEx_ReceiveToIdle_DMA) in DMA_NORMAL mode.
 * The IDLE interrupt fires once per frame burst; the callback calls
 * sen0306_irq_handler() then immediately re-arms the DMA transfer.
 *
 * UBC Rocket, 2026
 */

#include "sensors/sen0306_mmwave.h"
#include <string.h>

/* -------------------------------------------------------------------------- */
/* Global context instance                                                     */
/* -------------------------------------------------------------------------- */

sen0306_context_t sen0306_ctx;

/* -------------------------------------------------------------------------- */
/* Initialisation                                                              */
/* -------------------------------------------------------------------------- */

void sen0306_init(UART_HandleTypeDef *huart) {
    memset(&sen0306_ctx, 0, sizeof(sen0306_context_t));
    sen0306_ctx.huart = huart;
    sen0306_ctx.initialized = true;
    // Hardware reception is started in main.c after the hex-mode command is sent.
}

/* -------------------------------------------------------------------------- */
/* Frame parser (ISR context)                                                  */
/* -------------------------------------------------------------------------- */

/*
 * Feed one byte into the sliding-window frame parser.
 *
 * The parser accumulates bytes into parser_buf.  Whenever the first three
 * bytes are not the expected 0xFF 0xFF 0xFF header the buffer is shifted
 * left by one to stay aligned.  A complete 8-byte frame with a valid tail
 * (bytes 5–7 == 0x00) is decoded and pushed onto the output queue.
 *
 * Called exclusively from sen0306_irq_handler (ISR context).
 */
static void process_byte(uint8_t byte) {
    if (sen0306_ctx.parser_idx >= SEN0306_FRAME_SIZE) {
        sen0306_ctx.parser_idx = 0;
    }

    sen0306_ctx.parser_buf[sen0306_ctx.parser_idx++] = byte;

    // Align to 0xFF 0xFF 0xFF header; shift left if mismatch.
    if (sen0306_ctx.parser_idx >= 3) {
        if (sen0306_ctx.parser_buf[0] != 0xFF ||
            sen0306_ctx.parser_buf[1] != 0xFF ||
            sen0306_ctx.parser_buf[2] != 0xFF) {
            memmove(sen0306_ctx.parser_buf,
                    &sen0306_ctx.parser_buf[1],
                    --sen0306_ctx.parser_idx);
            return;
        }
    }

    if (sen0306_ctx.parser_idx == SEN0306_FRAME_SIZE) {
        if (sen0306_ctx.parser_buf[5] == 0x00 &&
            sen0306_ctx.parser_buf[6] == 0x00 &&
            sen0306_ctx.parser_buf[7] == 0x00) {

            uint16_t dist = ((uint16_t)sen0306_ctx.parser_buf[3] << 8) |
                             sen0306_ctx.parser_buf[4];

            uint8_t next_head = (sen0306_ctx.queue.head + 1) % SEN0306_QUEUE_LEN;
            sen0306_ctx.queue.samples[sen0306_ctx.queue.head].distance_cm = dist;
            sen0306_ctx.queue.samples[sen0306_ctx.queue.head].timestamp_ms =
                HAL_GetTick();

            if (next_head != sen0306_ctx.queue.tail) {
                sen0306_ctx.queue.head = next_head;
            } else {
                sen0306_ctx.queue.tail =
                    (sen0306_ctx.queue.tail + 1) % SEN0306_QUEUE_LEN;
                sen0306_ctx.queue.head = next_head;
                sen0306_ctx.queue_overflows++;
            }

            sen0306_ctx.samples_received++;
        } else {
            sen0306_ctx.parse_errors++;
        }

        sen0306_ctx.parser_idx = 0;
    }
}

/* -------------------------------------------------------------------------- */
/* IRQ handler                                                                 */
/* -------------------------------------------------------------------------- */

/*
 * Called from HAL_UARTEx_RxEventCallback in main.c.
 *
 * In DMA_NORMAL mode the DMA fills dma_buf from index 0 on each arm, so
 * dma_buf[0..size-1] holds exactly the bytes received in this burst.
 * The caller re-arms the DMA immediately after returning.
 */
void sen0306_irq_handler(UART_HandleTypeDef *huart, uint16_t size) {
    if (huart != sen0306_ctx.huart) return;
    for (uint16_t i = 0; i < size; i++) {
        process_byte(sen0306_ctx.dma_buf[i]);
    }
}

/* -------------------------------------------------------------------------- */
/* Consumer API                                                                */
/* -------------------------------------------------------------------------- */

/*
 * Returns a pointer directly into the queue ring buffer — valid until
 * the slot is overwritten by a future sen0306_irq_handler call.
 * Returns NULL when no samples are available.
 */
const sen0306_sample_t *sen0306_get_latest(void) {
    if (sen0306_ctx.queue.head == sen0306_ctx.queue.tail) return NULL;
    // head points to the next empty slot; the latest sample is one behind it.
    uint8_t idx = (sen0306_ctx.queue.head == 0) ?
                  (SEN0306_QUEUE_LEN - 1) : (sen0306_ctx.queue.head - 1);
    return &sen0306_ctx.queue.samples[idx];
}
