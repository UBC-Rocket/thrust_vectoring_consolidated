/**
 * @file sen0306_mmwave.h
 * @brief Driver for the DFRobot SEN0306 mmWave radar (distance-only mode).
 *
 * Reception uses UART DMA (HAL_UARTEx_ReceiveToIdle_DMA) in DMA_NORMAL mode.
 * The DMA channel (GPDMA2_Channel5) is configured in stm32h5xx_hal_msp.c and
 * linked to huart4. Call sen0306_irq_handler() from HAL_UARTEx_RxEventCallback
 * in main.c, then immediately re-arm with HAL_UARTEx_ReceiveToIdle_DMA.
 *
 * Frame format (8 bytes, distance-only mode):
 *   [0xFF][0xFF][0xFF][DIST_H][DIST_L][0x00][0x00][0x00]
 *   Distance is big-endian uint16 in centimetres.
 *
 * UBC Rocket, 2026
 */

#ifndef SEN0306_MMWAVE_H
#define SEN0306_MMWAVE_H

#include "stm32l4xx_hal.h"
#include "sensors/spsc_queue.h"
#include <stdbool.h>
#include <stdint.h>

/* -------------------------------------------------------------------------- */
/* Configuration                                                               */
/* -------------------------------------------------------------------------- */

#define SEN0306_FRAME_SIZE   8   /**< Bytes per frame in distance-only mode */
#define SEN0306_DMA_BUF_SIZE 64  /**< DMA reception buffer size (bytes).
                                      Sized to hold 8 complete frames;
                                      IDLE fires after each burst so the
                                      buffer rarely fills completely. */
#define SEN0306_QUEUE_LEN    16  /**< Output sample queue depth */

/* -------------------------------------------------------------------------- */
/* Data types                                                                  */
/* -------------------------------------------------------------------------- */

/**
 * @brief A single parsed radar sample.
 */
typedef struct {
    uint16_t distance_cm;   ///< Target distance in centimetres (0–2000)
    uint32_t timestamp_ms;  ///< HAL_GetTick() value at time of reception (ms)
} sen0306_sample_t;

/**
 * @brief SPSC output queue for parsed samples.
 *
 * Producer: sen0306_irq_handler (ISR context via HAL_UARTEx_RxEventCallback).
 * Consumer: application task via sen0306_get_latest().
 * SENSORS_MEMORY_BARRIER() separates the sample write from the head update.
 */
typedef struct {
    sen0306_sample_t samples[SEN0306_QUEUE_LEN];
    volatile uint8_t head;  ///< Written by producer only
    volatile uint8_t tail;  ///< Written by consumer only
} sen0306_queue_t;

/**
 * @brief Driver context. One global instance: sen0306_ctx.
 */
typedef struct {
    UART_HandleTypeDef *huart;  ///< UART handle passed to sen0306_init()
    bool initialized;           ///< Set true by sen0306_init()

    /* ── DMA reception ──────────────────────────────────────────────────── */
    /** DMA destination buffer. HAL_UARTEx_ReceiveToIdle_DMA fills this from
     *  index 0 on each arm. Bytes [0..size-1] are valid in the callback. */
    uint8_t dma_buf[SEN0306_DMA_BUF_SIZE];

    /* ── Frame parser ───────────────────────────────────────────────────── */
    uint8_t  parser_buf[SEN0306_FRAME_SIZE]; ///< Sliding window accumulator
    uint16_t parser_idx;                     ///< Valid bytes in parser_buf

    /* ── Statistics (read-only from application) ────────────────────────── */
    uint32_t samples_received;  ///< Total valid frames parsed
    uint32_t parse_errors;      ///< Frames with invalid tail bytes
    uint32_t queue_overflows;   ///< Samples dropped due to full queue

    /* ── Output ─────────────────────────────────────────────────────────── */
    sen0306_queue_t queue;
} sen0306_context_t;

extern sen0306_context_t sen0306_ctx;

/* -------------------------------------------------------------------------- */
/* Public API                                                                  */
/* -------------------------------------------------------------------------- */

/**
 * @brief Initialise the driver context.
 *
 * Zeroes the context and stores the UART handle. Does not start hardware
 * reception — call HAL_UARTEx_ReceiveToIdle_DMA() in main.c after sending
 * the hex-mode command to the sensor.
 *
 * @param huart  UART handle connected to the SEN0306 (huart4 on this board).
 */
void sen0306_init(UART_HandleTypeDef *huart);

/**
 * @brief Process a DMA burst and feed bytes to the frame parser.
 *
 * Call this from HAL_UARTEx_RxEventCallback() in main.c, then immediately
 * re-arm with HAL_UARTEx_ReceiveToIdle_DMA(). The DMA_NORMAL transfer always
 * fills dma_buf from index 0, so @p size bytes starting at dma_buf[0] are
 * the new data for this burst.
 *
 * @param huart  UART handle (guards against spurious calls for other UARTs).
 * @param size   Number of bytes received into dma_buf since the last arm.
 */
void sen0306_irq_handler(UART_HandleTypeDef *huart, uint16_t size);

/**
 * @brief Peek at the most recently parsed sample without consuming it.
 *
 * Returns a pointer directly into the queue — valid until the next call to
 * sen0306_irq_handler() that overwrites that slot (queue depth: SEN0306_QUEUE_LEN).
 *
 * @return Pointer to the latest sample, or NULL if the queue is empty.
 */
const sen0306_sample_t *sen0306_get_latest(void);

/* -------------------------------------------------------------------------- */
/* Queue helpers (inline, zero overhead)                                       */
/* -------------------------------------------------------------------------- */

/** @brief Returns true if no samples are available. */
static inline bool sen0306_queue_empty(void) {
    return sen0306_ctx.queue.head == sen0306_ctx.queue.tail;
}

/** @brief Returns the number of unread samples in the queue. */
static inline uint8_t sen0306_queue_count(void) {
    return (sen0306_ctx.queue.head - sen0306_ctx.queue.tail
            + SEN0306_QUEUE_LEN) % SEN0306_QUEUE_LEN;
}

#endif /* SEN0306_MMWAVE_H */
