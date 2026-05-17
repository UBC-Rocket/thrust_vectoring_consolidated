/**
 * @file spi1_bus.h
 * @brief SPI1 bus driver with simple FIFO scheduling
 *
 * This driver manages the SPI1 bus used for the GNSS Radio slave board.
 * It uses a simple FIFO job queue (same pattern as the sensor SPI buses)
 * with DMA-based transfers and interrupt-driven completion.
 *
 * Thread Safety:
 * - spi1_submit_job() is safe to call from both ISR and task contexts
 * - Uses critical sections to protect queue state
 *
 * UBC Rocket, Feb 2026
 */

#ifndef SPI1_BUS_H
#define SPI1_BUS_H

#include "stm32h5xx_hal.h"
#include "SPI_queue.h"
#include "sync.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* Configuration                                                              */
/* -------------------------------------------------------------------------- */

/** Number of jobs in the FIFO queue */
#define SPI1_QUEUE_SIZE       8

/** TX buffer size per job (largest transaction: 1 TYPE + 256 payload + margin) */
#define SPI1_TX_BUF_SIZE      272

/** RX staging buffer size (matches TX buffer) */
#define SPI1_RX_STAGING_SIZE  272

/* -------------------------------------------------------------------------- */
/* Job Structure                                                              */
/* -------------------------------------------------------------------------- */

/** Forward declaration */
typedef struct spi1_job_t spi1_job_t;

/**
 * @brief Callback type for job completion
 * @param job    The job that just finished
 * @param rx_buf Pointer to received data buffer
 * @param arg    User argument pointer
 */
typedef void (*spi1_done_cb_t)(spi1_job_t *job,
                               const uint8_t *rx_buf,
                               void *arg);

/**
 * @brief SPI1 job descriptor
 *
 * @attention The done callback must be safe to call from ISR context,
 * since it is invoked from the DMA complete ISR.
 */
struct spi1_job_t {
    GPIO_TypeDef *cs_port;       /**< Chip-select port */
    uint16_t cs_pin;             /**< Chip-select pin */
    uint8_t tx[SPI1_TX_BUF_SIZE]; /**< TX buffer (embedded) */
    uint16_t len;                /**< Transfer length in bytes */
    spi_xfer_type_t type;        /**< TX, RX, or TXRX */
    spi1_done_cb_t done;         /**< Completion callback (optional) */
    void *done_arg;              /**< User argument for callback */
    uint32_t task_notification_flag; /**< FreeRTOS notification flag */
};

/* -------------------------------------------------------------------------- */
/* Bus Context Structure                                                      */
/* -------------------------------------------------------------------------- */

/**
 * @brief SPI1 bus context with simple FIFO queue
 */
typedef struct {
    SPI_HandleTypeDef *hspi;

    spi1_job_t jobs[SPI1_QUEUE_SIZE];
    volatile uint8_t head;
    volatile uint8_t tail;

    spi1_job_t current_job;
    volatile bool bus_busy;

    /** DMA RX staging buffer - cache-line aligned for coherency */
    SYNC_CACHE_ALIGNED volatile uint8_t rx_staging[SPI1_RX_STAGING_SIZE];
    volatile HAL_StatusTypeDef last_status;
} spi1_bus_context_t;

/* -------------------------------------------------------------------------- */
/* Global Bus Context                                                         */
/* -------------------------------------------------------------------------- */

/** Global SPI1 bus context - defined in spi1_bus.c */
extern spi1_bus_context_t spi1_ctx;

/* -------------------------------------------------------------------------- */
/* Queue Helpers                                                              */
/* -------------------------------------------------------------------------- */

static inline bool spi1_queue_empty(const spi1_bus_context_t *ctx) {
    return ctx->head == ctx->tail;
}

static inline bool spi1_queue_full(const spi1_bus_context_t *ctx) {
    return ((ctx->head + 1) % SPI1_QUEUE_SIZE) == ctx->tail;
}

static inline bool spi1_is_busy(void) {
    return spi1_ctx.bus_busy;
}

/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

/**
 * @brief Initialize SPI1 bus driver
 *
 * Sets up the FIFO queue and links to the HAL SPI handle.
 * Call this after HAL_SPI_Init() for SPI1.
 *
 * @param hspi Pointer to HAL SPI handle for SPI1
 */
void spi1_bus_init(SPI_HandleTypeDef *hspi);

/* -------------------------------------------------------------------------- */
/* Job Submission                                                             */
/* -------------------------------------------------------------------------- */

/**
 * @brief Submit a job to the SPI1 bus
 *
 * Safe to call from both ISR and task contexts.
 * If bus is idle, job starts immediately. Otherwise it is enqueued.
 *
 * @param job Job descriptor (copied into queue)
 * @return true if job was started immediately, false if queued or failed
 */
bool spi1_submit_job(const spi1_job_t *job);

/* -------------------------------------------------------------------------- */
/* DMA Completion Handler                                                     */
/* -------------------------------------------------------------------------- */

/**
 * @brief Handle DMA completion for SPI1
 *
 * Called from HAL_SPI_TxRxCpltCallback when SPI1 transfer completes.
 * Invokes job callback, dequeues next job, and chains DMA.
 */
void spi1_dma_complete_handler(void);

#ifdef __cplusplus
}
#endif

#endif /* SPI1_BUS_H */
