/**
 * @file SPI_queue.h
 * @brief SPI job queue with DMA chaining.
 *
 * Supports multiple interrupt-driven devices on a shared SPI bus.
 * Each job describes a DMA transfer (CS pin, Tx buffer, Rx buffer, length,
 * timestamp, transfer type, and completion callback). The queue ensures safe,
 * serialized DMA usage. Device-specific handling is done in job callbacks.
 *
 * Thread Safety:
 * - spi_submit_job() is safe to call from both ISR and task contexts.
 * - The queue uses critical sections to protect concurrent access.
 * - Memory barriers ensure proper ordering for ring buffers.
 *
 * Cache Coherency:
 * - DMA buffers are cache-line aligned for proper cache maintenance.
 * - Call sync_dcache_invalidate() before reading DMA RX data if D-cache enabled.
 *
 *  UBC Rocket, Benedikt Howard, Sept 29th, 2025
 */

#ifndef SPI_QUEUE
#define SPI_QUEUE

#include "stm32h5xx_hal.h"
#include "sync.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "timestamp.h"

/* -------------------------------------------------------------------------- */
/* Configuration                                                              */
/* -------------------------------------------------------------------------- */

#define SPI_JOB_QUEUE_SIZE 16
#define SPI_RX_STAGING_SIZE 64

#define CS_ASSERT(port, pin)   ((port)->BSRR = (uint32_t)(pin) << 16)
#define CS_DEASSERT(port, pin) ((port)->BSRR = (uint32_t)(pin))


/* -------------------------------------------------------------------------- */
/* Job types and callbacks                                                    */
/* -------------------------------------------------------------------------- */

/**
 * @brief Transfer type for SPI job.
 */
typedef enum {
    SPI_XFER_TX,     ///< Transmit only
    SPI_XFER_RX,     ///< Receive only (rare; still requires dummy Tx)
    SPI_XFER_TXRX    ///< Transmit + receive (most common)
} spi_xfer_type_t;

/**
 * @brief Forward declaration for SPI job structure.
 */
typedef struct spi_job_t spi_job_t;

/**
 * @brief Callback type for job completion.
 * @param job    The job that just finished.
 * @param rx_buf Pointer to received data buffer (if any).
 * @param arg    User argument pointer.
 */
typedef void (*spi_done_cb_t)(spi_job_t *job,
                              const uint8_t *rx_buf,
                              void *arg);

typedef enum {
    SENSOR_ID_ACCEL,
    SENSOR_ID_GYRO,
    SENSOR_ID_BARO,
    SENSOR_ID_BARO2,
    SENSOR_ID_OTHER
} sensor_id_t;

/**
 * @brief SPI job descriptor. Represents one queued DMA transaction.
 * 
 * @attention The done function must be safte to be called from a ISR, 
 * since it will be invoked form the DMA complete ISR.
 */
struct spi_job_t {
    GPIO_TypeDef *cs_port;  ///< Chip-select port
    uint16_t cs_pin;        ///< Chip-select pin
    uint8_t tx[16];            ///< Tx buffer pointer
    uint16_t len;           ///< Transfer length in bytes
    uint64_t t_sample;      ///< Sample timestamp in µs
    spi_xfer_type_t type;   ///< Transfer type
    spi_done_cb_t done;     ///< Completion callback (optional)
    void *done_arg;         ///< User argument for callback
    sensor_id_t sensor;
    uint32_t task_notification_flag;
};

/* -------------------------------------------------------------------------- */
/* Queue structure                                                            */
/* -------------------------------------------------------------------------- */

/**
 * @brief Circular buffer of SPI jobs.
 *
 * @note spi_rx_staging is cache-line aligned for DMA coherency. If D-cache is
 *       enabled, call sync_dcache_invalidate() on spi_rx_staging before reading
 *       DMA results.
 */
typedef struct {
    SPI_HandleTypeDef *spi_bus;

    spi_job_t jobs[SPI_JOB_QUEUE_SIZE];
    volatile uint8_t head;
    volatile uint8_t tail;

    spi_job_t current_job;
    volatile bool spi_busy;

    /** @brief DMA RX staging buffer - cache-line aligned for coherency. */
    SYNC_CACHE_ALIGNED volatile uint8_t spi_rx_staging[SPI_RX_STAGING_SIZE];
    volatile HAL_StatusTypeDef last_submit_status;
} spi_job_queue_t;

/* -------------------------------------------------------------------------- */
/* Queue helpers                                                              */
/* -------------------------------------------------------------------------- */

/**
 * @brief Check if job queue is empty.
 */
static inline bool queue_empty(spi_job_queue_t *q) {
    return q->head == q->tail;
}

/**
 * @brief Check if job queue is full.
 */
static inline bool queue_full(spi_job_queue_t *q) {
    return ((q->head + 1) % SPI_JOB_QUEUE_SIZE) == q->tail;
}

/**
 * @brief Enqueue a new SPI job.
 * @return True if successful, false if queue full.
 * @note Uses memory barrier to ensure job data is visible before head update.
 */
static inline bool enqueue_job(spi_job_queue_t *q, const spi_job_t *job) {
    if (queue_full(q)) return false;
    q->jobs[q->head] = *job;
    SYNC_DMB();  /* Ensure job data written before head update is visible */
    q->head = (q->head + 1) % SPI_JOB_QUEUE_SIZE;
    return true;
}

/**
 * @brief Dequeue the next SPI job.
 * @return True if successful, false if queue empty.
 * @note Uses memory barriers to ensure proper ordering with producer.
 */
static inline bool dequeue_job(spi_job_queue_t *q, spi_job_t *job) {
    if (queue_empty(q)) return false;
    SYNC_DMB();  /* Ensure we see job data written before head was updated */
    *job = q->jobs[q->tail];
    SYNC_DMB();  /* Ensure job read completes before tail update */
    q->tail = (q->tail + 1) % SPI_JOB_QUEUE_SIZE;
    return true;
}


/* -------------------------------------------------------------------------- */
/* Core job runner                                                            */
/* -------------------------------------------------------------------------- */

/**
 * @brief Submit an SPI job to the shared bus queue.
 *        Safe to call from both ISR and task contexts.
 *
 * @param job Pointer to job descriptor (copied into queue if not started immediately).
 * @return true  if the job was started immediately
 *         false if it was queued (will run later)
 */
bool spi_submit_job(spi_job_t job, spi_job_queue_t *q);

#endif /* SPI_QUEUE */
