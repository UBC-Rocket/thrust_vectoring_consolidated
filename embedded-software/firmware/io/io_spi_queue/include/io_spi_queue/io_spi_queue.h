/**
 * @file io_spi_queue.h
 * @brief Generic SPI job queue with DMA chaining. One queue per physical SPI
 *        bus; queue entries serialise DMA transfers across multiple devices
 *        sharing that bus.
 *
 * Sits under the io_spi transport. Drivers do not see this header — the
 * per-target io_spi.c is the only consumer.
 *
 * Properties:
 *   - Caller-owned TX / RX buffers (no shared staging).
 *   - Opaque @c user pointer carried through to the completion callback.
 *   - ISR-safe submission via PRIMASK critical section.
 *   - Bounded queue with caller-provided ring storage.
 *   - Cache maintenance auto when @c __DCACHE_PRESENT is defined; compiled
 *     out otherwise.
 *   - Recovery on DMA error: CS released, error counter bumped, queue drains.
 *
 * UBC Rocket, 2026
 */

#ifndef IO_SPI_QUEUE_H
#define IO_SPI_QUEUE_H

#include "io_sys/io_types.h"
#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Hard cap on per-queue ring depth (uint8_t indices). */
#define IO_SPI_QUEUE_MAX_DEPTH 255U

typedef enum {
    IO_SPI_QUEUE_XFER_TX,
    IO_SPI_QUEUE_XFER_RX,
    IO_SPI_QUEUE_XFER_TXRX
} io_spi_queue_xfer_t;

typedef struct io_spi_queue io_spi_queue_t;
typedef struct io_spi_queue_job io_spi_queue_job_t;

/**
 * @brief Completion callback. Invoked from the SPI DMA ISR after CS is
 *        deasserted and (if applicable) the RX buffer cache-invalidated.
 *        Must be ISR-safe. @p ok is false on DMA / bus error.
 */
typedef void (*io_spi_queue_done_cb_t)(const io_spi_queue_job_t *job,
                                       void *user, bool ok);

struct io_spi_queue_job {
    GPIO_TypeDef           *cs_port;   ///< chip-select port (NULL = no CS)
    uint16_t                cs_pin;    ///< chip-select pin mask
    const uint8_t          *tx_buf;    ///< source buffer (NULL for RX-only)
    uint8_t                *rx_buf;    ///< destination buffer (NULL for TX-only)
    uint16_t                len;
    io_spi_queue_xfer_t     type;
    io_spi_queue_done_cb_t  done;      ///< completion callback (may be NULL)
    void                   *user;
    uint64_t                t_sample;  ///< caller-supplied timestamp (µs)
};

struct io_spi_queue {
    SPI_HandleTypeDef          *hspi;
    io_spi_queue_job_t         *jobs;   ///< caller-provided ring storage
    uint8_t                     depth;

    volatile uint8_t            head;
    volatile uint8_t            tail;
    io_spi_queue_job_t          current;
    volatile bool               busy;

    volatile HAL_StatusTypeDef  last_submit_status;
    volatile uint32_t           dma_error_count;
    volatile uint32_t           submit_drop_count;   ///< submissions rejected because queue was full
};

/**
 * @brief Initialise a queue.
 *
 * @return IO_OK on success; IO_ERR_PARAM if @p q, @p hspi, or @p jobs is NULL,
 *         or @p depth is 0 or > IO_SPI_QUEUE_MAX_DEPTH.
 */
io_status_t io_spi_queue_init(io_spi_queue_t *q,
                              SPI_HandleTypeDef *hspi,
                              io_spi_queue_job_t *jobs,
                              uint8_t depth);

/**
 * @brief Submit a job. ISR-safe.
 *
 * @return IO_OK if queued (or started immediately);
 *         IO_ERR_BUSY if the ring is full;
 *         IO_ERR_PARAM on invalid args.
 */
io_status_t io_spi_queue_submit(io_spi_queue_t *q, const io_spi_queue_job_t *job);

/**
 * @brief DMA completion hook. Per-target io_spi.c routes
 *        HAL_SPI_TxRx/Tx/RxCpltCallback to this with the matching queue.
 */
void io_spi_queue_on_complete(io_spi_queue_t *q);

/**
 * @brief DMA error hook. Per-target io_spi.c routes HAL_SPI_ErrorCallback
 *        to this with the matching queue.
 */
void io_spi_queue_on_error(io_spi_queue_t *q);

/**
 * @brief BSRR-based CS macros (single-cycle, atomic, no read-modify-write).
 */
#define IO_SPI_QUEUE_CS_ASSERT(port, pin)   ((port)->BSRR = (uint32_t)(pin) << 16)
#define IO_SPI_QUEUE_CS_DEASSERT(port, pin) ((port)->BSRR = (uint32_t)(pin))

#ifdef __cplusplus
}
#endif

#endif /* IO_SPI_QUEUE_H */
