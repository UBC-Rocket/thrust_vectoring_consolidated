/**
 * @file io_spi_queue.c
 * @brief Generic SPI job queue — implementation.
 *
 * Pure data-structure + DMA submit. The HAL_SPI_*Callback weak overrides
 * live in the per-target io_spi.c, which routes per-hspi to
 * io_spi_queue_on_complete / _on_error here.
 *
 * UBC Rocket, 2026
 */

#include "io_spi_queue/io_spi_queue.h"
#include "io_sys/io_status_led.h"
#include "io_sys/io_debug.h"

#include "stm32h7xx_hal.h"

#include <string.h>

/* -------------------------------------------------------------------------- */
/* Cache maintenance (compile out on cores without a D-cache, e.g. CM4)       */
/* -------------------------------------------------------------------------- */

static inline void dcache_clean(void *addr, uint32_t size) {
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U) {
        SCB_CleanDCache_by_Addr(addr, (int32_t)size);
    }
#else
    (void)addr;
    (void)size;
#endif
}

static inline void dcache_invalidate(void *addr, uint32_t size) {
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U) {
        SCB_InvalidateDCache_by_Addr(addr, (int32_t)size);
    }
#else
    (void)addr;
    (void)size;
#endif
}

/* -------------------------------------------------------------------------- */
/* Critical-section helpers (PRIMASK; ISR-safe)                               */
/* -------------------------------------------------------------------------- */

static inline uint32_t crit_enter(void) {
    uint32_t pri = __get_PRIMASK();
    __disable_irq();
    return pri;
}

static inline void crit_exit(uint32_t pri) {
    __set_PRIMASK(pri);
}

/* -------------------------------------------------------------------------- */
/* Ring helpers (single-producer / single-consumer is sufficient because the  */
/* PRIMASK critical section serialises submitters)                            */
/* -------------------------------------------------------------------------- */

static inline bool queue_empty(const io_spi_queue_t *q) {
    return q->head == q->tail;
}

static inline bool queue_full(const io_spi_queue_t *q) {
    return ((q->head + 1U) % q->depth) == q->tail;
}

static bool dequeue(io_spi_queue_t *q, io_spi_queue_job_t *out) {
    if (queue_empty(q)) return false;
    __DMB();
    *out = q->jobs[q->tail];
    __DMB();
    q->tail = (q->tail + 1U) % q->depth;
    return true;
}

/* -------------------------------------------------------------------------- */
/* Job runner                                                                 */
/* -------------------------------------------------------------------------- */

static bool start_job(io_spi_queue_t *q, io_spi_queue_job_t *job) {
    if (job->cs_port != NULL) {
        IO_SPI_QUEUE_CS_ASSERT(job->cs_port, job->cs_pin);
    }

    /* Some HAL paths leave the SPI handle locked until the ISR unwinds.
     * Force-unlock so a subsequent submit inside the ISR doesn't get
     * spurious HAL_BUSY. */
    __HAL_UNLOCK(q->hspi);

    if ((job->tx_buf != NULL) &&
        (job->type == IO_SPI_QUEUE_XFER_TX || job->type == IO_SPI_QUEUE_XFER_TXRX)) {
        dcache_clean((void *)job->tx_buf, job->len);
    }
    if ((job->rx_buf != NULL) &&
        (job->type == IO_SPI_QUEUE_XFER_RX || job->type == IO_SPI_QUEUE_XFER_TXRX)) {
        /* Pre-invalidate to drop any stale cache lines that might be
         * write-back-merged after DMA finishes. */
        dcache_invalidate(job->rx_buf, job->len);
    }

    HAL_StatusTypeDef status = HAL_ERROR;
    switch (job->type) {
    case IO_SPI_QUEUE_XFER_TX:
        status = HAL_SPI_Transmit_DMA(q->hspi, (uint8_t *)job->tx_buf, job->len);
        break;
    case IO_SPI_QUEUE_XFER_RX:
        status = HAL_SPI_Receive_DMA(q->hspi, job->rx_buf, job->len);
        break;
    case IO_SPI_QUEUE_XFER_TXRX:
    default:
        status = HAL_SPI_TransmitReceive_DMA(q->hspi,
                                             (uint8_t *)job->tx_buf,
                                             job->rx_buf,
                                             job->len);
        break;
    }

    q->last_submit_status = status;

    if (status != HAL_OK) {
        if (job->cs_port != NULL) {
            IO_SPI_QUEUE_CS_DEASSERT(job->cs_port, job->cs_pin);
        }
        q->busy = false;
        q->dma_error_count++;
        if (job->done != NULL) {
            job->done(job, job->user, false);
        }
        
        return false;
    }
    return true;
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */

io_status_t io_spi_queue_init(io_spi_queue_t *q,
                              SPI_HandleTypeDef *hspi,
                              io_spi_queue_job_t *jobs,
                              uint8_t depth) {
    if (q == NULL || hspi == NULL || jobs == NULL) return IO_ERR_PARAM;
    if (depth == 0U || depth > IO_SPI_QUEUE_MAX_DEPTH) return IO_ERR_PARAM;
    memset(q, 0, sizeof(*q));
    q->hspi  = hspi;
    q->jobs  = jobs;
    q->depth = depth;
    return IO_OK;
}

io_status_t io_spi_queue_submit(io_spi_queue_t *q, const io_spi_queue_job_t *job) {
    if (q == NULL || job == NULL) return IO_ERR_PARAM;

    bool                 should_start = false;
    io_spi_queue_job_t  *to_start     = NULL;
    io_status_t          ret          = IO_ERR_BUSY;

    uint32_t pri = crit_enter();

    if (!q->busy) {
        q->busy        = true;
        q->current     = *job;
        to_start       = &q->current;
        should_start   = true;
        ret            = IO_OK;
    } else if (!queue_full(q)) {
        q->jobs[q->head] = *job;
        __DMB();
        q->head = (q->head + 1U) % q->depth;
        ret     = IO_OK;
    } else {
        q->submit_drop_count++;
        ret = IO_ERR_BUSY;
    }

    crit_exit(pri);

    if (should_start) {
        (void)start_job(q, to_start);
        /* Even on start failure, ret stays IO_OK — the caller's done()
         * callback was already invoked with ok=false inside start_job. */
    } 

    return ret;
}

/* Shared "advance to the next pending job" tail. */
static void advance_to_next(io_spi_queue_t *q) {
    bool start_next = false;
    uint32_t pri = crit_enter();
    io_spi_queue_job_t next;
    if (dequeue(q, &next)) {
        q->current = next;
        q->busy    = true;
        start_next = true;
    } else {
        q->busy    = false;
    }
    crit_exit(pri);

    if (start_next) {
        (void)start_job(q, &q->current);
    }
}

void io_spi_queue_on_complete(io_spi_queue_t *q) {
    if (q == NULL) return;

    io_spi_queue_job_t completed = q->current;

    if (completed.cs_port != NULL) {
        IO_SPI_QUEUE_CS_DEASSERT(completed.cs_port, completed.cs_pin);
    }

    /* Make DMA-written RX bytes visible to the CPU before the callback
     * touches them. Idempotent on cores without a D-cache. */
    if ((completed.rx_buf != NULL) &&
        (completed.type == IO_SPI_QUEUE_XFER_RX || completed.type == IO_SPI_QUEUE_XFER_TXRX)) {
        dcache_invalidate(completed.rx_buf, completed.len);
    }

    if (completed.done != NULL) {
        completed.done(&completed, completed.user, true);
    }
    
    advance_to_next(q);
}

void io_spi_queue_on_error(io_spi_queue_t *q) {
    if (q == NULL) return;

    io_spi_queue_job_t failed = q->current;
    if (failed.cs_port != NULL) {
        IO_SPI_QUEUE_CS_DEASSERT(failed.cs_port, failed.cs_pin);
    }
    q->dma_error_count++;

    if (failed.done != NULL) {
        failed.done(&failed, failed.user, false);
    }

    advance_to_next(q);
}
