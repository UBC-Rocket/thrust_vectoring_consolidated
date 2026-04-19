#include "SPI_queue.h"
#include "SPI_device_interactions.h"
#include "spi1_bus.h"
#include "sync.h"
#include "stm32l4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "app_freertos.h"

/**
 * @file SPI_queue.c
 * @brief SPI job queue with DMA chaining - implementation.
 *
 * Supports multiple interrupt-driven devices on a shared SPI bus.
 * Each job describes a DMA transfer (CS pin, Tx buffer, Rx buffer, length,
 * timestamp, transfer type, and completion callback). The queue ensures safe,
 * serialized DMA usage. Device-specific handling is done in job callbacks.
 *
 * Thread Safety Notes:
 * - spi_submit_job() uses critical sections to protect both the busy flag
 *   check and queue enqueue operation atomically.
 * - DMA callbacks use ISR-safe critical sections.
 * - Memory barriers in queue operations ensure proper data visibility.
 *
 *  UBC Rocket, Benedikt Howard, Sept 29th, 2025
 */

/* -------------------------------------------------------------------------- */
/* Core job runner                                                            */
/* -------------------------------------------------------------------------- */

/**
 * @brief Start a new SPI job immediately.
 * @note Assumes bus is idle. DMA completion will chain the next job.
 *
 * @note Cache maintenance:
 *       - TX buffer is cleaned (written back) before DMA reads it.
 *       - RX staging buffer is invalidated after DMA completes (in callback).
 */
static void start_job(spi_job_t *job, spi_job_queue_t *q) {
    q->spi_busy = true;
    CS_ASSERT(job->cs_port, job->cs_pin);

    /* Some HAL paths leave the SPI handle locked until the ISR unwinds.
       Manually unlock here so the next DMA submission cannot spuriously
       return HAL_BUSY inside the ISR. */
    __HAL_UNLOCK(q->spi_bus);

    /*
     * Cache maintenance: clean TX buffer before DMA reads it.
     * This ensures DMA sees CPU's written data, not stale memory.
     * Note: job->tx is embedded in the struct, may not be perfectly aligned,
     * but clean operation will handle the containing cache line.
     */
    if (job->type == SPI_XFER_TX || job->type == SPI_XFER_TXRX) {
        sync_dcache_clean((void *)job->tx, sizeof(job->tx));
    }

    HAL_StatusTypeDef status = HAL_ERROR;

    switch (job->type) {
    case SPI_XFER_TX:
        status = HAL_SPI_Transmit_DMA(q->spi_bus, job->tx, job->len);
        break;
    case SPI_XFER_RX:
        status = HAL_SPI_Receive_DMA(q->spi_bus,
                                     (uint8_t *)q->spi_rx_staging,
                                     job->len);
        break;
    case SPI_XFER_TXRX:
    default:
        status = HAL_SPI_TransmitReceive_DMA(q->spi_bus,
                                             job->tx,
                                             (uint8_t *)q->spi_rx_staging,
                                             job->len);
        break;
    }

    q->last_submit_status = status;

    if (status != HAL_OK) {
        CS_DEASSERT(job->cs_port, job->cs_pin);
        q->spi_busy = false;
#if defined(DEBUG)
        __BKPT(0);
#endif
    }
}

/**
 * @brief Submit an SPI job to the shared bus queue.
 *        Safe to call from both ISR and task contexts.
 *
 * @param job Job descriptor (copied into queue or current_job).
 * @param q Pointer to the queue that the job is for.
 * @return true  if the job was started immediately
 *         false if it was queued (will run later), or dropped if queue full
 *
 * @note Critical section covers both busy check AND enqueue to prevent race
 *       conditions when called from multiple contexts (ISR + task).
 */

bool spi_submit_job(spi_job_t job, spi_job_queue_t *q)
{
    bool started = false;
    bool should_start = false;
    spi_job_t *active_job = NULL;

    /*
     * Critical section must cover:
     * 1. Check of spi_busy flag
     * 2. Setting spi_busy and copying job (if starting immediately)
     * 3. Enqueue operation (if busy)
     *
     * This prevents race when called from both ISR and task contexts.
     */
    uint32_t primask;
    volatile uint32_t basepre = __get_BASEPRI();
    SYNC_ENTER_CRITICAL_RAW(primask);


    if (!q->spi_busy) {
        /* Bus is free - start immediately */
        q->spi_busy = true;
        q->current_job = job;
        active_job = &q->current_job;
        should_start = true;
        started = true;
    } else {
        /* Bus is busy - enqueue for later */
        uint8_t next_head = (q->head + 1U) % SPI_JOB_QUEUE_SIZE;
        if (next_head != q->tail) {
            q->jobs[q->head] = job;
            SYNC_DMB();  /* Ensure job data visible before head update */
            q->head = next_head;
            started = false;  /* Queued, not started */
        } else {
            //TODO: Queue full - job dropped. Consider adding error callback.
            started = false;
        }
    }

    SYNC_EXIT_CRITICAL_RAW(primask);

    /* Start DMA outside critical section to minimize interrupt latency */
    if (should_start && active_job != NULL) {
        start_job(active_job, q);
    }

    return started;
}

/* -------------------------------------------------------------------------- */
/* DMA completion callbacks                                                   */
/* -------------------------------------------------------------------------- */

/**
 * @brief Common handler called at end of DMA transfer.
 *
 * @note start_job() is called OUTSIDE the critical section to minimize
 *       interrupt latency. The critical section only protects queue state.
 *
 * @note Cache invalidation is performed before reading DMA RX buffer to
 *       ensure CPU sees data written by DMA (if D-cache is enabled).
 */
static void spi_dma_complete_common(SPI_HandleTypeDef *hspi) {

    BaseType_t xWoken = pdFALSE;
    bool should_start_next = false;

    /* SPI1 uses the new priority bus driver */
    if (hspi == spi1_ctx.hspi) {
        spi1_dma_complete_handler();
        return;
    }

    /* Identify which queue this SPI belongs to (SPI2/SPI4) */
    spi_job_queue_t *jobq;
    if (hspi == jobq_spi_2.spi_bus) {
        jobq = &jobq_spi_2;
    } else if (hspi == jobq_spi_4.spi_bus) {
        jobq = &jobq_spi_4;
    } else {
        return;
    }

    /* Deassert chip select for completed job */
    CS_DEASSERT(jobq->current_job.cs_port, jobq->current_job.cs_pin);

    /*
     * Cache maintenance: invalidate RX buffer before CPU reads DMA data.
     * This ensures we see fresh data from DMA, not stale cached values.
     * Safe to call even if D-cache is disabled (becomes no-op).
     */
    sync_dcache_invalidate((void *)jobq->spi_rx_staging, SPI_RX_STAGING_SIZE);

    /* Copy current job info before potentially overwriting in critical section */
    spi_job_t completed_job = jobq->current_job;

    /* Invoke completion callback with RX data */
    if (completed_job.done) {
        completed_job.done(&completed_job,
                          (const uint8_t *)jobq->spi_rx_staging,
                          completed_job.done_arg);
    }

    /* Notify waiting task */
    if (completed_job.task_notification_flag != 0) {
        xTaskNotifyFromISR(StateEstimationHandle,
                          completed_job.task_notification_flag,
                          eSetBits,
                          &xWoken);
    }

    /*
     * Critical section: dequeue next job and update state.
     * Keep this section minimal - no HAL calls inside.
     */
    UBaseType_t s = SYNC_ENTER_CRITICAL_FROM_ISR();

    spi_job_t next;
    if (dequeue_job(jobq, &next)) {
        jobq->current_job = next;
        jobq->spi_busy = true;
        should_start_next = true;
    } else {
        jobq->spi_busy = false;
        should_start_next = false;
    }

    SYNC_EXIT_CRITICAL_FROM_ISR(s);

    /*
     * Start next job OUTSIDE critical section.
     * This minimizes time with interrupts disabled and allows other
     * high-priority interrupts to be serviced promptly.
     */
    if (should_start_next) {
        start_job(&jobq->current_job, jobq);
    }

    portYIELD_FROM_ISR(xWoken);
}

/**
 * @brief HAL weak callback for TxRx DMA complete.
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    spi_dma_complete_common(hspi);
}

/**
 * @brief HAL weak callback for Tx-only DMA complete.
 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
    spi_dma_complete_common(hspi);
}

/**
 * @brief HAL weak callback for Rx-only DMA complete.
 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
    spi_dma_complete_common(hspi);
}
