/**
 * @file spi1_bus.c
 * @brief SPI1 bus driver implementation with simple FIFO scheduling
 *
 * This driver manages the SPI1 bus for the GNSS Radio slave board.
 * It uses the same FIFO pattern as the sensor SPI buses (SPI2/SPI4)
 * with DMA-based transfers and interrupt-driven completion.
 *
 * UBC Rocket, Feb 2026
 */

#include "spi1_bus.h"
#include "timestamp.h"
#include "sync.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app_freertos.h"
#include <string.h>

/* -------------------------------------------------------------------------- */
/* Global Context                                                             */
/* -------------------------------------------------------------------------- */

spi1_bus_context_t spi1_ctx;

/* -------------------------------------------------------------------------- */
/* Chip Select Macros                                                         */
/* -------------------------------------------------------------------------- */

#define CS_ASSERT(port, pin)   ((port)->BSRR = (uint32_t)(pin) << 16)
#define CS_DEASSERT(port, pin) ((port)->BSRR = (uint32_t)(pin))

/* -------------------------------------------------------------------------- */
/* Forward Declarations                                                       */
/* -------------------------------------------------------------------------- */

static void spi1_start_job(spi1_job_t *job);

/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

void spi1_bus_init(SPI_HandleTypeDef *hspi)
{
    memset(&spi1_ctx, 0, sizeof(spi1_ctx));
    spi1_ctx.hspi = hspi;
    spi1_ctx.last_status = HAL_OK;
}

/* -------------------------------------------------------------------------- */
/* Job Execution                                                              */
/* -------------------------------------------------------------------------- */

/**
 * @brief Start a DMA transfer for a job
 *
 * Assumes bus is idle. Sets up CS, cache maintenance, and initiates DMA.
 */
static void spi1_start_job(spi1_job_t *job)
{
    spi1_ctx.bus_busy = true;
    CS_ASSERT(job->cs_port, job->cs_pin);

    /* Unlock HAL SPI handle (may be locked from previous ISR context) */
    __HAL_UNLOCK(spi1_ctx.hspi);

    /* Cache maintenance: clean TX buffer before DMA reads it */
    if (job->type == SPI_XFER_TX || job->type == SPI_XFER_TXRX) {
        sync_dcache_clean((void *)job->tx, sizeof(job->tx));
    }

    HAL_StatusTypeDef status = HAL_ERROR;

    switch (job->type) {
    case SPI_XFER_TX:
        status = HAL_SPI_Transmit_DMA(spi1_ctx.hspi, job->tx, job->len);
        break;
    case SPI_XFER_RX:
        status = HAL_SPI_Receive_DMA(spi1_ctx.hspi,
                                     (uint8_t *)spi1_ctx.rx_staging,
                                     job->len);
        break;
    case SPI_XFER_TXRX:
    default:
        status = HAL_SPI_TransmitReceive_DMA(spi1_ctx.hspi,
                                             job->tx,
                                             (uint8_t *)spi1_ctx.rx_staging,
                                             job->len);
        break;
    }

    spi1_ctx.last_status = status;

    if (status != HAL_OK) {
        CS_DEASSERT(job->cs_port, job->cs_pin);
        spi1_ctx.bus_busy = false;
#if defined(DEBUG)
        __BKPT(0);
#endif
    }
}

/* -------------------------------------------------------------------------- */
/* Job Submission                                                             */
/* -------------------------------------------------------------------------- */

bool spi1_submit_job(const spi1_job_t *job)
{
    bool started = false;
    bool should_start = false;
    spi1_job_t *active_job = NULL;

    uint32_t primask;
    SYNC_ENTER_CRITICAL_RAW(primask);

    if (!spi1_ctx.bus_busy) {
        /* Bus is free — start immediately */
        spi1_ctx.bus_busy = true;
        spi1_ctx.current_job = *job;
        active_job = &spi1_ctx.current_job;
        should_start = true;
        started = true;
    } else {
        /* Bus is busy — enqueue for later */
        uint8_t next_head = (spi1_ctx.head + 1U) % SPI1_QUEUE_SIZE;
        if (next_head != spi1_ctx.tail) {
            spi1_ctx.jobs[spi1_ctx.head] = *job;
            SYNC_DMB();
            spi1_ctx.head = next_head;
            started = false;
        } else {
            /* Queue full — job dropped */
            started = false;
        }
    }

    SYNC_EXIT_CRITICAL_RAW(primask);

    /* Start DMA outside critical section */
    if (should_start && active_job != NULL) {
        spi1_start_job(active_job);
    }

    return started;
}

/* -------------------------------------------------------------------------- */
/* DMA Completion Handler                                                     */
/* -------------------------------------------------------------------------- */

void spi1_dma_complete_handler(void)
{
    BaseType_t xWoken = pdFALSE;
    bool should_start_next = false;

    /* Deassert chip select for completed job */
    CS_DEASSERT(spi1_ctx.current_job.cs_port, spi1_ctx.current_job.cs_pin);

    /* Cache maintenance: invalidate RX buffer before CPU reads */
    sync_dcache_invalidate((void *)spi1_ctx.rx_staging, SPI1_RX_STAGING_SIZE);

    /* Copy current job info before potentially overwriting */
    spi1_job_t completed_job = spi1_ctx.current_job;

    /* Invoke completion callback with RX data */
    if (completed_job.done) {
        completed_job.done(&completed_job,
                          (const uint8_t *)spi1_ctx.rx_staging,
                          completed_job.done_arg);
    }

    /* Notify waiting task if requested */
    if (completed_job.task_notification_flag != 0) {
        xTaskNotifyFromISR(StateEstimationHandle,
                          completed_job.task_notification_flag,
                          eSetBits,
                          &xWoken);
    }

    /* Critical section: dequeue next job and update state */
    UBaseType_t s = SYNC_ENTER_CRITICAL_FROM_ISR();

    if (spi1_ctx.head != spi1_ctx.tail) {
        /* Dequeue next job */
        SYNC_DMB();
        spi1_ctx.current_job = spi1_ctx.jobs[spi1_ctx.tail];
        SYNC_DMB();
        spi1_ctx.tail = (spi1_ctx.tail + 1) % SPI1_QUEUE_SIZE;
        spi1_ctx.bus_busy = true;
        should_start_next = true;
    } else {
        spi1_ctx.bus_busy = false;
    }

    SYNC_EXIT_CRITICAL_FROM_ISR(s);

    /* Start next job outside critical section */
    if (should_start_next) {
        spi1_start_job(&spi1_ctx.current_job);
    }

    portYIELD_FROM_ISR(xWoken);
}
