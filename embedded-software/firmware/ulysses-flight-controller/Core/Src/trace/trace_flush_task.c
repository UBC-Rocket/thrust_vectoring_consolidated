#include "SD_logging/log_writer.h"
#include "SD_logging/log_service.h"
#include "log_records/log_records.h"
#include "log_records/log_frame.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timestamp.h"
#include <string.h>

#ifdef DEBUG
#include "trace/trace_hooks.h"
#endif

#define FLUSH_TIMEOUT_MS    10U
#define PERIODIC_FLUSH_MS   100U

/* Trace sector layout offsets */
#define TRACE_FRAME_OFFSET  0U
#define TRACE_BATCH_OFFSET  8U   /* sizeof(log_record_frame_t) */
#define TRACE_DATA_OFFSET   16U  /* frame + batch header */
#define TRACE_EVENT_SIZE    12U  /* sizeof(trace_event_t) */

/**
 * @brief Prepare a trace sector for DMA: fill frame header, batch header,
 *        pad, and compute CRC.
 */
#ifdef DEBUG
static void finalize_trace_sector(log_buffer_t *buf)
{
    size_t payload_len = buf->write_pos - sizeof(log_record_frame_t);
    uint8_t event_count = (uint8_t)(
        (buf->write_pos - TRACE_DATA_OFFSET) / TRACE_EVENT_SIZE);

    /* Read base_timestamp from first event (at offset 16) */
    uint32_t base_ts = 0;
    if (event_count > 0) {
        memcpy(&base_ts, &buf->data[TRACE_DATA_OFFSET], sizeof(uint32_t));
    }

    /* Fill batch header at offset 8 */
    log_record_trace_batch_t batch_hdr = {
        .base_timestamp_us = base_ts,
        .event_count       = event_count,
        .reserved1         = 0,
        .reserved2         = 0,
    };
    memcpy(&buf->data[TRACE_BATCH_OFFSET], &batch_hdr, sizeof(batch_hdr));

    /* Pad remaining bytes */
    if (buf->write_pos < LOG_SD_BLOCK_SIZE) {
        memset(&buf->data[buf->write_pos], 0xFF,
               LOG_SD_BLOCK_SIZE - buf->write_pos);
    }

    /* Build frame header with CRC */
    log_record_frame_t frame = {
        .magic    = LOG_RECORD_MAGIC,
        .type     = (uint8_t)LOG_RECORD_TYPE_trace_batch,
        .length   = (uint16_t)payload_len,
        .crc16    = 0U,
        .reserved = 0U,
    };

    /* Compute CRC over frame header (with crc=0) + payload */
    uint16_t crc = log_crc16_ccitt_compute(
        (const uint8_t *)&frame, sizeof(frame));
    crc = log_crc16_ccitt_accumulate(
        crc, &buf->data[sizeof(log_record_frame_t)], payload_len);
    frame.crc16 = crc;

    memcpy(&buf->data[TRACE_FRAME_OFFSET], &frame, sizeof(frame));
}
#endif /* DEBUG */

void sd_flush_task_start(void *argument)
{
    (void)argument;

    /* Initialize the SD log writer.  Retries until the SD card is ready
     * (HAL_SD_Init may need the peripheral to settle after power-on). */
    while (!log_writer_init()) {
        vTaskDelay(pdMS_TO_TICKS(50U));
    }
    log_service_mark_ready();

    /* Write flight header so the decoder can identify this flight */
    {
        uint32_t magic = HAL_GetTick() ^ 0x5A5AA5A5U;
        if (magic == 0U) magic = 0xA5A5A5A5U;
        log_service_log_flight_header(&(log_record_flight_header_t){
            .timestamp_us   = timestamp_us(),
            .flight_magic   = magic,
            .flight_counter = HAL_GetTick(),
        });
    }

    /* Register so the DMA ISR can wake us immediately after each write */
    log_writer_set_flush_task(xTaskGetCurrentTaskHandle());

#ifdef DEBUG
    trace_hooks_init();
#endif

    uint32_t last_periodic_flush = HAL_GetTick();

    for (;;) {
        /* Wake on DMA completion (from ISR) or 10ms timeout if idle */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(FLUSH_TIMEOUT_MS));

        if (!log_writer_ready()) continue;

        /* Continuously drain both trace and regular buffers until idle.
         * Re-scan after each DMA write so freed trace buffers are
         * reclaimed immediately by the hooks. */
        for (;;) {
            bool did_work = false;

#ifdef DEBUG
            for (uint8_t i = LOG_BUF_TRACE_START; i < LOG_BUF_TRACE_END; i++) {
                log_buffer_t *buf = log_writer_get_buffer(i);
                if (buf != NULL && buf->state == BUF_READY) {
                    finalize_trace_sector(buf);
                    log_writer_dma_write_block(i);
                    did_work = true;
                }
            }
#endif

            uint8_t buf_idx;
            if (log_writer_submit_dequeue(&buf_idx)) {
                log_writer_dma_write_block(buf_idx);
                did_work = true;
            }

            if (!did_work) break;
        }

        /* Periodic flush of partial log buffer */
        uint32_t now = HAL_GetTick();
        if ((now - last_periodic_flush) >= PERIODIC_FLUSH_MS) {
            log_writer_flush();
            last_periodic_flush = now;
        }

#ifdef DEBUG
        /* Report any dropped trace events */
        uint32_t dropped = trace_hooks_take_dropped();
        if (dropped > 0) {
            log_record_trace_overflow_t ovf = {
                .timestamp_us  = timestamp_us(),
                .dropped_count = dropped,
            };
            log_writer_append_record(LOG_RECORD_TYPE_trace_overflow,
                                     &ovf, sizeof(ovf));
        }
#endif
    }
}
