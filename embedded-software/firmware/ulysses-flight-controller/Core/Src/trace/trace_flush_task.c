#ifdef DEBUG

#include "trace/trace_ringbuf.h"
#include "SD_logging/log_writer.h"
#include "log_records/log_records.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timestamp.h"
#include <string.h>

#define TRACE_FLUSH_PERIOD_MS  50U
#define TRACE_FLUSH_BATCH_MAX  32U

void trace_flush_task_start(void *argument)
{
    (void)argument;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(TRACE_FLUSH_PERIOD_MS));

        if (!log_writer_ready()) {
            continue;
        }

        /* Drain up to TRACE_FLUSH_BATCH_MAX events per cycle. */
        trace_event_t batch[TRACE_FLUSH_BATCH_MAX];
        uint8_t count = 0;

        while (count < TRACE_FLUSH_BATCH_MAX && trace_ringbuf_pop(&batch[count])) {
            count++;
        }

        if (count > 0) {
            /* Build a trace_batch record:
             * Fixed header (8 bytes) followed by count * sizeof(trace_event_t). */
            log_record_trace_batch_t hdr = {
                .base_timestamp_us = batch[0].timestamp_us,
                .event_count           = count,
                .reserved1             = 0,
                .reserved2             = 0,
            };

            /* We need to send header + events as a contiguous payload.
             * Use a stack buffer sized for worst case. */
            uint8_t payload[sizeof(hdr) + TRACE_FLUSH_BATCH_MAX * sizeof(trace_event_t)];
            memcpy(payload, &hdr, sizeof(hdr));
            memcpy(payload + sizeof(hdr), batch, count * sizeof(trace_event_t));

            log_writer_append_record(LOG_RECORD_TYPE_trace_batch,
                                     payload,
                                     sizeof(hdr) + count * sizeof(trace_event_t));
        }

        /* Report any dropped events. */
        uint32_t dropped = trace_ringbuf_take_dropped();
        if (dropped > 0) {
            log_record_trace_overflow_t ovf = {
                .timestamp_us  = timestamp_us(),
                .dropped_count = dropped,
            };
            log_writer_append_record(LOG_RECORD_TYPE_trace_overflow,
                                     &ovf, sizeof(ovf));
        }
    }
}

#endif /* DEBUG */
