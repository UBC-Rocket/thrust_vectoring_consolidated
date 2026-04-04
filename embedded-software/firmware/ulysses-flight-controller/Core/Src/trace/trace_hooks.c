#ifdef DEBUG

#include "trace/trace_hooks.h"
#include "SD_logging/log_writer.h"
#include "timestamp.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

/*
 * Trace hooks write 12-byte events directly into a 512-byte sector buffer
 * owned by log_writer.  Layout per sector:
 *
 *   [0..7]   log_record_frame_t  (filled by flush task)
 *   [8..15]  log_record_trace_batch_t { base_timestamp, count, ... }
 *   [16..N]  trace_event_t[count]  (12 bytes each, max 41)
 *   [N..511] 0xFF padding
 *
 * write_pos starts at TRACE_DATA_OFFSET (16).
 * When write_pos + sizeof(trace_event_t) > 512, the buffer is full.
 */

#define TRACE_DATA_OFFSET  16U   /* skip frame header (8) + batch header (8) */
#define TRACE_EVENT_SIZE   ((size_t)sizeof(trace_event_t))

static log_buffer_t *s_trace_buf = NULL;
static volatile uint32_t s_dropped = 0;

void trace_hooks_init(void)
{
    s_trace_buf = log_writer_claim_trace_buffer();
    if (s_trace_buf != NULL) {
        s_trace_buf->write_pos = TRACE_DATA_OFFSET;
    }
}

uint32_t trace_hooks_take_dropped(void)
{
    uint32_t d = s_dropped;
    s_dropped = 0;
    return d;
}

static inline void trace_append(const trace_event_t *evt)
{
    /* If we lost our buffer, try to reclaim one */
    if (s_trace_buf == NULL) {
        s_trace_buf = log_writer_claim_trace_buffer();
        if (s_trace_buf == NULL) {
            s_dropped++;
            return;
        }
        s_trace_buf->write_pos = TRACE_DATA_OFFSET;
    }

    size_t pos = s_trace_buf->write_pos;

    /* Check if event fits */
    if (pos + TRACE_EVENT_SIZE > LOG_SD_BLOCK_SIZE) {
        /* Buffer full — swap to next empty buffer */
        log_buffer_t *next = log_writer_swap_trace_buffer(s_trace_buf);
        if (next == NULL) {
            /* All trace buffers busy — drop event but DON'T null out
             * s_trace_buf.  It's already marked READY by swap, so the
             * flush task will reclaim it.  Next call will try claim. */
            s_trace_buf = NULL;
            s_dropped++;
            return;
        }
        s_trace_buf = next;
        s_trace_buf->write_pos = TRACE_DATA_OFFSET;
        pos = TRACE_DATA_OFFSET;
    }

    memcpy(&s_trace_buf->data[pos], evt, TRACE_EVENT_SIZE);
    s_trace_buf->write_pos = pos + TRACE_EVENT_SIZE;
}

void trace_hook_task_switched_in(void)
{
    trace_event_t evt = {
        .timestamp_us     = timestamp_us(),
        .task_number      = (uint16_t)uxTaskGetTaskNumber(xTaskGetCurrentTaskHandle()),
        .event_type       = (uint8_t)TRACE_EVT_SWITCH_IN,
        .reserved         = 0,
        .aux              = 0,
    };
    trace_append(&evt);
}

void trace_hook_task_switched_out(void)
{
    trace_event_t evt = {
        .timestamp_us     = timestamp_us(),
        .task_number      = (uint16_t)uxTaskGetTaskNumber(xTaskGetCurrentTaskHandle()),
        .event_type       = (uint8_t)TRACE_EVT_SWITCH_OUT,
        .reserved         = 0,
        .aux              = 0,
    };
    trace_append(&evt);
}

void trace_hook_task_create(void *pxNewTCB)
{
    TaskHandle_t h = (TaskHandle_t)pxNewTCB;
    trace_event_t evt = {
        .timestamp_us     = timestamp_us(),
        .task_number      = (uint16_t)uxTaskGetTaskNumber(h),
        .event_type       = (uint8_t)TRACE_EVT_CREATE,
        .reserved         = 0,
        .aux              = (uint32_t)uxTaskPriorityGet(h),
    };
    trace_append(&evt);
}

#endif /* DEBUG */
