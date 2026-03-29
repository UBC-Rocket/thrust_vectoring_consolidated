#ifdef DEBUG

#include "trace/trace_hooks.h"
#include "trace/trace_ringbuf.h"
#include "timestamp.h"
#include "FreeRTOS.h"
#include "task.h"

void trace_hook_task_switched_in(void)
{
    trace_event_t evt = {
        .timestamp_us     = timestamp_us(),
        .task_number      = (uint16_t)uxTaskGetTaskNumber(NULL),
        .event_type       = (uint8_t)TRACE_EVT_SWITCH_IN,
        .reserved         = 0,
        .aux              = 0,
    };
    trace_ringbuf_push(&evt);
}

void trace_hook_task_switched_out(void)
{
    trace_event_t evt = {
        .timestamp_us     = timestamp_us(),
        .task_number      = (uint16_t)uxTaskGetTaskNumber(NULL),
        .event_type       = (uint8_t)TRACE_EVT_SWITCH_OUT,
        .reserved         = 0,
        .aux              = 0,
    };
    trace_ringbuf_push(&evt);
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
    trace_ringbuf_push(&evt);
}

#endif /* DEBUG */
