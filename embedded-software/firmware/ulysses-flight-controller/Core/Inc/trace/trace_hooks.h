#ifndef TRACE_HOOKS_H
#define TRACE_HOOKS_H

#ifdef DEBUG

#include <stdint.h>
#include <stdbool.h>

/* ── Trace event types and struct (formerly in trace_ringbuf.h) ── */

typedef enum {
    TRACE_EVT_SWITCH_IN  = 0,
    TRACE_EVT_SWITCH_OUT = 1,
    TRACE_EVT_CREATE     = 2,
} trace_event_type_t;

typedef struct __attribute__((packed)) {
    uint32_t timestamp_us;
    uint16_t task_number;
    uint8_t  event_type;
    uint8_t  reserved;
    uint32_t aux;
} trace_event_t;

/**
 * @brief Initialise trace hooks — claims a sector buffer from log_writer.
 * Must be called after log_writer_init(), before the scheduler starts
 * generating context switch events.
 */
void trace_hooks_init(void);

/**
 * @brief Return and reset the number of trace events dropped due to
 *        buffer exhaustion.
 */
uint32_t trace_hooks_take_dropped(void);

/* FreeRTOS trace hook callbacks (called from scheduler critical section) */
void trace_hook_task_switched_in(void);
void trace_hook_task_switched_out(void);
void trace_hook_task_create(void *pxNewTCB);

#endif /* DEBUG */
#endif /* TRACE_HOOKS_H */
