#ifdef DEBUG

#include "trace/trace_ringbuf.h"
#include "stm32h5xx.h"
#include <string.h>

#define TRACE_RINGBUF_MASK (TRACE_RINGBUF_CAPACITY - 1U)

static trace_event_t s_entries[TRACE_RINGBUF_CAPACITY];
static volatile uint32_t s_head; /* Written by producer */
static volatile uint32_t s_tail; /* Written by consumer */
static volatile uint32_t s_dropped;

bool trace_ringbuf_push(const trace_event_t *evt)
{
    uint32_t head = s_head;
    uint32_t next = (head + 1U) & TRACE_RINGBUF_MASK;

    if (next == s_tail) {
        s_dropped++;
        return false;
    }

    s_entries[head] = *evt;
    __DMB(); /* Ensure entry is written before head is updated */
    s_head = next;
    return true;
}

bool trace_ringbuf_pop(trace_event_t *evt)
{
    uint32_t tail = s_tail;

    if (tail == s_head) {
        return false;
    }

    __DMB(); /* Ensure we read head before reading the entry */
    *evt = s_entries[tail];
    __DMB();
    s_tail = (tail + 1U) & TRACE_RINGBUF_MASK;
    return true;
}

uint32_t trace_ringbuf_take_dropped(void)
{
    uint32_t d = s_dropped;
    s_dropped = 0;
    return d;
}

#endif /* DEBUG */
