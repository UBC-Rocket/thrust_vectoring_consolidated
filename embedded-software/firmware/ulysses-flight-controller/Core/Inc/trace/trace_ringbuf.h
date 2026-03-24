#ifndef TRACE_RINGBUF_H
#define TRACE_RINGBUF_H

#ifdef DEBUG

#include <stdint.h>
#include <stdbool.h>

#define TRACE_RINGBUF_CAPACITY 256U /* Must be power of 2 */

typedef enum {
    TRACE_EVT_SWITCH_IN  = 0,
    TRACE_EVT_SWITCH_OUT = 1,
    TRACE_EVT_CREATE     = 2,
} trace_event_type_t;

typedef struct __attribute__((packed)) {
    uint32_t timestamp_us;     /**< Microsecond timestamp from timestamp_us() */
    uint16_t task_number;      /**< uxTCBNumber of relevant task */
    uint8_t  event_type;       /**< trace_event_type_t */
    uint8_t  reserved;
    uint32_t aux;              /**< Extra data: priority for CREATE, 0 otherwise */
} trace_event_t;

/**
 * @brief Push an event into the trace ring buffer.
 *
 * Safe to call from ISR / kernel context (lock-free, single-producer).
 *
 * @return true if stored, false if buffer full (event dropped).
 */
bool trace_ringbuf_push(const trace_event_t *evt);

/**
 * @brief Pop an event from the trace ring buffer.
 *
 * Must only be called from a single consumer (the drain task).
 *
 * @param[out] evt  Destination for the dequeued event.
 * @return true if an event was dequeued, false if empty.
 */
bool trace_ringbuf_pop(trace_event_t *evt);

/**
 * @brief Return and reset the number of events dropped due to overflow.
 */
uint32_t trace_ringbuf_take_dropped(void);

#endif /* DEBUG */
#endif /* TRACE_RINGBUF_H */
