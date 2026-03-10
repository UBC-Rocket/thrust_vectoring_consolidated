#ifndef RADIO_QUEUE_H
#define RADIO_QUEUE_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define RADIO_MESSAGE_MAX_LEN 256
#define RADIO_MESSAGE_QUEUE_LEN 10

typedef struct {
    uint8_t radio_messages[RADIO_MESSAGE_QUEUE_LEN][RADIO_MESSAGE_MAX_LEN];

    volatile uint8_t head;
    volatile uint8_t tail;
} radio_message_queue_t;

static inline void radio_message_queue_init(radio_message_queue_t *q) {
    q->head = 0;
    q->tail = 0;
}

static inline bool radio_message_queue_empty(radio_message_queue_t *q) {
    return q->head == q->tail;
}

static inline bool radio_message_queue_full(radio_message_queue_t *q) {
    return ((q->head + 1) % RADIO_MESSAGE_QUEUE_LEN) == q->tail;
}

static inline bool radio_message_enqueue(uint16_t len, uint8_t *data, radio_message_queue_t *q) {
    if (radio_message_queue_full(q)) return false;

    if (len > RADIO_MESSAGE_MAX_LEN) return false;

    memcpy((void*)q->radio_messages[q->head], (void*)data, (size_t)len);

    memset((void*)(q->radio_messages[q->head] + len), 0, (size_t)(RADIO_MESSAGE_MAX_LEN - len));
    q->head = (q->head + 1) % RADIO_MESSAGE_QUEUE_LEN;

    return true;
}

static inline bool radio_message_dequeue(radio_message_queue_t *q, uint8_t *data) {
    if (radio_message_queue_empty(q)) return false;

    memcpy((void*)data, (void*)q->radio_messages[q->tail], RADIO_MESSAGE_MAX_LEN);

    q->tail = (q->tail + 1) % RADIO_MESSAGE_QUEUE_LEN;

    return true;
}

static inline bool radio_message_queue_tail_pointer(radio_message_queue_t *q, uint8_t **tail_pointer) {
    if (radio_message_queue_empty(q)) return false;

    *tail_pointer = q->radio_messages[q->tail];

    return true;
}

static inline void radio_message_queue_pop(radio_message_queue_t *q) {
    if (radio_message_queue_empty(q)) return;

    q->tail = (q->tail + 1) % RADIO_MESSAGE_QUEUE_LEN;
}

// LIFO operations (Last In, First Out) - returns most recent message
static inline bool radio_message_dequeue_lifo(radio_message_queue_t *q, uint8_t *data) {
    if (radio_message_queue_empty(q)) return false;

    // Get the most recent message (head - 1)
    uint8_t lifo_index = (q->head == 0) ? (RADIO_MESSAGE_QUEUE_LEN - 1) : (q->head - 1);
    memcpy((void*)data, (void*)q->radio_messages[lifo_index], RADIO_MESSAGE_MAX_LEN);

    // Decrement head to remove this message
    q->head = lifo_index;

    return true;
}

static inline bool radio_message_queue_head_pointer(radio_message_queue_t *q, uint8_t **head_pointer) {
    if (radio_message_queue_empty(q)) return false;

    // Get pointer to most recent message (head - 1)
    uint8_t lifo_index = (q->head == 0) ? (RADIO_MESSAGE_QUEUE_LEN - 1) : (q->head - 1);
    *head_pointer = q->radio_messages[lifo_index];

    return true;
}

static inline void radio_message_queue_pop_lifo(radio_message_queue_t *q) {
    if (radio_message_queue_empty(q)) return;

    // Decrement head (remove most recent)
    q->head = (q->head == 0) ? (RADIO_MESSAGE_QUEUE_LEN - 1) : (q->head - 1);
}

#endif
