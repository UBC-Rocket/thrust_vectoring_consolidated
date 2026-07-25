#ifndef GPS_QUEUE_H
#define GPS_QUEUE_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define GPS_SAMPLE_SIZE 87  // Max NMEA sentence length (82 chars + 5 byte margin)

/**
 * Double-buffer (ping-pong) for GPS samples.
 *
 * GPS data is time-sensitive — only the latest fix matters.
 * The writer (GPS driver, main loop) always writes to the inactive slot,
 * then atomically swaps `active`. The reader (SPI slave, ISR or main loop)
 * always reads from the active slot, which is always a complete sample.
 *
 * - enqueue: never blocks, always overwrites with latest data
 * - tail_pointer / empty: reflect whether unsent data is available
 * - pop: marks current data as consumed (prevents push-mode re-send)
 */
typedef struct {
    uint8_t samples[2][GPS_SAMPLE_SIZE];  /* Double buffer slots */
    volatile uint8_t active;              /* Index of latest complete sample (0 or 1) */
    volatile bool valid;                  /* Unsent data available */
} gps_sample_queue_t;

static inline void gps_sample_queue_init(gps_sample_queue_t *q) {
    q->active = 0;
    q->valid = false;
    memset(q->samples, 0, sizeof(q->samples));
}

static inline bool gps_sample_queue_empty(gps_sample_queue_t *q) {
    return !q->valid;
}

static inline bool gps_sample_queue_full(gps_sample_queue_t *q) {
    (void)q;
    return false;  /* Double buffer never blocks */
}

/**
 * Write a new GPS sample (always succeeds, overwrites previous).
 *
 * Writes to the inactive slot first, then swaps active index.
 * This ensures readers always see a complete, consistent sample.
 */
static inline bool gps_sample_enqueue(uint8_t *data, gps_sample_queue_t *q) {
    uint8_t write_slot = q->active ^ 1;  /* Write to inactive slot */
    memcpy((void*)q->samples[write_slot], (void*)data, GPS_SAMPLE_SIZE);
    q->active = write_slot;               /* Atomic swap: this is now the latest */
    q->valid = true;
    return true;
}

static inline bool gps_sample_dequeue(gps_sample_queue_t *q, uint8_t *data) {
    if (!q->valid) return false;
    memcpy((void*)data, (void*)q->samples[q->active], GPS_SAMPLE_SIZE);
    q->valid = false;
    return true;
}

/**
 * Peek at the latest GPS sample without consuming it.
 */
static inline bool gps_sample_queue_tail_pointer(gps_sample_queue_t *q, uint8_t **tail_pointer) {
    if (!q->valid) return false;
    *tail_pointer = q->samples[q->active];
    return true;
}

/**
 * Mark current sample as consumed (prevents push-mode re-send).
 */
static inline void gps_sample_queue_pop(gps_sample_queue_t *q) {
    q->valid = false;
}

#endif
