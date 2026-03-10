#ifndef GPS_FIX_QUEUE_H
#define GPS_FIX_QUEUE_H

#include "protocol_config.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/**
 * @brief Single-slot GPS fix holder — always stores the latest fix only.
 *
 * New fixes overwrite the previous one. The master always gets the most
 * recent fix when it reads via SPI push.
 */
typedef struct {
    gps_fix_t fix;
    volatile bool has_data;
} gps_fix_queue_t;

static inline void gps_fix_queue_init(gps_fix_queue_t *q) {
    q->has_data = false;
}

static inline bool gps_fix_queue_empty(const gps_fix_queue_t *q) {
    return !q->has_data;
}

static inline bool gps_fix_enqueue(const gps_fix_t *fix, gps_fix_queue_t *q) {
    memcpy((void*)&q->fix, (const void*)fix, sizeof(gps_fix_t));
    q->has_data = true;
    return true;
}

static inline bool gps_fix_queue_tail_pointer(const gps_fix_queue_t *q, const gps_fix_t **tail_pointer) {
    if (!q->has_data) return false;
    *tail_pointer = &q->fix;
    return true;
}

static inline void gps_fix_queue_pop(gps_fix_queue_t *q) {
    q->has_data = false;
}

#endif // GPS_FIX_QUEUE_H
