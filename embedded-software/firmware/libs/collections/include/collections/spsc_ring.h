/**
 * @file spsc_ring.h
 * @brief Generic Single-Producer Single-Consumer (SPSC) lock-free ring buffer.
 *
 * Usage:
 *   #include <collections/spsc_ring.h>
 *
 *   SPSC_RING_DECLARE(my_queue, my_element_t, 16)
 *
 * This generates:
 *   typedef struct { ... } my_queue_t;
 *   static inline bool my_queue_empty(my_queue_t *q);
 *   static inline bool my_queue_full(my_queue_t *q);
 *   static inline bool my_queue_push(my_queue_t *q, const my_element_t *item);
 *   static inline bool my_queue_pop(my_queue_t *q, my_element_t *item);
 *
 * Thread safety:
 *   - One producer (ISR/DMA callback) calls _push().
 *   - One consumer (task) calls _pop().
 *   - Memory barriers ensure proper ordering between producer and consumer.
 *
 * On ARM targets, define SPSC_RING_USE_ARM_BARRIER before including this
 * header (or via compile definition) to use the hardware __DMB() instruction.
 *
 * @ UBC Rocket, 2026
 */

#ifndef COLLECTIONS_SPSC_RING_H
#define COLLECTIONS_SPSC_RING_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* ------------------------------------------------------------------------ */
/* Portable memory barrier                                                   */
/* ------------------------------------------------------------------------ */

#ifndef SPSC_RING_MEMORY_BARRIER
  #if defined(SPSC_RING_USE_ARM_BARRIER)
    /* ARM Cortex-M: hardware Data Memory Barrier.
       __DMB() is provided by CMSIS (cmsis_gcc.h / cmsis_compiler.h),
       included transitively by any consumer that uses HAL. */
    #define SPSC_RING_MEMORY_BARRIER() __DMB()
  #elif defined(__GNUC__) || defined(__clang__)
    #define SPSC_RING_MEMORY_BARRIER() __asm volatile("" ::: "memory")
  #else
    #define SPSC_RING_MEMORY_BARRIER() ((void)0)
  #endif
#endif

/* ------------------------------------------------------------------------ */
/* SPSC_RING_DECLARE macro                                                   */
/* ------------------------------------------------------------------------ */

/**
 * @brief Declare a typed SPSC ring buffer with all inline operations.
 *
 * @param prefix  Name prefix for the generated type and functions.
 * @param type    Element type stored in the ring buffer.
 * @param cap     Capacity (max elements). Actual storage is cap slots;
 *                one slot is reserved as sentinel, so max occupancy is cap-1.
 */
#define SPSC_RING_DECLARE(prefix, type, cap)                                   \
                                                                               \
typedef struct {                                                               \
    type items[(cap)];                                                         \
    volatile uint8_t head; /**< Written by producer only */                    \
    volatile uint8_t tail; /**< Written by consumer only */                    \
} prefix##_t;                                                                  \
                                                                               \
/** @brief Check if the ring buffer is empty. */                               \
static inline bool prefix##_empty(prefix##_t *q) {                            \
    return q->head == q->tail;                                                 \
}                                                                              \
                                                                               \
/** @brief Check if the ring buffer is full. */                                \
static inline bool prefix##_full(prefix##_t *q) {                             \
    return ((q->head + 1) % (cap)) == q->tail;                                \
}                                                                              \
                                                                               \
/**                                                                            \
 * @brief Push an item into the ring buffer (producer side).                   \
 * @param q    Pointer to the ring buffer.                                     \
 * @param item Pointer to the item to enqueue.                                 \
 * @return true if successful, false if the queue is full.                     \
 * @note Memory barrier ensures item data is visible before head update.       \
 */                                                                            \
static inline bool prefix##_push(prefix##_t *q, const type *item) {           \
    if (prefix##_full(q)) return false;                                        \
    q->items[q->head] = *item;                                                \
    SPSC_RING_MEMORY_BARRIER();                                                \
    q->head = (q->head + 1) % (cap);                                          \
    return true;                                                               \
}                                                                              \
                                                                               \
/**                                                                            \
 * @brief Pop an item from the ring buffer (consumer side).                    \
 * @param q    Pointer to the ring buffer.                                     \
 * @param item Pointer to receive the dequeued item.                           \
 * @return true if successful, false if the queue is empty.                    \
 * @note Memory barriers ensure proper ordering with the producer.             \
 */                                                                            \
static inline bool prefix##_pop(prefix##_t *q, type *item) {                  \
    if (prefix##_empty(q)) return false;                                       \
    SPSC_RING_MEMORY_BARRIER();                                                \
    *item = q->items[q->tail];                                                 \
    SPSC_RING_MEMORY_BARRIER();                                                \
    q->tail = (q->tail + 1) % (cap);                                          \
    return true;                                                               \
}                                                                              \
                                                                               \
/**                                                                            \
 * @brief Peek at the oldest item without removing it (consumer side).         \
 * @param q    Pointer to the ring buffer.                                     \
 * @param item Pointer to receive a copy of the oldest item.                   \
 * @return true if an item is available, false if the queue is empty.          \
 */                                                                            \
static inline bool prefix##_peek(prefix##_t *q, type *item) {                 \
    if (prefix##_empty(q)) return false;                                       \
    SPSC_RING_MEMORY_BARRIER();                                                \
    *item = q->items[q->tail];                                                 \
    return true;                                                               \
}                                                                              \
                                                                               \
/**                                                                            \
 * @brief Return the number of items currently in the ring buffer.             \
 */                                                                            \
static inline uint8_t prefix##_count(prefix##_t *q) {                         \
    return (q->head - q->tail + (cap)) % (cap);                               \
}

#endif /* COLLECTIONS_SPSC_RING_H */
