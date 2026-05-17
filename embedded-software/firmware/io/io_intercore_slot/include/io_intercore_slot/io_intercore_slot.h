/**
 * @file io_intercore_slot.h
 * @brief Lock-free shared-memory primitives for CM4 ↔ CM7 data exchange.
 *
 * The H747 has an uncached SRAM4 accessible from both cores in the D3 domain;
 * allocating @ref io_intercore_slot_t backing storage there lets both cores
 * read/write without cache maintenance. Notifications (i.e. "wake the other
 * core because something changed") are delivered separately through
 * @ref io_intercore_signal.
 *
 * The slot uses the classic seqlock pattern: a sequence counter is
 * incremented to an odd value before a write, then to an even value after.
 * Readers retry as long as the counter is odd or changes mid-copy.
 *
 * Assumptions:
 *   - A slot has at most one concurrent writer (multiple readers are fine).
 *   - Backing storage lives in memory that is coherent without cache
 *     maintenance (SRAM4 on the H747; the writer must still execute __DMB
 *     around its publish, which this library handles).
 *
 * UBC Rocket, 2026
 */

#ifndef IO_INTERCORE_SLOT_H
#define IO_INTERCORE_SLOT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Header that precedes each slot's payload in shared memory.
 */
typedef struct {
    volatile uint32_t seq;      ///< even = stable, odd = write-in-progress
    uint32_t          size;     ///< payload size in bytes
} io_intercore_slot_hdr_t;

/**
 * @brief Handle held by producers/consumers. Stays in each core's own memory.
 */
typedef struct {
    io_intercore_slot_hdr_t *hdr;  ///< points into shared memory
    void                 *data; ///< points into shared memory (hdr + 1)
    size_t                size;
} io_intercore_slot_t;

/**
 * @brief Initialise a slot descriptor against a backing region.
 *
 * @param s        local descriptor (not shared)
 * @param backing  pointer into shared memory; must be 8-byte aligned and
 *                 sized to at least sizeof(io_intercore_slot_hdr_t) + @p size
 * @param size     payload size in bytes
 *
 * The caller is responsible for making sure every participating core agrees
 * on @p backing and @p size. Only one core should actually zero the header;
 * follow-up cores simply attach.
 */
void io_intercore_slot_init(io_intercore_slot_t *s, void *backing, size_t size);

/**
 * @brief Publish @p size bytes from @p src into the slot (writer side).
 *
 * The write is transactional — readers either see the previous contents or
 * the new contents, never a mixture.
 */
void io_intercore_slot_publish(io_intercore_slot_t *s, const void *src);

/**
 * @brief Try to read a consistent snapshot (reader side).
 *
 * @param s           slot descriptor
 * @param dst         destination buffer (size bytes)
 * @param out_seq     optional — filled with the sequence number of the read
 * @return true on success, false only if the writer is actively mid-update
 *         and @ref IO_INTERCORE_SLOT_MAX_RETRIES is exceeded (extremely rare).
 */
bool io_intercore_slot_read(io_intercore_slot_t *s, void *dst, uint32_t *out_seq);

#ifndef IO_INTERCORE_SLOT_MAX_RETRIES
#define IO_INTERCORE_SLOT_MAX_RETRIES 8U
#endif

#ifdef __cplusplus
}
#endif

#endif /* IO_INTERCORE_SLOT_H */
