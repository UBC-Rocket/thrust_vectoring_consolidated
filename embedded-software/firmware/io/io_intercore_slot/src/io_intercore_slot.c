/**
 * @file io_intercore_slot.c
 * @brief Seqlock slot for CM4 ↔ CM7 data exchange.
 *
 * UBC Rocket, 2026
 */

#include "io_intercore_slot/io_intercore_slot.h"

#include <string.h>

/* Portable DMB — works on both CM4 and CM7 when compiled with the CMSIS
 * device header. If that's not available (e.g. host build) fall back to the
 * GCC atomic thread fence which is almost as strong. */
#if defined(__CMSIS_GCC_H) || defined(__GNUC__)
#  if defined(__CMSIS_VERSION)
#    include "cmsis_compiler.h"
#    define IC_DMB() __DMB()
#  else
#    define IC_DMB() __atomic_thread_fence(__ATOMIC_SEQ_CST)
#  endif
#else
#  define IC_DMB() do { } while (0)
#endif

void io_intercore_slot_init(io_intercore_slot_t *s, void *backing, size_t size) {
    s->hdr  = (io_intercore_slot_hdr_t *)backing;
    s->data = (uint8_t *)backing + sizeof(io_intercore_slot_hdr_t);
    s->size = size;
    /* Do NOT zero the header here — multiple cores call init(). A separate
     * one-shot init (from CM7 boot, typically) should pre-clear shared
     * memory before CM4 is released. */
}

void io_intercore_slot_publish(io_intercore_slot_t *s, const void *src) {
    uint32_t seq = s->hdr->seq;
    /* Force odd to indicate write-in-progress. */
    seq++;
    s->hdr->seq = seq;
    IC_DMB();

    memcpy(s->data, src, s->size);

    IC_DMB();
    s->hdr->seq = seq + 1U;   /* back to even */
}

bool io_intercore_slot_read(io_intercore_slot_t *s, void *dst, uint32_t *out_seq) {
    for (uint32_t i = 0; i < IO_INTERCORE_SLOT_MAX_RETRIES; ++i) {
        uint32_t seq1 = s->hdr->seq;
        if (seq1 & 1U) {
            /* Writer in progress — spin. */
            continue;
        }
        IC_DMB();
        memcpy(dst, s->data, s->size);
        IC_DMB();
        uint32_t seq2 = s->hdr->seq;
        if (seq1 == seq2) {
            if (out_seq) *out_seq = seq2;
            return true;
        }
    }
    return false;
}
