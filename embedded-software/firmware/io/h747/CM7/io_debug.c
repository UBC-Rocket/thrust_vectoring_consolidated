/**
 * @file    io_debug.c
 * @brief   H745 CM7 IO impl: debug-console transport — forwards to CM4.
 *
 * CM7 has no debug UART of its own: the VCP (LPUART1) is owned by CM4. So CM7's
 * io_debug_write() pushes the formatted text into a lock-free SPSC byte ring in
 * uncached SRAM4 (APP_SLOT_DEBUG_CONSOLE_RING_*). CM4's io_debug_pump_remote()
 * drains that ring and emits the bytes to LPUART1, so io_debug_printf() works
 * the same from either core — the host serial terminal sees both cores' text.
 *
 * This is the producer side. The ring is single-producer (CM7) / single-
 * consumer (CM4); only DMB fences are needed because SRAM4 is uncached.
 *
 * UBC Rocket, 2026
 */
#include "io_common.h"
#include "io_sys/io_debug.h"
#include "app/shared_memory.h"

#include "stm32h7xx.h"   /* __DMB */

#define DBG_RING_BYTES  APP_SLOT_DEBUG_CONSOLE_RING_BYTES   /* power of two */
#define DBG_RING_MASK   (DBG_RING_BYTES - 1U)

/* Mirrors the 32-byte header documented in shared_memory.h. */
typedef struct {
    volatile uint32_t head;   /* producer-write, consumer-read */
    volatile uint32_t tail;   /* consumer-write, producer-read */
    volatile uint32_t drops;  /* bytes dropped when full       */
    volatile uint32_t magic;  /* APP_DEBUG_CONSOLE_RING_MAGIC  */
    uint32_t          _pad[4];
} dbg_ring_hdr_t;

static dbg_ring_hdr_t   *s_hdr;
static volatile uint8_t *s_data;
static bool              s_ready;

io_status_t io_debug_init(void) {
    if (s_ready) {
        return IO_OK;                  /* idempotent */
    }

    s_hdr  = (dbg_ring_hdr_t *)app_shared_slot(APP_SLOT_DEBUG_CONSOLE_RING_OFFSET);
    s_data = (volatile uint8_t *)app_shared_slot(
                 APP_SLOT_DEBUG_CONSOLE_RING_OFFSET + APP_SLOT_DEBUG_CONSOLE_RING_HDR);

    /* CM7 owns the one-shot init of the ring header. Zero head/tail/drops,
     * then publish the magic last so CM4's pump only starts draining once the
     * indices are coherent. */
    s_hdr->head  = 0U;
    s_hdr->tail  = 0U;
    s_hdr->drops = 0U;
    __DMB();
    s_hdr->magic = APP_DEBUG_CONSOLE_RING_MAGIC;

    s_ready = true;
    return IO_OK;
}

io_status_t io_debug_write(const uint8_t *data, size_t len) {
    if (data == NULL || len == 0U) {
        return IO_ERR_PARAM;
    }
    if (!s_ready) {
        return IO_ERR_NOT_READY;
    }

    uint32_t head = s_hdr->head;
    uint32_t tail = s_hdr->tail;                 /* snapshot consumer index   */
    uint32_t used = head - tail;                 /* wrap-safe (free-running)  */
    uint32_t freeb = DBG_RING_BYTES - 1U - used; /* keep one slot empty       */

    if ((uint32_t)len > freeb) {
        s_hdr->drops += (uint32_t)len - freeb;   /* count the overflow bytes  */
        len = freeb;                             /* write what fits, drop rest*/
    }
    for (size_t i = 0; i < len; ++i) {
        s_data[(head + (uint32_t)i) & DBG_RING_MASK] = data[i];
    }
    __DMB();                                     /* data visible before head  */
    s_hdr->head = head + (uint32_t)len;
    return IO_OK;
}
