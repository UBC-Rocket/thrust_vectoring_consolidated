/**
 * @file    log_service.c
 * @brief   Staging-buffer service behind the messages-runtime SD sink.
 *
 * Same TU lives on both cores (app_shared library). At runtime we branch
 * on CORE_CM4 / CORE_CM7 so each side hits the path it owns:
 *
 *   - CM4: appends from the messages runtime go into a CM4-local byte
 *     buffer; sd_log_task drains it in-task.
 *   - CM7: appends are pushed into the SRAM4 SPSC byte ring shared with
 *     CM4. The ring header (head/tail/drops/magic) and the data buffer
 *     both live at APP_SLOT_LOG_HANDOFF_RING_OFFSET. CM7 raises
 *     IO_IC_LOG_BUFFER_CM7_READY (HSEM) whenever the ring crosses a fill
 *     threshold so CM4's drain can wake out of vTaskDelay.
 *
 * Lock-free SPSC pattern (ring header in SRAM4):
 *   head — producer-write, consumer-read (CM7 owns)
 *   tail — consumer-write, producer-read (CM4 owns)
 *   __DMB() barriers around head/tail updates; SRAM4 is uncached on both
 *   cores so no cache maintenance is needed.
 *
 * UBC Rocket, 2026
 */

#include "app/log_service.h"
#include "app/shared_memory.h"
#include "io_sys/io_test_hooks.h"
#include "io_sys/io_intercore.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

/* Portable DMB. Works on both CM4 and CM7 when compiled with the CMSIS
 * device header; falls back to a sequentially-consistent atomic fence
 * otherwise (host builds / tests). */
#if defined(__GNUC__) && (defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__))
#  define LS_DMB() __asm__ volatile ("dmb" ::: "memory")
#elif defined(__GNUC__) || defined(__clang__)
#  define LS_DMB() __atomic_thread_fence(__ATOMIC_SEQ_CST)
#else
#  define LS_DMB() do { } while (0)
#endif

/* -------------------------------------------------------------------------- */
/* CM4-local staging buffer                                                   */
/* -------------------------------------------------------------------------- */

#if defined(CORE_CM4)
/* 4 kB CM4-local byte ring. Sized to a few SD blocks; the SD writer drains
 * it block-at-a-time. Sits in the CM4-only DTCM/SRAM (whatever .bss maps
 * to), so producer/consumer are both in-task on CM4 — no DMB needed. */
#  ifndef LOG_SERVICE_CM4_LOCAL_BYTES
#    define LOG_SERVICE_CM4_LOCAL_BYTES 4096U
#  endif
_Static_assert((LOG_SERVICE_CM4_LOCAL_BYTES & (LOG_SERVICE_CM4_LOCAL_BYTES - 1U)) == 0U,
               "LOG_SERVICE_CM4_LOCAL_BYTES must be a power of two");

static uint8_t  s_cm4_local[LOG_SERVICE_CM4_LOCAL_BYTES];
static uint32_t s_cm4_local_head;   /* producer index, masks with capacity-1 */
static uint32_t s_cm4_local_tail;   /* consumer index, masks with capacity-1 */
#endif

/* -------------------------------------------------------------------------- */
/* CM7→CM4 SRAM4 SPSC ring                                                    */
/* -------------------------------------------------------------------------- */

/** Magic stamped in the ring header so CM4 can detect "CM7 has not booted
 *  yet" and treat the ring as empty (rather than racing on uninitialised
 *  head/tail). */
#define LS_RING_MAGIC  0x4C4F4752U   /* "LOGR" */

/** Wake-threshold fraction: signal CM4 whenever the ring goes empty→non-empty
 *  AND additionally every time fill crosses a 25 % boundary upward. */
#define LS_WAKE_QUARTER  (APP_SLOT_LOG_HANDOFF_RING_BYTES / 4U)

typedef struct {
    volatile uint32_t head;     /* producer-write, consumer-read */
    volatile uint32_t tail;     /* consumer-write, producer-read */
    volatile uint32_t drops;    /* producer-write (bytes dropped on full)   */
    volatile uint32_t magic;    /* LS_RING_MAGIC once CM7 has init'd        */
    volatile uint32_t pushed;   /* producer-write (bytes successfully push) */
    uint32_t          _pad[3];  /* round header to 32 B */
} ls_ring_hdr_t;

_Static_assert(sizeof(ls_ring_hdr_t) == APP_SLOT_LOG_HANDOFF_RING_HDR,
               "ls_ring_hdr_t must equal APP_SLOT_LOG_HANDOFF_RING_HDR");
_Static_assert((APP_SLOT_LOG_HANDOFF_RING_BYTES &
                (APP_SLOT_LOG_HANDOFF_RING_BYTES - 1U)) == 0U,
               "APP_SLOT_LOG_HANDOFF_RING_BYTES must be a power of two");

#define LS_RING_MASK  (APP_SLOT_LOG_HANDOFF_RING_BYTES - 1U)

static ls_ring_hdr_t *s_ring_hdr;     /* points into SRAM4 */
static uint8_t       *s_ring_data;    /* points into SRAM4, capacity above */

#if defined(CORE_CM4)
/* CM4 wake callback: sd_log_task installs one that does the
 * xTaskNotifyGiveFromISR. Keeping FreeRTOS out of this TU lets app_shared
 * stay free of CMSIS/HAL link deps (mirrors how app_shared/crash_dump.c
 * is FreeRTOS-free). */
typedef void (*ls_wake_cb_t)(void);
static volatile ls_wake_cb_t s_cm4_wake_cb;

static void cm7_ready_isr_handler(void)
{
    ls_wake_cb_t cb = s_cm4_wake_cb;
    if (cb) cb();
}
#endif /* CORE_CM4 */

/* -------------------------------------------------------------------------- */
/* Common bookkeeping                                                         */
/* -------------------------------------------------------------------------- */

static bool s_ready;

/* "raw bytes appended" / "raw bytes dropped" are kept on both cores — they
 * cover the boundary between the messages runtime and *this* service. On
 * CM7 that's the ring producer; on CM4 that's the local byte buffer. */
static volatile uint32_t s_raw_bytes_appended;
static volatile uint32_t s_raw_bytes_dropped;

IO_TEST_HOOK_RW(s_ready, bool, log_service_ready)

/* -------------------------------------------------------------------------- */
/* Lifecycle                                                                  */
/* -------------------------------------------------------------------------- */

void log_service_init(void)
{
    s_ready = false;
    s_raw_bytes_appended = 0;
    s_raw_bytes_dropped = 0;

    /* Bind the ring header + data pointers to their SRAM4 backing. Both
     * cores agree on the offsets via shared_memory.h. */
    uint8_t *backing = (uint8_t *)app_shared_slot(APP_SLOT_LOG_HANDOFF_RING_OFFSET);
    s_ring_hdr  = (ls_ring_hdr_t *)backing;
    s_ring_data = backing + sizeof(ls_ring_hdr_t);

#if defined(CORE_CM7)
    /* CM7 owns the one-shot init: zero the header and stamp the magic so
     * CM4 knows the ring is live. We always reset on boot — the old head
     * / tail are not meaningful across a CM7 restart. */
    s_ring_hdr->head   = 0;
    s_ring_hdr->tail   = 0;
    s_ring_hdr->drops  = 0;
    s_ring_hdr->pushed = 0;
    LS_DMB();
    s_ring_hdr->magic  = LS_RING_MAGIC;
    LS_DMB();
#elif defined(CORE_CM4)
    s_cm4_local_head = 0;
    s_cm4_local_tail = 0;
    /* Subscribe to CM7's ring-ready signal. The handler wakes sd_log_task.
     * io_intercore_init must have already run (io_init_cm4 does it before
     * app_init_cm4). */
    io_intercore_register_handler(IO_IC_LOG_BUFFER_CM7_READY,
                                  cm7_ready_isr_handler);
#endif
}

bool log_service_ready(void) { return s_ready; }
void log_service_mark_ready(void) { s_ready = true; }

/* -------------------------------------------------------------------------- */
/* CM7→CM4 ring helpers                                                       */
/* -------------------------------------------------------------------------- */

#if defined(CORE_CM7)
/* Producer side. Atomic w.r.t. CM4's consumer: only CM7 writes head, only
 * CM4 writes tail. We read tail with a DMB to make sure we observe at
 * least CM4's most recent commit. */
static bool ring_push(const uint8_t *src, uint32_t len)
{
    if (!s_ring_hdr) return false;
    if (s_ring_hdr->magic != LS_RING_MAGIC) return false;   /* paranoia */

    uint32_t head = s_ring_hdr->head;
    LS_DMB();
    uint32_t tail = s_ring_hdr->tail;

    /* Free space = capacity - (head - tail). We reserve nothing — full
     * means we can fit (capacity - 1) bytes at most? No, actually with
     * separate head/tail counts as unbounded 32-bit indices we can use
     * the full capacity (full iff head - tail == capacity). */
    uint32_t used = head - tail;
    uint32_t freebytes = APP_SLOT_LOG_HANDOFF_RING_BYTES - used;
    if (len > freebytes) {
        s_ring_hdr->drops += len;
        return false;
    }

    /* Two-part copy across the wraparound boundary. */
    uint32_t mask = LS_RING_MASK;
    uint32_t off  = head & mask;
    uint32_t first = APP_SLOT_LOG_HANDOFF_RING_BYTES - off;
    if (first > len) first = len;
    memcpy(&s_ring_data[off], src, first);
    if (len > first) {
        memcpy(&s_ring_data[0], src + first, len - first);
    }

    LS_DMB();
    s_ring_hdr->head = head + len;
    s_ring_hdr->pushed += len;

    /* Wake policy: signal whenever the ring crosses a 25 % boundary
     * upward, including the empty→non-empty case. Cheap heuristic — the
     * fill before the push divided into quarters is different from the
     * fill after. */
    uint32_t fill_before = used;
    uint32_t fill_after  = used + len;
    if ((fill_before / LS_WAKE_QUARTER) < (fill_after / LS_WAKE_QUARTER) ||
        fill_before == 0U) {
        LS_DMB();
        io_intercore_signal(IO_IC_LOG_BUFFER_CM7_READY);
    }
    return true;
}
#endif /* CORE_CM7 */

#if defined(CORE_CM4)
/* Consumer side. */
static size_t ring_pop(uint8_t *dst, size_t max)
{
    if (!s_ring_hdr) return 0;
    if (s_ring_hdr->magic != LS_RING_MAGIC) return 0;

    uint32_t tail = s_ring_hdr->tail;
    LS_DMB();
    uint32_t head = s_ring_hdr->head;
    uint32_t avail = head - tail;
    if (avail == 0) return 0;
    if (avail > (uint32_t)max) avail = (uint32_t)max;

    uint32_t mask = LS_RING_MASK;
    uint32_t off  = tail & mask;
    uint32_t first = APP_SLOT_LOG_HANDOFF_RING_BYTES - off;
    if (first > avail) first = avail;
    memcpy(dst, &s_ring_data[off], first);
    if (avail > first) {
        memcpy(dst + first, &s_ring_data[0], avail - first);
    }

    LS_DMB();
    s_ring_hdr->tail = tail + avail;
    return (size_t)avail;
}
#endif /* CORE_CM4 */

/* -------------------------------------------------------------------------- */
/* Append path                                                                */
/* -------------------------------------------------------------------------- */

bool log_service_append_raw(const uint8_t *bytes, uint32_t len)
{
    if (len == 0U) return true;
    if (bytes == NULL) return false;

#if defined(CORE_CM7)
    if (ring_push(bytes, len)) {
        s_raw_bytes_appended += len;
        return true;
    }
    s_raw_bytes_dropped += len;
    return false;

#elif defined(CORE_CM4)
    /* CM4-local byte ring (single producer = whichever task publishes via
     * the messages runtime, single consumer = sd_log_task). Phase 1 uses
     * a simple non-atomic write — messages runtime is task-context only,
     * and sd_log_task is also a task. If we later admit ISR producers we
     * can switch to atomics, but the ring itself is correct. */
    uint32_t head = s_cm4_local_head;
    uint32_t tail = s_cm4_local_tail;
    uint32_t used = head - tail;
    uint32_t freebytes = LOG_SERVICE_CM4_LOCAL_BYTES - used;
    if (len > freebytes) {
        s_raw_bytes_dropped += len;
        return false;
    }
    uint32_t mask  = LOG_SERVICE_CM4_LOCAL_BYTES - 1U;
    uint32_t off   = head & mask;
    uint32_t first = LOG_SERVICE_CM4_LOCAL_BYTES - off;
    if (first > len) first = len;
    memcpy(&s_cm4_local[off], bytes, first);
    if (len > first) {
        memcpy(&s_cm4_local[0], bytes + first, len - first);
    }
    s_cm4_local_head = head + len;
    s_raw_bytes_appended += len;
    return true;

#else
    /* Host build (no CORE_CM4 / CORE_CM7 defined) — accept and count for
     * unit tests that exercise the messages runtime. */
    (void)bytes;
    s_raw_bytes_appended += len;
    return true;
#endif
}

uint32_t log_service_raw_bytes_appended(void) { return s_raw_bytes_appended; }
uint32_t log_service_raw_bytes_dropped(void)  { return s_raw_bytes_dropped; }

/* -------------------------------------------------------------------------- */
/* CM4 drain APIs                                                             */
/* -------------------------------------------------------------------------- */

size_t log_service_drain_local(uint8_t *out, size_t max)
{
#if defined(CORE_CM4)
    if (out == NULL || max == 0U) return 0;

    uint32_t head = s_cm4_local_head;
    uint32_t tail = s_cm4_local_tail;
    uint32_t avail = head - tail;
    if (avail == 0U) return 0;
    if (avail > (uint32_t)max) avail = (uint32_t)max;

    uint32_t mask  = LOG_SERVICE_CM4_LOCAL_BYTES - 1U;
    uint32_t off   = tail & mask;
    uint32_t first = LOG_SERVICE_CM4_LOCAL_BYTES - off;
    if (first > avail) first = avail;
    memcpy(out, &s_cm4_local[off], first);
    if (avail > first) {
        memcpy(out + first, &s_cm4_local[0], avail - first);
    }
    s_cm4_local_tail = tail + avail;
    return (size_t)avail;
#else
    (void)out; (void)max;
    return 0;
#endif
}

size_t log_service_drain_cm7_ring(uint8_t *out, size_t max)
{
#if defined(CORE_CM4)
    if (out == NULL || max == 0U) return 0;
    return ring_pop(out, max);
#else
    (void)out; (void)max;
    return 0;
#endif
}

void log_service_set_wake_callback(void (*cb)(void))
{
#if defined(CORE_CM4)
    s_cm4_wake_cb = (ls_wake_cb_t)cb;
#else
    (void)cb;
#endif
}

/* -------------------------------------------------------------------------- */
/* CM7→CM4 ring statistics                                                    */
/* -------------------------------------------------------------------------- */

uint32_t log_service_cm7_ring_bytes_pushed(void)
{
    if (!s_ring_hdr) return 0;
    if (s_ring_hdr->magic != LS_RING_MAGIC) return 0;
    return s_ring_hdr->pushed;
}

uint32_t log_service_cm7_ring_bytes_dropped(void)
{
    if (!s_ring_hdr) return 0;
    if (s_ring_hdr->magic != LS_RING_MAGIC) return 0;
    return s_ring_hdr->drops;
}

uint32_t log_service_cm7_ring_fill(void)
{
    if (!s_ring_hdr) return 0;
    if (s_ring_hdr->magic != LS_RING_MAGIC) return 0;
    uint32_t head = s_ring_hdr->head;
    LS_DMB();
    uint32_t tail = s_ring_hdr->tail;
    return head - tail;
}
