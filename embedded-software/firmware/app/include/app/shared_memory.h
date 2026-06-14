/**
 * @file    shared_memory.h
 * @brief   Layout of the cross-core shared-memory region (SRAM4 on H747).
 *
 * Each intercore slot gets a fixed byte offset inside a single shared region.
 * Both cores must agree on this layout at compile time, so the symbols live
 * in a single header. The region itself is provided by the linker script
 * under the section @c .shared.
 *
 * UBC Rocket, 2026
 */
#ifndef APP_SHARED_MEMORY_H
#define APP_SHARED_MEMORY_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Total shared region size — 16 kB out of SRAM4's 64 kB. The bulk of this
 * is the CM7→CM4 log handoff ring (8 kB data + header); the seqlock slots
 * and crash dumps occupy the first ~2 kB. Must match the linker script. */
#define APP_SHARED_REGION_SIZE  16384U

/* Slot offsets inside the shared region. Each seqlock slot begins with an
 * io_intercore_slot_hdr_t (8 bytes) followed by payload. Keep slots 8-byte
 * aligned. */
#define APP_SLOT_STATE_OFFSET           0x0000
#define APP_SLOT_STATE_PAYLOAD          512

#define APP_SLOT_CONTROL_OUTPUT_OFFSET  0x0240   /* 576 */
#define APP_SLOT_CONTROL_OUTPUT_PAYLOAD 256

#define APP_SLOT_FLIGHT_STATE_OFFSET    0x0380   /* 896 */
#define APP_SLOT_FLIGHT_STATE_PAYLOAD   32

#define APP_SLOT_ARMED_OFFSET           0x03C0   /* 960 */
#define APP_SLOT_ARMED_PAYLOAD          16

/* (0x0400 was a stub LOG_HANDOFF slot in phase 1 — the real ring lives at
 * APP_SLOT_LOG_HANDOFF_RING_OFFSET below.) */

/* Shared TIM13 overflow accumulator. CM4 increments on each TIM13 update
 * IRQ; both cores read for io_timestamp_us(). 32-bit upper word + the
 * 16-bit TIM13 CNT gives 48-bit ticks at 0.1 µs/tick → ~325 days. */
#define APP_TIMESTAMP_UPPER_OFFSET      0x0440   /* 1088 */
#define APP_TIMESTAMP_UPPER_PAYLOAD     8        /* uint32_t + pad */

/* Per-core crash-dump slots. One fixed-size record per core; the slot survives
 * soft reset because .shared is NOLOAD in SRAM4 and the boot code does not
 * zero-fill NOLOAD sections. Layout of each slot is owned by crash_dump.c
 * (struct crash_dump_record_t). 256 bytes per core is enough for the fault
 * registers, the exception stack frame, and a 32-word stack snapshot. */
#define APP_SLOT_CRASH_DUMP_CM4_OFFSET  0x0480   /* 1152 */
#define APP_SLOT_CRASH_DUMP_CM4_PAYLOAD 256
#define APP_SLOT_CRASH_DUMP_CM7_OFFSET  0x0580   /* 1408 */
#define APP_SLOT_CRASH_DUMP_CM7_PAYLOAD 256

/* ---------------------------------------------------------------------------
 * Tunables-from-radio: PID gains, reference setpoints, vehicle physical
 * config. mission_manager on CM4 publishes; controls_task on CM7 polls and
 * merges into its live snapshot for the TIM16 ISR. Each slot is 128 B total
 * (8-byte intercore header + up-to-120-byte payload), well over the actual
 * struct sizes (40 / 36 / 20 bytes today) so adding fields later doesn't
 * force a layout migration.
 * --------------------------------------------------------------------------- */
#define APP_SLOT_PID_GAINS_OFFSET       0x0680   /* 1664 — after CM7 crash dump */
#define APP_SLOT_PID_GAINS_PAYLOAD      120

#define APP_SLOT_REFERENCE_OFFSET       0x0700   /* 1792 */
#define APP_SLOT_REFERENCE_PAYLOAD      120

#define APP_SLOT_VEHICLE_CONFIG_OFFSET  0x0780   /* 1920 */
#define APP_SLOT_VEHICLE_CONFIG_PAYLOAD 120
/* Tail: 0x0780 + 128 = 0x0800, exactly meeting the log handoff ring. */

/* ---------------------------------------------------------------------------
 * CM7 → CM4 log handoff ring (lock-free SPSC byte ring).
 *
 * Producer: CM7 messages-runtime SD sink (log_service_append_raw on CM7).
 * Consumer: CM4 sd_log_task drain.
 *
 * Layout in shared memory:
 *
 *   +0x0000  uint32_t head      (producer-write, consumer-read)   --+
 *   +0x0004  uint32_t tail      (consumer-write, producer-read)     |  ring
 *   +0x0008  uint32_t drops     (producer-write, consumer-read)     |  header
 *   +0x000C  uint32_t magic     (set once at init by CM7)         --+  (32 B)
 *   +0x0010  uint32_t reserved[4]
 *   +0x0020  uint8_t  data[APP_SLOT_LOG_HANDOFF_RING_BYTES]
 *
 * head/tail are byte-positions modulo APP_SLOT_LOG_HANDOFF_RING_BYTES.
 * "drops" is a uint32 counter of dropped *bytes* (not records). SRAM4 is
 * uncached on both cores; the producer/consumer pair only need DMB fences
 * around their head/tail updates, no cache maintenance.
 * --------------------------------------------------------------------------- */
#define APP_SLOT_LOG_HANDOFF_RING_OFFSET  0x0800   /* 2048 */
#define APP_SLOT_LOG_HANDOFF_RING_HDR     32U      /* must be 8-byte aligned */
#define APP_SLOT_LOG_HANDOFF_RING_BYTES   8192U    /* 8 kB ring (power of two) */
#define APP_SLOT_LOG_HANDOFF_RING_TOTAL \
    (APP_SLOT_LOG_HANDOFF_RING_HDR + APP_SLOT_LOG_HANDOFF_RING_BYTES)
/* Tail of ring: 0x0800 + 32 + 8192 = 0x2820. Leaves 16384 - 0x2820 = ~7 kB
 * headroom for future slots. */

/* ---------------------------------------------------------------------------
 * SD log cursor — tracks the next SD block address to write across boots.
 *
 * Survives soft reset because .shared is NOLOAD (linker does not zero it).
 * Layout:
 *   +0x00  uint32_t magic       (must equal APP_SD_CURSOR_MAGIC)
 *   +0x04  uint32_t next_block  (next free block on the card)
 *   +0x08  uint32_t boot_count  (incremented each boot — diagnostic)
 *   +0x0C  uint32_t reserved
 *
 * sd_log_task on CM4 reads this at boot; if magic mismatches (cold power-up,
 * SRAM4 not preserved, or first ever boot), it falls back to SD_LOG_START_
 * BLOCK and stamps the magic. Every write to the card advances next_block;
 * a sustained soft-reset loop therefore still produces strictly-increasing
 * block addresses, so previous-boot data is never overwritten in-place.
 * --------------------------------------------------------------------------- */
#define APP_SLOT_SD_CURSOR_OFFSET   0x2840    /* after the 8 kB log ring */
#define APP_SLOT_SD_CURSOR_PAYLOAD  16U
#define APP_SD_CURSOR_MAGIC         0x53444C47U  /* "SDLG" */

/**
 * @brief Pointer to the base of the shared region (defined by the linker).
 */
extern uint8_t __shared_region_start__[];

static inline void *app_shared_slot(size_t offset) {
    return (void *)(__shared_region_start__ + offset);
}

#ifdef __cplusplus
}
#endif

#endif /* APP_SHARED_MEMORY_H */
