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

/* Total shared region size — 4 kB out of SRAM4's 64 kB is plenty for the
 * slot payloads and any future additions. Must match the linker script. */
#define APP_SHARED_REGION_SIZE  4096U

/* Slot offsets inside the shared region. Each slot begins with an
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

#define APP_SLOT_LOG_HANDOFF_OFFSET     0x0400   /* 1024 */
#define APP_SLOT_LOG_HANDOFF_PAYLOAD    64

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
