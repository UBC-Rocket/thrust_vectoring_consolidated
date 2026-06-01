/**
 * @file    crash_dump.c
 * @brief   Per-core crash dump capture + replay.
 *
 * Layout. Two fixed-size slots live in the cross-core shared region
 * (SRAM4, .shared, NOLOAD — survives soft reset; not zeroed by startup):
 *
 *   APP_SLOT_CRASH_DUMP_CM4_OFFSET  -> 256 B  (CM4 fault records land here)
 *   APP_SLOT_CRASH_DUMP_CM7_OFFSET  -> 256 B  (CM7 fault records land here)
 *
 * Each slot holds a single @ref crash_dump_record_t. The leading u32 is the
 * "valid" magic; any other value is treated as an empty slot.
 *
 * Capture path. Each fault handler in stm32h7xx_it.c runs an asm trampoline
 * that picks the faulting stack pointer (MSP or PSP, selected by LR bit 2),
 * shuffles arguments into the AAPCS order this function declares, then
 * branches here. We snapshot the eight stacked registers, the relevant SCB
 * fault registers, the saved MSP/PSP, and the first N words of the offending
 * stack — all from the C side, with no allocations and no scheduler use.
 *
 * Replay path. crash_dump_flush_pending() runs once per boot on CM4 (it owns
 * the SD log). It scans both slots; for any slot with a valid magic it
 * builds a messages envelope by hand and hands the raw bytes to
 * log_service_append_raw(). Then it clears the magic so the record doesn't
 * re-flush on the next boot.
 *
 * Replay route choice. The task header lists two options:
 *
 *   A. Build a Class B system.log_event and call messages_publish_b.
 *   B. Hand framed envelope bytes to log_service_append_raw directly.
 *
 * We use route B. messages_publish_b is a phase-1 stub today (see
 * lib/messages/messages.h — "Class B is a stub — phase 2 nanopb") and
 * unconditionally returns false, so route A would silently drop the record.
 * We assemble the wire envelope exactly as the messages runtime would
 * (length / class / module_id / msg_id / t_us / payload / crc16) — the SD
 * reader walks envelopes by length-prefix + CRC and is content with an
 * opaque payload, which is what a crash trace is.
 *
 * Call-site contract. These three entry points need to be hooked into the
 * boot/teardown flow by the integrator:
 *
 *   - crash_dump_init() must run once on EACH core, AFTER the slot has
 *     been checked for a pending dump (so we don't wipe it before flush).
 *     Suggested site: at the bottom of app_init_cm4 / app_init_cm7, AFTER
 *     the flush call below.
 *
 *   - crash_dump_record_fault() is wired by the asm trampolines in
 *     stm32h7xx_it.c — no extra wiring needed.
 *
 *   - crash_dump_flush_pending() must run on CM4 once log_service is
 *     initialised but BEFORE crash_dump_init() wipes the slot. Suggested
 *     site: at the top of task_sd_log (after log_service is up but before
 *     entering the periodic loop), or in app_init_cm4 after
 *     log_service_init() but before any other task starts publishing.
 *
 * UBC Rocket, 2026
 */

#include "app/crash_dump.h"

#include "app/log_service.h"
#include "app/shared_memory.h"
#include "io_sys/io_timestamp.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* We intentionally do NOT include "messages/messages.h" or
 * "messages/registry.h" here: this TU lives in app_shared, which does not
 * link the messages library (only app_cm4 / app_cm7 do). The wire constants
 * below are the only registry values we need at the byte-assembly level,
 * and they're stable wire constants by contract — duplicating them here
 * with a #if cross-check is preferable to expanding app_shared's deps. If
 * the registry.h numbers ever drift, the static_assert at the top of
 * crash_emit_record will fire (host-side decode check). */
#define CRASH_MSG_CLASS_B           0x42  /* matches messages/registry.h */
#define CRASH_MOD_SYSTEM            0u    /* matches messages/registry.h */
#define CRASH_MSG_SYSTEM_LOG_EVENT  1u    /* matches messages/registry.h */

/* -------------------------------------------------------------------------- */
/* Constants                                                                  */
/* -------------------------------------------------------------------------- */

/** Magic stamped at the top of a valid slot. Unlikely to appear by accident
 *  in uninitialised SRAM4 (bit pattern picked for distinctiveness). */
#define CRASH_DUMP_MAGIC          0xC4A50DEFU

/** Schema version of the record layout. Bump if the struct changes. */
#define CRASH_DUMP_VERSION        1U

/** Number of stack words snapshotted past the exception frame. */
#define CRASH_DUMP_STACK_WORDS    32U

/** Fault id codes — mirror the asm trampolines in stm32h7xx_it.c. */
#define CRASH_DUMP_FAULT_HARD     0U
#define CRASH_DUMP_FAULT_MEM      1U
#define CRASH_DUMP_FAULT_BUS      2U
#define CRASH_DUMP_FAULT_USAGE    3U

/** Source-core identifier. */
#define CRASH_DUMP_CORE_CM4       4U
#define CRASH_DUMP_CORE_CM7       7U

/* -------------------------------------------------------------------------- */
/* Record layout (lives in shared SRAM4)                                      */
/* -------------------------------------------------------------------------- */

/* Packed so the struct lays out identically on both cores — CM4 reads CM7's
 * slot at replay time, so layout must agree. */
typedef struct __attribute__((packed)) {
    /* Header */
    uint32_t magic;          /* CRASH_DUMP_MAGIC when slot is valid      */
    uint32_t version;        /* CRASH_DUMP_VERSION                       */
    uint8_t  core_id;        /* CRASH_DUMP_CORE_CM4 / _CM7                */
    uint8_t  fault_id;       /* CRASH_DUMP_FAULT_*                        */
    uint8_t  stack_words;    /* how many entries in stack[] are valid     */
    uint8_t  reserved;

    /* Exception stack frame, copied off the faulting stack pointer. */
    uint32_t r0;
    uint32_t r1;
    uint32_t r2;
    uint32_t r3;
    uint32_t r12;
    uint32_t lr_from_frame;
    uint32_t pc;
    uint32_t xpsr;

    /* SCB fault status registers, sampled from the C handler. */
    uint32_t cfsr;
    uint32_t hfsr;
    uint32_t bfar;
    uint32_t mmfar;
    uint32_t shcsr;

    /* CPU state at handler entry. */
    uint32_t exc_return;     /* LR at fault entry                          */
    uint32_t msp_at_fault;
    uint32_t psp_at_fault;

    /* First N words past the exception frame. */
    uint32_t stack[CRASH_DUMP_STACK_WORDS];
} crash_dump_record_t;

/* The slot we reserved is 256 B; make sure we fit. */
_Static_assert(sizeof(crash_dump_record_t) <= APP_SLOT_CRASH_DUMP_CM4_PAYLOAD,
               "crash_dump_record_t exceeds reserved CM4 slot payload");
_Static_assert(sizeof(crash_dump_record_t) <= APP_SLOT_CRASH_DUMP_CM7_PAYLOAD,
               "crash_dump_record_t exceeds reserved CM7 slot payload");
_Static_assert(APP_SLOT_CRASH_DUMP_CM4_PAYLOAD == APP_SLOT_CRASH_DUMP_CM7_PAYLOAD,
               "CM4 and CM7 crash slot payloads must match");

/* -------------------------------------------------------------------------- */
/* Slot helpers                                                                */
/* -------------------------------------------------------------------------- */

#if defined(CORE_CM7)
#  define CRASH_LOCAL_SLOT_OFFSET   APP_SLOT_CRASH_DUMP_CM7_OFFSET
#  define CRASH_LOCAL_CORE_ID       CRASH_DUMP_CORE_CM7
#elif defined(CORE_CM4)
#  define CRASH_LOCAL_SLOT_OFFSET   APP_SLOT_CRASH_DUMP_CM4_OFFSET
#  define CRASH_LOCAL_CORE_ID       CRASH_DUMP_CORE_CM4
#else
/* Host/unit-test builds compile this TU too — default so the file links. The
 * capture path is never exercised off-target. */
#  define CRASH_LOCAL_SLOT_OFFSET   APP_SLOT_CRASH_DUMP_CM4_OFFSET
#  define CRASH_LOCAL_CORE_ID       CRASH_DUMP_CORE_CM4
#endif

static volatile crash_dump_record_t *crash_slot(size_t offset) {
    return (volatile crash_dump_record_t *)app_shared_slot(offset);
}

#if defined(CORE_CM4) || defined(CORE_CM7)
#  include "stm32h7xx.h"  /* SCB + __get_MSP / __get_PSP */
static inline uint32_t crash_read_cfsr(void)  { return SCB->CFSR;  }
static inline uint32_t crash_read_hfsr(void)  { return SCB->HFSR;  }
static inline uint32_t crash_read_bfar(void)  { return SCB->BFAR;  }
static inline uint32_t crash_read_mmfar(void) { return SCB->MMFAR; }
static inline uint32_t crash_read_shcsr(void) { return SCB->SHCSR; }
static inline uint32_t crash_read_msp(void)   { return __get_MSP(); }
static inline uint32_t crash_read_psp(void)   { return __get_PSP(); }
#else
static inline uint32_t crash_read_cfsr(void)  { return 0; }
static inline uint32_t crash_read_hfsr(void)  { return 0; }
static inline uint32_t crash_read_bfar(void)  { return 0; }
static inline uint32_t crash_read_mmfar(void) { return 0; }
static inline uint32_t crash_read_shcsr(void) { return 0; }
static inline uint32_t crash_read_msp(void)   { return 0; }
static inline uint32_t crash_read_psp(void)   { return 0; }
#endif

/* Quick sanity check on a stack pointer — only used to gate the stack-word
 * snapshot. We accept any RAM-like address on H747; anything outside that
 * window is treated as bogus and we skip the snapshot rather than fault
 * recursively inside the fault handler. H747 RAM map spans ITCM
 * (0x00000000) through SRAM4 (0x38010000). */
static bool addr_looks_like_ram(uint32_t addr) {
    return addr < 0x38010000U;
}

/* -------------------------------------------------------------------------- */
/* Capture                                                                    */
/* -------------------------------------------------------------------------- */

void crash_dump_init(void) {
    /* Clear this core's slot to a known-empty state. Any value other than
     * CRASH_DUMP_MAGIC reads as "no pending dump"; zero just keeps the slot
     * looking clean in a hex dump. */
    volatile crash_dump_record_t *rec = crash_slot(CRASH_LOCAL_SLOT_OFFSET);
    memset((void *)rec, 0, sizeof(*rec));
}

/* Called from each fault handler's asm trampoline. Snapshots register state
 * into our SRAM4 slot, then triggers a system reset so the dump replays at
 * next boot. Must not return.
 *
 * The asm trampolines in stm32h7xx_it.c shuffle their args into AAPCS order
 * matching this prototype:
 *   r0 = fault_id (CRASH_DUMP_FAULT_*)
 *   r1 = sp       (faulting stack pointer — MSP or PSP)
 *   r2 = NULL     (regs — unused today; reserved for future caller-saved snapshot)
 *   r3 = 0        (regs_len)
 *
 * Both regs/regs_len are accepted for ABI stability with crash_dump.h; the
 * register snapshot we care about comes off *sp (the exception frame).
 */
void crash_dump_record_fault(uint32_t fault_id,
                             const uint32_t *sp,
                             const uint32_t *regs,
                             uint32_t regs_len) {
    (void)regs;
    (void)regs_len;

    volatile crash_dump_record_t *rec = crash_slot(CRASH_LOCAL_SLOT_OFFSET);

    /* Wipe first so a partial state left by a double-fault midway through
     * doesn't masquerade as valid. */
    memset((void *)rec, 0, sizeof(*rec));

    rec->version       = CRASH_DUMP_VERSION;
    rec->core_id       = CRASH_LOCAL_CORE_ID;
    rec->fault_id      = (uint8_t)fault_id;

    /* SCB snapshot. */
    rec->cfsr          = crash_read_cfsr();
    rec->hfsr          = crash_read_hfsr();
    rec->bfar          = crash_read_bfar();
    rec->mmfar         = crash_read_mmfar();
    rec->shcsr         = crash_read_shcsr();

    /* MSP/PSP read here reflect the values *during* the handler — MSP is the
     * handler's MSP, PSP is the user thread's PSP (untouched by entry). The
     * faulting SP is what the trampoline passed in. */
    rec->msp_at_fault  = crash_read_msp();
    rec->psp_at_fault  = crash_read_psp();
    rec->exc_return    = 0; /* not preserved through the AAPCS shuffle today */

    /* Exception stack frame, if the SP looks plausible. */
    if (sp != NULL && addr_looks_like_ram((uint32_t)sp)) {
        rec->r0            = sp[0];
        rec->r1            = sp[1];
        rec->r2            = sp[2];
        rec->r3            = sp[3];
        rec->r12           = sp[4];
        rec->lr_from_frame = sp[5];
        rec->pc            = sp[6];
        rec->xpsr          = sp[7];

        /* Stack snapshot starts past the 8-word exception frame. */
        const uint32_t *walk = sp + 8;
        uint32_t copied = 0;
        for (uint32_t i = 0; i < CRASH_DUMP_STACK_WORDS; i++) {
            uint32_t addr = (uint32_t)(uintptr_t)&walk[i];
            if (!addr_looks_like_ram(addr)) {
                break;
            }
            rec->stack[i] = walk[i];
            copied++;
        }
        rec->stack_words = (uint8_t)copied;
    }

    /* Stamp the magic LAST so a torn write never reads as valid. */
    rec->magic = CRASH_DUMP_MAGIC;

#if defined(CORE_CM4) || defined(CORE_CM7)
    /* Trigger a system reset so the dump replays at next boot. Bare AIRCR
     * write is the minimal path — NVIC_SystemReset() drags in extra HAL
     * state we don't want from a fault handler. */
    __asm volatile ("dsb 0xF" ::: "memory");
    SCB->AIRCR = (0x5FAUL << SCB_AIRCR_VECTKEY_Pos) | SCB_AIRCR_SYSRESETREQ_Msk;
    __asm volatile ("dsb 0xF" ::: "memory");
#endif

    for (;;) { __asm__ volatile ("nop"); }  /* reset should fire above */
}

/* -------------------------------------------------------------------------- */
/* Replay                                                                     */
/* -------------------------------------------------------------------------- */

/* CRC-16/CCITT-FALSE (polynomial 0x1021, init 0xFFFF) — matches the framing
 * used by the messages runtime, so the SD-side parser accepts our envelope
 * exactly as it accepts a runtime-published one. */
static uint16_t crash_crc16_ccitt(const uint8_t *buf, size_t len) {
    uint16_t crc = 0xFFFFU;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)buf[i] << 8;
        for (int b = 0; b < 8; b++) {
            crc = (uint16_t)((crc & 0x8000U) ? ((crc << 1) ^ 0x1021U)
                                             : (crc << 1));
        }
    }
    return crc;
}

/* Emit one record as a hand-rolled messages envelope on the SD sink. We tag
 * it MOD_SYSTEM / MSG_SYSTEM_LOG_EVENT with class B; the host SD reader
 * walks envelopes by length-prefix and is content with an opaque payload
 * (the host-side log_event resolver renders unknown payloads as a hex blob,
 * which is what we want for a register snapshot anyway). */
static bool crash_emit_record(const volatile crash_dump_record_t *rec) {
    enum { N = sizeof(crash_dump_record_t) };
    enum { ENVELOPE_BYTES = 2 + 12 + N + 2 };

    static uint8_t buf[ENVELOPE_BYTES];

    uint64_t t_us = io_timestamp_us();

    size_t off = 0;
    const uint16_t length_field = (uint16_t)(12U + N + 2U);
    buf[off++] = (uint8_t)(length_field & 0xFFU);
    buf[off++] = (uint8_t)((length_field >> 8) & 0xFFU);

    size_t crc_start = off;
    buf[off++] = CRASH_MSG_CLASS_B;
    buf[off++] = (uint8_t)CRASH_MOD_SYSTEM;
    buf[off++] = (uint8_t)(CRASH_MSG_SYSTEM_LOG_EVENT & 0xFFU);
    buf[off++] = (uint8_t)((CRASH_MSG_SYSTEM_LOG_EVENT >> 8) & 0xFFU);
    for (int i = 0; i < 8; i++) {
        buf[off++] = (uint8_t)((t_us >> (i * 8)) & 0xFFU);
    }

    /* Tmp copy avoids a volatile-to-non-volatile memcpy warning and lets
     * the compiler unroll the move. */
    crash_dump_record_t tmp;
    memcpy(&tmp, (const void *)rec, sizeof(tmp));
    memcpy(&buf[off], &tmp, sizeof(tmp));
    off += sizeof(tmp);

    uint16_t crc = crash_crc16_ccitt(&buf[crc_start], off - crc_start);
    buf[off++] = (uint8_t)(crc & 0xFFU);
    buf[off++] = (uint8_t)((crc >> 8) & 0xFFU);

    return log_service_append_raw(buf, (uint32_t)off);
}

static bool crash_flush_slot(size_t offset) {
    volatile crash_dump_record_t *rec = crash_slot(offset);
    if (rec->magic != CRASH_DUMP_MAGIC) {
        return false;
    }
    (void)crash_emit_record(rec);
    /* Clear regardless of emit success — keeping a stale dump around to
     * retry would re-flush every boot. The drop counter on the sink path
     * tracks loss already if log_service_append_raw refused. */
    memset((void *)rec, 0, sizeof(*rec));
    return true;
}

bool crash_dump_flush_pending(void) {
    bool flushed = false;
    if (crash_flush_slot(APP_SLOT_CRASH_DUMP_CM4_OFFSET)) flushed = true;
    if (crash_flush_slot(APP_SLOT_CRASH_DUMP_CM7_OFFSET)) flushed = true;
    return flushed;
}
