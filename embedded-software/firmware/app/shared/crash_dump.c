/**
 * @file    crash_dump.c
 * @brief   Per-core crash dump helpers — skeleton.
 *
 * UBC Rocket, 2026
 */

#include "app/crash_dump.h"

#include <string.h>

void crash_dump_init(void) {
    /* TODO: clear this core's section of the shared crash region. */
}

void crash_dump_record_fault(uint32_t fault_id,
                             const uint32_t *sp,
                             const uint32_t *regs,
                             uint32_t regs_len) {
    (void)fault_id;
    (void)sp;
    (void)regs;
    (void)regs_len;
    /* TODO: persist to the shared crash region, then reset or spin. */
}

bool crash_dump_flush_pending(void) {
    /* TODO: read shared region, emit records via log_service, clear region. */
    return false;
}
