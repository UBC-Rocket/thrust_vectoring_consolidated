/**
 * @file    crash_dump.h
 * @brief   Per-core crash dump helpers.
 *
 * Each core writes into its own section of a shared crash-dump region. After
 * reset, CM4 reads both sections and flushes them to the SD card in its
 * startup sequence.
 *
 * UBC Rocket, 2026
 */
#ifndef APP_CRASH_DUMP_H
#define APP_CRASH_DUMP_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void crash_dump_init(void);

/**
 * @brief Record a fault (HardFault, BusFault, etc.) on the current core.
 *        Safe to call from a fault handler.
 */
void crash_dump_record_fault(uint32_t fault_id,
                             const uint32_t *sp,
                             const uint32_t *regs,
                             uint32_t regs_len);

/**
 * @brief Try to flush pending crash dumps from both cores to the log service.
 *        Intended to be called early from CM4's sd_log_task on boot.
 * @return true if at least one dump was flushed.
 */
bool crash_dump_flush_pending(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_CRASH_DUMP_H */
