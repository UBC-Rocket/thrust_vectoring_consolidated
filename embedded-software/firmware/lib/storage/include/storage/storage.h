/**
 * @file storage.h
 * @brief Two-sector ping-pong flash storage with a deferred-save task.
 *
 * Architecture:
 *   - Codegen owns the wire layout (firmware/generated/storage/types.h)
 *     and the linker placement (storage.ld).
 *   - This runtime owns the load/save protocol + the storage task.
 *
 * Boot sequence:
 *   1. storage_init() reads both sector copies, picks the higher-
 *      generation copy whose CRC32 matches, populates g_storage,
 *      spawns the storage task.
 *   2. App code modifies g_storage under storage_lock() / unlock()
 *      and pokes storage_save_async() (or storage_save_sync() for
 *      critical points like end-of-calibration).
 *   3. The storage task debounces, snapshots g_storage, erases the
 *      OTHER sector, programs header+payload, verifies CRC, and
 *      atomically swings cur_sector. Power loss anywhere leaves the
 *      previous-generation sector intact.
 *
 * @ UBC Rocket, 2026
 */
#ifndef STORAGE_STORAGE_H
#define STORAGE_STORAGE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "generated/storage/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** RAM working copy. Modify under storage_lock() / storage_unlock().
 *  Defined in firmware/generated/storage/storage_data.c. */
extern storage_t g_storage;

typedef enum {
    STORAGE_LOAD_FRESH = 0,       /**< Neither sector valid → defaults applied */
    STORAGE_LOAD_FROM_A,          /**< Loaded from sector A (higher generation) */
    STORAGE_LOAD_FROM_B,          /**< Loaded from sector B (higher generation) */
} storage_load_result_t;

/**
 * @brief Bring up the storage subsystem. Reads both sectors, picks
 *        the current one, populates g_storage, spawns the storage
 *        task. Call once from app_init_cm4 BEFORE the scheduler
 *        starts (the task is StaticTask-allocated so it lives in
 *        BSS, no malloc).
 *
 * @return STORAGE_LOAD_FRESH if no valid record was found (both
 *         sectors invalid / first boot). The payload is zeroed and
 *         the next save will program sector A with generation = 1.
 */
storage_load_result_t storage_init(void);

/** Take / release the modify mutex. */
void storage_lock(void);
void storage_unlock(void);

/**
 * @brief Mark the working copy dirty + signal the storage task.
 *        Returns immediately. Coalesces with any other pending dirty
 *        marks; the task debounces before flushing.
 */
void storage_save_async(void);

/**
 * @brief Force a flush, block until it lands or @p timeout_ms elapses.
 *        Used at safety-critical points (boot calibration complete,
 *        crash dump record, controlled shutdown). NOT safe to call
 *        from an ISR.
 *
 * @return true on successful flush + verify, false on timeout or
 *         flash error.
 */
bool storage_save_sync(uint32_t timeout_ms);

/**
 * @brief Diagnostic getters. Cheap; no flash IO.
 */
uint32_t storage_current_generation(void);
uint32_t storage_save_count(void);          /**< successful saves since boot */
uint32_t storage_save_errors(void);         /**< failed verify / HAL errors  */
bool     storage_current_is_a(void);        /**< true if A is the active sector */

#ifdef __cplusplus
}
#endif

#endif /* STORAGE_STORAGE_H */
