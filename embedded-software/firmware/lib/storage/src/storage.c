/**
 * @file storage.c
 * @brief Two-sector ping-pong storage runtime.
 *
 * Wire format (per sector):
 *   [storage_header_t : 16 B][storage_payload_t : N B][pad to 32 B]
 *   header.crc32 covers [magic..generation] + payload.
 *
 * The flash region is split into two sectors A and B; codegen reserves
 * them with the linker fragment in generated/storage/storage.ld. On
 * boot we pick the sector with the higher generation whose CRC matches.
 * Saves go to the OTHER sector; commit is implicit (next boot picks
 * the higher generation).
 *
 * @ UBC Rocket, 2026
 */
#include "storage/storage.h"
#include "storage_crc32.h"

#include "generated/storage/layout.h"

#include "stm32h7xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <string.h>

/* -------------------------------------------------------------------------- */
/* Tunables                                                                    */
/* -------------------------------------------------------------------------- */
#define STORAGE_HEARTBEAT_MS    60000U   /* periodic flush even if no poke */
#define STORAGE_DEBOUNCE_MS     2000U    /* coalesce update bursts         */
#define STORAGE_TASK_PRIO       2U
#define STORAGE_TASK_STACK_W    1024U    /* 4 KB stack                      */
#define STORAGE_SYNC_NOTIFY_OK  (1U << 0)
#define STORAGE_SYNC_NOTIFY_ERR (1U << 1)

/* -------------------------------------------------------------------------- */
/* State                                                                       */
/* -------------------------------------------------------------------------- */
static SemaphoreHandle_t   s_lock;
static StaticSemaphore_t   s_lock_buf;

static TaskHandle_t        s_task;
static StaticTask_t        s_task_tcb;
static StackType_t         s_task_stack[STORAGE_TASK_STACK_W];

/* Generation counter — read from the active flash copy at boot, bumped
 * inside the storage task on every save. */
static uint32_t            s_generation;

/* Which sector currently holds the most recent valid record. We always
 * write the OTHER sector. */
static bool                s_cur_is_a;

/* Dirty + last-dirty-tick for debounce. Producer side does atomic
 * stores; the task does relaxed reads inside the debounce loop. */
static volatile bool       s_dirty;
static volatile TickType_t s_last_dirty_tick;

/* If non-NULL, the task notifies this handle on flush completion (via
 * storage_save_sync). Only one waiter at a time — sync calls take s_lock
 * to serialise. */
static volatile TaskHandle_t s_sync_waiter;

/* Diagnostic counters. */
static uint32_t s_save_count;
static uint32_t s_save_errors;

/* -------------------------------------------------------------------------- */
/* Header / payload utilities                                                  */
/* -------------------------------------------------------------------------- */

/* CRC scope: [magic..generation] of the header (12 bytes) + payload. */
static uint32_t compute_crc(const storage_t *s) {
    /* Two-phase CRC: header front, then payload. We can't pass a single
     * memory range because crc32 field sits between them; compute over
     * the header's first 12 bytes, then over the payload. */
    uint32_t c;
    /* IEEE/zlib CRC is byte-wise; build a fresh CRC by running both
     * regions through it. The helper does init+finalize per call, so
     * we'd need an incremental variant for true concatenation. Simpler:
     * stage into a small scratch and CRC once. */
    uint8_t scratch[sizeof(storage_header_t) - 4 + sizeof(storage_payload_t)];
    memcpy(&scratch[0], &s->header, sizeof(storage_header_t) - 4);
    memcpy(&scratch[sizeof(storage_header_t) - 4], &s->payload,
           sizeof(storage_payload_t));
    c = storage_crc32(scratch, sizeof(scratch));
    return c;
}

static bool is_record_valid(const storage_t *s) {
    if (s->header.magic   != STORAGE_MAGIC)            return false;
    if (s->header.version != STORAGE_SCHEMA_VERSION)   return false;
    if (s->header.length  != sizeof(storage_payload_t)) return false;
    if (s->header.crc32   != compute_crc(s))           return false;
    return true;
}

/* -------------------------------------------------------------------------- */
/* Flash IO                                                                    */
/* -------------------------------------------------------------------------- */

static bool erase_sector(uint32_t bank, uint32_t sector) {
    FLASH_EraseInitTypeDef e = {
        .TypeErase    = FLASH_TYPEERASE_SECTORS,
        .Banks        = (bank == 1) ? FLASH_BANK_1 : FLASH_BANK_2,
        .Sector       = sector,
        .NbSectors    = 1,
        .VoltageRange = FLASH_VOLTAGE_RANGE_3,
    };
    uint32_t sector_error = 0xFFFFFFFFU;
    HAL_FLASH_Unlock();
    HAL_StatusTypeDef st = HAL_FLASHEx_Erase(&e, &sector_error);
    HAL_FLASH_Lock();
    return st == HAL_OK && sector_error == 0xFFFFFFFFU;
}

/* Program one record (sizeof(storage_t) bytes) into the target sector.
 * H7 flashword = 256 bits = 32 bytes. Codegen pads storage_t to a
 * multiple of 32 so we always program whole flashwords. */
static bool program_record(uint32_t target_origin, const storage_t *src) {
    const uint32_t flashword_bytes = 32U;
    const uint32_t total           = sizeof(storage_t);
    HAL_FLASH_Unlock();
    bool ok = true;
    for (uint32_t off = 0; off < total; off += flashword_bytes) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
                              target_origin + off,
                              (uint32_t)((const uint8_t *)src + off)) != HAL_OK) {
            ok = false;
            break;
        }
    }
    HAL_FLASH_Lock();
    return ok;
}

/* -------------------------------------------------------------------------- */
/* Save (called from storage task only)                                        */
/* -------------------------------------------------------------------------- */

static bool storage_commit_locked(void) {
    /* Build the record in RAM from g_storage (which has been snapshotted
     * by the caller under s_lock). HAL_FLASH_Program FLASHWORD requires
     * the source address to be 32-byte aligned; ensure that here. */
    __attribute__((aligned(32))) storage_t rec;
    memset(&rec, 0, sizeof(rec));
    rec.header.magic      = STORAGE_MAGIC;
    rec.header.version    = STORAGE_SCHEMA_VERSION;
    rec.header.length     = sizeof(storage_payload_t);
    rec.header.generation = s_generation + 1U;
    rec.payload           = g_storage.payload;
    rec.header.crc32      = compute_crc(&rec);

    /* Target = OTHER sector. */
    bool target_is_a = !s_cur_is_a;
    uint32_t target_origin = target_is_a ? STORAGE_ORIGIN_A : STORAGE_ORIGIN_B;
    uint32_t target_sector = target_is_a ? STORAGE_SECTOR_A : STORAGE_SECTOR_B;

    if (!erase_sector(STORAGE_BANK, target_sector)) return false;
    if (!program_record(target_origin, &rec))      return false;

    /* Verify by reading back via the mapped flash address. The compiler
     * shouldn't have cached anything because the placeholders are
     * declared volatile in the runtime view. */
    const storage_t *readback = (const storage_t *)target_origin;
    if (!is_record_valid(readback)) return false;
    if (memcmp(&readback->payload, &rec.payload,
               sizeof(storage_payload_t)) != 0) {
        return false;
    }

    /* Implicit commit: now that the new sector is valid with a higher
     * generation, swing our view. */
    s_cur_is_a   = target_is_a;
    s_generation = rec.header.generation;
    return true;
}

/* -------------------------------------------------------------------------- */
/* Load                                                                        */
/* -------------------------------------------------------------------------- */

static storage_load_result_t storage_load(void) {
    const storage_t *a = &__storage_a_in_flash;
    const storage_t *b = &__storage_b_in_flash;
    bool a_ok = is_record_valid(a);
    bool b_ok = is_record_valid(b);
    const storage_t *pick;
    bool pick_is_a;

    if (a_ok && b_ok) {
        if (a->header.generation >= b->header.generation) {
            pick = a; pick_is_a = true;
        } else {
            pick = b; pick_is_a = false;
        }
    } else if (a_ok) {
        pick = a; pick_is_a = true;
    } else if (b_ok) {
        pick = b; pick_is_a = false;
    } else {
        /* Cold boot or both corrupt — zero the working copy. The first
         * save will program sector A with generation = 1. */
        memset(&g_storage, 0, sizeof(g_storage));
        s_generation = 0;
        s_cur_is_a   = false;   /* so the first write goes to A */
        return STORAGE_LOAD_FRESH;
    }
    /* Copy the valid record into the RAM working copy. */
    memcpy(&g_storage.payload, &pick->payload, sizeof(storage_payload_t));
    g_storage.header = pick->header;
    s_generation     = pick->header.generation;
    s_cur_is_a       = pick_is_a;
    return pick_is_a ? STORAGE_LOAD_FROM_A : STORAGE_LOAD_FROM_B;
}

/* -------------------------------------------------------------------------- */
/* Storage task                                                                */
/* -------------------------------------------------------------------------- */

static void storage_task(void *arg) {
    (void)arg;
    for (;;) {
        /* Wait for a dirty poke OR the periodic heartbeat. */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(STORAGE_HEARTBEAT_MS));
        if (!s_dirty) continue;

        /* Debounce: coalesce a flurry of edits. Loop while last_dirty_
         * tick keeps moving — once it stays put for STORAGE_DEBOUNCE_MS,
         * we flush. */
        TickType_t last;
        do {
            last = s_last_dirty_tick;
            vTaskDelay(pdMS_TO_TICKS(STORAGE_DEBOUNCE_MS));
        } while (s_last_dirty_tick != last);

        /* Snapshot under lock; release before the slow flash IO so
         * callers aren't blocked behind us. */
        storage_lock();
        s_dirty = false;
        storage_t snapshot_payload_only;
        snapshot_payload_only.payload = g_storage.payload;
        storage_unlock();

        /* g_storage's header gets overwritten with the new generation
         * inside the commit. Stage the snapshot back into g_storage so
         * commit reads from a coherent state. We already have the lock
         * released here; commit only reads g_storage.payload + writes
         * generation, so racing is fine for our single-writer (only the
         * task itself) pattern. */
        storage_lock();
        g_storage.payload = snapshot_payload_only.payload;
        storage_unlock();

        bool ok = storage_commit_locked();
        if (ok) {
            s_save_count++;
        } else {
            s_save_errors++;
        }

        /* Wake any sync waiter. */
        TaskHandle_t waiter = s_sync_waiter;
        if (waiter != NULL) {
            xTaskNotify(waiter,
                        ok ? STORAGE_SYNC_NOTIFY_OK : STORAGE_SYNC_NOTIFY_ERR,
                        eSetBits);
            s_sync_waiter = NULL;
        }
    }
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                  */
/* -------------------------------------------------------------------------- */

storage_load_result_t storage_init(void) {
    s_lock = xSemaphoreCreateMutexStatic(&s_lock_buf);
    storage_load_result_t r = storage_load();
    s_task = xTaskCreateStatic(storage_task, "storage",
                               STORAGE_TASK_STACK_W, NULL,
                               STORAGE_TASK_PRIO,
                               s_task_stack, &s_task_tcb);
    return r;
}

void storage_lock(void)   { xSemaphoreTake(s_lock, portMAX_DELAY); }
void storage_unlock(void) { xSemaphoreGive(s_lock); }

void storage_save_async(void) {
    s_dirty           = true;
    s_last_dirty_tick = xTaskGetTickCount();
    if (s_task != NULL) {
        xTaskNotifyGive(s_task);
    }
}

bool storage_save_sync(uint32_t timeout_ms) {
    /* Serialise sync calls so we don't lose a waiter handle. */
    storage_lock();
    s_sync_waiter = xTaskGetCurrentTaskHandle();
    s_dirty       = true;
    s_last_dirty_tick = xTaskGetTickCount();
    storage_unlock();

    /* Bypass the debounce by waking the task immediately. The task
     * still runs its debounce loop; pass a near-zero debounce by
     * marking dirty and then sleeping until our notify fires. */
    if (s_task != NULL) {
        xTaskNotifyGive(s_task);
    }

    uint32_t notif = 0;
    BaseType_t ok = xTaskNotifyWait(
        STORAGE_SYNC_NOTIFY_OK | STORAGE_SYNC_NOTIFY_ERR,
        STORAGE_SYNC_NOTIFY_OK | STORAGE_SYNC_NOTIFY_ERR,
        &notif, pdMS_TO_TICKS(timeout_ms));
    if (ok == pdFALSE) return false;
    return (notif & STORAGE_SYNC_NOTIFY_OK) != 0U;
}

uint32_t storage_current_generation(void) { return s_generation; }
uint32_t storage_save_count(void)         { return s_save_count; }
uint32_t storage_save_errors(void)        { return s_save_errors; }
bool     storage_current_is_a(void)       { return s_cur_is_a; }
