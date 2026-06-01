/**
 * @file    sd_log_task.c
 * @brief   CM4 task: drain local + CM7 log staging buffers, write to SD.
 *
 * Lifecycle on entry:
 *   1. Replay any pending crash dump into the SD byte stream (so the first
 *      blocks on the card describe the fault that triggered the reboot).
 *   2. Install the SD-channel messages sink (already done by app_init_cm4
 *      via log_service_append_raw) and the CM7→CM4 ring wake callback.
 *      Mark log_service ready.
 *   3. Loop forever:
 *        - drain the CM7→CM4 SRAM4 ring + CM4-local staging buffer into
 *          a 512-byte block-aligned scratch
 *        - whenever the scratch contains one or more 512-byte blocks,
 *          io_sd_write_blocks_async the front-most ones, block on a task
 *          notification from the SD DMA-done callback
 *        - block-pad and flush partials every SD_LOG_FLUSH_INTERVAL_MS
 *
 * The card itself is laid out raw (no FATFS — FATFS is NOT enabled in
 * the CubeMX project, only LwIP + FreeRTOS). Writes are append-only to
 * monotonically increasing block addresses starting at SD_LOG_START_BLOCK,
 * which gives us a single contiguous "current log" partition per power
 * cycle.
 *
 * Per-boot rotation. We could either:
 *   - scan the card for the highest "magic" block and resume past it
 *     (a la log_NNNN.bin in the deprecated build), or
 *   - keep a small "next-start-block" cursor in SRAM4 that survives soft
 *     reset (.shared is NOLOAD), bumped per boot.
 * Phase 2 / TODO: implement the cursor. For now every boot starts at
 * SD_LOG_START_BLOCK and overwrites whatever was there — fine for bench
 * work, NOT fine for a flight. Flagged as NEEDS USER INPUT in the
 * handoff notes.
 *
 * Backpressure / degraded mode. If io_sd_write_blocks_async returns
 * IO_ERR_BUSY we drain into the scratch buffer until the SD signals done.
 * If a write errors twice in a row we go into a degraded mode where the
 * drain still runs (so the CM7 ring doesn't back up) but the bytes are
 * discarded; sd_write_errors counter tracks this so the host sees it
 * via the system.drop_counter publish.
 *
 * UBC Rocket, 2026
 */

#include "app/tasks.h"
#include "app/log_service.h"
#include "app/crash_dump.h"
#include "io_sys/io_sd.h"
#include "io_sys/io_types.h"

#include "messages/messages.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* ---- Tunables ---- */

/** SD block size — hard-wired for SD cards. */
#define SD_BLOCK_SIZE             512U

/** Scratch buffer for assembling block-aligned writes. Sized to comfortably
 *  hold one drain pass from both the local buffer (4 kB) and the SRAM4 ring
 *  (8 kB). Bytes beyond a multiple of 512 are carried into the next pass. */
#define SD_SCRATCH_BYTES          (16U * SD_BLOCK_SIZE)   /* 8 kB */

/** Periodic publish + flush tick. Matches the existing 1 Hz drop-counter
 *  cadence the previous stub used. */
#define SD_LOG_TICK_MS            1000U

/** Wait time for a block to assemble before falling through to publish /
 *  flush. The HSEM IRQ from CM7 cuts this short whenever there's traffic. */
#define SD_LOG_DRAIN_TICK_MS      50U

/** First block address used by the SD log writer. Block 0 is left clear so
 *  a future MBR / partition table can be added without colliding. */
#define SD_LOG_START_BLOCK        2048U

/** Max bytes we'll attempt per io_sd_write_blocks_async call (DMA scatter
 *  is contiguous-only; we still split if scratch wraps but keep the chunks
 *  bounded so a single DMA timeout doesn't stall many seconds of logs). */
#define SD_LOG_MAX_WRITE_BLOCKS   8U   /* 4 kB per DMA submit */

/** How long to wait on the SD DMA-done callback before giving up on a
 *  block. The deprecated builder allowed up to 15 s for pre-erased blocks;
 *  500 ms is plenty for the per-block case and bounds the worst-case
 *  blocking time on the task. */
#define SD_LOG_DMA_TIMEOUT_MS     500U

/** Consecutive write errors before entering degraded (drain-only) mode. */
#define SD_LOG_ERROR_TRIP_COUNT   2U

/* ---- Task-local state ---- */

static TaskHandle_t s_self_handle;   /* cached at task entry for the wake callback */

/* Scratch ring. Bytes flow in via append → drain → here; we then submit
 * whole 512-byte chunks to io_sd_write_blocks_async. */
static uint8_t  s_scratch[SD_SCRATCH_BYTES];
static uint32_t s_scratch_len;

/* SD bookkeeping. */
static uint32_t s_next_block;
static uint32_t s_sd_blocks_written;
static uint32_t s_sd_write_errors;
static bool     s_card_ready;
static bool     s_degraded;

/* ---- Wake callback (called from HSEM IRQ) ---- */

static void sd_log_wake_from_isr(void)
{
    if (s_self_handle == NULL) return;
    BaseType_t hp_woken = pdFALSE;
    vTaskNotifyGiveFromISR(s_self_handle, &hp_woken);
    portYIELD_FROM_ISR(hp_woken);
}

/* ---- SD write completion ---- */

/* The io_sd_done_cb fires from the SDMMC ISR; we hand the result back
 * to the task via a notification value (bit0 = ok, bit1 = error). The
 * task arms a 0-cleared notification, kicks the async write, then blocks
 * on xTaskNotifyWait. */
#define SD_NOTIFY_OK     (1U << 0)
#define SD_NOTIFY_ERR    (1U << 1)

static void sd_log_io_done_cb(io_status_t status, void *arg)
{
    TaskHandle_t target = (TaskHandle_t)arg;
    if (target == NULL) return;
    BaseType_t hp_woken = pdFALSE;
    uint32_t notify_bits = (status == IO_OK) ? SD_NOTIFY_OK : SD_NOTIFY_ERR;
    xTaskNotifyFromISR(target, notify_bits, eSetBits, &hp_woken);
    portYIELD_FROM_ISR(hp_woken);
}

/* ---- Drain helpers ---- */

/* Pull as much as the scratch can hold from both sources. Returns total
 * bytes added to the scratch this pass. */
static size_t drain_into_scratch(void)
{
    size_t added = 0;
    while (s_scratch_len < SD_SCRATCH_BYTES) {
        size_t room = SD_SCRATCH_BYTES - s_scratch_len;
        size_t got = log_service_drain_cm7_ring(&s_scratch[s_scratch_len], room);
        if (got == 0) break;
        s_scratch_len += (uint32_t)got;
        added += got;
    }
    while (s_scratch_len < SD_SCRATCH_BYTES) {
        size_t room = SD_SCRATCH_BYTES - s_scratch_len;
        size_t got = log_service_drain_local(&s_scratch[s_scratch_len], room);
        if (got == 0) break;
        s_scratch_len += (uint32_t)got;
        added += got;
    }
    return added;
}

/* Write up to one DMA chunk's worth of full 512-byte blocks. Returns the
 * number of blocks committed (0 if nothing to write or write failed). */
static uint32_t flush_full_blocks(void)
{
    uint32_t n_blocks = s_scratch_len / SD_BLOCK_SIZE;
    if (n_blocks == 0U) return 0U;
    if (n_blocks > SD_LOG_MAX_WRITE_BLOCKS) n_blocks = SD_LOG_MAX_WRITE_BLOCKS;

    if (!s_card_ready || s_degraded) {
        /* Degraded / no card: discard the front blocks rather than letting
         * the scratch back up forever. */
        uint32_t bytes = n_blocks * SD_BLOCK_SIZE;
        memmove(&s_scratch[0], &s_scratch[bytes], s_scratch_len - bytes);
        s_scratch_len -= bytes;
        return 0U;
    }

    /* Arm the notification slot. xTaskNotifyWait with a 0-mask-to-clear
     * here would lose bits the IRQ might have set since the last wait;
     * we explicitly clear by reading-and-discarding. */
    uint32_t junk = 0;
    (void)xTaskNotifyWait(SD_NOTIFY_OK | SD_NOTIFY_ERR,
                          SD_NOTIFY_OK | SD_NOTIFY_ERR,
                          &junk, 0);

    io_status_t st = io_sd_write_blocks_async(
        s_next_block, &s_scratch[0], n_blocks,
        sd_log_io_done_cb, (void *)s_self_handle);

    if (st == IO_ERR_BUSY) {
        /* SDMMC currently busy with another (impossible in this codebase —
         * sd_log_task is the only producer — but defensive). Retry next
         * tick. */
        return 0U;
    }
    if (st != IO_OK) {
        s_sd_write_errors++;
        if (s_sd_write_errors >= SD_LOG_ERROR_TRIP_COUNT) {
            s_degraded = true;
        }
        return 0U;
    }

    /* Block until the SDMMC ISR notifies us. */
    uint32_t bits = 0;
    BaseType_t ok = xTaskNotifyWait(0, SD_NOTIFY_OK | SD_NOTIFY_ERR,
                                    &bits, pdMS_TO_TICKS(SD_LOG_DMA_TIMEOUT_MS));
    if (ok != pdTRUE || (bits & SD_NOTIFY_ERR) != 0U) {
        s_sd_write_errors++;
        if (s_sd_write_errors >= SD_LOG_ERROR_TRIP_COUNT) {
            s_degraded = true;
        }
        return 0U;
    }

    /* Successful write: advance the card cursor, slide leftover bytes
     * down in the scratch, reset the error counter. */
    uint32_t bytes = n_blocks * SD_BLOCK_SIZE;
    if (s_scratch_len > bytes) {
        memmove(&s_scratch[0], &s_scratch[bytes], s_scratch_len - bytes);
    }
    s_scratch_len -= bytes;
    s_next_block += n_blocks;
    s_sd_blocks_written += n_blocks;
    s_sd_write_errors = 0U;
    return n_blocks;
}

/* Block-pad whatever is in the scratch so a flush leaves the card aligned
 * to a block boundary. Padding is 0xFF (matches the deprecated builder's
 * convention; SD cards prefer 0xFF for un-programmed bytes). Then writes
 * the (one) partial block via flush_full_blocks. */
static void flush_partial_block(void)
{
    if (s_scratch_len == 0U) return;
    if (!s_card_ready || s_degraded) {
        /* Drop the partial. We don't want to leave bytes in scratch for
         * ever in degraded mode. */
        s_scratch_len = 0U;
        return;
    }
    uint32_t mod = s_scratch_len % SD_BLOCK_SIZE;
    if (mod != 0U) {
        uint32_t pad = SD_BLOCK_SIZE - mod;
        if (s_scratch_len + pad > SD_SCRATCH_BYTES) {
            /* No room to pad — wait for next tick. */
            return;
        }
        memset(&s_scratch[s_scratch_len], 0xFF, pad);
        s_scratch_len += pad;
    }
    (void)flush_full_blocks();
}

/* ---- Task entry ---- */

void task_sd_log(void *arg)
{
    (void)arg;
    s_self_handle = xTaskGetCurrentTaskHandle();

    /* Wire the CM7→CM4 ring wake callback. We do this from task context
     * to dodge the chicken/egg: log_service_init() ran before any task
     * existed, so it couldn't have a task handle. */
    log_service_set_wake_callback(sd_log_wake_from_isr);

    /* Replay any pending crash dump first — the resulting envelopes are
     * pushed into log_service before we mark the service ready, which is
     * fine because they're going into the local CM4 buffer that we own
     * the only consumer of. */
    crash_dump_flush_pending();

    /* Bring up the card. io_sd_init() was already called by io_init_cm4;
     * we just need to confirm a card is present. */
    s_card_ready  = io_sd_card_present();
    s_next_block  = SD_LOG_START_BLOCK;
    s_degraded    = !s_card_ready;

    /* From this point onward log_service_append_raw is "real" — both
     * cores' producers can push. */
    log_service_mark_ready();

    TickType_t last_publish = xTaskGetTickCount();

    for (;;) {
        /* Wait up to SD_LOG_DRAIN_TICK_MS for a CM7 wake or local timeout.
         * Either path drops us into the drain. ulTaskNotifyTake clears the
         * notify count to 0 on wake, which is what we want — we'll
         * re-evaluate the drain state every iteration regardless. */
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(SD_LOG_DRAIN_TICK_MS));

        /* Drain into scratch, then flush whatever full blocks accumulated. */
        (void)drain_into_scratch();
        while (s_scratch_len >= SD_BLOCK_SIZE) {
            uint32_t wrote = flush_full_blocks();
            if (wrote == 0U) break;   /* err / busy / degraded — stop spinning */
        }

        /* Periodic housekeeping: publish drop counters + flush any
         * partial block so a clean reboot doesn't lose the tail. */
        TickType_t now = xTaskGetTickCount();
        if ((TickType_t)(now - last_publish) >= pdMS_TO_TICKS(SD_LOG_TICK_MS)) {
            last_publish = now;
            flush_partial_block();
            messages_publish_drop_counters();
            /* If we were degraded but the card came back, try recovery. */
            if (s_degraded && io_sd_card_present()) {
                s_card_ready = true;
                s_degraded   = false;
                s_sd_write_errors = 0U;
            }
        }
    }
}
