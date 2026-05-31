#include "SD_logging/log_writer.h"
#include "log_records/log_frame.h"

#include "main.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include <string.h>

extern SD_HandleTypeDef hsd1;

#define LOG_ERASE_BLOCK_COUNT      262144U   /* 128 MB */
#define LOG_ERASE_TIMEOUT_MS       15000U
#define LOG_ERASE_AHEAD_THRESHOLD  4096U
#define LOG_ERASE_AHEAD_SIZE       32768U

/* ── Submit queue (lock-free MPSC via atomic CAS on head) ── */
#define SUBMIT_QUEUE_SIZE  16U  /* power of 2, > LOG_BUFFER_COUNT */
#define SUBMIT_QUEUE_MASK  (SUBMIT_QUEUE_SIZE - 1U)

typedef struct {
    bool ready;
    bool error;
    log_buffer_t buffers[LOG_BUFFER_COUNT];
    uint8_t active_log_idx;       /* Index of the current log-writing buffer */
    uint32_t block_address;
    uint32_t erased_up_to_block;
    bool dma_in_progress;
    SemaphoreHandle_t dma_done_sem;
    StaticSemaphore_t dma_done_sem_buf;
    SemaphoreHandle_t buffer_mutex;   /* Protects active_log_idx and log buffer writes */
    StaticSemaphore_t buffer_mutex_buf;
    TaskHandle_t flush_task_handle;   /* Set by flush task; used ONLY by DMA ISR */
    /* Submit queue */
    volatile uint8_t sq_entries[SUBMIT_QUEUE_SIZE];
    volatile uint32_t sq_head;
    volatile uint32_t sq_tail;
} log_writer_ctx_t;

static log_writer_ctx_t g_ctx;
static bool s_preflight_erased = false;

/* ── Forward declarations ── */
static bool ensure_preflight_erase(void);
static bool wait_for_card_ready(uint32_t timeout_ms);
static void erase_ahead_if_needed(void);
static void wait_for_dma_completion(void);
static uint8_t find_empty_buffer_in(uint8_t start, uint8_t end);

/* ── Submit queue ── */

bool log_writer_submit_enqueue(uint8_t buf_idx)
{
    uint32_t head, next;
    do {
        head = g_ctx.sq_head;
        next = (head + 1U) & SUBMIT_QUEUE_MASK;
        if (next == g_ctx.sq_tail) {
            return false;  /* queue full */
        }
    } while (!__atomic_compare_exchange_n(
        (uint32_t *)&g_ctx.sq_head, &head, next,
        false, __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST));

    g_ctx.sq_entries[head] = buf_idx;
    __DMB();
    return true;
}

bool log_writer_submit_dequeue(uint8_t *buf_idx)
{
    uint32_t tail = g_ctx.sq_tail;
    if (tail == g_ctx.sq_head) {
        return false;
    }
    __DMB();
    *buf_idx = g_ctx.sq_entries[tail];
    __DMB();
    g_ctx.sq_tail = (tail + 1U) & SUBMIT_QUEUE_MASK;
    return true;
}

/* ── Buffer helpers ── */

log_buffer_t *log_writer_get_buffer(uint8_t idx)
{
    if (idx >= LOG_BUFFER_COUNT) return NULL;
    return &g_ctx.buffers[idx];
}

uint8_t log_writer_buffer_count(void)
{
    return LOG_BUFFER_COUNT;
}

static uint8_t find_empty_buffer_in(uint8_t start, uint8_t end)
{
    for (uint8_t i = start; i < end; i++) {
        if (g_ctx.buffers[i].state == BUF_EMPTY) {
            return i;
        }
    }
    return UINT8_MAX;
}

/* ── Init ── */

bool log_writer_init(void)
{
    memset(&g_ctx, 0, sizeof(g_ctx));

    if (!g_sd_card_initialized) {
        return false;
    }

    if (!ensure_preflight_erase()) {
        return false;
    }

    g_ctx.dma_done_sem = xSemaphoreCreateBinaryStatic(&g_ctx.dma_done_sem_buf);
    if (!g_ctx.dma_done_sem) return false;

    g_ctx.buffer_mutex = xSemaphoreCreateMutexStatic(&g_ctx.buffer_mutex_buf);
    if (!g_ctx.buffer_mutex) return false;

    xSemaphoreGive(g_ctx.dma_done_sem);

    /* Reserve buffer 0 as the initial active log buffer */
    g_ctx.active_log_idx = 0U;
    g_ctx.buffers[0].state = BUF_ACTIVE_LOG;
    g_ctx.buffers[0].write_pos = 0U;

    g_ctx.block_address = 0U;
    g_ctx.erased_up_to_block = LOG_ERASE_BLOCK_COUNT;
    g_ctx.ready = true;
    return true;
}

bool log_writer_ready(void)
{
    return g_ctx.ready && !g_ctx.error;
}

void log_writer_set_flush_task(void *task_handle)
{
    g_ctx.flush_task_handle = (TaskHandle_t)task_handle;
}

/* ── Trace buffer API (lock-free) ── */

log_buffer_t *log_writer_claim_trace_buffer(void)
{
    uint8_t idx = find_empty_buffer_in(LOG_BUF_TRACE_START, LOG_BUF_TRACE_END);
    if (idx == UINT8_MAX) return NULL;
    g_ctx.buffers[idx].state = BUF_ACTIVE_TRACE;
    g_ctx.buffers[idx].write_pos = 0U;
    return &g_ctx.buffers[idx];
}

log_buffer_t *log_writer_swap_trace_buffer(log_buffer_t *filled)
{
    /* Mark filled buffer as READY — the flush task scans the trace
     * partition directly, so no need to enqueue. */
    filled->state = BUF_READY;

    /* Find next empty buffer from the trace partition */
    uint8_t next = find_empty_buffer_in(LOG_BUF_TRACE_START, LOG_BUF_TRACE_END);
    if (next == UINT8_MAX) return NULL;
    g_ctx.buffers[next].state = BUF_ACTIVE_TRACE;
    g_ctx.buffers[next].write_pos = 0U;
    return &g_ctx.buffers[next];
}

/* ── Record append (task context, mutex-protected) ── */

bool log_writer_append_record(log_record_type_t type,
                              const void *payload,
                              size_t payload_size)
{
    if (!log_writer_ready()) return false;

    /* Non-blocking: if the mutex is held (e.g. by flush or another task's
     * log call), drop the record rather than stalling the caller.  Critical
     * for the Controls task which must never block on logging. */
    if (xSemaphoreTake(g_ctx.buffer_mutex, 0) != pdTRUE) {
        return false;
    }

    if (payload_size > UINT16_MAX || (payload_size > 0U && payload == NULL)) {
        xSemaphoreGive(g_ctx.buffer_mutex);
        return false;
    }

    size_t total_size = sizeof(log_record_frame_t) + payload_size;
    if (total_size > LOG_SD_BLOCK_SIZE) {
        xSemaphoreGive(g_ctx.buffer_mutex);
        return false;
    }

    log_buffer_t *buf = &g_ctx.buffers[g_ctx.active_log_idx];

    /* If record doesn't fit, submit current buffer and get a new one */
    if (buf->write_pos + total_size > LOG_SD_BLOCK_SIZE) {
        /* Pad and submit */
        if (buf->write_pos < LOG_SD_BLOCK_SIZE) {
            memset(&buf->data[buf->write_pos], 0xFF,
                   LOG_SD_BLOCK_SIZE - buf->write_pos);
        }
        buf->state = BUF_READY;
        log_writer_submit_enqueue(g_ctx.active_log_idx);

        /* Grab next empty buffer */
        uint8_t next = find_empty_buffer_in(LOG_BUF_LOG_START, LOG_BUF_LOG_END);
        if (next == UINT8_MAX) {
            /* No buffers free — very unlikely with 8 buffers */
            xSemaphoreGive(g_ctx.buffer_mutex);
            return false;
        }
        g_ctx.active_log_idx = next;
        g_ctx.buffers[next].state = BUF_ACTIVE_LOG;
        g_ctx.buffers[next].write_pos = 0U;
        buf = &g_ctx.buffers[next];
    }

    /* Build frame header with CRC */
    log_record_frame_t frame = {
        .magic = LOG_RECORD_MAGIC,
        .type = (uint8_t)type,
        .length = (uint16_t)payload_size,
        .crc16 = 0U,
        .reserved = 0U
    };
    uint16_t crc = log_crc16_ccitt_compute((const uint8_t *)&frame, sizeof(frame));
    if (payload_size > 0U) {
        crc = log_crc16_ccitt_accumulate(crc, (const uint8_t *)payload, payload_size);
    }
    frame.crc16 = crc;

    memcpy(&buf->data[buf->write_pos], &frame, sizeof(frame));
    buf->write_pos += sizeof(frame);
    if (payload_size > 0U) {
        memcpy(&buf->data[buf->write_pos], payload, payload_size);
        buf->write_pos += payload_size;
    }

    /* If buffer is exactly full, submit immediately */
    if (buf->write_pos == LOG_SD_BLOCK_SIZE) {
        buf->state = BUF_READY;
        log_writer_submit_enqueue(g_ctx.active_log_idx);

        uint8_t next = find_empty_buffer_in(LOG_BUF_LOG_START, LOG_BUF_LOG_END);
        if (next != UINT8_MAX) {
            g_ctx.active_log_idx = next;
            g_ctx.buffers[next].state = BUF_ACTIVE_LOG;
            g_ctx.buffers[next].write_pos = 0U;
        }
        /* If no empty buffer, next append will fail — acceptable */
    }

    xSemaphoreGive(g_ctx.buffer_mutex);
    return true;
}

bool log_writer_flush(void)
{
    if (!log_writer_ready()) return false;

    /* Short timeout — flush task can retry next cycle */
    if (xSemaphoreTake(g_ctx.buffer_mutex, pdMS_TO_TICKS(5)) != pdTRUE) {
        return false;
    }

    log_buffer_t *buf = &g_ctx.buffers[g_ctx.active_log_idx];
    if (buf->write_pos > 0U) {
        /* Pad and submit */
        if (buf->write_pos < LOG_SD_BLOCK_SIZE) {
            memset(&buf->data[buf->write_pos], 0xFF,
                   LOG_SD_BLOCK_SIZE - buf->write_pos);
        }
        buf->state = BUF_READY;
        log_writer_submit_enqueue(g_ctx.active_log_idx);

        uint8_t next = find_empty_buffer_in(LOG_BUF_LOG_START, LOG_BUF_LOG_END);
        if (next != UINT8_MAX) {
            g_ctx.active_log_idx = next;
            g_ctx.buffers[next].state = BUF_ACTIVE_LOG;
            g_ctx.buffers[next].write_pos = 0U;
        }
    }

    xSemaphoreGive(g_ctx.buffer_mutex);
    return true;
}

/* ── DMA write (called only from flush task) ── */

bool log_writer_dma_write_block(uint8_t buf_idx)
{
    if (buf_idx >= LOG_BUFFER_COUNT) return false;
    if (!log_writer_ready()) return false;

    wait_for_dma_completion();

    /* Erase-ahead disabled: the preflight erase covers 128 MB (262144 blocks)
     * which is ~9 minutes at current write rates.  Runtime erase causes
     * 25ms busy-wait stalls in HAL_SD_Erase that disrupt trace capture.
     * Stop writing if we've exhausted the pre-erased region. */
    if (g_ctx.block_address >= LOG_ERASE_BLOCK_COUNT) {
        return false;
    }

    if (!wait_for_card_ready(LOG_ERASE_TIMEOUT_MS)) {
        g_ctx.error = true;
        g_ctx.ready = false;
        return false;
    }

    g_ctx.buffers[buf_idx].state = BUF_FLUSHING;

    HAL_StatusTypeDef status = HAL_SD_WriteBlocks_DMA(
        &hsd1,
        g_ctx.buffers[buf_idx].data,
        g_ctx.block_address,
        1U);

    if (status != HAL_OK) {
        g_ctx.error = true;
        g_ctx.ready = false;
        g_ctx.buffers[buf_idx].state = BUF_EMPTY;
        return false;
    }

    g_ctx.block_address += 1U;
    g_ctx.dma_in_progress = true;
    /* Reset semaphore so wait blocks until ISR releases it */
    xSemaphoreTake(g_ctx.dma_done_sem, 0);

    /* Block until DMA finishes — buffer must stay valid */
    wait_for_dma_completion();

    /* Buffer is now free */
    g_ctx.buffers[buf_idx].write_pos = 0U;
    g_ctx.buffers[buf_idx].state = BUF_EMPTY;
    return true;
}

/* ── DMA completion ── */

static void wait_for_dma_completion(void)
{
    if (g_ctx.dma_in_progress) {
        xSemaphoreTake(g_ctx.dma_done_sem, portMAX_DELAY);
        g_ctx.dma_in_progress = false;
    }
}

void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd)
{
    if (hsd != &hsd1) return;

    BaseType_t higher_priority_task_woken = pdFALSE;
    g_ctx.dma_in_progress = false;
    xSemaphoreGiveFromISR(g_ctx.dma_done_sem, &higher_priority_task_woken);

    /* Also wake the flush task so it can start the next write immediately */
    if (g_ctx.flush_task_handle != NULL) {
        BaseType_t woken2 = pdFALSE;
        vTaskNotifyGiveFromISR(g_ctx.flush_task_handle, &woken2);
        higher_priority_task_woken |= woken2;
    }

    portYIELD_FROM_ISR(higher_priority_task_woken);
}

void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd)
{
    if (hsd != &hsd1) return;

    (void)HAL_SD_GetError(hsd);
    (void)HAL_SD_GetCardState(hsd);

    g_ctx.error = true;
    g_ctx.ready = false;
    BaseType_t higher_priority_task_woken = pdFALSE;
    g_ctx.dma_in_progress = false;
    xSemaphoreGiveFromISR(g_ctx.dma_done_sem, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

/* ── Erase logic ── */

static bool ensure_preflight_erase(void)
{
    if (s_preflight_erased) return true;

    HAL_SD_CardInfoTypeDef card_info = {0};
    if (HAL_SD_GetCardInfo(&hsd1, &card_info) != HAL_OK) return false;

    uint32_t blocks_to_erase = LOG_ERASE_BLOCK_COUNT;
    if (card_info.LogBlockNbr > 0U && blocks_to_erase > card_info.LogBlockNbr) {
        blocks_to_erase = card_info.LogBlockNbr;
    }
    if (blocks_to_erase == 0U) return false;

    if (HAL_SD_Erase(&hsd1, 0U, blocks_to_erase - 1U) != HAL_OK) return false;
    if (!wait_for_card_ready(LOG_ERASE_TIMEOUT_MS)) return false;

    s_preflight_erased = true;
    return true;
}

static void erase_ahead_if_needed(void)
{
    if (g_ctx.block_address + LOG_ERASE_AHEAD_THRESHOLD
            >= g_ctx.erased_up_to_block) {
        uint32_t erase_start = g_ctx.erased_up_to_block;
        uint32_t erase_end   = erase_start + LOG_ERASE_AHEAD_SIZE - 1U;
        if (HAL_SD_Erase(&hsd1, erase_start, erase_end) == HAL_OK) {
            if (wait_for_card_ready(LOG_ERASE_TIMEOUT_MS)) {
                g_ctx.erased_up_to_block = erase_end + 1U;
            }
        }
    }
}

static bool wait_for_card_ready(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    while (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER) {
        if ((HAL_GetTick() - start) > timeout_ms) return false;
        if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
            /* Card programming typically completes in <1ms for pre-erased
             * sectors.  Use 1ms delay to avoid busy-spinning while still
             * keeping throughput high. */
            vTaskDelay(pdMS_TO_TICKS(1U));
        } else {
            HAL_Delay(1U);
        }
    }
    return true;
}
