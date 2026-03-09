#include "SD_logging/log_writer.h"

#include "main.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include <string.h>

extern SD_HandleTypeDef hsd1;

#define LOG_SD_BLOCK_SIZE   512U
#define LOG_BUFFER_COUNT    2U
#define LOG_ERASE_BLOCK_COUNT 8192U
#define LOG_ERASE_TIMEOUT_MS 5000U

typedef struct {
    uint8_t data[LOG_SD_BLOCK_SIZE];
    size_t used;
} log_buffer_t;

typedef struct {
    bool ready;
    bool error;
    log_buffer_t buffers[LOG_BUFFER_COUNT];
    uint8_t active_idx;
    uint8_t flushing_idx;
    uint32_t block_address;
    bool dma_in_progress;
    SemaphoreHandle_t dma_done_sem;
    StaticSemaphore_t dma_done_sem_buf;
    SemaphoreHandle_t buffer_mutex;
    StaticSemaphore_t buffer_mutex_buf;
} log_writer_ctx_t;

static log_writer_ctx_t g_log_ctx;
static bool s_preflight_erased = false;

static uint16_t crc16_ccitt_update(uint16_t crc, uint8_t data);
static uint16_t crc16_ccitt_compute(const uint8_t *data, size_t len);
static uint16_t crc16_ccitt_accumulate(uint16_t crc, const uint8_t *data, size_t len);
static log_buffer_t *get_active_buffer(void);
static uint8_t next_buffer_index(uint8_t idx);
static void pad_buffer_to_block(log_buffer_t *buf);
static bool start_dma_flush(uint8_t buffer_index);
static void wait_for_dma_completion(void);
static bool flush_active_buffer(void);
static bool ensure_preflight_erase(void);
static bool wait_for_card_ready(uint32_t timeout_ms);

bool log_writer_init(void)
{
    memset(&g_log_ctx, 0, sizeof(g_log_ctx));

    if (!g_sd_card_initialized) {
        return false;
    }

    if (!ensure_preflight_erase()) {
        return false;
    }

    g_log_ctx.dma_done_sem = xSemaphoreCreateBinaryStatic(&g_log_ctx.dma_done_sem_buf);
    if (!g_log_ctx.dma_done_sem) {
        return false;
    }

    g_log_ctx.buffer_mutex = xSemaphoreCreateMutexStatic(&g_log_ctx.buffer_mutex_buf);
    if (!g_log_ctx.buffer_mutex) {
        return false;
    }

    /* Allow first wait to pass immediately. */
    xSemaphoreGive(g_log_ctx.dma_done_sem);

    g_log_ctx.active_idx = 0U;
    g_log_ctx.flushing_idx = UINT8_MAX;
    g_log_ctx.block_address = 0U;
    g_log_ctx.ready = true;
    return true;
}

bool log_writer_ready(void)
{
    return g_log_ctx.ready && !g_log_ctx.error;
}

bool log_writer_append_record(log_record_type_t type,
                              const void *payload,
                              size_t payload_size)
{
    if (!log_writer_ready()) {
        return false;
    }

    if (xSemaphoreTake(g_log_ctx.buffer_mutex, portMAX_DELAY) != pdTRUE) {
        return false;
    }

    if (payload_size > UINT16_MAX) {
        xSemaphoreGive(g_log_ctx.buffer_mutex);
        return false;
    }

    if (payload_size > 0U && payload == NULL) {
        xSemaphoreGive(g_log_ctx.buffer_mutex);
        return false;
    }

    size_t total_size = sizeof(log_record_frame_t) + payload_size;
    if (total_size > LOG_SD_BLOCK_SIZE) {
        /* Record too large for current block size. */
        xSemaphoreGive(g_log_ctx.buffer_mutex);
        return false;
    }

    log_buffer_t *buf = get_active_buffer();
    if (buf->used + total_size > LOG_SD_BLOCK_SIZE) {
        if (!flush_active_buffer()) {
            xSemaphoreGive(g_log_ctx.buffer_mutex);
            return false;
        }
        buf = get_active_buffer();
    }

    log_record_frame_t frame = {
        .magic = LOG_RECORD_MAGIC,
        .type = (uint8_t)type,
        .length = (uint16_t)payload_size,
        .crc16 = 0U,
        .reserved = 0U
    };

    uint16_t crc = crc16_ccitt_compute((const uint8_t *)&frame, sizeof(frame));
    if (payload_size > 0U) {
        crc = crc16_ccitt_accumulate(crc, (const uint8_t *)payload, payload_size);
    }
    frame.crc16 = crc;

    memcpy(&buf->data[buf->used], &frame, sizeof(frame));
    buf->used += sizeof(frame);

    if (payload_size > 0U) {
        memcpy(&buf->data[buf->used], payload, payload_size);
        buf->used += payload_size;
    }

    if (buf->used == LOG_SD_BLOCK_SIZE) {
        bool ok = flush_active_buffer();
        xSemaphoreGive(g_log_ctx.buffer_mutex);
        return ok;
    }

    xSemaphoreGive(g_log_ctx.buffer_mutex);
    return true;
}

bool log_writer_flush(void)
{
    if (!log_writer_ready()) {
        return false;
    }

    if (xSemaphoreTake(g_log_ctx.buffer_mutex, portMAX_DELAY) != pdTRUE) {
        return false;
    }

    log_buffer_t *buf = get_active_buffer();
    if (buf->used > 0U) {
        if (!flush_active_buffer()) {
            xSemaphoreGive(g_log_ctx.buffer_mutex);
            return false;
        }
    }

    xSemaphoreGive(g_log_ctx.buffer_mutex);

    if (!log_writer_ready()) {
        return false;
    }

    wait_for_dma_completion();
    return !g_log_ctx.error;
}

static log_buffer_t *get_active_buffer(void)
{
    return &g_log_ctx.buffers[g_log_ctx.active_idx];
}

static uint8_t next_buffer_index(uint8_t idx)
{
    return (uint8_t)((idx + 1U) % LOG_BUFFER_COUNT);
}

static void pad_buffer_to_block(log_buffer_t *buf)
{
    if (buf->used < LOG_SD_BLOCK_SIZE) {
        memset(&buf->data[buf->used], 0xFF, LOG_SD_BLOCK_SIZE - buf->used);
        buf->used = LOG_SD_BLOCK_SIZE;
    }
}

static bool flush_active_buffer(void)
{
    log_buffer_t *buf = get_active_buffer();
    if (buf->used == 0U) {
        return true;
    }

    pad_buffer_to_block(buf);
    uint8_t flush_idx = g_log_ctx.active_idx;
    uint8_t next_idx = next_buffer_index(flush_idx);

    /* Ensure next buffer is available. */
    if (g_log_ctx.buffers[next_idx].used != 0U) {
        wait_for_dma_completion();
        g_log_ctx.buffers[next_idx].used = 0U;
    }

    g_log_ctx.active_idx = next_idx;
    g_log_ctx.buffers[next_idx].used = 0U;

    return start_dma_flush(flush_idx);
}

static bool start_dma_flush(uint8_t buffer_index)
{
    wait_for_dma_completion();

    if (!wait_for_card_ready(LOG_ERASE_TIMEOUT_MS)) {
        g_log_ctx.error = true;
        g_log_ctx.ready = false;
        return false;
    }

    HAL_StatusTypeDef status = HAL_SD_WriteBlocks_DMA(
        &hsd1,
        g_log_ctx.buffers[buffer_index].data,
        g_log_ctx.block_address,
        1U);

    if (status != HAL_OK) {
        g_log_ctx.error = true;
        g_log_ctx.ready = false;
        return false;
    }

    g_log_ctx.block_address += 1U;
    g_log_ctx.dma_in_progress = true;
    g_log_ctx.flushing_idx = buffer_index;
    /* Reset semaphore so wait blocks until ISR releases it. */
    xSemaphoreTake(g_log_ctx.dma_done_sem, 0);
    return true;
}

static void wait_for_dma_completion(void)
{
    if (g_log_ctx.dma_in_progress) {
        xSemaphoreTake(g_log_ctx.dma_done_sem, portMAX_DELAY);
        g_log_ctx.dma_in_progress = false;
        if (g_log_ctx.flushing_idx < LOG_BUFFER_COUNT) {
            g_log_ctx.buffers[g_log_ctx.flushing_idx].used = 0U;
        }
        g_log_ctx.flushing_idx = UINT8_MAX;
    }
}

static bool ensure_preflight_erase(void)
{
    if (s_preflight_erased) {
        return true;
    }

    HAL_SD_CardInfoTypeDef card_info = {0};
    if (HAL_SD_GetCardInfo(&hsd1, &card_info) != HAL_OK) {
        return false;
    }

    uint32_t blocks_to_erase = LOG_ERASE_BLOCK_COUNT;
    if (card_info.LogBlockNbr > 0U && blocks_to_erase > card_info.LogBlockNbr) {
        blocks_to_erase = card_info.LogBlockNbr;
    }

    if (blocks_to_erase == 0U) {
        return false;
    }

    if (HAL_SD_Erase(&hsd1, 0U, blocks_to_erase - 1U) != HAL_OK) {
        return false;
    }

    if (!wait_for_card_ready(LOG_ERASE_TIMEOUT_MS)) {
        return false;
    }

    s_preflight_erased = true;
    return true;
}

static bool wait_for_card_ready(uint32_t timeout_ms)
{
    const uint32_t poll_ms = 10U;
    uint32_t start = HAL_GetTick();

    while (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER) {
        if ((HAL_GetTick() - start) > timeout_ms) {
            return false;
        }

        if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
            vTaskDelay(pdMS_TO_TICKS(poll_ms));
        } else {
            HAL_Delay(poll_ms);
        }
    }

    return true;
}

static uint16_t crc16_ccitt_update(uint16_t crc, uint8_t data)
{
    crc ^= (uint16_t)data << 8;
    for (uint8_t i = 0; i < 8U; ++i) {
        if (crc & 0x8000U) {
            crc = (crc << 1U) ^ 0x1021U;
        } else {
            crc <<= 1U;
        }
    }
    return crc;
}

static uint16_t crc16_ccitt_compute(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFU;
    return crc16_ccitt_accumulate(crc, data, len);
}

static uint16_t crc16_ccitt_accumulate(uint16_t crc, const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; ++i) {
        crc = crc16_ccitt_update(crc, data[i]);
    }
    return crc;
}

void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd)
{
    if (hsd != &hsd1) {
        return;
    }

    BaseType_t higher_priority_task_woken = pdFALSE;
    g_log_ctx.dma_in_progress = false;
    if (g_log_ctx.flushing_idx < LOG_BUFFER_COUNT) {
        g_log_ctx.buffers[g_log_ctx.flushing_idx].used = 0U;
    }
    g_log_ctx.flushing_idx = UINT8_MAX;
    xSemaphoreGiveFromISR(g_log_ctx.dma_done_sem, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd)
{
    if (hsd != &hsd1) {
        return;
    }

    (void)HAL_SD_GetError(hsd);
    (void)HAL_SD_GetCardState(hsd);

    g_log_ctx.error = true;
    g_log_ctx.ready = false;
    BaseType_t higher_priority_task_woken = pdFALSE;
    g_log_ctx.dma_in_progress = false;
    g_log_ctx.flushing_idx = UINT8_MAX;
    xSemaphoreGiveFromISR(g_log_ctx.dma_done_sem, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}
