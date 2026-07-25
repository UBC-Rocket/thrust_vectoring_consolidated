#ifndef LOG_WRITER_H
#define LOG_WRITER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "log_records/log_records.h"

#define LOG_SD_BLOCK_SIZE   512U
#define LOG_BUFFER_COUNT    10U
#define LOG_BUF_LOG_START   0U   /* Buffers [0..3] reserved for regular log records */
#define LOG_BUF_LOG_END     4U
#define LOG_BUF_TRACE_START 4U   /* Buffers [4..9] reserved for trace events */
#define LOG_BUF_TRACE_END   10U

typedef enum {
    BUF_EMPTY = 0,
    BUF_ACTIVE_LOG,
    BUF_ACTIVE_TRACE,
    BUF_READY,
    BUF_FLUSHING,
} log_buf_state_t;

typedef struct {
    uint8_t data[LOG_SD_BLOCK_SIZE] __attribute__((aligned(4)));
    volatile size_t write_pos;
    volatile log_buf_state_t state;
} log_buffer_t;

bool log_writer_init(void);
bool log_writer_ready(void);

/**
 * @brief Append a typed record to the log stream (task context, takes mutex).
 */
bool log_writer_append_record(log_record_type_t type,
                              const void *payload,
                              size_t payload_size);

/**
 * @brief Force partial log buffer into submit queue. Non-blocking.
 */
bool log_writer_flush(void);

/* ── Trace buffer API (lock-free, safe from scheduler critical section) ── */

/**
 * @brief Claim an empty buffer as the active trace buffer.
 * Called once at startup from the flush task. Returns NULL if no buffer free.
 */
log_buffer_t *log_writer_claim_trace_buffer(void);

/**
 * @brief Submit a full trace buffer and claim the next empty one.
 * Called from trace hooks (scheduler critical section). Lock-free.
 *
 * @param filled  The buffer that is full (will be set to BUF_READY).
 * @return        The next empty buffer (set to BUF_ACTIVE_TRACE), or NULL if
 *                all buffers are busy (caller should drop events).
 */
log_buffer_t *log_writer_swap_trace_buffer(log_buffer_t *filled);

/* ── Submit queue API ── */

/**
 * @brief Push a buffer index into the DMA submit queue. Lock-free.
 * Called by log_writer_append_record (log buffers) and trace hooks (trace buffers).
 * @return true if enqueued, false if queue full.
 */
bool log_writer_submit_enqueue(uint8_t buf_idx);

/**
 * @brief Pop the next buffer index from the DMA submit queue.
 * Called only from the flush task (single consumer).
 * @param[out] buf_idx  Dequeued buffer index.
 * @return true if dequeued, false if empty.
 */
bool log_writer_submit_dequeue(uint8_t *buf_idx);

/**
 * @brief Get buffer by index.
 */
log_buffer_t *log_writer_get_buffer(uint8_t idx);

/**
 * @brief Get total buffer count.
 */
uint8_t log_writer_buffer_count(void);

/**
 * @brief Issue a single-block DMA write. Called only from flush task.
 * Blocks until DMA completion.
 * @param buf_idx  Buffer index to write.
 * @return true on success.
 */
bool log_writer_dma_write_block(uint8_t buf_idx);

/**
 * @brief Set the flush task handle so the DMA ISR can wake it.
 * Called once from the flush task at startup.
 */
void log_writer_set_flush_task(void *task_handle);

#endif /* LOG_WRITER_H */
