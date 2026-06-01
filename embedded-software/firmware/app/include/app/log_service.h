/**
 * @file    log_service.h
 * @brief   Per-core staging buffer behind the messages-runtime SD sink.
 *
 * The old typed-record APIs (one log_service_log_<name> per LOG_RECORD_LIST
 * entry) were removed when the message registry became the single schema
 * source — log lines are now PUB_<MODULE>_<NAME>(...) calls (see
 * generated/messages/publish.h), which the messages runtime hands here as
 * already-framed envelope bytes via the sink installed by
 * messages_sd_sink_set().
 *
 * Two cores, two backings:
 *   - CM4: appends go into a CM4-local byte buffer that sd_log_task drains
 *     directly in-task.
 *   - CM7: appends are pushed into a lock-free SPSC byte ring in SRAM4
 *     (APP_SLOT_LOG_HANDOFF_RING_*). sd_log_task on CM4 drains the same
 *     ring through log_service_drain_cm7_ring(). CM7 raises
 *     IO_IC_LOG_BUFFER_CM7_READY (HSEM) to wake CM4 when the ring crosses
 *     a fill threshold.
 *
 * UBC Rocket, 2026
 */
#ifndef APP_LOG_SERVICE_H
#define APP_LOG_SERVICE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Initialise the local staging buffer and handoff machinery. Must be called
 *  once per core, after io_intercore_init (CM7 also clears the SRAM4 ring
 *  header here on first boot). */
void log_service_init(void);

bool log_service_ready(void);
void log_service_mark_ready(void);

/**
 * @brief Raw-bytes sink used by the structured messages runtime.
 *
 * The messages library hands fully-assembled envelopes (length + class +
 * module_id + msg_id + t_us + payload + crc16) to this function via the
 * sink installed by messages_sd_sink_set().
 *
 * Returns true on accept, false if the staging buffer / ring is full
 * (messages runtime then bumps the per-channel drop counter; this function
 * also bumps an internal byte-level drop counter so the bytes-dropped
 * metric is reported on both boundaries).
 */
bool log_service_append_raw(const uint8_t *bytes, uint32_t len);

/** Cumulative bytes successfully appended via log_service_append_raw. */
uint32_t log_service_raw_bytes_appended(void);

/** Cumulative bytes dropped at log_service_append_raw (sink full). */
uint32_t log_service_raw_bytes_dropped(void);

/* -------------------------------------------------------------------------- */
/* CM4 drain APIs — used by sd_log_task to feed the SD writer.                */
/* -------------------------------------------------------------------------- */

/**
 * @brief (CM4 only) Drain bytes from the CM4-local staging buffer.
 *
 * Copies up to @p max bytes into @p out, advances the local tail, and
 * returns the number of bytes copied (0 if the local buffer is empty).
 *
 * On cores other than CM4 this is a no-op that returns 0.
 */
size_t log_service_drain_local(uint8_t *out, size_t max);

/**
 * @brief (CM4 only) Drain bytes from the CM7→CM4 SRAM4 SPSC ring.
 *
 * The ring is set up by CM7's log_service_init(); on CM4 this function
 * simply pops bytes from the consumer side. Copies up to @p max bytes,
 * advances the tail in shared memory, and returns the byte count.
 *
 * On cores other than CM4 this is a no-op that returns 0.
 */
size_t log_service_drain_cm7_ring(uint8_t *out, size_t max);

/**
 * @brief (CM4 only) Register a wake callback fired from the HSEM IRQ when
 *        CM7 signals IO_IC_LOG_BUFFER_CM7_READY.
 *
 * The callback runs in interrupt context (HSEM IRQ); the typical
 * implementation does xTaskNotifyGiveFromISR + portYIELD_FROM_ISR to wake
 * sd_log_task. log_service.c stays FreeRTOS-free; the caller handles the
 * RTOS-specific wake.
 *
 * @param cb Wake callback (NULL to clear).
 */
void log_service_set_wake_callback(void (*cb)(void));

/* -------------------------------------------------------------------------- */
/* CM7→CM4 ring statistics (readable from either core).                       */
/* -------------------------------------------------------------------------- */

/** Bytes the CM7 producer successfully pushed into the SRAM4 ring. */
uint32_t log_service_cm7_ring_bytes_pushed(void);

/** Bytes the CM7 producer dropped because the ring was full. */
uint32_t log_service_cm7_ring_bytes_dropped(void);

/** Current fill level (head - tail mod capacity) in bytes. */
uint32_t log_service_cm7_ring_fill(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_LOG_SERVICE_H */
