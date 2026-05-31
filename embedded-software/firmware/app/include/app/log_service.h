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
 * This service owns the staging buffer the sd_log_task drains onto the
 * SD card. Phase-1 implementation just counts bytes appended + drops;
 * the real ring lands when SD writing comes online.
 *
 * UBC Rocket, 2026
 */
#ifndef APP_LOG_SERVICE_H
#define APP_LOG_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Initialise the local staging buffer and handoff machinery. */
void log_service_init(void);

bool log_service_ready(void);
void log_service_mark_ready(void);

/**
 * @brief Raw-bytes sink used by the structured messages runtime.
 *
 * The messages library hands fully-assembled envelopes (length + class +
 * module_id + msg_id + t_us + payload + crc16) to this function via the
 * sink installed by messages_sd_sink_set(). Phase-1 implementation just
 * counts bytes appended + drops; the real SD path lands when sd_log_task
 * grows a body. Returns true on accept, false if the staging buffer is
 * full (messages runtime then bumps the per-channel drop counter).
 */
bool log_service_append_raw(const uint8_t *bytes, uint32_t len);

/** Cumulative bytes successfully appended via log_service_append_raw. */
uint32_t log_service_raw_bytes_appended(void);

/** Cumulative bytes dropped at log_service_append_raw (sink full). */
uint32_t log_service_raw_bytes_dropped(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_LOG_SERVICE_H */
