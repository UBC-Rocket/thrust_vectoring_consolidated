/**
 * @file    log_service.h
 * @brief   Per-core record producer for the unified SD log.
 *
 * On CM4 the service pushes records directly into the SD log task's staging
 * buffer. On CM7 records are pushed into a CM7-local staging buffer that is
 * handed off to CM4 via the intercore log-buffer slot + HSEM notification.
 *
 * UBC Rocket, 2026
 */
#ifndef APP_LOG_SERVICE_H
#define APP_LOG_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#include "log_records/log_records.h"
#include "state_estimation/state.h"
#include "app/state_exchange.h"

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

/* One log function per record type, auto-generated from LOG_RECORD_LIST. */
#define APP_LOG_DECLARE_FN_(id, name, fields, enable) \
    void log_service_log_##name(const log_record_##name##_t *record);
LOG_RECORD_LIST(APP_LOG_DECLARE_FN_)
#undef APP_LOG_DECLARE_FN_

/**
 * @brief Convenience: map state_t + flight_state to a state-snapshot record
 *        and enqueue it.
 */
void log_service_log_state(const state_t *state, app_flight_state_t flight_state);

#ifdef __cplusplus
}
#endif

#endif /* APP_LOG_SERVICE_H */
