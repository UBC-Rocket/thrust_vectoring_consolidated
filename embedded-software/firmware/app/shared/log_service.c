/**
 * @file    log_service.c
 * @brief   Per-core record producer — skeleton pending adaptation from the
 *          deprecated H5 implementation.
 *
 * The deprecated CM4-only version used a FreeRTOS queue into a single SD log
 * task. The new dual-core version keeps the same public API but backs the
 * producer with a per-core staging ring that the SD log task drains on CM4.
 * CM7 handoff is layered on top via the log-handoff intercore slot + HSEM.
 *
 * UBC Rocket, 2026
 */

#include "app/log_service.h"
#include "io_sys/io_test_hooks.h"

#include <stdbool.h>
#include <string.h>

static bool s_ready;

/* Phase-1 placeholders for the raw-bytes sink. Real implementation
 * (ring buffer drained by sd_log_task) lands when SD writing comes online. */
static volatile uint32_t s_raw_bytes_appended;
static volatile uint32_t s_raw_bytes_dropped;

IO_TEST_HOOK_RW(s_ready, bool, log_service_ready)

void log_service_init(void) {
    /* TODO: allocate per-core staging ring, wire CM7 → CM4 handoff. */
    s_ready = false;
}

bool log_service_ready(void) { return s_ready; }
void log_service_mark_ready(void) { s_ready = true; }

/* Default: every typed log function is a no-op until the staging ring is
 * wired. This lets the rest of the project link and run before the full
 * logging pipeline is migrated. */
#define APP_LOG_DEFINE_FN_(id, name, fields, enable)                       \
    void log_service_log_##name(const log_record_##name##_t *record) {     \
        (void)record;                                                      \
    }
LOG_RECORD_LIST(APP_LOG_DEFINE_FN_)
#undef APP_LOG_DEFINE_FN_

void log_service_log_state(const state_t *state, app_flight_state_t flight_state) {
    (void)state;
    (void)flight_state;
}

bool log_service_append_raw(const uint8_t *bytes, uint32_t len) {
    (void)bytes;
    /* Phase-1: accept everything; just count bytes so the messages publish
     * path exercises end-to-end. When the real staging ring lands the
     * memcpy + ring write goes here, and the bool return reflects whether
     * the ring had room. */
    s_raw_bytes_appended += len;
    return true;
}

uint32_t log_service_raw_bytes_appended(void) { return s_raw_bytes_appended; }
uint32_t log_service_raw_bytes_dropped(void)  { return s_raw_bytes_dropped; }
