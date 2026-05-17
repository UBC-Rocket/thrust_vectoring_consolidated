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
