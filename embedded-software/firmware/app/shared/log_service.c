/**
 * @file    log_service.c
 * @brief   Staging-buffer service behind the messages-runtime SD sink.
 *
 * Phase-1 implementation is a byte counter (no actual ring yet) — the
 * real staging ring + sd_log_task drain lands when SD writing comes
 * online. The typed log_service_log_<type> APIs that used to live here
 * were removed: the message registry is now the single schema source,
 * and producers call PUB_<MODULE>_<NAME>(...) instead.
 *
 * UBC Rocket, 2026
 */

#include "app/log_service.h"
#include "io_sys/io_test_hooks.h"

#include <stdbool.h>
#include <string.h>

static bool s_ready;

/* Phase-1 placeholders for the raw-bytes sink. Real implementation
 * (ring buffer drained by sd_log_task) lands when SD writing comes
 * online. */
static volatile uint32_t s_raw_bytes_appended;
static volatile uint32_t s_raw_bytes_dropped;

IO_TEST_HOOK_RW(s_ready, bool, log_service_ready)

void log_service_init(void) {
    /* TODO: allocate per-core staging ring, wire CM7 → CM4 handoff. */
    s_ready = false;
    s_raw_bytes_appended = 0;
    s_raw_bytes_dropped = 0;
}

bool log_service_ready(void) { return s_ready; }
void log_service_mark_ready(void) { s_ready = true; }

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
