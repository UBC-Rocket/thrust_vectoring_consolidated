#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "SD_logging/log_writer.h"
#include "SD_logging/log_service.h"
#include "log_records/log_records.h"
#include "state_exchange.h"
#include "timestamp.h"

#ifdef DEBUG
#include "trace/trace_ringbuf.h"
#endif

/* ── Configuration ── */
#define SD_LOG_FLUSH_PERIOD_MS   50U   /* ~20 Hz flush rate */
#define SD_LOG_INIT_RETRY_MS     200U  /* Retry interval for log_writer_init */

#ifdef DEBUG
#define TRACE_FLUSH_BATCH_MAX    32U
#endif

extern bool g_sd_card_initialized;

/* ── Flight header state ── */
static bool s_flight_header_logged = false;
static uint32_t s_flight_magic = 0U;
static uint32_t s_flight_counter = 0U;

/* ── Flight state tracking ── */
static flight_state_t s_last_logged_flight_state = IDLE;

static uint32_t next_flight_magic(void)
{
    uint32_t seed = HAL_GetTick() ^ 0x5A5AA5A5U;
    if (seed == 0U) {
        seed = 0xA5A5A5A5U;
    }
    return seed;
}

static void log_flight_header_if_ready(uint32_t timestamp_us_val)
{
    if (s_flight_header_logged || !log_service_ready()) {
        return;
    }

    if (s_flight_magic == 0U) {
        s_flight_magic = next_flight_magic();
        s_flight_counter = HAL_GetTick();
    }

    log_service_log_flight_header(&(log_record_flight_header_t){
        .timestamp_us = timestamp_us_val,
        .flight_magic = s_flight_magic,
        .flight_counter = s_flight_counter,
    });
    s_flight_header_logged = true;
}

static void log_flight_state_if_changed(flight_state_t flight_state,
                                        uint32_t ts)
{
    if (flight_state == s_last_logged_flight_state) {
        return;
    }

    log_service_log_event(&(log_record_event_t){
        .timestamp_us = ts,
        .event_code = LOG_EVENT_CODE_FLIGHT_STATE,
        .data_u16 = (uint16_t)flight_state,
    });
    s_last_logged_flight_state = flight_state;
}

#ifdef DEBUG
static void drain_trace_ringbuf(void)
{
    if (!log_writer_ready()) {
        return;
    }

    trace_event_t batch[TRACE_FLUSH_BATCH_MAX];
    uint8_t count = 0;

    while (count < TRACE_FLUSH_BATCH_MAX && trace_ringbuf_pop(&batch[count])) {
        count++;
    }

    if (count > 0) {
        log_record_trace_batch_t hdr = {
            .base_timestamp_us = batch[0].timestamp_us,
            .event_count       = count,
            .reserved1         = 0,
            .reserved2         = 0,
        };

        uint8_t payload[sizeof(hdr) + TRACE_FLUSH_BATCH_MAX * sizeof(trace_event_t)];
        memcpy(payload, &hdr, sizeof(hdr));
        memcpy(payload + sizeof(hdr), batch, count * sizeof(trace_event_t));

        log_writer_append_record(LOG_RECORD_TYPE_trace_batch,
                                 payload,
                                 sizeof(hdr) + count * sizeof(trace_event_t));
    }

    uint32_t dropped = trace_ringbuf_take_dropped();
    if (dropped > 0) {
        log_record_trace_overflow_t ovf = {
            .timestamp_us  = timestamp_us(),
            .dropped_count = dropped,
        };
        log_writer_append_record(LOG_RECORD_TYPE_trace_overflow,
                                 &ovf, sizeof(ovf));
    }
}
#endif /* DEBUG */

void sd_log_task_start(void *argument)
{
    (void)argument;

    /* ── One-time init: wait for log_writer to be ready ── */
    while (!g_sd_card_initialized) {
        vTaskDelay(pdMS_TO_TICKS(SD_LOG_INIT_RETRY_MS));
    }

    while (!log_writer_init()) {
        vTaskDelay(pdMS_TO_TICKS(SD_LOG_INIT_RETRY_MS));
    }

    /* Mark log service as initialised so other tasks' log calls succeed */
    log_service_mark_ready();

    /* ── Main loop ── */
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(SD_LOG_FLUSH_PERIOD_MS));

        /* Log flight header once */
        uint32_t ts = timestamp_us();
        log_flight_header_if_ready(ts);

        /* Track flight state changes */
        flight_state_t fs = IDLE;
        state_exchange_get_flight_state(&fs);
        log_flight_state_if_changed(fs, ts);

#ifdef DEBUG
        /* Drain FreeRTOS trace events */
        drain_trace_ringbuf();
#endif

        /* Flush pending data to SD card */
        log_writer_flush();
    }
}
