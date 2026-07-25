/**
 * @file debug_uart.c
 * @brief Debug UART implementation - bidirectional injection and logging
 *
 * Provides message injection (RX) and system logging (TX) via UART1.
 * Only compiled when DEBUG macro is defined.
 */

#ifdef DEBUG

#include "debug_uart.h"
#include "spi_slave.h"
#include "lwgps/lwgps.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <ctype.h>

/* ============================================================================
 * External UART Handle
 * ============================================================================ */

extern UART_HandleTypeDef huart1;

/* ============================================================================
 * Module Variables
 * ============================================================================ */

/** Log message queue */
static debug_log_queue_t log_queue;

/** DMA TX in progress flag — set before HAL_UART_Transmit_DMA, cleared in TxCplt callback */
static volatile bool s_dma_tx_busy = false;

/** Tick when current DMA TX started (for timeout recovery) */
static uint32_t s_dma_start_tick = 0;

/** Diagnostic: number of DMA TX timeout recoveries */
static uint32_t s_dma_tx_timeouts = 0;

/** Diagnostic: number of messages skipped due to persistent DMA failures */
static uint32_t s_dma_tx_skipped = 0;

/** Consecutive DMA start failures on current tail message */
static uint8_t s_dma_fail_count = 0;

/** Max consecutive DMA failures before skipping a message */
#define DMA_TX_MAX_RETRIES 5

/** Radio message queue pointer (for injection) */
static radio_message_queue_t *radio_queue = NULL;

/** GPS NMEA queue pointer (for injection) */
static gps_sample_queue_t *gps_queue = NULL;

/** lwgps parser instance for injection parsing */
static lwgps_t s_debug_lwgps;

/* ============================================================================
 * Internal Function Prototypes
 * ============================================================================ */

static void process_radio_injection(const uint8_t *payload, uint16_t len);
static void process_gps_injection(const uint8_t *payload, uint16_t len);
static void log_enqueue(const char *format, ...);
static bool log_queue_full(void);
static bool log_queue_empty(void);
static void format_hex_ascii(char *out, size_t out_size, const uint8_t *data, uint16_t len);

/* ============================================================================
 * Public API - Initialization
 * ============================================================================ */

void debug_uart_init(radio_message_queue_t *radio_q, gps_sample_queue_t *gps_q)
{
    radio_queue = radio_q;
    gps_queue = gps_q;

    log_queue.head = 0;
    log_queue.tail = 0;

    lwgps_init(&s_debug_lwgps);
    /* USART1 DMA reception is started by uart_callbacks_init() */
}

/* ============================================================================
 * Public API - Injection (RX)
 * ============================================================================ */

void debug_uart_rx_line_callback(const uint8_t *data, uint16_t len, uint8_t type)
{
    if (type == DEBUG_UART_TYPE_RADIO) {
        process_radio_injection(data, len);
    } else if (type == DEBUG_UART_TYPE_GPS) {
        process_gps_injection(data, len);
    }
}

/* ============================================================================
 * Public API - Logging (TX)
 * ============================================================================ */

void debug_uart_log_radio(const uint8_t *msg, uint16_t len)
{
    if (msg == NULL || len == 0) {
        return;
    }

    char hex_ascii[128];
    format_hex_ascii(hex_ascii, sizeof(hex_ascii), msg, len);
    log_enqueue("[RADIO RX] %s\r\n", hex_ascii);
}

void debug_uart_log_gps_nmea(const char *nmea)
{
    if (nmea == NULL) return;
    log_enqueue("[GPS NMEA] %s", nmea);
}

void debug_uart_log_gps_fix(const gps_fix_t *fix)
{
    if (fix == NULL) {
        return;
    }

    const char *q_str;
    switch (fix->fix_quality) {
        case 0:  q_str = "No Fix"; break;
        case 1:  q_str = "GPS";    break;
        case 2:  q_str = "DGPS";   break;
        case 3:  q_str = "PPS";    break;
        default: q_str = "?";      break;
    }

    uint8_t hrs = (uint8_t)(fix->time_of_week_ms / 3600000u);
    uint8_t min = (uint8_t)((fix->time_of_week_ms % 3600000u) / 60000u);
    uint8_t sec = (uint8_t)((fix->time_of_week_ms % 60000u) / 1000u);

    log_enqueue("[GPS FIX] Fix: %u (%s) | Sats: %u | HDOP: %.1f\r\n"
                "          Lat: %.6f | Lon: %.6f | Alt: %.1fm\r\n"
                "          Spd: %.1fm/s | Crs: %.1f | Time: %02u:%02u:%02u UTC\r\n",
                fix->fix_quality, q_str, fix->num_satellites, (double)fix->hdop,
                fix->latitude, fix->longitude, (double)fix->altitude_msl,
                (double)fix->ground_speed, (double)fix->course,
                hrs, min, sec);
}

void debug_uart_log_gps_no_fix(uint8_t fix_status, uint8_t sats)
{
    log_enqueue("[GPS NOFIX] fix=%u sats=%u\r\n", fix_status, sats);
}

void debug_uart_log_spi_radio_tx(const uint8_t *msg, uint16_t len)
{
    if (msg == NULL || len == 0) {
        return;
    }

    char hex_ascii[128];
    format_hex_ascii(hex_ascii, sizeof(hex_ascii), msg, len);
    log_enqueue("[SPI TX] Radio msg from master: %s\r\n", hex_ascii);
}

void debug_uart_log_spi_radio_read(const uint8_t *msg, uint16_t len)
{
    if (msg == NULL || len == 0) {
        return;
    }

    char hex_ascii[128];
    format_hex_ascii(hex_ascii, sizeof(hex_ascii), msg, len);
    log_enqueue("[SPI->M] Radio: %s\r\n", hex_ascii);
}

void debug_uart_log_spi_gps_read(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0) {
        return;
    }

    // Print as string (NMEA sentences are ASCII)
    log_enqueue("[SPI->M] GPS: %.*s\r\n", (int)len, (const char *)data);
}

void debug_uart_log_spi_buflen(uint8_t count)
{
    log_enqueue("[SPI->M] BufLen: %u\r\n", count);
}

void debug_uart_process_logs(void)
{
    if (s_dma_tx_busy) {
        /* Fast recovery: callback already ran, gState is READY */
        if (huart1.gState == HAL_UART_STATE_READY) {
            s_dma_tx_busy = false;
        }
        /* Timeout recovery: DMA chain stuck (gState still BUSY_TX).
           Longest message (256 bytes) at 115200 baud = ~22ms.
           50ms timeout gives generous margin. */
        else if (HAL_GetTick() - s_dma_start_tick > 50) {
            HAL_UART_AbortTransmit(&huart1);
            log_queue.tail = (log_queue.tail + 1) % DEBUG_UART_LOG_QUEUE_DEPTH;
            s_dma_tx_busy = false;
            s_dma_tx_timeouts++;
        }
        return;
    }

    if (log_queue_empty()) {
        return;
    }

    const char *msg = log_queue.messages[log_queue.tail];
    uint16_t msg_len = strlen(msg);

    if (msg_len == 0) {
        log_queue.tail = (log_queue.tail + 1) % DEBUG_UART_LOG_QUEUE_DEPTH;
        return;
    }

    s_dma_tx_busy = true;
    s_dma_start_tick = HAL_GetTick();
    if (HAL_UART_Transmit_DMA(&huart1, (const uint8_t *)msg, msg_len) != HAL_OK) {
        s_dma_tx_busy = false;
        s_dma_fail_count++;
        if (s_dma_fail_count >= DMA_TX_MAX_RETRIES) {
            /* Persistent failure — skip this message to unblock the queue */
            log_queue.tail = (log_queue.tail + 1) % DEBUG_UART_LOG_QUEUE_DEPTH;
            s_dma_fail_count = 0;
            s_dma_tx_skipped++;
        }
    } else {
        s_dma_fail_count = 0;
    }
}

void debug_uart_dma_tx_cplt(void)
{
    log_queue.tail = (log_queue.tail + 1) % DEBUG_UART_LOG_QUEUE_DEPTH;
    s_dma_tx_busy = false;
}

/* ============================================================================
 * Internal Functions - Injection
 * ============================================================================ */

static void process_radio_injection(const uint8_t *payload, uint16_t len)
{
    if (radio_queue == NULL || payload == NULL || len == 0) {
        return;
    }

    // Find null terminator
    uint16_t msg_len = 0;
    for (uint16_t i = 0; i < len; i++) {
        if (payload[i] == 0x00) {
            msg_len = i;
            break;
        }
    }

    // If no null terminator found, use entire payload
    if (msg_len == 0) {
        msg_len = len;
    }

    // Validate message length (must fit in radio queue)
    if (msg_len > RADIO_MESSAGE_MAX_LEN) {
        return; // Too long
    }

    // Create temporary buffer with message
    uint8_t msg_buffer[RADIO_MESSAGE_MAX_LEN];
    memcpy(msg_buffer, payload, msg_len);

    // Enqueue to radio RX queue
    radio_message_enqueue(msg_len, msg_buffer, radio_queue);

    // Log the injected message
    debug_uart_log_radio(msg_buffer, msg_len);
}

static void process_gps_injection(const uint8_t *payload, uint16_t len)
{
    if (gps_queue == NULL || payload == NULL || len == 0) {
        return;
    }

    // Validate NMEA format (should start with '$')
    if (payload[0] != '$') {
        return; // Invalid NMEA
    }

    // Validate length (max 82 chars per NMEA spec, need room for \r\n)
    if (len > 80 || (len + 2) > GPS_SAMPLE_SIZE) {
        return; // Too long
    }

    // Create buffer, copy payload, append \r\n as per NMEA spec
    // (CM handler strips \r\n, but upstream parser needs the delimiters)
    uint8_t gps_buffer[GPS_SAMPLE_SIZE];
    memset(gps_buffer, 0, GPS_SAMPLE_SIZE);
    memcpy(gps_buffer, payload, len);
    gps_buffer[len] = '\r';
    gps_buffer[len + 1] = '\n';

    // Enqueue to GPS NMEA queue
    gps_sample_enqueue(gps_buffer, gps_queue);

    // Log the injected GPS sentence
    debug_uart_log_gps_nmea((const char *)payload);

    // Parse the injected NMEA with lwgps and log the result
    lwgps_process(&s_debug_lwgps, gps_buffer, len + 2);

    gps_fix_t injected_fix;
    memset(&injected_fix, 0, sizeof(injected_fix));
    injected_fix.latitude      = s_debug_lwgps.latitude;
    injected_fix.longitude     = s_debug_lwgps.longitude;
    injected_fix.altitude_msl  = s_debug_lwgps.altitude;
    injected_fix.ground_speed  = s_debug_lwgps.speed * 0.514444f;
    injected_fix.course        = s_debug_lwgps.course;
    injected_fix.fix_quality   = (uint8_t)s_debug_lwgps.fix;
    injected_fix.num_satellites = s_debug_lwgps.sats_in_use;
    injected_fix.hdop          = s_debug_lwgps.dop_h;
    injected_fix.time_of_week_ms = ((uint32_t)s_debug_lwgps.hours * 3600u +
                                     (uint32_t)s_debug_lwgps.minutes * 60u +
                                     (uint32_t)s_debug_lwgps.seconds) * 1000u;
    debug_uart_log_gps_fix(&injected_fix);
}

/* ============================================================================
 * Internal Functions - Logging
 * ============================================================================ */

static void log_enqueue(const char *format, ...)
{
    /* Reserve slot atomically — ISR callers and main loop callers can't collide */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (log_queue_full()) {
        __set_PRIMASK(primask);
        return;
    }

    uint8_t slot = log_queue.head;
    log_queue.head = (slot + 1) % DEBUG_UART_LOG_QUEUE_DEPTH;
    __set_PRIMASK(primask);

    /* Format into reserved slot — safe, no other writer can touch this slot */
    va_list args;
    va_start(args, format);
    vsnprintf(log_queue.messages[slot], DEBUG_UART_LOG_MSG_SIZE, format, args);
    va_end(args);
}

static bool log_queue_full(void)
{
    return ((log_queue.head + 1) % DEBUG_UART_LOG_QUEUE_DEPTH) == log_queue.tail;
}

static bool log_queue_empty(void)
{
    return log_queue.head == log_queue.tail;
}

/* ============================================================================
 * Internal Functions - Utilities
 * ============================================================================ */

static void format_hex_ascii(char *out, size_t out_size, const uint8_t *data, uint16_t len)
{
    if (out == NULL || data == NULL || len == 0 || out_size < 10) {
        if (out != NULL && out_size > 0) {
            out[0] = '\0';
        }
        return;
    }

    size_t pos = 0;

    // Format hex bytes (limit to prevent overflow)
    uint16_t hex_limit = (len < 16) ? len : 16;
    for (uint16_t i = 0; i < hex_limit && pos < out_size - 20; i++) {
        pos += snprintf(&out[pos], out_size - pos, "%02X ", data[i]);
    }

    if (len > hex_limit) {
        pos += snprintf(&out[pos], out_size - pos, "... ");
    }

    // Add ASCII representation if printable
    pos += snprintf(&out[pos], out_size - pos, "(");
    bool has_printable = false;
    for (uint16_t i = 0; i < hex_limit && pos < out_size - 10; i++) {
        if (isprint(data[i])) {
            out[pos++] = (char)data[i];
            has_printable = true;
        }
    }

    if (!has_printable) {
        pos += snprintf(&out[pos], out_size - pos, "binary");
    }

    snprintf(&out[pos], out_size - pos, ")");
}

/* ============================================================================
 * Public API - Push Mode Debug Logging
 * ============================================================================ */

void debug_uart_log_spi_push_event(const char *event, uint8_t type,
                                    uint16_t tx_sent, uint16_t rx_received)
{
    log_enqueue("[SPI PUSH] %s type=0x%02X tx=%u rx=%u\r\n",
                event, type, tx_sent, rx_received);
}

void debug_uart_log_spi_idle_arm(void)
{
    log_enqueue("[SPI IDLE] Armed for master TX capture\r\n");
}

void debug_uart_log_spi_push_arm(uint8_t type, uint16_t len)
{
    log_enqueue("[SPI PUSH] Armed type=0x%02X len=%u\r\n", type, len);
}

/* ============================================================================
 * Public API - SPI Debug Logging
 * ============================================================================ */

void debug_uart_log_spi_stats(uint32_t push_txns, uint32_t total_txns,
                               uint32_t incomplete, uint32_t master_tx,
                               uint32_t overruns)
{
    log_enqueue("[SPI STATS] push=%lu txn=%lu inc=%lu mtx=%lu ovr=%lu dto=%lu dsk=%lu\r\n",
                (unsigned long)push_txns,
                (unsigned long)total_txns,
                (unsigned long)incomplete,
                (unsigned long)master_tx,
                (unsigned long)overruns,
                (unsigned long)s_dma_tx_timeouts,
                (unsigned long)s_dma_tx_skipped);
}

void debug_uart_log_spi_nss_event(bool falling, uint8_t state)
{
    log_enqueue("[SPI NSS] %s state=%u\r\n", falling ? "FALL" : "RISE", state);
}

void debug_uart_log_radio_diag(uint32_t cm_events, uint32_t bytes_fed,
                                uint32_t msgs_enqueued, uint32_t uart_errors,
                                uint32_t uart_cr1, uint32_t uart_cr3,
                                uint32_t uart_isr, uint16_t dma_cndtr)
{
    log_enqueue("[RADIO DIAG] cm=%lu bytes=%lu msgs=%lu err=%lu\r\n"
                "  CR1=%04lX CR3=%04lX ISR=%04lX CNDTR=%u\r\n",
                (unsigned long)cm_events, (unsigned long)bytes_fed,
                (unsigned long)msgs_enqueued, (unsigned long)uart_errors,
                (unsigned long)uart_cr1, (unsigned long)uart_cr3,
                (unsigned long)uart_isr, (unsigned)dma_cndtr);
}

void debug_uart_log_spi_arm(const spi_debug_capture_t *dbg)
{
    if (dbg == NULL) return;

    log_enqueue("[SPI ARM#%lu] pre_SR=%04lX post_SR=%04lX CR2=%04lX\r\n"
                "  TXCN=%u CMAR=%08lX DMX=%02lX/%02lX\r\n",
                (unsigned long)dbg->arm_count,
                (unsigned long)dbg->pre_arm_sr,
                (unsigned long)dbg->arm_sr,
                (unsigned long)dbg->arm_cr2,
                (unsigned)dbg->arm_tx_cndtr,
                (unsigned long)dbg->arm_tx_cmar,
                (unsigned long)dbg->arm_dmamux0,
                (unsigned long)dbg->arm_dmamux1);
}

void debug_uart_log_spi_txn(const spi_debug_capture_t *dbg)
{
    if (dbg == NULL) return;

    log_enqueue("[SPI ISR#%lu] cmd=%02X SR=%04lX TXCN=%u\r\n"
                "  tx: %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                (unsigned long)dbg->isr_count,
                (unsigned)dbg->isr_cmd,
                (unsigned long)dbg->isr_sr,
                (unsigned)dbg->isr_tx_cndtr,
                dbg->isr_tx_snap[0], dbg->isr_tx_snap[1],
                dbg->isr_tx_snap[2], dbg->isr_tx_snap[3],
                dbg->isr_tx_snap[4], dbg->isr_tx_snap[5],
                dbg->isr_tx_snap[6], dbg->isr_tx_snap[7]);

    log_enqueue("[SPI EXTI#%lu] SR=%04lX TXCN=%u RXCN=%u\r\n"
                "  tx: %02X %02X %02X %02X %02X %02X %02X %02X\r\n"
                "  rx: %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                (unsigned long)dbg->exti_count,
                (unsigned long)dbg->exti_sr,
                (unsigned)dbg->exti_tx_cndtr,
                (unsigned)dbg->exti_rx_cndtr,
                dbg->exti_tx_snap[0], dbg->exti_tx_snap[1],
                dbg->exti_tx_snap[2], dbg->exti_tx_snap[3],
                dbg->exti_tx_snap[4], dbg->exti_tx_snap[5],
                dbg->exti_tx_snap[6], dbg->exti_tx_snap[7],
                dbg->exti_rx_snap[0], dbg->exti_rx_snap[1],
                dbg->exti_rx_snap[2], dbg->exti_rx_snap[3],
                dbg->exti_rx_snap[4], dbg->exti_rx_snap[5],
                dbg->exti_rx_snap[6], dbg->exti_rx_snap[7]);

    log_enqueue("[SPI ARM#%lu] pre_SR=%04lX post_SR=%04lX CR2=%04lX\r\n"
                "  TXCN=%u CMAR=%08lX DMX=%02lX/%02lX\r\n",
                (unsigned long)dbg->arm_count,
                (unsigned long)dbg->pre_arm_sr,
                (unsigned long)dbg->arm_sr,
                (unsigned long)dbg->arm_cr2,
                (unsigned)dbg->arm_tx_cndtr,
                (unsigned long)dbg->arm_tx_cmar,
                (unsigned long)dbg->arm_dmamux0,
                (unsigned long)dbg->arm_dmamux1);
}

#endif // DEBUG
