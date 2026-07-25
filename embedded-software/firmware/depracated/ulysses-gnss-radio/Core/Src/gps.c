/**
 * @file gps.c
 * @brief GPS Driver using DMA circular + Character Match on '\n'
 *
 * Receives NMEA sentences from GPS module via UART6 using DMA in circular mode.
 * Interrupts only on Character Match ('\n') — fires once per complete NMEA sentence.
 * This matches the USART1 and USART5 CM pattern used throughout the project.
 *
 * Author: Ernie Han (original), refactored for DMA by UBC Rocket
 */

#include "gps.h"
#include "gps_nema_queue.h"
#include "gps_fix_queue.h"
#include "protocol_config.h"
#include "lwgps/lwgps.h"
#include <stdbool.h>
#include <string.h>
#ifdef DEBUG
#include "debug_uart.h"
#endif

/* ============================================================================
 * Configuration
 * ============================================================================ */

#define GPS_DMA_BUF_SIZE    256u   /**< DMA circular buffer size (power of 2) */
#define GPS_NMEA_MAX        87u    /**< Max NMEA sentence length (82 chars + safety margin) */

#if (GPS_DMA_BUF_SIZE & (GPS_DMA_BUF_SIZE - 1u))
#error "GPS_DMA_BUF_SIZE must be power of two"
#endif

/* ============================================================================
 * Private State
 * ============================================================================ */

static UART_HandleTypeDef *s_gps = NULL;
static UART_HandleTypeDef *s_out = NULL;

/** DMA circular receive buffer */
static uint8_t s_dma_buf[GPS_DMA_BUF_SIZE];

/** Last processed position in DMA buffer */
static volatile uint16_t s_last_pos = 0;

/** NMEA sentence builder */
static uint8_t s_line[GPS_NMEA_MAX];
static uint16_t s_line_len = 0;
static bool s_in_sentence = false;

/** Shared GPS queue for SPI push/pull mode (raw NMEA) */
static gps_sample_queue_t *s_gps_queue = NULL;

/** Shared GPS fix queue for push mode (parsed fixes) */
static gps_fix_queue_t *s_gps_fix_queue = NULL;

/** lwgps parser instance */
static lwgps_t s_lwgps;

/** Parsed fix for push mode */
static gps_fix_t s_parsed_fix;

/** Current protocol mode (PULL = raw NMEA, PUSH = parsed fix) */
static spi_protocol_mode_t s_protocol_mode = SPI_MODE_PULL;

/** Debug output state */
static volatile bool s_out_tx_busy = false;
static uint8_t s_out_tx_buf[GPS_NMEA_MAX];
static uint16_t s_out_tx_len = 0;

/* ============================================================================
 * Forward Declarations
 * ============================================================================ */

static void process_dma_data(uint16_t new_pos);
static void feed_byte(uint8_t b);
static void nmea_reset(void);
static void out_try_start_tx(void);
static void populate_gps_fix(gps_fix_t *fix, const lwgps_t *gps);

/* ============================================================================
 * Public API
 * ============================================================================ */

void gps_set_queue(gps_sample_queue_t *queue)
{
    s_gps_queue = queue;
}

void gps_set_fix_queue(gps_fix_queue_t *queue)
{
    s_gps_fix_queue = queue;
}

void gps_set_protocol_mode(spi_protocol_mode_t mode)
{
    s_protocol_mode = mode;
}

void gps_init(UART_HandleTypeDef *gps_uart, UART_HandleTypeDef *out_uart)
{
    s_gps = gps_uart;
    s_out = out_uart;

    /* Reset state */
    s_last_pos = 0;
    s_line_len = 0;
    s_in_sentence = false;
    s_out_tx_busy = false;
    s_out_tx_len = 0;

    memset(s_dma_buf, 0, sizeof(s_dma_buf));
    memset(s_line, 0, sizeof(s_line));
    memset(&s_parsed_fix, 0, sizeof(s_parsed_fix));

    /* Initialize lwgps parser */
    lwgps_init(&s_lwgps);

    /* Start DMA circular receive with Character Match on '\n' */
    if (s_gps) {
        /* Set Character Match address to '\n' (NMEA sentence terminator) */
        __HAL_UART_DISABLE(s_gps);
        MODIFY_REG(s_gps->Instance->CR2, USART_CR2_ADD,
                   ((uint32_t)'\n' << USART_CR2_ADD_Pos));
        __HAL_UART_ENABLE(s_gps);

        /* Start DMA circular reception (CM is the only trigger) */
        HAL_UART_Receive_DMA(s_gps, s_dma_buf, GPS_DMA_BUF_SIZE);
        __HAL_DMA_DISABLE_IT(s_gps->hdmarx, DMA_IT_HT | DMA_IT_TC);
        __HAL_UART_ENABLE_IT(s_gps, UART_IT_CM);
    }
}

void gps_process(void)
{
    /* 
     * In DMA mode, most processing happens in callbacks.
     * This function can be used for any deferred/non-ISR processing if needed.
     * Currently empty since callbacks handle everything.
     */
}

/* ============================================================================
 * UART Callbacks (called from main.c or stm32g0xx_it.c)
 * ============================================================================ */

/**
 * @brief Called when Character Match ('\n') detected on USART6
 *
 * This is the main receive callback for DMA circular + CM mode.
 * Called from uart_cm_handler() when CMF fires on '\n'.
 *
 * @param huart UART handle
 * @param Size Current position in DMA buffer
 */
void gps_rx_event_callback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (!s_gps || huart != s_gps) {
        return;
    }

    /* Size is the current position in the DMA buffer */
    process_dma_data(Size);
}

/**
 * @brief Called on UART error
 *
 * Restarts DMA reception on error.
 */
void gps_uart_error_callback(UART_HandleTypeDef *huart)
{
    if (!s_gps || huart != s_gps) {
        return;
    }

    /* Reset and restart DMA circular reception with CM */
    s_last_pos = 0;
    HAL_UART_Receive_DMA(s_gps, s_dma_buf, GPS_DMA_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(s_gps->hdmarx, DMA_IT_HT | DMA_IT_TC);
    __HAL_UART_ENABLE_IT(s_gps, UART_IT_CM);
}

/**
 * @brief Called when debug UART TX completes
 */
void gps_uart_tx_cplt_callback(UART_HandleTypeDef *huart)
{
    if (!s_out || huart != s_out) {
        return;
    }

    s_out_tx_busy = false;
    s_out_tx_len = 0;
}

/* Legacy callback - no longer used in DMA mode but kept for compatibility */
void gps_uart_rx_cplt_callback(UART_HandleTypeDef *huart)
{
    (void)huart;
    /* Not used in DMA mode */
}

/* ============================================================================
 * Private Functions
 * ============================================================================ */

/**
 * @brief Process new data in DMA buffer
 *
 * Handles circular buffer wrap-around and feeds bytes to NMEA parser.
 *
 * @param new_pos Current DMA buffer position (from callback)
 */
static void process_dma_data(uint16_t new_pos)
{
    uint16_t last = s_last_pos;
    uint16_t current = new_pos;

    if (current == last) {
        /* No new data */
        return;
    }

    if (current > last) {
        /* Simple case: no wrap-around */
        for (uint16_t i = last; i < current; i++) {
            feed_byte(s_dma_buf[i]);
        }
    } else {
        /* Wrap-around: process end of buffer, then start */
        for (uint16_t i = last; i < GPS_DMA_BUF_SIZE; i++) {
            feed_byte(s_dma_buf[i]);
        }
        for (uint16_t i = 0; i < current; i++) {
            feed_byte(s_dma_buf[i]);
        }
    }

    s_last_pos = current;
}

/**
 * @brief Feed one byte to NMEA sentence builder
 *
 * Accumulates bytes into complete NMEA sentences.
 * In PUSH mode: parses with lwgps and enqueues parsed fix
 * In PULL mode: enqueues raw NMEA sentence
 *
 * @param b Received byte
 */
static void feed_byte(uint8_t b)
{
    /* Feed byte to lwgps parser */
    /* PUSH mode: always needed for parsed fix enqueuing */
    /* DEBUG PULL mode: needed for debug logging of parsed fix */
#ifdef DEBUG
    lwgps_process(&s_lwgps, &b, 1);
#else
    if (s_protocol_mode == SPI_MODE_PUSH) {
        lwgps_process(&s_lwgps, &b, 1);
    }
#endif

    /* Wait for '$' to start a new sentence */
    if (!s_in_sentence) {
        if (b == '$') {
            s_in_sentence = true;
            s_line_len = 0;
            s_line[s_line_len++] = b;
        }
        return;
    }

    /* Check for buffer overflow */
    if (s_line_len >= (GPS_NMEA_MAX - 1u)) {
        nmea_reset();
        return;
    }

    /* Accumulate byte */
    s_line[s_line_len++] = b;

    /* Check for end of sentence */
    if (b == '\n') {
        /* Null-terminate for safety */
        if (s_line_len < GPS_NMEA_MAX) {
            s_line[s_line_len] = '\0';
        }

        /* Mode-specific enqueuing */
        if (s_protocol_mode == SPI_MODE_PULL) {
            /* PULL MODE: Enqueue raw NMEA sentence */
            if (s_gps_queue != NULL) {
                gps_sample_enqueue(s_line, s_gps_queue);
#ifdef DEBUG
                /* Log NMEA sentence to debug console */
                debug_uart_log_gps_nmea((const char*)s_line);
                /* Always log parsed GPS state (shows fields even before valid fix) */
                populate_gps_fix(&s_parsed_fix, &s_lwgps);
                debug_uart_log_gps_fix(&s_parsed_fix);
#endif
            }
        } else if (s_protocol_mode == SPI_MODE_PUSH) {
            /* PUSH MODE: Enqueue parsed fix if valid */
#ifdef DEBUG
            /* Log every NMEA sentence in push mode too */
            debug_uart_log_gps_nmea((const char*)s_line);
#endif
            if (lwgps_is_valid(&s_lwgps) == 1) {
                populate_gps_fix(&s_parsed_fix, &s_lwgps);
                if (s_gps_fix_queue != NULL) {
                    gps_fix_enqueue(&s_parsed_fix, s_gps_fix_queue);
#ifdef DEBUG
                    /* Log decoded GPS fix to debug console */
                    debug_uart_log_gps_fix(&s_parsed_fix);
#endif
                }
            }
#ifdef DEBUG
            else {
                debug_uart_log_gps_no_fix(s_lwgps.fix, s_lwgps.sats_in_use);
            }
#endif
        }

        /* Debug output to UART1 */
        if (s_out && !s_out_tx_busy) {
            memcpy(s_out_tx_buf, s_line, s_line_len);
            s_out_tx_len = s_line_len;
            out_try_start_tx();
        }

        nmea_reset();
    }
}

/**
 * @brief Reset NMEA sentence builder state
 */
static void nmea_reset(void)
{
    s_in_sentence = false;
    s_line_len = 0;
}

/**
 * @brief Start non-blocking debug output transmission
 */
static void out_try_start_tx(void)
{
    if (!s_out || s_out_tx_busy || s_out_tx_len == 0) {
        return;
    }

    s_out_tx_busy = true;

    if (HAL_UART_Transmit_IT(s_out, s_out_tx_buf, s_out_tx_len) != HAL_OK) {
        s_out_tx_busy = false;
    }
}

/**
 * @brief Populate gps_fix_t structure from lwgps parsed data
 *
 * Maps lwgps fields to protocol-defined gps_fix_t structure.
 * Handles unit conversions (knots to m/s).
 *
 * @param fix Destination gps_fix_t structure
 * @param gps Source lwgps parser instance
 */
static void populate_gps_fix(gps_fix_t *fix, const lwgps_t *gps)
{
    /* Position */
    fix->latitude = gps->latitude;
    fix->longitude = gps->longitude;
    fix->altitude_msl = gps->altitude;

    /* Velocity & Heading */
    /* Note: lwgps speed is in knots, convert to m/s */
    fix->ground_speed = gps->speed * 0.514444f;  // knots to m/s
    fix->course = gps->course;

    /* Quality indicators */
    fix->fix_quality = (uint8_t)gps->fix;  // 0=invalid, 1=GPS, 2=DGPS
    fix->num_satellites = gps->sats_in_use;
    fix->hdop = gps->dop_h;

    /* Timestamp - milliseconds since midnight UTC */
    /* Note: GPS time of week requires GPS week number and leap seconds */
    /* Using simplified approach: milliseconds since midnight UTC */
    fix->time_of_week_ms = ((uint32_t)gps->hours * 3600 +
                             (uint32_t)gps->minutes * 60 +
                             (uint32_t)gps->seconds) * 1000;

    /* Clear padding for deterministic SPI transfers */
    fix->padding1[0] = 0;
    fix->padding1[1] = 0;
    memset(fix->padding2, 0, sizeof(fix->padding2));
}
