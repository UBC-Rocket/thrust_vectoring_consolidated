/**
 * @file debug_uart.h
 * @brief Debug UART interface - bidirectional injection and logging
 *
 * RX (Injection): Line-based protocol via Character Match on '\n'
 *   - 'R' prefix → Inject radio message to radio queue
 *   - 'G' prefix → Inject GPS NMEA to GPS queue
 *
 * TX (Logging): Human-readable printf-style logging of all system events
 *   - Radio messages from UART5
 *   - GPS NMEA sentences from UART6
 *   - Decoded GPS fixes
 *   - SPI radio TX messages from master
 *
 * Only compiled when DEBUG macro is defined (zero impact on Release builds).
 */

#ifndef DEBUG_UART_H
#define DEBUG_UART_H

#ifdef DEBUG

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "stm32g0xx_hal.h"
#include "radio_queue.h"
#include "gps_nema_queue.h"
#include "protocol_config.h"
#include "spi_slave.h"

/* ============================================================================
 * Protocol Constants
 * ============================================================================ */

/** Injection type byte for radio messages (ASCII 'R') */
#define DEBUG_UART_TYPE_RADIO 0x52

/** Injection type byte for GPS messages (ASCII 'G') */
#define DEBUG_UART_TYPE_GPS   0x47

/** Maximum pending log messages in queue */
#define DEBUG_UART_LOG_QUEUE_DEPTH 32

/** Maximum characters per log message */
#define DEBUG_UART_LOG_MSG_SIZE 256

/* ============================================================================
 * Data Structures
 * ============================================================================ */

/**
 * @brief Log message queue for buffering debug output
 *
 * Ring buffer of formatted text messages waiting to be transmitted
 * via UART1. Populated by logging functions, drained by debug_uart_process_logs().
 */
typedef struct {
    char messages[DEBUG_UART_LOG_QUEUE_DEPTH][DEBUG_UART_LOG_MSG_SIZE];
    volatile uint8_t head;  /**< Next write position */
    volatile uint8_t tail;  /**< Next read position */
} debug_log_queue_t;

/* ============================================================================
 * Public API - Initialization
 * ============================================================================ */

/**
 * @brief Initialize debug UART system (injection and logging)
 *
 * Initializes log queue and stores queue pointers for injection.
 * USART1 DMA reception is started by uart_callbacks_init(), not here.
 *
 * @param radio_q Pointer to radio message queue (for injection)
 * @param gps_q Pointer to GPS NMEA queue (for injection)
 */
void debug_uart_init(radio_message_queue_t *radio_q, gps_sample_queue_t *gps_q);

/* ============================================================================
 * Public API - Injection (RX)
 * ============================================================================ */

/**
 * @brief Process a complete line received on UART1 for injection
 *
 * Called from uart_callbacks.c when Character Match fires on '\n'
 * and the line has an injection prefix ('R' or 'G').
 * The prefix byte has already been stripped by the caller.
 *
 * @param data Line payload (prefix stripped, no newline)
 * @param len Length of payload
 * @param type DEBUG_UART_TYPE_RADIO or DEBUG_UART_TYPE_GPS
 */
void debug_uart_rx_line_callback(const uint8_t *data, uint16_t len, uint8_t type);

/* ============================================================================
 * Public API - Logging (TX)
 * ============================================================================ */

/**
 * @brief Log radio message to debug console
 *
 * Formats and enqueues message: "[RADIO RX] <hex> (<ASCII>)"
 * Called from radio_driver.c when message received from UART5.
 *
 * @param msg Pointer to radio message bytes
 * @param len Length of message in bytes
 */
void debug_uart_log_radio(const uint8_t *msg, uint16_t len);

/**
 * @brief Log GPS NMEA sentence to debug console
 *
 * Formats and enqueues message: "[GPS NMEA] <sentence>"
 * Called from gps.c when NMEA sentence received from UART6.
 *
 * @param nmea Pointer to null-terminated NMEA sentence string
 */
void debug_uart_log_gps_nmea(const char *nmea);

/**
 * @brief Log decoded GPS fix to debug console
 *
 * Formats and enqueues message: "[GPS FIX] Lat: X, Lon: Y, Alt: Z, ..."
 * Called from gps.c after successful GPS fix parse.
 *
 * @param fix Pointer to decoded GPS fix structure
 */
void debug_uart_log_gps_fix(const gps_fix_t *fix);

/**
 * @brief Log GPS parse with no valid fix
 *
 * Formats: "[GPS NOFIX] fix=%u sats=%u"
 * Called from gps.c when NMEA sentence parsed but no valid fix.
 *
 * @param fix_status Fix quality from lwgps (0=invalid)
 * @param sats Satellites in use
 */
void debug_uart_log_gps_no_fix(uint8_t fix_status, uint8_t sats);

/**
 * @brief Log SPI radio TX message to debug console
 *
 * Formats and enqueues message: "[SPI TX] Radio msg from master: <hex> (<ASCII>)"
 * Called from spi_slave.c when radio TX received from SPI master.
 *
 * @param msg Pointer to radio message bytes
 * @param len Length of message in bytes
 */
void debug_uart_log_spi_radio_tx(const uint8_t *msg, uint16_t len);

/**
 * @brief Log successful radio message read by SPI master
 *
 * Formats and enqueues message: "[SPI->M] Radio: <hex> (<ASCII>)"
 * Called from spi_slave.c when radio data is dequeued and sent to master.
 *
 * @param msg Pointer to radio message bytes
 * @param len Length of message in bytes
 */
void debug_uart_log_spi_radio_read(const uint8_t *msg, uint16_t len);

/**
 * @brief Log successful GPS data read by SPI master
 *
 * Formats and enqueues message: "[SPI->M] GPS: <data>"
 * Called from spi_slave.c when GPS data is dequeued and sent to master.
 *
 * @param data Pointer to GPS data bytes (NMEA sentence or fix struct)
 * @param len Length of data in bytes
 */
void debug_uart_log_spi_gps_read(const uint8_t *data, uint16_t len);

/**
 * @brief Log radio buffer length response sent to SPI master
 *
 * Formats and enqueues message: "[SPI->M] BufLen: <count>"
 * Called from spi_slave.c when CMD_RADIO_RXBUF_LEN is processed.
 *
 * @param count Number of messages in radio buffer
 */
void debug_uart_log_spi_buflen(uint8_t count);

/**
 * @brief Process and transmit pending log messages
 *
 * Starts a DMA transfer for the next queued message. Non-blocking —
 * returns immediately after kicking DMA. Call regularly from main loop.
 */
void debug_uart_process_logs(void);

/**
 * @brief DMA TX complete callback
 *
 * Called from HAL_UART_TxCpltCallback when USART1 DMA TX finishes.
 * Advances the log queue tail and clears the DMA busy flag.
 */
void debug_uart_dma_tx_cplt(void);

/**
 * @brief Log push mode event with key metrics
 *
 * Formats: "[SPI PUSH] <event> type=0x%02X tx=%u rx=%u"
 * Called from spi_slave.c NSS handler for push mode state transitions.
 *
 * @param event Short event description (e.g. "COMPLETE", "INCOMPLETE", "NOT-YET-READ")
 * @param type Push type byte (PUSH_TYPE_RADIO or PUSH_TYPE_GPS)
 * @param tx_sent Number of TX bytes clocked out
 * @param rx_received Number of RX bytes received from master
 */
void debug_uart_log_spi_push_event(const char *event, uint8_t type,
                                    uint16_t tx_sent, uint16_t rx_received);

/**
 * @brief Log push mode idle arm
 *
 * Formats: "[SPI IDLE] Armed for master TX capture"
 * Called from arm_push_idle() after RX DMA is armed.
 */
void debug_uart_log_spi_idle_arm(void);

/**
 * @brief Log push mode data arm
 *
 * Formats: "[SPI PUSH] Armed type=0x%02X len=%u"
 * Called from arm_push() / prepare_push() when push data is loaded.
 *
 * @param type Push type byte
 * @param len Total transaction length
 */
void debug_uart_log_spi_push_arm(uint8_t type, uint16_t len);

/**
 * @brief Log SPI ARM state (idle state after arm)
 *
 * Prints SPI register state, DMA configuration, and DMAMUX routing
 * captured after spi_slave_arm() completes. Use for one-shot dump
 * after initialization or after each re-arm.
 *
 * @param dbg Pointer to debug capture structure
 */
void debug_uart_log_spi_arm(const spi_debug_capture_t *dbg);

/**
 * @brief Log SPI transaction debug data
 *
 * Prints ISR capture (command byte, SR, DMA state, tx_buf snapshot),
 * EXTI capture (end-of-transaction state), and post-arm state.
 * Call from main loop when a new transaction is detected.
 *
 * @param dbg Pointer to debug capture structure
 */
void debug_uart_log_spi_txn(const spi_debug_capture_t *dbg);

/**
 * @brief Log periodic SPI statistics
 *
 * Formats: "[SPI STATS] push=N txn=N inc=N mtx=N ovr=N"
 * Called from spi_slave_tick() every few seconds.
 */
/**
 * @brief Log NSS edge event with current SPI state
 *
 * Formats: "[SPI NSS] FALL state=N" or "[SPI NSS] RISE state=N"
 * Called from EXTI handler on NSS transitions.
 *
 * @param falling true for falling edge (CS assert), false for rising (CS deassert)
 * @param state Current SPI state machine state
 */
void debug_uart_log_spi_nss_event(bool falling, uint8_t state);

void debug_uart_log_spi_stats(uint32_t push_txns, uint32_t total_txns,
                               uint32_t incomplete, uint32_t master_tx,
                               uint32_t overruns);

/**
 * @brief Log radio UART5 diagnostic snapshot
 *
 * Prints CM event count, bytes processed, messages enqueued, UART errors,
 * plus raw USART5 register state and DMA counter to diagnose radio intake.
 * Call periodically from main loop (e.g. every 5 seconds).
 *
 * @param cm_events Character Match ISR invocations
 * @param bytes_fed Total bytes through feed_byte
 * @param msgs_enqueued Messages completed and enqueued
 * @param uart_errors UART error callback invocations
 * @param uart_cr1 USART5->CR1 (check CMIE bit 14)
 * @param uart_cr3 USART5->CR3 (check DMAR bit 6, EIE bit 0)
 * @param uart_isr USART5->ISR (check ORE/FE/NE flags)
 * @param dma_cndtr DMA1_Channel6->CNDTR (512 = no data received)
 */
void debug_uart_log_radio_diag(uint32_t cm_events, uint32_t bytes_fed,
                                uint32_t msgs_enqueued, uint32_t uart_errors,
                                uint32_t uart_cr1, uint32_t uart_cr3,
                                uint32_t uart_isr, uint16_t dma_cndtr);

#ifdef __cplusplus
}
#endif

#endif // DEBUG

#endif // DEBUG_UART_H
