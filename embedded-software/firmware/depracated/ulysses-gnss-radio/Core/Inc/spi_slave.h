#ifndef SPI_SLAVE_H
#define SPI_SLAVE_H

/**
 * @file spi_slave.h
 * @brief SPI slave driver for STM32G0B1 - Pull and Push mode support
 *
 * Hardware Configuration:
 * - SPI1 on PA4(NSS), PA5(SCK), PA6(MISO), PA7(MOSI)
 * - IRQ output on PB1 (active high, directly driving GPIO for push mode)
 * - DMA1 Channel 1 (RX), Channel 2 (TX)
 * - EXTI line 4 for NSS transaction end detection
 *
 * Protocol:
 * - Pull mode: Master sends [CMD:1][DUMMY:4][PAYLOAD:0-256]
 * - Push mode: Slave asserts IRQ (high), sends [TYPE:1][PAYLOAD:N]
 */

#include <stdint.h>
#include <stdbool.h>
#include "stm32g0xx.h"
#include "protocol_config.h"
#include "radio_queue.h"
#include "gps_nema_queue.h"
#include "gps_fix_queue.h"

// ============================================================================
// HARDWARE DEFINITIONS
// ============================================================================

// SPI Peripheral Selection
#define SPI_PERIPHERAL          SPI1

// DMA Configuration
#define DMA_CONTROLLER          DMA1
#define RX_DMA_CHANNEL          DMA1_Channel1
#define TX_DMA_CHANNEL          DMA1_Channel2
#define RX_DMAMUX_CHANNEL       DMAMUX1_Channel0
#define TX_DMAMUX_CHANNEL       DMAMUX1_Channel1

// DMAMUX Request IDs (RM0444 Table 59)
#define DMAREQ_SPI_RX           16  // SPI1_RX
#define DMAREQ_SPI_TX           17  // SPI1_TX

// ============================================================================
// GPIO PIN DEFINITIONS
// ============================================================================

#define SPI_NSS_PIN             GPIO_PIN_4   // PA4
#define SPI_SCK_PIN             GPIO_PIN_5   // PA5
#define SPI_MISO_PIN            GPIO_PIN_6   // PA6
#define SPI_MOSI_PIN            GPIO_PIN_7   // PA7
#define SPI_GPIO_PORT           GPIOA

// For STM32G0B1, SPI1 is AF0 on PA4-7
#define SPI_AF_NUM              0

// NSS EXTI Configuration
#define NSS_EXTI_LINE           4
#define NSS_EXTI_PORT           0x00  // Port A = 0x00 (for EXTICR register)

// IRQ Line for Push Mode (PB1)
#define IRQ_GPIO_PORT           GPIOB
#define IRQ_GPIO_PIN            GPIO_PIN_1

// ============================================================================
// STATE MACHINE
// ============================================================================

typedef enum {
    SPI_STATE_UNCONFIGURED,  // Waiting for configuration frame from master
    SPI_STATE_IDLE,          // No transaction, RX DMA armed, waiting for NSS or data
    SPI_STATE_HAVE_DATA,     // Push mode: IRQ asserted, TX+RX DMA armed, waiting for NSS
    SPI_STATE_ACTIVE,        // Mid-transaction (both TX and RX DMA running)
} spi_slave_state_t;

// ============================================================================
// PUSH MODE CONFIGURATION
// ============================================================================

// Minimum bytes to sample when checking if RX buffer has real data
#define RX_PATTERN_SAMPLE_SIZE  16

// Threshold: if this many bytes are 0x00 or 0xFF, consider it dummy data
#define RX_PATTERN_THRESHOLD    14

// ============================================================================
// CONTEXT STRUCTURE
// ============================================================================

typedef struct {
    // ── State ──
    volatile spi_slave_state_t state;
    volatile uint8_t current_cmd;
    volatile bool payload_processed;  // Prevents double-processing in DMA TC + EXTI

    // ── Buffers ──
    // NOTE: Aligned to 32-byte boundary for optimal DMA performance
    uint8_t tx_buf[MAX_TRANSACTION_SIZE] __attribute__((aligned(32)));
    uint8_t rx_buf[MAX_TRANSACTION_SIZE] __attribute__((aligned(32)));

    // ── Queue Pointers ──
    radio_message_queue_t *radio_queue;      // RX: messages received from radio UART5
    radio_message_queue_t *radio_tx_queue;   // TX: messages from SPI master to send over radio
    gps_sample_queue_t *gps_queue;           // Pull mode: raw NMEA sentences (87 bytes)
    gps_fix_queue_t *gps_fix_queue;          // Push mode: parsed GPS fixes (48 bytes)

    // ── Statistics & Diagnostics ──
    volatile uint32_t transactions_completed;
    volatile uint32_t overrun_errors;
    volatile uint32_t transfer_errors;
    volatile uint32_t unknown_commands;

    // ── Push Mode State ──
    volatile bool irq_asserted;           // True when IRQ line is asserted (active low)
    volatile bool nss_busy;               // True when NSS low (transaction active, set by EXTI falling edge)
    volatile uint8_t pending_push_type;   // PUSH_TYPE_RADIO or PUSH_TYPE_GPS (0 = none)
    volatile uint16_t tx_dma_length;      // Length of current TX DMA transfer (expected)
    volatile uint16_t tx_dma_sent;        // Actual bytes transmitted (measured from CNDTR)
    volatile uint32_t push_transactions;  // Count of push mode transactions
    volatile uint32_t master_tx_received; // Count of master TX while pushing (simultaneous)

    // ── Push Mode Diagnostics ──
    volatile uint32_t nss_collisions;         // spi_slave_tick() called while NSS low
    volatile uint32_t irq_deferred_busy;      // IRQ assertion deferred due to busy flag
    volatile uint32_t tx_incomplete_count;    // TX DMA incomplete at NSS rising
    volatile uint32_t nss_falling_events;     // NSS falling edges detected
    volatile uint32_t nss_rising_events;      // NSS rising edges detected

} spi_slave_context_t;

// ============================================================================
// PUBLIC API
// ============================================================================

/**
 * @brief Initialize SPI slave peripheral
 *
 * Configures SPI1 as slave with hardware NSS, sets up DMA channels,
 * and arms for operation based on the protocol mode.
 *
 * IMPORTANT: Must call spi_slave_set_protocol_mode() BEFORE this function
 * to configure whether to operate in pull or push mode.
 *
 * @param radio_queue Pointer to radio RX message queue
 * @param radio_tx_queue Pointer to radio TX message queue (SPI master → radio UART5)
 * @param gps_queue Pointer to GPS sample queue
 */
void spi_slave_init(radio_message_queue_t *radio_queue, radio_message_queue_t *radio_tx_queue, gps_sample_queue_t *gps_queue, gps_fix_queue_t *gps_fix_queue);

/**
 * @brief Set the protocol mode (pull or push)
 *
 * Must be called after receiving and parsing the configuration frame
 * from the master, but before calling spi_slave_init().
 *
 * @param mode SPI_MODE_PULL or SPI_MODE_PUSH
 */
void spi_slave_set_protocol_mode(spi_protocol_mode_t mode);

/**
 * @brief Get current state (for debugging)
 */
spi_slave_state_t spi_slave_get_state(void);

/**
 * @brief Get statistics structure (for debugging)
 */
const spi_slave_context_t* spi_slave_get_context(void);

/**
 * @brief Reset error counters
 */
void spi_slave_reset_errors(void);

// ============================================================================
// INTERRUPT HANDLERS (must be called from stm32g0xx_it.c)
// ============================================================================
//
// These functions contain the actual interrupt logic. They are called by the
// corresponding IRQHandler functions in stm32g0xx_it.c.
//
// NOTE: The functions are named with _handler suffix to avoid duplicate symbol
// errors with the standard IRQHandler names which are defined in stm32g0xx_it.c

/**
 * @brief SPI1 RXNE interrupt logic
 * Called when command byte arrives. This is the critical handoff point.
 * Must be called from SPI1_IRQHandler() in stm32g0xx_it.c
 */
void spi_slave_spi1_irq_handler(void);

/**
 * @brief DMA1 Channel 1 (RX) interrupt logic
 * Called when RX DMA transfer completes or errors
 * Must be called from DMA1_Channel1_IRQHandler() in stm32g0xx_it.c
 */
void spi_slave_dma1_ch1_irq_handler(void);

/**
 * @brief NSS EXTI interrupt logic (line 4)
 * Called when NSS rising edge detected (transaction end)
 * Must be called from EXTI4_15_IRQHandler() in stm32g0xx_it.c
 */
void spi_slave_nss_exti_handler(void);

// ============================================================================
// PUSH MODE API
// ============================================================================

/**
 * @brief Assert IRQ line to master (push mode)
 *
 * Sets PB1 high (active high) to signal the master that
 * the slave has data ready to send.
 */
void spi_slave_assert_irq(void);

/**
 * @brief Deassert IRQ line to master (push mode)
 *
 * Sets PB1 low (inactive) after transaction completes.
 */
void spi_slave_deassert_irq(void);

/**
 * @brief Check for pending push data and assert IRQ if needed
 *
 * Call this periodically from the main loop. When data is available
 * (radio RX message or GPS fix) and the slave is idle, this function
 * prepares the TX buffer, arms DMAs, and asserts IRQ.
 */
void spi_slave_tick(void);

/**
 * @brief Check if RX buffer contains valid data (not dummy bytes)
 *
 * Used to detect if master sent a radio TX message during a push
 * transaction. Checks for non-zero/non-0xFF pattern.
 *
 * @return true if RX buffer appears to contain real data
 */
bool spi_slave_rx_has_valid_data(void);

// ============================================================================
// DEBUG INSTRUMENTATION
// ============================================================================

#ifdef DEBUG

/**
 * @brief Debug capture structure for SPI transaction instrumentation
 *
 * Captures register state at three critical points:
 * 1. ARM: After spi_slave_arm() completes (pre-transaction idle state)
 * 2. ISR: At RXNE ISR entry (command byte received, before processing)
 * 3. EXTI: At NSS rising edge (transaction complete, before re-arm)
 *
 * Key fields for diagnosing TX FIFO offset issues:
 * - pre_arm_sr: FTLVL bits [12:11] show stale TX FIFO bytes after SPE=0
 * - arm_tx_cndtr: Should be MAX_TRANSACTION_SIZE-4 after DMA prefetch
 * - isr_tx_cndtr: Shows how many TX bytes DMA has consumed at ISR entry
 */
typedef struct spi_debug_capture {
    // ARM state (captured in spi_slave_arm())
    uint32_t arm_count;        // Number of times arm has been called
    uint32_t pre_arm_sr;       // SPI1->SR right after SPE=0 (FTLVL shows stale TX FIFO)
    uint32_t arm_sr;           // SPI1->SR after full arm sequence (post-TXDMAEN prefetch)
    uint32_t arm_cr2;          // SPI1->CR2 after arm
    uint16_t arm_tx_cndtr;     // TX DMA CNDTR after arm (expect MAX_TRANSACTION_SIZE - 4)
    uint32_t arm_tx_cmar;      // TX DMA CMAR (should equal &tx_buf[0])
    uint32_t arm_dmamux0;      // DMAMUX1_Channel0->CCR (RX routing, expect 16)
    uint32_t arm_dmamux1;      // DMAMUX1_Channel1->CCR (TX routing, expect 17)

    // ISR state (captured in spi_slave_spi1_irq_handler())
    uint32_t isr_count;        // Number of RXNE ISR fires
    uint8_t  isr_cmd;          // Command byte received
    uint32_t isr_sr;           // SPI1->SR at ISR entry
    uint16_t isr_tx_cndtr;     // TX DMA CNDTR at ISR entry
    uint8_t  isr_tx_snap[8];   // tx_buf[0..7] at ISR entry

    // EXTI state (captured in spi_slave_nss_exti_handler())
    uint32_t exti_count;       // Number of NSS rising edges
    uint32_t exti_sr;          // SPI1->SR at transaction end
    uint16_t exti_tx_cndtr;    // TX DMA CNDTR at transaction end
    uint16_t exti_rx_cndtr;    // RX DMA CNDTR at transaction end
    uint8_t  exti_tx_snap[8];  // tx_buf[0..7] at transaction end
    uint8_t  exti_rx_snap[8];  // rx_buf[0..7] at transaction end
} spi_debug_capture_t;

/**
 * @brief Get pointer to debug capture data
 *
 * Returns a pointer to the static debug capture structure.
 * Fields are updated by ISR/EXTI handlers. Read from main loop
 * to print diagnostic data after each transaction.
 */
const spi_debug_capture_t* spi_slave_get_debug_capture(void);

#endif // DEBUG

#endif // SPI_SLAVE_H
