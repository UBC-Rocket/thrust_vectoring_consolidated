/**
 * @file spi_slave_test.c
 * @brief Test harness and debugging utilities for SPI slave
 *
 * Provides test functions to validate SPI slave operation and
 * diagnostic functions for debugging protocol issues.
 */

#include "spi_slave.h"
#include <stdio.h>
#include <string.h>

// ============================================================================
// TEST DATA GENERATION
// ============================================================================

/**
 * @brief Fill radio queue with test data
 */
void spi_test_populate_radio_queue(radio_message_queue_t *q) {
    uint8_t test_msg[256];

    // Message 0: All 0x00
    memset(test_msg, 0x00, 256);
    radio_message_enqueue(256, test_msg, q);

    // Message 1: All 0xFF
    memset(test_msg, 0xFF, 256);
    radio_message_enqueue(256, test_msg, q);

    // Message 2: Incrementing pattern
    for (int i = 0; i < 256; i++) {
        test_msg[i] = (uint8_t)i;
    }
    radio_message_enqueue(256, test_msg, q);

    // Message 3: Alternating 0xAA / 0x55
    for (int i = 0; i < 256; i++) {
        test_msg[i] = (i & 1) ? 0x55 : 0xAA;
    }
    radio_message_enqueue(256, test_msg, q);

    // Message 4: ASCII text
    const char *text = "HELLO FROM GNSS/RADIO BOARD - SPI SLAVE TEST MESSAGE";
    memset(test_msg, 0x00, 256);
    strncpy((char*)test_msg, text, 255);
    radio_message_enqueue(256, test_msg, q);
}

/**
 * @brief Fill GPS queue with test data
 */
void spi_test_populate_gps_queue(gps_sample_queue_t *q) {
    uint8_t test_nmea[GPS_SAMPLE_SIZE];

    // NMEA sentence example: $GPGGA (Global Positioning System Fix Data)
    const char *nmea1 = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";
    memset(test_nmea, 0x00, GPS_SAMPLE_SIZE);
    strncpy((char*)test_nmea, nmea1, GPS_SAMPLE_SIZE - 1);
    gps_sample_enqueue(test_nmea, q);

    // NMEA sentence example: $GPRMC (Recommended Minimum)
    const char *nmea2 = "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A";
    memset(test_nmea, 0x00, GPS_SAMPLE_SIZE);
    strncpy((char*)test_nmea, nmea2, GPS_SAMPLE_SIZE - 1);
    gps_sample_enqueue(test_nmea, q);
}

// ============================================================================
// DIAGNOSTIC FUNCTIONS
// ============================================================================

/**
 * @brief Print SPI slave status to UART (requires printf redirect)
 *
 * Example output format:
 * SPI Slave Status:
 *   State: IDLE
 *   Transactions: 42
 *   Overruns: 0
 *   Transfer Errors: 0
 *   Unknown Commands: 0
 */
void spi_test_print_status(void) {
    const spi_slave_context_t *ctx = spi_slave_get_context();

    const char *state_names[] = {
        "IDLE",
        "ACTIVE",
        "HAVE_DATA",
        "PUSH_TYPE",
        "PUSH_PAYLOAD",
        "COLLISION"
    };

    printf("\n=== SPI Slave Status ===\n");
    printf("  State: %s\n", state_names[ctx->state]);
    printf("  Current CMD: 0x%02X\n", ctx->current_cmd);
    printf("  Transactions: %lu\n", ctx->transactions_completed);
    printf("  Overrun Errors: %lu\n", ctx->overrun_errors);
    printf("  Transfer Errors: %lu\n", ctx->transfer_errors);
    printf("  Unknown Commands: %lu\n", ctx->unknown_commands);
    printf("  Master TX Received: %lu\n", ctx->master_tx_received);
    printf("  Push Transactions: %lu\n", ctx->push_transactions);
    printf("  IRQ Asserted: %s\n", ctx->irq_asserted ? "YES" : "NO");
    printf("========================\n\n");
}

/**
 * @brief Print queue status
 */
void spi_test_print_queues(radio_message_queue_t *radio_q, gps_sample_queue_t *gps_q) {
    printf("\n=== Queue Status ===\n");

    // Radio queue
    uint8_t radio_count = radio_message_queue_empty(radio_q) ? 0 :
        (radio_q->head - radio_q->tail + RADIO_MESSAGE_QUEUE_LEN) % RADIO_MESSAGE_QUEUE_LEN;
    printf("  Radio Queue: %u/%u messages\n", radio_count, RADIO_MESSAGE_QUEUE_LEN);

    // GPS double buffer
    uint8_t gps_count = gps_sample_queue_empty(gps_q) ? 0 : 1;
    printf("  GPS Buffer: %s (active slot %u)\n",
           gps_count ? "has data" : "empty", gps_q->active);

    printf("====================\n\n");
}

/**
 * @brief Dump buffer contents in hex
 */
void spi_test_dump_buffer(const char *name, const uint8_t *buf, uint16_t len) {
    printf("\n=== %s (%u bytes) ===\n", name, len);

    for (uint16_t i = 0; i < len; i++) {
        if (i % 16 == 0) {
            printf("%04X: ", i);
        }

        printf("%02X ", buf[i]);

        if ((i + 1) % 16 == 0 || i == len - 1) {
            // Print ASCII representation
            uint16_t start = (i / 16) * 16;
            uint16_t end = i + 1;
            printf(" ");
            for (uint16_t j = start; j < end; j++) {
                char c = buf[j];
                printf("%c", (c >= 32 && c <= 126) ? c : '.');
            }
            printf("\n");
        }
    }

    printf("=========================\n\n");
}

// ============================================================================
// TEST SEQUENCES
// ============================================================================

/**
 * @brief Test 1: Buffer Length Query (Simplest Test)
 *
 * Master sends: [0x03][0xFF][0xFF][0xFF][0xFF][0xFF]
 * Slave responds with count at byte 5
 *
 * This is the simplest transaction (6 bytes) and a good first test.
 */
void spi_test_1_buffer_length(void) {
    printf("\n╔════════════════════════════════════════╗\n");
    printf("║  Test 1: Buffer Length Query (0x03)   ║\n");
    printf("╚════════════════════════════════════════╝\n\n");

    printf("Instructions:\n");
    printf("1. Master should send: 0x03 0xFF 0xFF 0xFF 0xFF 0xFF (6 bytes)\n");
    printf("2. Check byte 5 of MISO for queue count\n");
    printf("3. Expected result: 0x05 (5 messages in queue)\n\n");

    printf("Waiting for transaction...\n");
}

/**
 * @brief Test 2: Radio Read LIFO (Most Recent Message)
 *
 * Master sends: [0x01][0xFF][0xFF][0xFF][0xFF][256 dummy bytes]
 * Slave responds with most recent message starting at byte 5
 */
void spi_test_2_radio_read_lifo(void) {
    printf("\n╔════════════════════════════════════════╗\n");
    printf("║  Test 2: Radio Read LIFO (0x01)       ║\n");
    printf("╚════════════════════════════════════════╝\n\n");

    printf("Instructions:\n");
    printf("1. Master should send: 0x01 + 4 dummy + 256 dummy (261 bytes)\n");
    printf("2. Check bytes 5-260 of MISO for message data\n");
    printf("3. Expected: ASCII text message (most recent)\n\n");

    printf("Waiting for transaction...\n");
}

/**
 * @brief Test 3: Radio Read FIFO (Oldest Message)
 */
void spi_test_3_radio_read_fifo(void) {
    printf("\n╔════════════════════════════════════════╗\n");
    printf("║  Test 3: Radio Read FIFO (0x02)       ║\n");
    printf("╚════════════════════════════════════════╝\n\n");

    printf("Instructions:\n");
    printf("1. Master should send: 0x02 + 4 dummy + 256 dummy (261 bytes)\n");
    printf("2. Check bytes 5-260 of MISO for message data\n");
    printf("3. Expected: All zeros (oldest message)\n\n");

    printf("Waiting for transaction...\n");
}

/**
 * @brief Test 4: GPS Read
 */
void spi_test_4_gps_read(void) {
    printf("\n╔════════════════════════════════════════╗\n");
    printf("║  Test 4: GPS Read (0x05)              ║\n");
    printf("╚════════════════════════════════════════╝\n\n");

    printf("Instructions:\n");
    printf("1. Master should send: 0x05 + 4 dummy + 87 dummy (92 bytes)\n");
    printf("2. Check bytes 5-91 of MISO for NMEA sentence\n");
    printf("3. Expected: $GPGGA... NMEA sentence\n\n");

    printf("Waiting for transaction...\n");
}

/**
 * @brief Test 5: Radio Write (TX)
 */
void spi_test_5_radio_write(void) {
    printf("\n╔════════════════════════════════════════╗\n");
    printf("║  Test 5: Radio Write (0x04)           ║\n");
    printf("╚════════════════════════════════════════╝\n\n");

    printf("Instructions:\n");
    printf("1. Master should send: 0x04 + 4 dummy + 256 data bytes (261 bytes)\n");
    printf("2. Send test pattern (e.g., incrementing 0x00-0xFF repeated)\n");
    printf("3. Slave will enqueue the message\n\n");

    printf("Waiting for transaction...\n");
}

/**
 * @brief Test 6: Rapid Transactions
 */
void spi_test_6_rapid_transactions(void) {
    printf("\n╔════════════════════════════════════════╗\n");
    printf("║  Test 6: Rapid Transactions           ║\n");
    printf("╚════════════════════════════════════════╝\n\n");

    printf("Instructions:\n");
    printf("1. Master should send multiple transactions back-to-back\n");
    printf("2. Mix of short (0x03) and long (0x01) commands\n");
    printf("3. Check for errors in status\n\n");

    printf("Waiting for transactions...\n");
}

// ============================================================================
// MASTER TEST RUNNER
// ============================================================================

/**
 * @brief Run all tests (interactive)
 *
 * Call this from main loop and trigger tests via UART commands
 */
void spi_test_run_interactive(radio_message_queue_t *radio_q, gps_sample_queue_t *gps_q) {
    printf("\n");
    printf("╔════════════════════════════════════════════════╗\n");
    printf("║   SPI Slave Test Harness - Pull Mode          ║\n");
    printf("╚════════════════════════════════════════════════╝\n\n");

    printf("Commands:\n");
    printf("  1 - Test buffer length query\n");
    printf("  2 - Test radio read LIFO\n");
    printf("  3 - Test radio read FIFO\n");
    printf("  4 - Test GPS read\n");
    printf("  5 - Test radio write\n");
    printf("  6 - Test rapid transactions\n");
    printf("  s - Print status\n");
    printf("  q - Print queue status\n");
    printf("  p - Populate test data\n");
    printf("  r - Reset error counters\n");
    printf("  h - Print this help\n\n");

    printf("Test data loaded:\n");
    spi_test_print_queues(radio_q, gps_q);
    spi_test_print_status();

    printf("Ready for SPI transactions from master.\n");
    printf("Slave is in IDLE state, waiting for command byte.\n\n");
}

/**
 * @brief Handle test command character
 */
void spi_test_handle_command(char cmd, radio_message_queue_t *radio_q, gps_sample_queue_t *gps_q) {
    switch (cmd) {
        case '1':
            spi_test_1_buffer_length();
            break;
        case '2':
            spi_test_2_radio_read_lifo();
            break;
        case '3':
            spi_test_3_radio_read_fifo();
            break;
        case '4':
            spi_test_4_gps_read();
            break;
        case '5':
            spi_test_5_radio_write();
            break;
        case '6':
            spi_test_6_rapid_transactions();
            break;
        case 's':
            spi_test_print_status();
            break;
        case 'q':
            spi_test_print_queues(radio_q, gps_q);
            break;
        case 'p':
            printf("Populating test data...\n");
            spi_test_populate_radio_queue(radio_q);
            spi_test_populate_gps_queue(gps_q);
            printf("Done.\n");
            spi_test_print_queues(radio_q, gps_q);
            break;
        case 'r':
            printf("Resetting error counters...\n");
            spi_slave_reset_errors();
            printf("Done.\n");
            break;
        case 'h':
            spi_test_run_interactive(radio_q, gps_q);
            break;
        default:
            printf("Unknown command: '%c'\n", cmd);
            break;
    }
}

// ============================================================================
// VALIDATION HELPERS
// ============================================================================

/**
 * @brief Validate that a transaction completed successfully
 */
bool spi_test_validate_transaction(void) {
    const spi_slave_context_t *ctx = spi_slave_get_context();

    // Check for errors
    if (ctx->overrun_errors > 0) {
        printf("❌ FAIL: Overrun error detected\n");
        return false;
    }

    if (ctx->transfer_errors > 0) {
        printf("❌ FAIL: Transfer error detected\n");
        return false;
    }

    // Check state returned to IDLE
    if (ctx->state != SPI_STATE_IDLE) {
        printf("❌ FAIL: State not IDLE after transaction (state=%d)\n", ctx->state);
        return false;
    }

    printf("✓ PASS: Transaction completed successfully\n");
    return true;
}

/**
 * @brief Check if slave is ready for next transaction
 */
bool spi_test_is_ready(void) {
    return (spi_slave_get_state() == SPI_STATE_IDLE);
}
