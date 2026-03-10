#ifndef PROTOCOL_CONFIG_H
#define PROTOCOL_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// SPI SLAVE PROTOCOL CONFIGURATION
// Rev 0.1 - Pull & Push Mode Support
// ============================================================================

// ----------------------------------------------------------------------------
// Protocol Mode Selection
// ----------------------------------------------------------------------------
typedef enum {
    SPI_MODE_PULL = 0x00,  // Master-initiated, command-based
    SPI_MODE_PUSH = 0x01,  // Slave-initiated via IRQ
} spi_protocol_mode_t;

// ----------------------------------------------------------------------------
// Pull Mode Command Definitions
// ----------------------------------------------------------------------------
typedef enum {
    CMD_RADIO_RX_LIFO    = 0x01,  // Read newest radio message (LIFO)
    CMD_RADIO_RX_FIFO    = 0x02,  // Read oldest radio message (FIFO)
    CMD_RADIO_RXBUF_LEN  = 0x03,  // Read radio buffer message count
    CMD_RADIO_TX         = 0x04,  // Write radio message to transmit
    CMD_GPS_RX           = 0x05,  // Read raw NMEA sentence (pull mode)
} spi_pull_command_t;

// ----------------------------------------------------------------------------
// Push Mode Data Type Identifiers
// ----------------------------------------------------------------------------
typedef enum {
    PUSH_TYPE_RADIO = 0x01,  // Radio message available
    PUSH_TYPE_GPS   = 0x05,  // GPS fix available
} spi_push_type_t;

// ----------------------------------------------------------------------------
// Protocol Transaction Sizes
// ----------------------------------------------------------------------------

// Pull mode transaction sizes (CMD + DUMMY + DATA)
#define PULL_CMD_BYTES           1
#define PULL_DUMMY_BYTES         4    // CRITICAL: 4 bytes needed to avoid TX FIFO prefetch race
#define CMD_OVERHEAD             (PULL_CMD_BYTES + PULL_DUMMY_BYTES)  // 5 bytes
#define PULL_RADIO_PAYLOAD       256
#define PULL_GPS_PAYLOAD         87   // Max NMEA sentence length (82 chars + margin)
#define PULL_BUFLEN_PAYLOAD      1    // Single byte count

#define PULL_RADIO_TOTAL         (PULL_CMD_BYTES + PULL_DUMMY_BYTES + PULL_RADIO_PAYLOAD)  // 261
#define PULL_GPS_TOTAL           (PULL_CMD_BYTES + PULL_DUMMY_BYTES + PULL_GPS_PAYLOAD)    // 92
#define PULL_BUFLEN_TOTAL        (PULL_CMD_BYTES + PULL_DUMMY_BYTES + PULL_BUFLEN_PAYLOAD) // 6

// Push mode transaction sizes (TYPE + PAYLOAD)
#define PUSH_TYPE_BYTES          1
#define PUSH_RADIO_PAYLOAD       256
#define PUSH_GPS_PAYLOAD         48   // Parsed gps_fix_t structure (sizeof(gps_fix_t))

#define PUSH_RADIO_TOTAL         (PUSH_TYPE_BYTES + PUSH_RADIO_PAYLOAD)  // 257
#define PUSH_GPS_TOTAL           (PUSH_TYPE_BYTES + PUSH_GPS_PAYLOAD)    // 49

// Maximum transaction size (for buffer allocation)
#define MAX_TRANSACTION_SIZE     PULL_RADIO_TOTAL  // 261 bytes

// ----------------------------------------------------------------------------
// Configuration Frame Structure (Startup)
// ----------------------------------------------------------------------------
typedef struct {
    uint8_t mode;                 // 0x00 = Pull, 0x01 = Push
    uint8_t gps_rate;             // GPS update rate (Hz)
    uint8_t gps_constellation;    // GNSS constellation mask
    uint8_t reserved[5];          // Future use
} __attribute__((packed)) config_frame_t;

#define CONFIG_FRAME_SIZE sizeof(config_frame_t)  // 8 bytes

// ----------------------------------------------------------------------------
// GPS Fix Structure (Push Mode)
// ----------------------------------------------------------------------------
typedef struct {
    // Position
    double   latitude;         // Degrees, signed (-90 to +90)          [8 bytes]
    double   longitude;        // Degrees, signed (-180 to +180)        [8 bytes]
    float    altitude_msl;     // Meters above sea level                [4 bytes]

    // Velocity & Heading
    float    ground_speed;     // m/s                                   [4 bytes]
    float    course;           // Degrees true north (0-360)            [4 bytes]

    // Quality indicators
    uint8_t  fix_quality;      // 0=invalid, 1=GPS, 2=DGPS, etc.        [1 byte]
    uint8_t  num_satellites;   // Number of satellites used             [1 byte]
    uint8_t  padding1[2];      // Alignment padding                     [2 bytes]
    float    hdop;             // Horizontal dilution of precision      [4 bytes]

    // Timestamp
    uint32_t time_of_week_ms;  // GPS time of week in milliseconds      [4 bytes]

    uint8_t  padding2[8];      // Padding to 48 bytes total             [8 bytes]
} __attribute__((packed)) gps_fix_t;  // Total: 48 bytes (reserved for future parsed GPS)

// ----------------------------------------------------------------------------
// Buffer Limits
// ----------------------------------------------------------------------------
#define RADIO_QUEUE_DEPTH        10   // Max buffered radio messages
#define GPS_QUEUE_DEPTH          10   // Max buffered GPS samples

#endif // PROTOCOL_CONFIG_H
