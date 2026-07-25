#include "dshot/bdshot.h"

#include "dshot/protocol/dshot.h"
#include "dshot/protocol/bdshot.h"

#include <stdbool.h>
#include <stdint.h>

#define MINUTES_TO_US(minutes)  ((minutes) * 60 * 1000000)
#define BYTES_TO_NIBBLES(bytes) (bytes * 2)

#define GCR_SYMBOL_MASK (0x1F)
#define GCR_SYMBOL_BITS (5)
#define GCR_NULL        (0xFF)

// clang-format off: clang-format is not good at formatting large tables
static const uint8_t GCR_DECODE_TABLE[32] = {
    GCR_NULL, GCR_NULL, GCR_NULL, GCR_NULL, GCR_NULL, GCR_NULL, GCR_NULL, GCR_NULL,
    GCR_NULL, 0x09,     0x0A,     0x0B,     GCR_NULL, 0x0D,     0x0E,     0x0F,
    GCR_NULL, GCR_NULL, 0x02,     0x03,     GCR_NULL, 0x05,     0x06,     0x07,
    GCR_NULL, 0x00,     0x08,     0x01,     GCR_NULL, 0x04,     0x0C,     GCR_NULL,
};
// clang-format on

static uint8_t dshot_frame_checksum(dshot_frame_t frame);
static bool dshot_to_frame_throttle(uint16_t *frame_throttle, uint16_t throttle);

static void bdshot_frame_pack(dshot_frame_t *frame, uint16_t data, bool request_telemetry);

static uint8_t dshot_frame_checksum(dshot_frame_t frame)
{
    // Checksum is calculated over the throttle and telemetry bits
    frame >>= DSHOT_CHECKSUM_BITS;

    uint8_t checksum = frame ^ (frame >> 4) ^ (frame >> 8);

    return checksum & DSHOT_CHECKSUM_MASK;
}

static bool dshot_to_frame_throttle(uint16_t *frame_throttle, uint16_t throttle)
{
    if (throttle >= (DSHOT_MAX_THROTTLE - DSHOT_MIN_THROTTLE + 1)) {
        return false;
    }

    *frame_throttle = throttle + DSHOT_MIN_THROTTLE;

    return true;
}

static void bdshot_frame_pack(dshot_frame_t *frame, uint16_t data, bool request_telemetry)
{
    uint8_t bdshot_telemetry = request_telemetry ? 1 : 0;

    *frame = 0x0000;

    *frame |= (data << DSHOT_PAYLOAD_SHIFT) & DSHOT_PAYLOAD_MASK;
    *frame |= (bdshot_telemetry << DSHOT_TELEMETRY_SHIFT) & DSHOT_TELEMETRY_MASK;
    *frame |= (bdshot_frame_checksum(*frame) << DSHOT_CHECKSUM_SHIFT) & DSHOT_CHECKSUM_MASK;
}

bool bdshot_throttle_frame_pack(dshot_frame_t *frame, uint16_t throttle, bool request_telemetry)
{
    uint16_t frame_throttle = 0;

    if (!dshot_to_frame_throttle(&frame_throttle, throttle)) {
        return false;
    }

    bdshot_frame_pack(frame, frame_throttle, request_telemetry);

    return true;
}

bool bdshot_command_frame_pack(dshot_frame_t *frame, dshot_command_t command,
                               bool request_telemetry)
{
    bdshot_frame_pack(frame, command, request_telemetry);

    return true;
}

uint8_t bdshot_frame_checksum(dshot_frame_t frame)
{
    uint8_t dshot_checksum = dshot_frame_checksum(frame);

    // Bidirectional DShot uses the inverse of the normal DShot checksum
    return (~dshot_checksum) & DSHOT_CHECKSUM_MASK;
}

bool bdshot_frame_decode_from_wire(dshot_frame_t *frame, uint32_t wire_frame)
{
    bool success = true;

    uint32_t gcr_frame = (wire_frame ^ (wire_frame >> 1)) & BDSHOT_TELEMETRY_GCR_MASK;

    *frame = 0x0000;

    // Decode the GCR frame by mapping each 5-bit symbol in the 20-bit encoded
    // frame to its corresponding 4-bit nibble
    for (uint8_t i = 0; i < BYTES_TO_NIBBLES(sizeof(dshot_frame_t)); i++) {
        // Iterate from MSb to LSb in groups of 5 bits
        uint8_t shift = 15 - (i * GCR_SYMBOL_BITS);

        uint8_t symbol = (gcr_frame >> shift) & GCR_SYMBOL_MASK;
        uint8_t nibble = GCR_DECODE_TABLE[symbol];

        if (nibble != GCR_NULL) {
            *frame = (*frame << 4) | nibble;
        } else {
            success = false;
            break;
        }
    }

    return success;
}

bool bdshot_frame_is_edt(dshot_frame_t frame)
{
    // See: https://github.com/bird-sanctuary/extended-dshot-telemetry
    return (frame & 0xF000) != 0 && (frame & 0x1000) == 0;
}

float bdshot_frame_calculate_rpm(dshot_frame_t frame, uint8_t motor_pole_count)
{
    uint8_t exponent = (frame & BDSHOT_ERPM_EXPONENT_MASK) >> BDSHOT_ERPM_EXPONENT_SHIFT;
    uint16_t mantissa = (frame & BDSHOT_ERPM_MANTISSA_MASK) >> BDSHOT_ERPM_MANTISSA_SHIFT;

    float erpm = 0;

    if (mantissa == 0) {
        // Avoid division by 0 in the general calculation
        erpm = 0;
    } else if (exponent == UINT8_MAX && mantissa == UINT16_MAX) {
        erpm = 0;
    } else {
        uint32_t period_us = (uint32_t)mantissa << (uint32_t)exponent;

        erpm = MINUTES_TO_US(1.0f) / (float)period_us;
    }

    return (erpm / motor_pole_count) * 2.0f;
}
