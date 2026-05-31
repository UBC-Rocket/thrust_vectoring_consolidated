#include "motor_drivers/dshot/dshot.h"

#include <stdbool.h>
#include <stdint.h>

#define MINUTES_TO_US(minutes)  ((minutes) * 60 * 1000000)
#define BYTES_TO_NIBBLES(bytes) (bytes * 2)

#define GCR_FRAME_MASK  (0x000FFFFF)
#define GCR_SYMBOL_MASK (0x1F)
#define GCR_DUMMY       (0xFF)

static const uint8_t GCR_DECODE_TABLE[32] = {
    GCR_DUMMY, GCR_DUMMY, GCR_DUMMY, GCR_DUMMY, GCR_DUMMY, GCR_DUMMY, GCR_DUMMY, GCR_DUMMY,
    GCR_DUMMY, 0x09,      0x0A,      0x0B,      GCR_DUMMY, 0x0D,      0x0E,      0x0F,
    GCR_DUMMY, GCR_DUMMY, 0x02,      0x03,      GCR_DUMMY, 0x05,      0x06,      0x07,
    GCR_DUMMY, 0x00,      0x08,      0x01,      GCR_DUMMY, 0x04,      0x0C,      GCR_DUMMY,
};

static bool to_bdshot_throttle(uint16_t *bdshot_throttle, uint16_t throttle);
static void bdshot_frame_pack(bdshot_frame_t *frame, uint16_t data, bool request_telemetry);

static bool to_bdshot_throttle(uint16_t *bdshot_throttle, uint16_t throttle)
{
    if (throttle >= (BDSHOT_MAX_THROTTLE - BDSHOT_MIN_THROTTLE + 1)) {
        return false;
    }

    *bdshot_throttle = throttle + BDSHOT_MIN_THROTTLE;
    return true;
}

static void bdshot_frame_pack(bdshot_frame_t *frame, uint16_t data, bool request_telemetry)
{
    uint8_t bdshot_telemetry = request_telemetry ? 1 : 0;

    *frame = 0x0000;

    *frame |= (data << BDSHOT_PAYLOAD_SHIFT) & BDSHOT_PAYLOAD_MASK;
    *frame |= (bdshot_telemetry << BDSHOT_TELEMETRY_SHIFT) & BDSHOT_TELEMETRY_MASK;
    *frame |= (bdshot_frame_checksum(*frame, true) << BDSHOT_CHECKSUM_SHIFT) & BDSHOT_CHECKSUM_MASK;
}

bool bdshot_throttle_frame_pack(bdshot_frame_t *frame, uint16_t throttle, bool request_telemetry)
{
    uint16_t bdshot_throttle;

    if (!to_bdshot_throttle(&bdshot_throttle, throttle)) {
        return false;
    }

    bdshot_frame_pack(frame, bdshot_throttle, request_telemetry);

    return true;
}

bool bdshot_command_frame_pack(bdshot_frame_t *frame, bdshot_command_t command,
                               bool request_telemetry)
{
    bdshot_frame_pack(frame, command, request_telemetry);

    return true;
}

uint8_t bdshot_frame_checksum(bdshot_frame_t frame, bool inverted)
{
    // Checksum is calculated over the throttle and telemetry bits
    frame >>= BDSHOT_CHECKSUM_BITS;

    uint8_t checksum = frame ^ (frame >> 4) ^ (frame >> 8);

    if (inverted) {
        checksum = ~checksum;
    }

    return checksum & BDSHOT_CHECKSUM_MASK;
}

float bdshot_frame_calculate_rpm(bdshot_frame_t frame, uint8_t motor_pole_count)
{
    uint8_t exponent = (frame & BDSHOT_ERPM_EXPONENT_MASK) >> BDSHOT_ERPM_EXPONENT_SHIFT;
    uint16_t mantissa = (frame & BDSHOT_ERPM_MANTISSA_MASK) >> BDSHOT_ERPM_MANTISSA_SHIFT;

    // Have to handle 0 RPM values separately, since it could be a
    // division by 0 in the general case
    if ((mantissa == 0) || (exponent == UINT8_MAX && mantissa == UINT16_MAX)) {
        return 0.0f;
    }

    uint32_t period_us = (uint32_t)mantissa << (uint32_t)exponent;
    float erpm = MINUTES_TO_US(1.0f) / (float)period_us;

    return (erpm / motor_pole_count) * 2.0f;
}

bool bdshot_frame_is_edt(bdshot_frame_t frame)
{
    // SEE: https://github.com/bird-sanctuary/extended-dshot-telemetry
    return (frame & 0xF000) != 0 && (frame & 0x1000) == 0;
}

bool bdshot_frame_decode_from_wire(bdshot_frame_t *bdshot_frame, uint32_t wire_frame)
{
    bdshot_frame_t frame = 0x0000;
    uint32_t gcr_frame = (wire_frame ^ (wire_frame >> 1)) & GCR_FRAME_MASK;

    for (uint8_t i = 0; i < BYTES_TO_NIBBLES(sizeof(bdshot_frame_t)); i++) {
        // Iterate from MSb to LSb in groups of 5 bits
        uint8_t shift = 15 - (i * 5);

        uint8_t symbol = (gcr_frame >> shift) & GCR_SYMBOL_MASK;
        uint8_t nibble = GCR_DECODE_TABLE[symbol];

        if (nibble == GCR_DUMMY) {
            return false;
        }

        frame = (frame << 4) | nibble;
    }

    *bdshot_frame = frame;

    return true;
}
