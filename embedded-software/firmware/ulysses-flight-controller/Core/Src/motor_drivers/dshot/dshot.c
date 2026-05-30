#include "motor_drivers/dshot/dshot.h"

#include <stdbool.h>
#include <stdint.h>

#define MINUTES_TO_US(minutes) ((minutes) * 60 * 1000000)

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
