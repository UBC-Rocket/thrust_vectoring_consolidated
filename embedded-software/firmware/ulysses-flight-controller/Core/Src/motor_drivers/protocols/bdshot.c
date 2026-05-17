#include "motor_drivers/protocols/bdshot.h"

#include <stdbool.h>
#include <stdint.h>

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

    *frame |= (data << BDSHOT_THROTTLE_SHIFT) & BDSHOT_THROTTLE_MASK;
    *frame |= (bdshot_telemetry << BDSHOT_TELEMETRY_SHIFT) & BDSHOT_TELEMETRY_MASK;
    *frame |= (bdshot_frame_checksum(*frame) << BDSHOT_CHECKSUM_SHIFT) & BDSHOT_CHECKSUM_MASK;
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

uint8_t bdshot_frame_checksum(bdshot_frame_t frame)
{
    // Checksum is calculated over the throttle and telemetry bits
    frame >>= BDSHOT_CHECKSUM_BITS;

    return ~(frame ^ (frame >> 4) ^ (frame >> 8)) & BDSHOT_CHECKSUM_MASK;
}

bool bdshot_is_edt_frame(bdshot_frame_t frame)
{
    // SEE: https://github.com/bird-sanctuary/extended-dshot-telemetry
    return (frame & 0xF000) != 0 && (frame & 0x1000) == 0;
}
