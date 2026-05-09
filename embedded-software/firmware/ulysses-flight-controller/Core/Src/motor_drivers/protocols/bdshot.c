#include "motor_drivers/protocols/bdshot.h"

#include <stdbool.h>
#include <stdint.h>

static bool to_bdshot_throttle(uint16_t *bdshot_throttle, uint16_t throttle);

bool bdshot_frame_pack(bdshot_frame_t *frame, uint16_t throttle, bool request_telemetry)
{
    uint16_t bdshot_throttle;
    uint8_t bdshot_telemetry;

    if (!to_bdshot_throttle(&bdshot_throttle, throttle)) {
        return false;
    }

    bdshot_telemetry = request_telemetry ? 1 : 0;

    *frame = 0x0000;

    *frame |= (bdshot_throttle << BDSHOT_THROTTLE_SHIFT) & BDSHOT_THROTTLE_MASK;
    *frame |= (bdshot_telemetry << BDSHOT_TELEMETRY_SHIFT) & BDSHOT_TELEMETRY_MASK;
    *frame |= (bdshot_frame_checksum(*frame) << BDSHOT_CHECKSUM_SHIFT) & BDSHOT_CHECKSUM_MASK;

    return true;
}

uint8_t bdshot_frame_checksum(bdshot_frame_t frame)
{
    // Checksum is calculated over the throttle and telemetry bits
    frame >>= BDSHOT_CHECKSUM_BITS;

    return ~(frame ^ (frame >> 4) ^ (frame >> 8)) & BDSHOT_CHECKSUM_MASK;
}

static bool to_bdshot_throttle(uint16_t *bdshot_throttle, uint16_t throttle)
{
    if (throttle > (BDSHOT_MAX_THROTTLE - BDSHOT_MIN_THROTTLE + 1)) {
        return false;
    }

    *bdshot_throttle = throttle + BDSHOT_MIN_THROTTLE;
    return true;
}
