#ifndef LIB_DSHOT_BDSHOT_H
#define LIB_DSHOT_BDSHOT_H

#include "dshot/protocol/dshot.h"

#include <stdbool.h>
#include <stdint.h>

bool bdshot_throttle_frame_pack(dshot_frame_t *frame, uint16_t throttle, bool request_telemetry);
bool bdshot_command_frame_pack(dshot_frame_t *frame, dshot_command_t command,
                               bool request_telemetry);

uint8_t bdshot_frame_checksum(dshot_frame_t frame);

bool bdshot_frame_decode_from_wire(dshot_frame_t *frame, uint32_t wire_frame);
bool bdshot_frame_is_edt(dshot_frame_t frame);
float bdshot_frame_calculate_rpm(dshot_frame_t frame, uint8_t motor_pole_count);

#endif // LIB_DSHOT_BDSHOT_H
