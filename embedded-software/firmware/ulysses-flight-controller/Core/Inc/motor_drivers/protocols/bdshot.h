#ifndef ULYSSES_MOTOR_DRIVER_PROTOCOLS_BDSHOT_H
#define ULYSSES_MOTOR_DRIVER_PROTOCOLS_BDSHOT_H

#include <stdint.h>
#include <stdbool.h>

#define BDSHOT_MOTOR_COUNT (2)

#define BDSHOT_MIN_THROTTLE (48)
#define BDSHOT_MAX_THROTTLE (2047)

#define BDSHOT_FRAME_BITS          (16)
#define BDSHOT_TELEMETRY_GCR_BITS  (20)
#define BDSHOT_TELEMETRY_WIRE_BITS (BDSHOT_TELEMETRY_GCR_BITS + 1)

#define BDSHOT_THROTTLE_MASK  (0xFFE0)
#define BDSHOT_THROTTLE_BITS  (11)
#define BDSHOT_THROTTLE_SHIFT (5)

#define BDSHOT_TELEMETRY_MASK  (0x0010)
#define BDSHOT_TELEMETRY_BITS  (1)
#define BDSHOT_TELEMETRY_SHIFT (4)

#define BDSHOT_CHECKSUM_MASK  (0x000F)
#define BDSHOT_CHECKSUM_BITS  (4)
#define BDSHOT_CHECKSUM_SHIFT (0)

#define BDSHOT_ERPM_EXPONENT_MASK  (0xE000)
#define BDSHOT_ERPM_EXPONENT_BITS  (3)
#define BDSHOT_ERPM_EXPONENT_SHIFT (13)

#define BDSHOT_ERPM_MANTISSA_MASK  (0x1FF0)
#define BDSHOT_ERPM_MANTISSA_BITS  (9)
#define BDSHOT_ERPM_MANTISSA_SHIFT (4)

typedef uint16_t bdshot_frame_t;

typedef enum bdshot_motor_index {
    BDSHOT_MOTOR_INDEX_UPPER = 0,
    BDSHOT_MOTOR_INDEX_LOWER = 1,
} bdshot_motor_index_t;

typedef struct bdshot_motor_telemetry {
    float rpm;
} bdshot_motor_telemetry_t;

typedef enum bdshot_command {
    BDSHOT_COMMAND_MOTOR_STOP = 0
} bdshot_command_t;

bool bdshot_throttle_frame_pack(bdshot_frame_t *frame, uint16_t throttle, bool request_telemetry);
bool bdshot_command_frame_pack(bdshot_frame_t *frame, bdshot_command_t command,
                               bool request_telemetry);

uint8_t bdshot_frame_checksum(bdshot_frame_t frame, bool inverted);

bool bdshot_is_edt_frame(bdshot_frame_t frame);

#endif // ULYSSES_MOTOR_DRIVER_PROTOCOLS_BDSHOT_H
