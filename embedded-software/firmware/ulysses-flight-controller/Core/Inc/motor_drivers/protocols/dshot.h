#ifndef ULYSSES_MOTOR_DRIVER_PROTOCOLS_DSHOT_H
#define ULYSSES_MOTOR_DRIVER_PROTOCOLS_DSHOT_H

#include <stdint.h>
#include <stdbool.h>

#define DSHOT_RESERVED_THROTTLE_COUNT (48)

typedef uint16_t dshot_frame_t;

dshot_frame_t dshot_pack(uint16_t throttle, bool request_telemetry);

uint8_t dshot_checksum(uint16_t throttle, bool request_telemetry);

#endif // ULYSSES_MOTOR_DRIVER_PROTOCOLS_DSHOT_H
