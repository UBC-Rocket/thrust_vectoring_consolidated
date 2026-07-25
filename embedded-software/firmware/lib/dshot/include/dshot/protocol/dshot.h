#ifndef LIB_DSHOT_PROTOCOL_DSHOT_H
#define LIB_DSHOT_PROTOCOL_DSHOT_H

#include <stdint.h>

#define DSHOT_FRAME_BITS (16)

#define DSHOT_MIN_THROTTLE (48)
#define DSHOT_MAX_THROTTLE (2047)

#define DSHOT_PAYLOAD_MASK  (0xFFE0)
#define DSHOT_PAYLOAD_BITS  (11)
#define DSHOT_PAYLOAD_SHIFT (5)

#define DSHOT_TELEMETRY_MASK  (0x0010)
#define DSHOT_TELEMETRY_BITS  (1)
#define DSHOT_TELEMETRY_SHIFT (4)

#define DSHOT_CHECKSUM_MASK  (0x000F)
#define DSHOT_CHECKSUM_BITS  (4)
#define DSHOT_CHECKSUM_SHIFT (0)

typedef enum dshot_command {
    DSHOT_COMMAND_MOTOR_STOP = 0
} dshot_command_t;

typedef uint16_t dshot_frame_t;

#endif // LIB_DSHOT_PROTOCOL_DSHOT_H
