/* Test fake mirroring the codegen contract for generated/messages/types.h. */
#ifndef FAKE_GENERATED_MESSAGES_TYPES_H
#define FAKE_GENERATED_MESSAGES_TYPES_H

#include <stdint.h>

typedef struct __attribute__((packed)) {
    uint32_t sd_drops;
    uint32_t vcp_drops;
    uint32_t udp_drops;
} msg_system_drop_counter_t;

typedef struct __attribute__((packed)) {
    uint32_t seq;
    int32_t  value;
} msg_test_ping_t;

typedef struct __attribute__((packed)) {
    uint8_t b;
} msg_test_tiny_t;

typedef struct __attribute__((packed)) {
    uint32_t counter;
} msg_test_limited_t;

typedef struct __attribute__((packed)) {
    uint32_t counter;
} msg_test_vcp_only_t;

#endif /* FAKE_GENERATED_MESSAGES_TYPES_H */
