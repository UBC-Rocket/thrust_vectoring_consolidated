/* Test fake mirroring the codegen contract for generated/messages/routing.h. */
#ifndef FAKE_GENERATED_MESSAGES_ROUTING_H
#define FAKE_GENERATED_MESSAGES_ROUTING_H

#include <stddef.h>
#include <stdint.h>

#include "generated/messages/registry.h"

typedef struct {
    uint8_t  module_id;
    uint16_t msg_id;
    uint8_t  class;
    uint16_t payload_size;
    uint8_t  enabled_channels;          /* bitmask: bit i for channel i */
    uint16_t max_rate_hz[CH_COUNT];
    const void *pb_desc;                /* Class B: pb_msgdesc_t*; Class A: NULL */
} messages_routing_entry_t;

extern const messages_routing_entry_t messages_routing_table[];
extern const size_t                   messages_routing_table_len;

#endif /* FAKE_GENERATED_MESSAGES_ROUTING_H */
