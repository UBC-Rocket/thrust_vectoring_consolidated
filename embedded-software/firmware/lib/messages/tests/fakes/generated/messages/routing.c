/* Test fake routing table. */
#include "generated/messages/routing.h"
#include "generated/messages/types.h"

const messages_routing_entry_t messages_routing_table[] = {
    /* system.drop_counter — SD only, 1 Hz max. */
    {
        .module_id        = MOD_SYSTEM,
        .msg_id           = MSG_SYSTEM_DROP_COUNTER,
        .class            = MSG_CLASS_A,
        .payload_size     = sizeof(msg_system_drop_counter_t),
        .enabled_channels = (uint8_t)(1U << CH_SD),
        .max_rate_hz      = { [CH_SD] = 1, [CH_VCP] = 0, [CH_UDP] = 0 },
    },
    /* test.ping — SD + VCP + UDP, high rate. */
    {
        .module_id        = MOD_TEST,
        .msg_id           = MSG_TEST_PING,
        .class            = MSG_CLASS_A,
        .payload_size     = sizeof(msg_test_ping_t),
        .enabled_channels = (uint8_t)((1U << CH_SD) | (1U << CH_VCP) | (1U << CH_UDP)),
        .max_rate_hz      = { [CH_SD] = 1000, [CH_VCP] = 1000, [CH_UDP] = 1000 },
    },
    /* test.tiny — SD only, single-byte payload (envelope edge case). */
    {
        .module_id        = MOD_TEST,
        .msg_id           = MSG_TEST_TINY,
        .class            = MSG_CLASS_A,
        .payload_size     = sizeof(msg_test_tiny_t),
        .enabled_channels = (uint8_t)(1U << CH_SD),
        .max_rate_hz      = { [CH_SD] = 100, [CH_VCP] = 0, [CH_UDP] = 0 },
    },
    /* test.limited — SD only at 10 Hz, used by the rate-limit test. */
    {
        .module_id        = MOD_TEST,
        .msg_id           = MSG_TEST_LIMITED,
        .class            = MSG_CLASS_A,
        .payload_size     = sizeof(msg_test_limited_t),
        .enabled_channels = (uint8_t)(1U << CH_SD),
        .max_rate_hz      = { [CH_SD] = 10, [CH_VCP] = 0, [CH_UDP] = 0 },
    },
    /* test.vcp_only — exercises the VCP stub drop path. */
    {
        .module_id        = MOD_TEST,
        .msg_id           = MSG_TEST_VCP_ONLY,
        .class            = MSG_CLASS_A,
        .payload_size     = sizeof(msg_test_vcp_only_t),
        .enabled_channels = (uint8_t)(1U << CH_VCP),
        .max_rate_hz      = { [CH_SD] = 0, [CH_VCP] = 1000, [CH_UDP] = 0 },
    },
};

const size_t messages_routing_table_len =
    sizeof(messages_routing_table) / sizeof(messages_routing_table[0]);
