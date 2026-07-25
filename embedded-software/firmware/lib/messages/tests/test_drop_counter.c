#include "unity.h"

#include "messages/messages.h"
#include "generated/messages/registry.h"
#include "generated/messages/types.h"
#include "timestamp/timestamp.h" /* fake */

#include <stddef.h>
#include <string.h>

void test_drop_counter_struct_offsets(void)
{
    /* Wire format: sd, vcp, udp — each u32, packed (no padding). */
    TEST_ASSERT_EQUAL_size_t(0U, offsetof(msg_system_drop_counter_t, sd_drops));
    TEST_ASSERT_EQUAL_size_t(4U, offsetof(msg_system_drop_counter_t, vcp_drops));
    TEST_ASSERT_EQUAL_size_t(8U, offsetof(msg_system_drop_counter_t, udp_drops));
    TEST_ASSERT_EQUAL_size_t(12U, sizeof(msg_system_drop_counter_t));
}

/* Sink capture for drop-counter test. */
static uint8_t g_last[64];
static size_t  g_last_len;
static int     g_calls;
static bool capture_sink(const uint8_t *r, size_t n)
{
    if (n > sizeof(g_last)) return false;
    memcpy(g_last, r, n);
    g_last_len = n;
    g_calls++;
    return true;
}

void test_drop_counter_publish_emits_envelope(void)
{
    fake_clock_set(1000U);
    messages_init();
    messages_sd_sink_set(capture_sink);
    g_last_len = 0; g_calls = 0;

    /* Inject some drops by publishing a message on a channel with no real
     * sink (CH_VCP). */
    msg_test_vcp_only_t v = { .counter = 0 };
    (void)messages_publish_a(MOD_TEST, MSG_TEST_VCP_ONLY, &v, sizeof(v));
    (void)messages_publish_a(MOD_TEST, MSG_TEST_VCP_ONLY, &v, sizeof(v));
    TEST_ASSERT_EQUAL_UINT32(2U, messages_get_drops(CH_VCP));

    /* Now snapshot. */
    messages_publish_drop_counters();
    TEST_ASSERT_EQUAL_INT(1, g_calls);

    /* Payload N = sizeof(msg_system_drop_counter_t) = 12 -> total = 28 bytes. */
    TEST_ASSERT_EQUAL_size_t(28U, g_last_len);

    /* Payload starts at byte 14 (2 length + 12 header). */
    msg_system_drop_counter_t s;
    memcpy(&s, &g_last[14], sizeof(s));
    TEST_ASSERT_EQUAL_UINT32(0U, s.sd_drops);
    TEST_ASSERT_EQUAL_UINT32(2U, s.vcp_drops);
    TEST_ASSERT_EQUAL_UINT32(0U, s.udp_drops);

    /* And the class byte is Class A. */
    TEST_ASSERT_EQUAL_HEX8(0x41, g_last[2]);
    TEST_ASSERT_EQUAL_HEX8(MOD_SYSTEM, g_last[3]);
}

void test_vcp_only_publish_bumps_vcp_drops(void)
{
    fake_clock_set(0U);
    messages_init();
    /* No SD sink set — but message has only VCP enabled anyway. */
    msg_test_vcp_only_t v = { .counter = 1 };
    bool ok = messages_publish_a(MOD_TEST, MSG_TEST_VCP_ONLY, &v, sizeof(v));
    TEST_ASSERT_FALSE(ok);
    TEST_ASSERT_EQUAL_UINT32(1U, messages_get_drops(CH_VCP));
    TEST_ASSERT_EQUAL_UINT32(0U, messages_get_drops(CH_SD));
    TEST_ASSERT_EQUAL_UINT32(0U, messages_get_drops(CH_UDP));
}

void test_no_sink_bumps_sd_drops(void)
{
    fake_clock_set(0U);
    messages_init();
    /* No sink registered, but TEST_TINY has SD enabled. */
    msg_test_tiny_t t = { .b = 1 };
    bool ok = messages_publish_a(MOD_TEST, MSG_TEST_TINY, &t, sizeof(t));
    TEST_ASSERT_FALSE(ok);
    TEST_ASSERT_EQUAL_UINT32(1U, messages_get_drops(CH_SD));
}

void test_publish_b_is_stub(void)
{
    fake_clock_set(0U);
    messages_init();
    uint8_t buf[4] = {0};
    bool ok = messages_publish_b(MOD_TEST, MSG_TEST_PING, buf, sizeof(buf));
    TEST_ASSERT_FALSE(ok);
}
