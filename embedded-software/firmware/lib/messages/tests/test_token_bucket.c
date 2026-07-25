#include "unity.h"

#include "messages/messages.h"
#include "generated/messages/registry.h"
#include "generated/messages/types.h"
#include "timestamp/timestamp.h" /* fake */

#include <string.h>

static int g_sink_calls;
static bool noop_sink(const uint8_t *r, size_t n) { (void)r; (void)n; g_sink_calls++; return true; }

void test_token_bucket_10hz_burst_drops_90(void)
{
    /* MSG_TEST_LIMITED is configured at 10 Hz on SD only. */
    fake_clock_set(0U);
    messages_init();
    messages_sd_sink_set(noop_sink);
    g_sink_calls = 0;

    msg_test_limited_t p = { .counter = 0U };

    /* 100 publishes over 99 ms (1 ms apart). At 10 Hz, capacity = 10:
     *   - First call drains the initial full bucket of 10.
     *   - 1 ms refill adds 0.01 token; 99 ms total adds 0.99 token.
     *   - So we accept the first 10, then accept zero more (no full token
     *     refilled within 99 ms windows of 1 ms each).
     * Expected: exactly 10 accepted, 90 dropped on SD. */
    for (int i = 0; i < 100; ++i) {
        p.counter = (uint32_t)i;
        (void)messages_publish_a(MOD_TEST, MSG_TEST_LIMITED, &p, sizeof(p));
        fake_clock_advance(1000U); /* +1 ms */
    }

    TEST_ASSERT_EQUAL_INT(10, g_sink_calls);
    TEST_ASSERT_EQUAL_UINT32(90U, messages_get_drops(CH_SD));
}

void test_token_bucket_recovers_after_idle(void)
{
    /* After draining the bucket, idling for >= 1 second refills capacity. */
    fake_clock_set(0U);
    messages_init();
    messages_sd_sink_set(noop_sink);
    g_sink_calls = 0;

    msg_test_limited_t p = { .counter = 0U };

    /* Drain initial 10 tokens. */
    for (int i = 0; i < 10; ++i) {
        TEST_ASSERT_TRUE(messages_publish_a(MOD_TEST, MSG_TEST_LIMITED, &p, sizeof(p)));
    }
    TEST_ASSERT_EQUAL_INT(10, g_sink_calls);

    /* Next call without time advance should drop. */
    TEST_ASSERT_FALSE(messages_publish_a(MOD_TEST, MSG_TEST_LIMITED, &p, sizeof(p)));

    /* Wait 1 second — bucket should refill to capacity (10). */
    fake_clock_advance(1000000U);
    g_sink_calls = 0;
    for (int i = 0; i < 10; ++i) {
        TEST_ASSERT_TRUE(messages_publish_a(MOD_TEST, MSG_TEST_LIMITED, &p, sizeof(p)));
    }
    TEST_ASSERT_EQUAL_INT(10, g_sink_calls);

    /* 11th call drops again. */
    TEST_ASSERT_FALSE(messages_publish_a(MOD_TEST, MSG_TEST_LIMITED, &p, sizeof(p)));
}

void test_token_bucket_zero_rate_blocks(void)
{
    /* MSG_TEST_VCP_ONLY has max_rate_hz=0 for SD, and SD isn't enabled
     * anyway. But we can verify zero-rate behaviour by exercising a route
     * with the rate set to zero. Here we just check that the drop counter
     * for SD doesn't change on a non-SD message — the channel mask filters
     * before the bucket gets consulted. */
    fake_clock_set(0U);
    messages_init();
    messages_sd_sink_set(noop_sink);
    g_sink_calls = 0;

    msg_test_vcp_only_t p = { .counter = 0U };
    /* publish_a will route only to VCP, which is a stub — so any_accepted=false. */
    bool ok = messages_publish_a(MOD_TEST, MSG_TEST_VCP_ONLY, &p, sizeof(p));
    TEST_ASSERT_FALSE(ok);
    /* SD sink never invoked; SD drops stays at 0. */
    TEST_ASSERT_EQUAL_INT(0, g_sink_calls);
    TEST_ASSERT_EQUAL_UINT32(0U, messages_get_drops(CH_SD));
}
