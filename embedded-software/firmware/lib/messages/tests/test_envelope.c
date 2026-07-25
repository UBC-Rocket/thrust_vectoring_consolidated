#include "unity.h"

#include "messages/messages.h"
#include "messages/crc16.h"
#include "generated/messages/registry.h"
#include "generated/messages/routing.h"
#include "generated/messages/types.h"
#include "timestamp/timestamp.h" /* fake */

#include <string.h>

/* ------- Sink capture ------- */

#define CAP_MAX 8

static struct {
    uint8_t buf[CAP_MAX][512];
    size_t  len[CAP_MAX];
    int     count;
} g_cap;

static void cap_reset(void)
{
    memset(&g_cap, 0, sizeof(g_cap));
}

static bool cap_sink(const uint8_t *record, size_t len)
{
    if (g_cap.count >= CAP_MAX) return false;
    memcpy(g_cap.buf[g_cap.count], record, len);
    g_cap.len[g_cap.count] = len;
    g_cap.count++;
    return true;
}

/* ------- Tests ------- */

void test_envelope_byte_for_byte(void)
{
    fake_clock_set(0x0123456789ABCDEFULL);
    messages_init();
    messages_sd_sink_set(cap_sink);
    cap_reset();

    msg_test_ping_t ping = { .seq = 0x11223344U, .value = -1 };
    bool ok = messages_publish_a(MOD_TEST, MSG_TEST_PING, &ping, sizeof(ping));
    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_INT(1, g_cap.count);

    /* Expected layout for payload size N=8:
     * length = 12 + 8 + 2 = 22 = 0x0016
     * total  = 2 + 22 = 24 bytes
     */
    TEST_ASSERT_EQUAL_size_t(24U, g_cap.len[0]);

    const uint8_t *r = g_cap.buf[0];
    /* [u16 length] LE = 0x0016 */
    TEST_ASSERT_EQUAL_HEX8(0x16, r[0]);
    TEST_ASSERT_EQUAL_HEX8(0x00, r[1]);
    /* [u8 class] = 0x41 */
    TEST_ASSERT_EQUAL_HEX8(0x41, r[2]);
    /* [u8 module_id] */
    TEST_ASSERT_EQUAL_HEX8(MOD_TEST, r[3]);
    /* [u16 msg_id] LE */
    TEST_ASSERT_EQUAL_HEX8((uint8_t)(MSG_TEST_PING & 0xFF), r[4]);
    TEST_ASSERT_EQUAL_HEX8((uint8_t)((MSG_TEST_PING >> 8) & 0xFF), r[5]);
    /* [u64 t_us_publish] LE = 0x0123456789ABCDEF */
    TEST_ASSERT_EQUAL_HEX8(0xEF, r[6]);
    TEST_ASSERT_EQUAL_HEX8(0xCD, r[7]);
    TEST_ASSERT_EQUAL_HEX8(0xAB, r[8]);
    TEST_ASSERT_EQUAL_HEX8(0x89, r[9]);
    TEST_ASSERT_EQUAL_HEX8(0x67, r[10]);
    TEST_ASSERT_EQUAL_HEX8(0x45, r[11]);
    TEST_ASSERT_EQUAL_HEX8(0x23, r[12]);
    TEST_ASSERT_EQUAL_HEX8(0x01, r[13]);
    /* payload [seq u32 LE][value i32 LE] */
    TEST_ASSERT_EQUAL_HEX8(0x44, r[14]);
    TEST_ASSERT_EQUAL_HEX8(0x33, r[15]);
    TEST_ASSERT_EQUAL_HEX8(0x22, r[16]);
    TEST_ASSERT_EQUAL_HEX8(0x11, r[17]);
    TEST_ASSERT_EQUAL_HEX8(0xFF, r[18]);
    TEST_ASSERT_EQUAL_HEX8(0xFF, r[19]);
    TEST_ASSERT_EQUAL_HEX8(0xFF, r[20]);
    TEST_ASSERT_EQUAL_HEX8(0xFF, r[21]);
    /* CRC over r[2..22) (class..end-of-payload), LE in r[22..24). */
    uint16_t expected_crc = messages_crc16(&r[2], 20U);
    TEST_ASSERT_EQUAL_HEX8((uint8_t)(expected_crc & 0xFF), r[22]);
    TEST_ASSERT_EQUAL_HEX8((uint8_t)((expected_crc >> 8) & 0xFF), r[23]);
}

void test_envelope_length_field(void)
{
    fake_clock_set(0U);
    messages_init();
    messages_sd_sink_set(cap_sink);
    cap_reset();

    msg_test_tiny_t t = { .b = 0xAB };
    TEST_ASSERT_TRUE(messages_publish_a(MOD_TEST, MSG_TEST_TINY, &t, sizeof(t)));

    /* N = 1 -> length = 12 + 1 + 2 = 15 = 0x000F; total = 17 bytes. */
    TEST_ASSERT_EQUAL_size_t(17U, g_cap.len[0]);
    TEST_ASSERT_EQUAL_HEX8(0x0F, g_cap.buf[0][0]);
    TEST_ASSERT_EQUAL_HEX8(0x00, g_cap.buf[0][1]);
}

void test_envelope_crc_matches_independent(void)
{
    fake_clock_set(123456789ULL);
    messages_init();
    messages_sd_sink_set(cap_sink);
    cap_reset();

    msg_test_ping_t ping = { .seq = 42U, .value = -7 };
    TEST_ASSERT_TRUE(messages_publish_a(MOD_TEST, MSG_TEST_PING, &ping, sizeof(ping)));

    size_t n = g_cap.len[0];
    const uint8_t *r = g_cap.buf[0];
    /* CRC covers everything between the length field and the trailing CRC. */
    uint16_t computed = messages_crc16(&r[2], n - 2U - 2U);
    uint16_t on_wire  = (uint16_t)r[n - 2] | ((uint16_t)r[n - 1] << 8);
    TEST_ASSERT_EQUAL_HEX16(computed, on_wire);
}

void test_envelope_unknown_message_drops(void)
{
    fake_clock_set(0U);
    messages_init();
    messages_sd_sink_set(cap_sink);
    cap_reset();

    msg_test_ping_t p = { 0 };
    /* MOD 99 / MSG 99 is not in the routing table. */
    bool ok = messages_publish_a(99U, 99U, &p, sizeof(p));
    TEST_ASSERT_FALSE(ok);
    TEST_ASSERT_EQUAL_INT(0, g_cap.count);
}

void test_envelope_wrong_payload_size_drops(void)
{
    fake_clock_set(0U);
    messages_init();
    messages_sd_sink_set(cap_sink);
    cap_reset();

    uint8_t too_small = 0x5A;
    bool ok = messages_publish_a(MOD_TEST, MSG_TEST_PING, &too_small, 1U);
    TEST_ASSERT_FALSE(ok);
    TEST_ASSERT_EQUAL_INT(0, g_cap.count);
}
