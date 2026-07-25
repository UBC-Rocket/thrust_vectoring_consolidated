#include "cobs/cobs.h"
#include "unity.h"

#include <string.h>

void setUp(void) {}
void tearDown(void) {}

static void roundtrip(const uint8_t *src, size_t n) {
    uint8_t enc[COBS_ENCODE_MAX(512) + 16];
    uint8_t dec[512];
    size_t enc_len = cobs_encode(src, n, enc, sizeof(enc));
    TEST_ASSERT_TRUE(enc_len > 0 || n == 0);
    /* No 0x00 must appear in the encoded buffer. */
    for (size_t i = 0; i < enc_len; ++i) {
        TEST_ASSERT_NOT_EQUAL_HEX8(0x00, enc[i]);
    }
    size_t dec_len = cobs_decode(enc, enc_len, dec, sizeof(dec));
    TEST_ASSERT_EQUAL_size_t(n, dec_len);
    if (n > 0U) {
        TEST_ASSERT_EQUAL_MEMORY(src, dec, n);
    }
}

void test_single_zero(void) {
    const uint8_t src[] = {0x00};
    uint8_t enc[8] = {0};
    size_t enc_len = cobs_encode(src, sizeof(src), enc, sizeof(enc));
    TEST_ASSERT_EQUAL_size_t(2, enc_len);
    TEST_ASSERT_EQUAL_HEX8(0x01, enc[0]);
    TEST_ASSERT_EQUAL_HEX8(0x01, enc[1]);
}

void test_zero_in_middle(void) {
    const uint8_t src[] = {0x11, 0x22, 0x00, 0x33};
    uint8_t enc[8] = {0};
    size_t enc_len = cobs_encode(src, sizeof(src), enc, sizeof(enc));
    TEST_ASSERT_EQUAL_size_t(5, enc_len);
    TEST_ASSERT_EQUAL_HEX8(0x03, enc[0]);
    TEST_ASSERT_EQUAL_HEX8(0x11, enc[1]);
    TEST_ASSERT_EQUAL_HEX8(0x22, enc[2]);
    TEST_ASSERT_EQUAL_HEX8(0x02, enc[3]);
    TEST_ASSERT_EQUAL_HEX8(0x33, enc[4]);
}

void test_roundtrip_empty(void) {
    roundtrip(NULL, 0);
}

void test_roundtrip_no_zeros(void) {
    const uint8_t src[] = {0x11, 0x22, 0x33, 0x44, 0x55};
    roundtrip(src, sizeof(src));
}

void test_roundtrip_mixed(void) {
    uint8_t src[256];
    for (size_t i = 0; i < sizeof(src); ++i) src[i] = (uint8_t)i;
    roundtrip(src, sizeof(src));
}

void test_roundtrip_all_zero(void) {
    uint8_t src[20] = {0};
    roundtrip(src, sizeof(src));
}

void test_roundtrip_envelope_sized(void) {
    /* Stress at the size we'll actually carry over VCP — a Class A
     * state_estimate record is ~88 bytes including envelope + CRC. */
    uint8_t src[88];
    for (size_t i = 0; i < sizeof(src); ++i) {
        src[i] = (uint8_t)((i * 13U + (i % 17U == 0U ? 0U : 7U)) & 0xFFU);
    }
    roundtrip(src, sizeof(src));
}

void test_malformed_zero_byte(void) {
    /* A literal 0x00 inside the frame is invalid — decoder must reject. */
    const uint8_t bad[] = {0x02, 0x11, 0x00};
    uint8_t dec[8];
    size_t dec_len = cobs_decode(bad, sizeof(bad), dec, sizeof(dec));
    TEST_ASSERT_EQUAL_size_t(0, dec_len);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_single_zero);
    RUN_TEST(test_zero_in_middle);
    RUN_TEST(test_roundtrip_empty);
    RUN_TEST(test_roundtrip_no_zeros);
    RUN_TEST(test_roundtrip_mixed);
    RUN_TEST(test_roundtrip_all_zero);
    RUN_TEST(test_roundtrip_envelope_sized);
    RUN_TEST(test_malformed_zero_byte);
    return UNITY_END();
}
