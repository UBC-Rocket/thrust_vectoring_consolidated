#include "unity.h"
#include "messages/crc16.h"

#include <string.h>

void test_crc_ccitt_false_check_vector(void)
{
    /* Standard CCITT-FALSE check vector: "123456789" -> 0x29B1. */
    const char *s = "123456789";
    uint16_t crc = messages_crc16((const uint8_t *)s, strlen(s));
    TEST_ASSERT_EQUAL_HEX16(0x29B1, crc);
}

void test_crc_empty_buffer(void)
{
    uint16_t crc = messages_crc16(NULL, 0U);
    TEST_ASSERT_EQUAL_HEX16(MESSAGES_CRC16_INIT, crc);
}

void test_crc_resumable(void)
{
    const char *s = "123456789";
    /* Split: "1234" + "56789" must equal the whole. */
    uint16_t a = messages_crc16_update(MESSAGES_CRC16_INIT, (const uint8_t *)s, 4U);
    uint16_t b = messages_crc16_update(a, (const uint8_t *)s + 4, 5U);
    TEST_ASSERT_EQUAL_HEX16(0x29B1, b);
}
