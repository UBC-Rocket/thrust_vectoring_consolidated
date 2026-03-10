#include "unity.h"

#include <stdint.h>

#include "rp/crc/crc.h"

void setUp(void)
{
}

void tearDown(void)
{
}

void test_crc16_ccitt_checksum_correct(void)
{
    const uint8_t data[] = "123456789";
    const uint16_t expected = 0x2189;

    const uint16_t checksum = crc16_ccitt(data, sizeof(data) - 1);

    TEST_ASSERT_EQUAL_UINT16(expected, checksum);
}

void test_crc16_ccitt_check_with_correct_codeword(void)
{
    const uint8_t codeword[] = "123456789\x89\x21";

    const uint16_t residue = crc16_ccitt(codeword, sizeof(codeword) - 1);

    TEST_ASSERT_EQUAL_UINT16(CRC16_CCITT_RESIDUE, residue);
}

void test_crc16_ccitt_check_with_incorrect_codeword(void)
{
    const uint8_t codeword[] = "023456789\x89\x21";

    const uint16_t residue = crc16_ccitt(codeword, sizeof(codeword) - 1);

    TEST_ASSERT_NOT_EQUAL_UINT16(CRC16_CCITT_RESIDUE, residue);
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_crc16_ccitt_checksum_correct);
    RUN_TEST(test_crc16_ccitt_check_with_correct_codeword);
    RUN_TEST(test_crc16_ccitt_check_with_incorrect_codeword);

    return UNITY_END();
}
