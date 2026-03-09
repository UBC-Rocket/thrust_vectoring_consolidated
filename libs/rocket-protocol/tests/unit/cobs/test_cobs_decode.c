#include "unity.h"

#include <stdint.h>

#include "rp/cobs/cobs.h"

void setUp(void)
{
}

void tearDown(void)
{
}

void test_cobs_decode_0_size_data(void)
{
    uint8_t data[] = {0x01, COBS_DELIMITER_BYTE};

    uint8_t actual[1];

    cobs_result_t result = cobs_decode(data, sizeof(data), actual, sizeof(actual));

    TEST_ASSERT_EQUAL(COBS_OK, result.status);
    TEST_ASSERT_EQUAL(0, result.written);
}

void test_cobs_decode_only_1_delimiter(void)
{
    uint8_t data[] = {0x01, 0x01, COBS_DELIMITER_BYTE};
    uint8_t expected[] = {COBS_DELIMITER_BYTE};

    uint8_t actual[sizeof(expected)];

    cobs_result_t result = cobs_decode(data, sizeof(data), actual, sizeof(actual));

    TEST_ASSERT_EQUAL(COBS_OK, result.status);
    TEST_ASSERT_EQUAL(sizeof(expected), result.written);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, actual, sizeof(actual));
}

void test_cobs_decode_only_2_delimeters(void)
{
    uint8_t data[] = {0x01, 0x01, 0x01, COBS_DELIMITER_BYTE};
    uint8_t expected[] = {COBS_DELIMITER_BYTE, COBS_DELIMITER_BYTE};

    uint8_t actual[sizeof(expected)];

    cobs_result_t result = cobs_decode(data, sizeof(data), actual, sizeof(actual));

    TEST_ASSERT_EQUAL(COBS_OK, result.status);
    TEST_ASSERT_EQUAL(sizeof(expected), result.written);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, actual, sizeof(actual));
}

void test_cobs_decode_data_and_delimiter(void)
{
    uint8_t data[] = {0x03, 0x11, 0x22, 0x02, 0x33, COBS_DELIMITER_BYTE};
    uint8_t expected[] = {0x11, 0x22, COBS_DELIMITER_BYTE, 0x33};

    uint8_t actual[sizeof(expected)];

    cobs_result_t result = cobs_decode(data, sizeof(data), actual, sizeof(actual));

    TEST_ASSERT_EQUAL(COBS_OK, result.status);
    TEST_ASSERT_EQUAL(sizeof(expected), result.written);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, actual, sizeof(actual));
}

void test_cobs_decode_no_delimiter(void)
{
    uint8_t data[] = {0x05, 0x11, 0x22, 0x33, 0x44, COBS_DELIMITER_BYTE};
    uint8_t expected[] = {0x11, 0x22, 0x33, 0x44};

    uint8_t actual[sizeof(expected)];

    cobs_result_t result = cobs_decode(data, sizeof(data), actual, sizeof(actual));

    TEST_ASSERT_EQUAL(COBS_OK, result.status);
    TEST_ASSERT_EQUAL(sizeof(expected), result.written);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, actual, sizeof(actual));
}

void test_cobs_decode_max_offset_no_restart(void)
{
    // FF 01 02 03 ... FD FE 00
    uint8_t data[0xFE + 1 + 1];

    // 01 02 03 ... FD FE
    uint8_t expected[0xFE];

    for (size_t byte = 0x01; byte <= 0xFE; byte++) {
        expected[byte - 1] = byte;
        data[byte] = byte;
    }

    data[0] = 0xFF;
    data[sizeof(data) - 1] = COBS_DELIMITER_BYTE;

    uint8_t actual[sizeof(expected)];

    cobs_result_t result = cobs_decode(data, sizeof(data), actual, sizeof(actual));

    TEST_ASSERT_EQUAL(COBS_OK, result.status);
    TEST_ASSERT_EQUAL(sizeof(expected), result.written);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, actual, sizeof(actual));
}

void test_cobs_decode_max_offset_with_restart(void)
{
    // FF 01 02 03 ... FD FE 02 FF 00
    uint8_t data[0xFF + 2 + 1];

    // 01 02 03 ... FD FE FF
    uint8_t expected[0xFF];

    for (size_t byte = 0x01; byte <= 0xFF; byte++) {
        expected[byte - 1] = byte;
        data[byte] = byte;
    }

    data[0] = 0xFF;
    data[sizeof(data) - 3] = 0x02;
    data[sizeof(data) - 2] = 0xFF;
    data[sizeof(data) - 1] = COBS_DELIMITER_BYTE;

    uint8_t actual[sizeof(expected)];

    cobs_result_t result = cobs_decode(data, sizeof(data), actual, sizeof(actual));

    TEST_ASSERT_EQUAL(COBS_OK, result.status);
    TEST_ASSERT_EQUAL(sizeof(expected), result.written);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, actual, sizeof(actual));
}

void test_cobs_decode_input_too_short_minimum(void)
{
    uint8_t data[1];
    uint8_t actual[1];

    cobs_result_t result = cobs_decode(data, 1, actual, sizeof(actual));

    TEST_ASSERT_EQUAL(COBS_INPUT_TOO_SHORT, result.status);
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_cobs_decode_0_size_data);
    RUN_TEST(test_cobs_decode_only_1_delimiter);
    RUN_TEST(test_cobs_decode_only_2_delimeters);
    RUN_TEST(test_cobs_decode_data_and_delimiter);
    RUN_TEST(test_cobs_decode_no_delimiter);
    RUN_TEST(test_cobs_decode_max_offset_no_restart);
    RUN_TEST(test_cobs_decode_max_offset_with_restart);
    RUN_TEST(test_cobs_decode_input_too_short_minimum);

    return UNITY_END();
}
