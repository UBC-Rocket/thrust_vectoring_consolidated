#include "unity.h"

#include <stdint.h>

#include "rp/cobs/cobs.h"

void setUp(void)
{
}

void tearDown(void)
{
}

void test_cobs_encode_0_size_data(void)
{
    uint8_t data[] = {};
    uint8_t expected[] = {0x01, COBS_DELIMITER_BYTE};

    uint8_t actual[sizeof(expected)];

    cobs_result_t result = cobs_encode(data, sizeof(data), actual, sizeof(actual));

    TEST_ASSERT_EQUAL(COBS_OK, result.status);
    TEST_ASSERT_EQUAL(sizeof(expected), result.written);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, actual, sizeof(actual));
}

void test_cobs_encode_only_1_delimiter(void)
{
    uint8_t data[] = {COBS_DELIMITER_BYTE};
    uint8_t expected[] = {0x01, 0x01, COBS_DELIMITER_BYTE};

    uint8_t actual[sizeof(expected)];

    cobs_result_t result = cobs_encode(data, sizeof(data), actual, sizeof(actual));

    TEST_ASSERT_EQUAL(COBS_OK, result.status);
    TEST_ASSERT_EQUAL(sizeof(expected), result.written);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, actual, sizeof(actual));
}

void test_cobs_encode_only_2_delimeters(void)
{
    uint8_t data[] = {COBS_DELIMITER_BYTE, COBS_DELIMITER_BYTE};
    uint8_t expected[] = {0x01, 0x01, 0x01, COBS_DELIMITER_BYTE};

    uint8_t actual[sizeof(expected)];

    cobs_result_t result = cobs_encode(data, sizeof(data), actual, sizeof(actual));

    TEST_ASSERT_EQUAL(COBS_OK, result.status);
    TEST_ASSERT_EQUAL(sizeof(expected), result.written);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, actual, sizeof(actual));
}

void test_cobs_encode_data_and_delimiter(void)
{
    uint8_t data[] = {0x11, 0x22, COBS_DELIMITER_BYTE, 0x33};
    uint8_t expected[] = {0x03, 0x11, 0x22, 0x02, 0x33, COBS_DELIMITER_BYTE};

    uint8_t actual[sizeof(expected)];

    cobs_result_t result = cobs_encode(data, sizeof(data), actual, sizeof(actual));

    TEST_ASSERT_EQUAL(COBS_OK, result.status);
    TEST_ASSERT_EQUAL(sizeof(expected), result.written);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, actual, sizeof(actual));
}

void test_cobs_encode_no_delimiter(void)
{
    uint8_t data[] = {0x11, 0x22, 0x33, 0x44};
    uint8_t expected[] = {0x05, 0x11, 0x22, 0x33, 0x44, COBS_DELIMITER_BYTE};

    uint8_t actual[sizeof(expected)];

    cobs_result_t result = cobs_encode(data, sizeof(data), actual, sizeof(actual));

    TEST_ASSERT_EQUAL(COBS_OK, result.status);
    TEST_ASSERT_EQUAL(sizeof(expected), result.written);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, actual, sizeof(actual));
}

void test_cobs_encode_max_offset_no_restart(void)
{
    // 01 02 03 ... FD FE
    uint8_t data[0xFE];

    // FF 01 02 03 ... FD FE 00
    uint8_t expected[0xFE + 1 + 1];

    for (size_t byte = 0x01; byte <= 0xFE; byte++) {
        data[byte - 1] = byte;
        expected[byte] = byte;
    }

    expected[0] = 0xFF;
    expected[sizeof(expected) - 1] = COBS_DELIMITER_BYTE;

    uint8_t actual[sizeof(expected)];

    cobs_result_t result = cobs_encode(data, sizeof(data), actual, sizeof(actual));

    TEST_ASSERT_EQUAL(COBS_OK, result.status);
    TEST_ASSERT_EQUAL(sizeof(expected), result.written);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, actual, sizeof(actual));
}

void test_cobs_encode_max_offset_with_restart(void)
{
    // 01 02 03 ... FD FE FF
    uint8_t data[0xFF];

    // FF 01 02 03 ... FD FE 02 FF 00
    uint8_t expected[0xFF + 2 + 1];

    for (size_t byte = 0x01; byte <= 0xFF; byte++) {
        data[byte - 1] = byte;
        expected[byte] = byte;
    }

    expected[0] = 0xFF;
    expected[sizeof(expected) - 3] = 0x02;
    expected[sizeof(expected) - 2] = 0xFF;
    expected[sizeof(expected) - 1] = COBS_DELIMITER_BYTE;

    uint8_t actual[sizeof(expected)];

    cobs_result_t result = cobs_encode(data, sizeof(data), actual, sizeof(actual));

    TEST_ASSERT_EQUAL(COBS_OK, result.status);
    TEST_ASSERT_EQUAL(sizeof(expected), result.written);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, actual, sizeof(actual));
}

void test_cobs_encode_overflow_minimum(void)
{
    uint8_t data[1];
    uint8_t actual[1];

    cobs_result_t result = cobs_encode(data, 0, actual, sizeof(actual));

    TEST_ASSERT_EQUAL(COBS_OUTPUT_OVERFLOW, result.status);
}

void test_cobs_encode_overflow_long(void)
{
    // 01 02 03 ... FD FE
    uint8_t data[0xFE];
    uint8_t actual[0xFE];

    for (size_t byte = 0x01; byte <= 0xFE; byte++) {
        data[byte - 1] = byte;
    }

    cobs_result_t result = cobs_encode(data, sizeof(data), actual, sizeof(actual));

    // Should fail somewhere in the middle of a block
    TEST_ASSERT_EQUAL(COBS_OUTPUT_OVERFLOW, result.status);
}

void test_cobs_encode_overflow_final_delimiter(void)
{
    // 01 02 03 ... FD FE FF
    uint8_t data[0xFF];
    uint8_t actual[0xFF + 2];

    for (size_t byte = 0x01; byte <= 0xFF; byte++) {
        data[byte - 1] = byte;
    }

    cobs_result_t result = cobs_encode(data, sizeof(data), actual, sizeof(actual));

    // Should not be able to write in final delimiter
    TEST_ASSERT_EQUAL(COBS_OUTPUT_OVERFLOW, result.status);
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_cobs_encode_0_size_data);
    RUN_TEST(test_cobs_encode_only_1_delimiter);
    RUN_TEST(test_cobs_encode_only_2_delimeters);
    RUN_TEST(test_cobs_encode_data_and_delimiter);
    RUN_TEST(test_cobs_encode_no_delimiter);
    RUN_TEST(test_cobs_encode_max_offset_no_restart);
    RUN_TEST(test_cobs_encode_max_offset_with_restart);
    RUN_TEST(test_cobs_encode_overflow_minimum);
    RUN_TEST(test_cobs_encode_overflow_long);
    RUN_TEST(test_cobs_encode_overflow_final_delimiter);

    return UNITY_END();
}
