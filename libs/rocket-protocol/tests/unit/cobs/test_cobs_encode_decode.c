#include "unity.h"

#include <stdint.h>

#include "rp/cobs/cobs.h"

#define COBS_MAX_TEST_ENCODED_SIZE (1024)

void setUp(void)
{
}

void tearDown(void)
{
}

static void test_encode_decode_success(uint8_t *data, size_t data_size, uint8_t *encoded,
                                       size_t encoded_size, uint8_t *decoded, size_t decoded_size)
{
    cobs_result_t encode_result = cobs_encode(data, data_size, encoded, encoded_size);
    size_t encode_written = encode_result.written;

    TEST_ASSERT_EQUAL(COBS_OK, encode_result.status);

    cobs_result_t decode_result = cobs_decode(encoded, encode_written, decoded, decoded_size);

    TEST_ASSERT_EQUAL(COBS_OK, decode_result.status);
    TEST_ASSERT_EQUAL(data_size, decode_result.written);

    if (data_size > 0) {
        TEST_ASSERT_EQUAL_UINT8_ARRAY(data, decoded, data_size);
    }
}

void test_cobs_decode_0_size_data(void)
{
    // Dummy buffer so that we have something to pass to the functions,
    // actually unused since the data size is 0
    uint8_t data[1];

    uint8_t encoded[COBS_MAX_TEST_ENCODED_SIZE];
    uint8_t decoded[sizeof(data)];

    test_encode_decode_success(data, 0, encoded, sizeof(encoded), decoded, sizeof(decoded));
}

void test_cobs_decode_only_delimiter_data(void)
{
    uint8_t data[] = {COBS_DELIMITER_BYTE};

    uint8_t encoded[COBS_MAX_TEST_ENCODED_SIZE];
    uint8_t decoded[sizeof(data)];

    test_encode_decode_success(data, sizeof(data), encoded, sizeof(encoded), decoded,
                               sizeof(decoded));
}

void test_cobs_decode_delimiter_interleaved_data(void)
{
    uint8_t data[] = {0x11,
                      0x22,
                      COBS_DELIMITER_BYTE,
                      0x33,
                      COBS_DELIMITER_BYTE,
                      0x55,
                      COBS_DELIMITER_BYTE,
                      COBS_DELIMITER_BYTE,
                      0x66};

    uint8_t encoded[COBS_MAX_TEST_ENCODED_SIZE];
    uint8_t decoded[sizeof(data)];

    test_encode_decode_success(data, sizeof(data), encoded, sizeof(encoded), decoded,
                               sizeof(decoded));
}

void test_cobs_decode_random_data_1(void)
{
    uint8_t data[] = {0x65, 0x32, 0x34, 0x34, 0x39, 0x62, 0x38, 0x66, 0x62, 0x65, 0x30,
                      0x33, 0x63, 0x63, 0x36, 0x36, 0x39, 0x39, 0x64, 0x66, 0x35, 0x38,
                      0x65, 0x37, 0x32, 0x62, 0x66, 0x35, 0x38, 0x61, 0x37, 0x36};

    uint8_t encoded[COBS_MAX_TEST_ENCODED_SIZE];
    uint8_t decoded[sizeof(data)];

    test_encode_decode_success(data, sizeof(data), encoded, sizeof(encoded), decoded,
                               sizeof(decoded));
}

void test_cobs_decode_random_data_2(void)
{
    uint8_t data[] = {0x34, 0x61, 0x64, 0x39, 0x34, 0x35, 0x65, 0x31, 0x63, 0x30, 0x39, 0x61, 0x38,
                      0x32, 0x36, 0x61, 0x31, 0x32, 0x61, 0x61, 0x35, 0x38, 0x35, 0x61, 0x39, 0x38,
                      0x35, 0x33, 0x31, 0x37, 0x34, 0x63, 0x37, 0x39, 0x37, 0x66, 0x39, 0x31, 0x35,
                      0x66, 0x31, 0x36, 0x30, 0x34, 0x33, 0x65, 0x34, 0x36, 0x32, 0x39, 0x30, 0x30,
                      0x37, 0x35, 0x39, 0x62, 0x38, 0x64, 0x39, 0x37, 0x31, 0x63, 0x63, 0x33};

    uint8_t encoded[COBS_MAX_TEST_ENCODED_SIZE];
    uint8_t decoded[sizeof(data)];

    test_encode_decode_success(data, sizeof(data), encoded, sizeof(encoded), decoded,
                               sizeof(decoded));
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_cobs_decode_0_size_data);
    RUN_TEST(test_cobs_decode_only_delimiter_data);
    RUN_TEST(test_cobs_decode_delimiter_interleaved_data);
    RUN_TEST(test_cobs_decode_random_data_1);
    RUN_TEST(test_cobs_decode_random_data_2);

    return UNITY_END();
}
