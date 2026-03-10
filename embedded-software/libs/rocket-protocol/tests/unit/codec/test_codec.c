#include "rp/codec.h"
#include "unity.h"

#include <stdbool.h>
#include <stdint.h>
#include "pb_encode.h"

#include "proto/codec_test_data.pb.h"
#include "unity_internals.h"

void setUp(void)
{
}

void tearDown(void)
{
}

void test_codec_encode_should_overflow(void)
{
    codec_test_data_t input_message = {
        .d = 3.1415926,
        .ui32 = 1234567890,
        .f = 0.0,
        .b1 = true,
        .b2 = false,
        .which_oo = CODEC_TEST_DATA_MO_TAG,
        .oo =
            {
                .mo = MY_OPTION_MY_OPTIONS_VALUE2,
            },
    };

    uint8_t packet[1];

    rp_packet_encode_result_t encode_result =
        rp_packet_encode(packet, sizeof(packet), CODEC_TEST_DATA_FIELDS, &input_message);

    TEST_ASSERT_EQUAL(RP_CODEC_OVERFLOW, encode_result.status);
}

void test_codec_decode_should_error(void)
{
    codec_test_data_t input_message = {
        .d = 3.1415926,
        .ui32 = 1234567890,
        .f = 0.0,
        .b1 = true,
        .b2 = false,
        .which_oo = CODEC_TEST_DATA_MO_TAG,
        .oo =
            {
                .mo = MY_OPTION_MY_OPTIONS_VALUE2,
            },
    };

    codec_test_data_t output_message = CODEC_TEST_DATA_INIT_DEFAULT;

    uint8_t packet[RP_PACKET_MAX_SIZE];

    rp_packet_encode_result_t encode_result =
        rp_packet_encode(packet, sizeof(packet), CODEC_TEST_DATA_FIELDS, &input_message);

    TEST_ASSERT_EQUAL(RP_CODEC_OK, encode_result.status);
    TEST_ASSERT_GREATER_THAN(0, encode_result.written);

    // Purposefully mess with the COBS encoding
    packet[0] += 0xDA;

    rp_packet_decode_result_t decode_result =
        rp_packet_decode(packet, encode_result.written, CODEC_TEST_DATA_FIELDS, &output_message);

    TEST_ASSERT_NOT_EQUAL(RP_CODEC_OK, decode_result.status);
}

void test_codec_decode_should_checksum_mismatch(void)
{
    codec_test_data_t input_message = {
        .d = 3.1415926,
        .ui32 = 1234567890,
        .f = 0.0,
        .b1 = true,
        .b2 = false,
        .which_oo = CODEC_TEST_DATA_MO_TAG,
        .oo =
            {
                .mo = MY_OPTION_MY_OPTIONS_VALUE2,
            },
    };

    codec_test_data_t output_message = CODEC_TEST_DATA_INIT_DEFAULT;

    uint8_t packet[RP_PACKET_MAX_SIZE];

    rp_packet_encode_result_t encode_result =
        rp_packet_encode(packet, sizeof(packet), CODEC_TEST_DATA_FIELDS, &input_message);

    TEST_ASSERT_EQUAL(RP_CODEC_OK, encode_result.status);
    TEST_ASSERT_GREATER_THAN(0, encode_result.written);

    // Corrupt a data byte
    packet[1]++;

    rp_packet_decode_result_t decode_result =
        rp_packet_decode(packet, encode_result.written, CODEC_TEST_DATA_FIELDS, &output_message);

    TEST_ASSERT_EQUAL(RP_CODEC_CHECKSUM_MISMATCH, decode_result.status);
}

void test_codec_encode_decode_should_succeed(void)
{
    codec_test_data_t input_message = {
        .d = 3.1415926,
        .ui32 = 1234567890,
        .f = 0.0,
        .b1 = true,
        .b2 = false,
        .which_oo = CODEC_TEST_DATA_MO_TAG,
        .oo =
            {
                .mo = MY_OPTION_MY_OPTIONS_VALUE2,
            },
    };

    codec_test_data_t output_message = CODEC_TEST_DATA_INIT_DEFAULT;

    uint8_t packet[RP_PACKET_MAX_SIZE];

    rp_packet_encode_result_t encode_result =
        rp_packet_encode(packet, sizeof(packet), CODEC_TEST_DATA_FIELDS, &input_message);

    TEST_ASSERT_EQUAL(RP_CODEC_OK, encode_result.status);
    TEST_ASSERT_GREATER_THAN(0, encode_result.written);

    rp_packet_decode_result_t decode_result =
        rp_packet_decode(packet, encode_result.written, CODEC_TEST_DATA_FIELDS, &output_message);

    TEST_ASSERT_EQUAL(RP_CODEC_OK, decode_result.status);

    TEST_ASSERT(input_message.d == output_message.d);
    TEST_ASSERT(input_message.ui32 == output_message.ui32);
    TEST_ASSERT(input_message.f == output_message.f);
    TEST_ASSERT(input_message.b1 == output_message.b1);
    TEST_ASSERT(input_message.b2 == output_message.b2);
    TEST_ASSERT(input_message.which_oo == output_message.which_oo);
    TEST_ASSERT(input_message.oo.mo == output_message.oo.mo);
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_codec_encode_should_overflow);
    RUN_TEST(test_codec_decode_should_error);
    RUN_TEST(test_codec_decode_should_checksum_mismatch);
    RUN_TEST(test_codec_encode_decode_should_succeed);

    return UNITY_END();
}
