#include "unity.h"

#include <stdint.h>

#include "rp/cobs/cobs.h"
#include "unity_internals.h"

void setUp(void)
{
}

void tearDown(void)
{
}

void test_cobs_get_max_encoded_size_0_size(void)
{
    size_t data_size = 0;

    size_t encoded_size = cobs_get_max_encoded_size(data_size);

    TEST_ASSERT_EQUAL(1 + 1, encoded_size);
}

void test_cobs_get_max_encoded_size_1_size(void)
{
    size_t data_size = 1;

    size_t encoded_size = cobs_get_max_encoded_size(data_size);

    TEST_ASSERT_EQUAL(1 + 1 + 1, encoded_size);
}

void test_cobs_get_max_encoded_size_1_block(void)
{
    size_t data_size = 254;

    size_t encoded_size = cobs_get_max_encoded_size(data_size);

    TEST_ASSERT_EQUAL(1 + 254 + 1, encoded_size);
}

void test_cobs_get_max_encoded_size_2_blocks(void)
{
    size_t data_size = 254 + 1;

    size_t encoded_size = cobs_get_max_encoded_size(data_size);

    TEST_ASSERT_EQUAL(1 + 254 + 1 + 1 + 1, encoded_size);
}

void test_cobs_get_max_encoded_size_3_blocks(void)
{
    size_t data_size = 254 + 254 + 1;

    size_t encoded_size = cobs_get_max_encoded_size(data_size);

    TEST_ASSERT_EQUAL(1 + 254 + 1 + 254 + 1 + 1 + 1, encoded_size);
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_cobs_get_max_encoded_size_0_size);
    RUN_TEST(test_cobs_get_max_encoded_size_1_size);
    RUN_TEST(test_cobs_get_max_encoded_size_1_block);
    RUN_TEST(test_cobs_get_max_encoded_size_2_blocks);
    RUN_TEST(test_cobs_get_max_encoded_size_3_blocks);

    return UNITY_END();
}
