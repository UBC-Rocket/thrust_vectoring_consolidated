#include "unity.h"

void setUp(void)
{
}

void tearDown(void)
{
}

void test_icm40609_init_with_config_success(void);
void test_icm40609_init_with_config_rejects_wrong_who_am_i(void);
void test_icm40609_init_with_config_rejects_bad_power_readback(void);
void test_icm40609_init_with_config_rejects_missing_data_ready(void);
void test_icm40609_init_with_config_returns_parse_error_when_dev_null(void);
void test_icm40609_init_uses_default_config(void);
void test_icm40609_data_ready_interrupt_submits_expected_job(void);
void test_icm40609_done_enqueues_sample_from_captured_job(void);

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_icm40609_init_with_config_success);
    RUN_TEST(test_icm40609_init_with_config_rejects_wrong_who_am_i);
    RUN_TEST(test_icm40609_init_with_config_rejects_bad_power_readback);
    RUN_TEST(test_icm40609_init_with_config_rejects_missing_data_ready);
    RUN_TEST(test_icm40609_init_with_config_returns_parse_error_when_dev_null);
    RUN_TEST(test_icm40609_init_uses_default_config);
    RUN_TEST(test_icm40609_data_ready_interrupt_submits_expected_job);
    RUN_TEST(test_icm40609_done_enqueues_sample_from_captured_job);

    return UNITY_END();
}
