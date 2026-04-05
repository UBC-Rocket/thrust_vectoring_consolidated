#include "unity.h"

void setUp(void)
{
}

void tearDown(void)
{
}

void test_icm40609_build_soft_reset(void);
void test_icm40609_build_set_bank(void);
void test_icm40609_build_set_bank_masks_high_bits(void);
void test_icm40609_build_read_reg(void);
void test_icm40609_build_read_reg_masks_high_bit(void);
void test_icm40609_build_write_reg(void);
void test_icm40609_build_write_reg_masks_high_bit(void);
void test_icm40609_build_pwr_mgmt0(void);
void test_icm40609_build_pwr_mgmt0_masks_mode_bits(void);
void test_icm40609_build_gyro_config_updates_device_state(void);
void test_icm40609_build_gyro_config_accepts_null_dev(void);
void test_icm40609_build_gyro_config_masks_inputs(void);
void test_icm40609_build_accel_config_updates_device_state(void);
void test_icm40609_build_accel_config_accepts_null_dev(void);
void test_icm40609_build_accel_config_masks_inputs(void);
void test_icm40609_build_int_config(void);
void test_icm40609_build_int_source0(void);
void test_icm40609_build_drive_config(void);
void test_icm40609_build_drive_config_masks_inputs(void);
void test_icm40609_build_intf_config1(void);
void test_icm40609_build_intf_config1_masks_inputs(void);
void test_icm40609_build_read_accel_gyro(void);
void test_icm40609_build_read_int_status(void);
void test_icm40609_build_fifo_config(void);
void test_icm40609_build_fifo_config_masks_inputs(void);
void test_icm40609_build_read_fifo_count(void);
void test_icm40609_build_read_fifo_data(void);
void test_icm40609_build_read_fifo_data_zero_bytes(void);
void test_icm40609_build_fifo_flush(void);

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_icm40609_build_soft_reset);
    RUN_TEST(test_icm40609_build_set_bank);
    RUN_TEST(test_icm40609_build_set_bank_masks_high_bits);
    RUN_TEST(test_icm40609_build_read_reg);
    RUN_TEST(test_icm40609_build_read_reg_masks_high_bit);
    RUN_TEST(test_icm40609_build_write_reg);
    RUN_TEST(test_icm40609_build_write_reg_masks_high_bit);
    RUN_TEST(test_icm40609_build_pwr_mgmt0);
    RUN_TEST(test_icm40609_build_pwr_mgmt0_masks_mode_bits);
    RUN_TEST(test_icm40609_build_gyro_config_updates_device_state);
    RUN_TEST(test_icm40609_build_gyro_config_accepts_null_dev);
    RUN_TEST(test_icm40609_build_gyro_config_masks_inputs);
    RUN_TEST(test_icm40609_build_accel_config_updates_device_state);
    RUN_TEST(test_icm40609_build_accel_config_accepts_null_dev);
    RUN_TEST(test_icm40609_build_accel_config_masks_inputs);
    RUN_TEST(test_icm40609_build_int_config);
    RUN_TEST(test_icm40609_build_int_source0);
    RUN_TEST(test_icm40609_build_drive_config);
    RUN_TEST(test_icm40609_build_drive_config_masks_inputs);
    RUN_TEST(test_icm40609_build_intf_config1);
    RUN_TEST(test_icm40609_build_intf_config1_masks_inputs);
    RUN_TEST(test_icm40609_build_read_accel_gyro);
    RUN_TEST(test_icm40609_build_read_int_status);
    RUN_TEST(test_icm40609_build_fifo_config);
    RUN_TEST(test_icm40609_build_fifo_config_masks_inputs);
    RUN_TEST(test_icm40609_build_read_fifo_count);
    RUN_TEST(test_icm40609_build_read_fifo_data);
    RUN_TEST(test_icm40609_build_read_fifo_data_zero_bytes);
    RUN_TEST(test_icm40609_build_fifo_flush);

    return UNITY_END();
}
