#include "unity.h"

#include "sensors/icm40609.h"

#include <stddef.h>
#include <stdint.h>

static void assert_tx2(const uint8_t *tx, uint8_t b0, uint8_t b1)
{
    TEST_ASSERT_EQUAL_UINT8(b0, tx[0]);
    TEST_ASSERT_EQUAL_UINT8(b1, tx[1]);
}

static void fill_sentinel(uint8_t *buf, size_t len)
{
    for (size_t i = 0; i < len; ++i) {
        buf[i] = 0xA5;
    }
}

void test_icm40609_build_soft_reset(void)
{
    uint8_t tx[2];
    fill_sentinel(tx, sizeof(tx));

    TEST_ASSERT_EQUAL_UINT8(2, icm40609_build_soft_reset(tx));
    assert_tx2(tx, 0x11, 0x01);
}

void test_icm40609_build_set_bank(void)
{
    uint8_t tx[2];
    fill_sentinel(tx, sizeof(tx));

    TEST_ASSERT_EQUAL_UINT8(2, icm40609_build_set_bank(ICM40609_BANK_4, tx));
    assert_tx2(tx, 0x76, 0x04);
}

void test_icm40609_build_set_bank_masks_high_bits(void)
{
    uint8_t tx[2];
    fill_sentinel(tx, sizeof(tx));

    TEST_ASSERT_EQUAL_UINT8(2, icm40609_build_set_bank((icm40609_bank_t)0xFF, tx));
    assert_tx2(tx, 0x76, 0x07);
}

void test_icm40609_build_read_reg(void)
{
    uint8_t tx[2];
    fill_sentinel(tx, sizeof(tx));

    TEST_ASSERT_EQUAL_UINT8(2, icm40609_build_read_reg(ICM40609_REG_WHO_AM_I, tx));
    assert_tx2(tx, 0xF5, 0x00);
}

void test_icm40609_build_read_reg_masks_high_bit(void)
{
    uint8_t tx[2];
    fill_sentinel(tx, sizeof(tx));

    TEST_ASSERT_EQUAL_UINT8(2, icm40609_build_read_reg(0xFF, tx));
    assert_tx2(tx, 0xFF, 0x00);
}

void test_icm40609_build_write_reg(void)
{
    uint8_t tx[2];
    fill_sentinel(tx, sizeof(tx));

    TEST_ASSERT_EQUAL_UINT8(2, icm40609_build_write_reg(ICM40609_REG_PWR_MGMT0, 0x2F, tx));
    assert_tx2(tx, 0x4E, 0x2F);
}

void test_icm40609_build_write_reg_masks_high_bit(void)
{
    uint8_t tx[2];
    fill_sentinel(tx, sizeof(tx));

    TEST_ASSERT_EQUAL_UINT8(2, icm40609_build_write_reg(0xFF, 0xAA, tx));
    assert_tx2(tx, 0x7F, 0xAA);
}

void test_icm40609_build_pwr_mgmt0(void)
{
    uint8_t tx[2];
    fill_sentinel(tx, sizeof(tx));

    TEST_ASSERT_EQUAL_UINT8(2, icm40609_build_pwr_mgmt0(ICM40609_GYRO_MODE_LN,
                                                        ICM40609_ACCEL_MODE_LP,
                                                        true,
                                                        tx));
    assert_tx2(tx, 0x4E, 0x2E);
}

void test_icm40609_build_pwr_mgmt0_masks_mode_bits(void)
{
    uint8_t tx[2];
    fill_sentinel(tx, sizeof(tx));

    TEST_ASSERT_EQUAL_UINT8(2, icm40609_build_pwr_mgmt0((icm40609_gyro_mode_t)0xFF,
                                                        (icm40609_accel_mode_t)0xFF,
                                                        false,
                                                        tx));
    assert_tx2(tx, 0x4E, 0x0F);
}

void test_icm40609_build_gyro_config_updates_device_state(void)
{
    uint8_t tx[2];
    icm40609_t dev = {0};
    fill_sentinel(tx, sizeof(tx));

    TEST_ASSERT_EQUAL_UINT8(2, icm40609_build_gyro_config(ICM40609_GYRO_FS_250DPS,
                                                          ICM40609_ODR_1KHZ,
                                                          tx,
                                                          &dev));
    assert_tx2(tx, 0x4F, 0x66);
    TEST_ASSERT_EQUAL_UINT8(ICM40609_GYRO_FS_250DPS, dev.gyro_fs_sel);
    TEST_ASSERT_EQUAL_UINT8(ICM40609_ODR_1KHZ, dev.gyro_odr);
    TEST_ASSERT_EQUAL_UINT16(1000, dev.gyro_odr_hz);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.0f / 131.0f, dev.gyro_scale);
}

void test_icm40609_build_gyro_config_accepts_null_dev(void)
{
    uint8_t tx[2];
    fill_sentinel(tx, sizeof(tx));

    TEST_ASSERT_EQUAL_UINT8(2, icm40609_build_gyro_config(ICM40609_GYRO_FS_1000DPS,
                                                          ICM40609_ODR_200HZ,
                                                          tx,
                                                          NULL));
    assert_tx2(tx, 0x4F, 0x27);
}

void test_icm40609_build_gyro_config_masks_inputs(void)
{
    uint8_t tx[2];
    icm40609_t dev = {0};
    fill_sentinel(tx, sizeof(tx));

    TEST_ASSERT_EQUAL_UINT8(2, icm40609_build_gyro_config((icm40609_gyro_fs_t)0xFF,
                                                          (icm40609_odr_t)0xFF,
                                                          tx,
                                                          &dev));
    assert_tx2(tx, 0x4F, 0xEF);
}

void test_icm40609_build_accel_config_updates_device_state(void)
{
    uint8_t tx[2];
    icm40609_t dev = {0};
    fill_sentinel(tx, sizeof(tx));

    TEST_ASSERT_EQUAL_UINT8(2, icm40609_build_accel_config(ICM40609_ACCEL_FS_8G,
                                                           ICM40609_ODR_500HZ,
                                                           tx,
                                                           &dev));
    assert_tx2(tx, 0x50, 0x4F);
    TEST_ASSERT_EQUAL_UINT8(ICM40609_ACCEL_FS_8G, dev.accel_fs_sel);
    TEST_ASSERT_EQUAL_UINT8(ICM40609_ODR_500HZ, dev.accel_odr);
    TEST_ASSERT_EQUAL_UINT16(500, dev.accel_odr_hz);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.0f / 4096.0f, dev.accel_scale);
}

void test_icm40609_build_accel_config_accepts_null_dev(void)
{
    uint8_t tx[2];
    fill_sentinel(tx, sizeof(tx));

    TEST_ASSERT_EQUAL_UINT8(2, icm40609_build_accel_config(ICM40609_ACCEL_FS_16G,
                                                           ICM40609_ODR_100HZ,
                                                           tx,
                                                           NULL));
    assert_tx2(tx, 0x50, 0x28);
}

void test_icm40609_build_accel_config_masks_inputs(void)
{
    uint8_t tx[2];
    icm40609_t dev = {0};
    fill_sentinel(tx, sizeof(tx));

    TEST_ASSERT_EQUAL_UINT8(2, icm40609_build_accel_config((icm40609_accel_fs_t)0xFF,
                                                           (icm40609_odr_t)0xFF,
                                                           tx,
                                                           &dev));
    assert_tx2(tx, 0x50, 0xEF);
}

void test_icm40609_build_int_config(void)
{
    uint8_t tx[2];
    fill_sentinel(tx, sizeof(tx));

    TEST_ASSERT_EQUAL_UINT8(2, icm40609_build_int_config(ICM40609_INT_ACTIVE_HIGH,
                                                        ICM40609_INT_PUSH_PULL,
                                                        ICM40609_INT_LATCHED,
                                                        tx));
    assert_tx2(tx, 0x14, 0x07);
}

void test_icm40609_build_int_source0(void)
{
    uint8_t tx[2];
    fill_sentinel(tx, sizeof(tx));

    TEST_ASSERT_EQUAL_UINT8(2, icm40609_build_int_source0(true, false, true, tx));
    assert_tx2(tx, 0x65, 0x0A);
}

void test_icm40609_build_drive_config(void)
{
    uint8_t tx[2];
    fill_sentinel(tx, sizeof(tx));

    TEST_ASSERT_EQUAL_UINT8(2, icm40609_build_drive_config(5, tx));
    assert_tx2(tx, 0x13, 0x05);
}

void test_icm40609_build_drive_config_masks_inputs(void)
{
    uint8_t tx[2];
    fill_sentinel(tx, sizeof(tx));

    TEST_ASSERT_EQUAL_UINT8(2, icm40609_build_drive_config(0xFF, tx));
    assert_tx2(tx, 0x13, 0x07);
}

void test_icm40609_build_intf_config1(void)
{
    uint8_t tx[2];
    fill_sentinel(tx, sizeof(tx));

    TEST_ASSERT_EQUAL_UINT8(2, icm40609_build_intf_config1(2, tx));
    assert_tx2(tx, 0x4D, 0x42);
}

void test_icm40609_build_intf_config1_masks_inputs(void)
{
    uint8_t tx[2];
    fill_sentinel(tx, sizeof(tx));

    TEST_ASSERT_EQUAL_UINT8(2, icm40609_build_intf_config1(0xFF, tx));
    assert_tx2(tx, 0x4D, 0x43);
}

void test_icm40609_build_read_accel_gyro(void)
{
    uint8_t tx[16];
    fill_sentinel(tx, sizeof(tx));

    TEST_ASSERT_EQUAL_UINT8(15, icm40609_build_read_accel_gyro(tx));
    TEST_ASSERT_EQUAL_UINT8(0x9D, tx[0]);
    for (size_t i = 1; i < 15; ++i) {
        TEST_ASSERT_EQUAL_UINT8(0x00, tx[i]);
    }
}

void test_icm40609_build_read_int_status(void)
{
    uint8_t tx[2];
    fill_sentinel(tx, sizeof(tx));

    TEST_ASSERT_EQUAL_UINT8(2, icm40609_build_read_int_status(tx));
    assert_tx2(tx, 0xAD, 0x00);
}

void test_icm40609_build_fifo_config(void)
{
    uint8_t tx[10];
    fill_sentinel(tx, sizeof(tx));

    TEST_ASSERT_EQUAL_UINT8(8, icm40609_build_fifo_config(ICM40609_FIFO_STOP_ON_FULL,
                                                         true,
                                                         false,
                                                         true,
                                                         0x02AB,
                                                         tx));
    TEST_ASSERT_EQUAL_UINT8(0x16, tx[0]);
    TEST_ASSERT_EQUAL_UINT8(0x80, tx[1]);
    TEST_ASSERT_EQUAL_UINT8(0x5F, tx[2]);
    TEST_ASSERT_EQUAL_UINT8(0x05, tx[3]);
    TEST_ASSERT_EQUAL_UINT8(0x60, tx[4]);
    TEST_ASSERT_EQUAL_UINT8(0xAB, tx[5]);
    TEST_ASSERT_EQUAL_UINT8(0x61, tx[6]);
    TEST_ASSERT_EQUAL_UINT8(0x02, tx[7]);
}

void test_icm40609_build_fifo_config_masks_inputs(void)
{
    uint8_t tx[10];
    fill_sentinel(tx, sizeof(tx));

    TEST_ASSERT_EQUAL_UINT8(8, icm40609_build_fifo_config((icm40609_fifo_mode_t)0xFF,
                                                         true,
                                                         true,
                                                         true,
                                                         0x0FFF,
                                                         tx));
    TEST_ASSERT_EQUAL_UINT8(0x16, tx[0]);
    TEST_ASSERT_EQUAL_UINT8(0xC0, tx[1]);
    TEST_ASSERT_EQUAL_UINT8(0x5F, tx[2]);
    TEST_ASSERT_EQUAL_UINT8(0x07, tx[3]);
    TEST_ASSERT_EQUAL_UINT8(0x60, tx[4]);
    TEST_ASSERT_EQUAL_UINT8(0xFF, tx[5]);
    TEST_ASSERT_EQUAL_UINT8(0x61, tx[6]);
    TEST_ASSERT_EQUAL_UINT8(0x0F, tx[7]);
}

void test_icm40609_build_read_fifo_count(void)
{
    uint8_t tx[3];
    fill_sentinel(tx, sizeof(tx));

    TEST_ASSERT_EQUAL_UINT8(3, icm40609_build_read_fifo_count(tx));
    TEST_ASSERT_EQUAL_UINT8(0xAE, tx[0]);
    TEST_ASSERT_EQUAL_UINT8(0x00, tx[1]);
    TEST_ASSERT_EQUAL_UINT8(0x00, tx[2]);
}

void test_icm40609_build_read_fifo_data(void)
{
    uint8_t tx[6];
    fill_sentinel(tx, sizeof(tx));

    TEST_ASSERT_EQUAL_UINT8(5, icm40609_build_read_fifo_data(4, tx));
    TEST_ASSERT_EQUAL_UINT8(0xB0, tx[0]);
    for (size_t i = 1; i < 5; ++i) {
        TEST_ASSERT_EQUAL_UINT8(0x00, tx[i]);
    }
}

void test_icm40609_build_read_fifo_data_zero_bytes(void)
{
    uint8_t tx[2];
    fill_sentinel(tx, sizeof(tx));

    TEST_ASSERT_EQUAL_UINT8(1, icm40609_build_read_fifo_data(0, tx));
    assert_tx2(tx, 0xB0, 0xA5);
}

void test_icm40609_build_fifo_flush(void)
{
    uint8_t tx[2];
    fill_sentinel(tx, sizeof(tx));

    TEST_ASSERT_EQUAL_UINT8(2, icm40609_build_fifo_flush(tx));
    assert_tx2(tx, 0x4B, 0x02);
}
