#include "unity.h"

#include "SPI_device_interactions.h"
#include "fake_stm32_runtime.h"
#include "sensors/icm40609.h"

#include <string.h>

static void queue_init_success_responses(uint8_t expected_pwr)
{
    const uint8_t who_am_i[2] = {0x00, ICM40609_WHO_AM_I_VALUE};
    const uint8_t pwr_mgmt0[2] = {0x00, expected_pwr};
    const uint8_t int_status[2] = {0x00, ICM40609_INT_STATUS_DATA_RDY};
    const uint8_t first_sample[15] = {
        0x00,
        0x00, 0x00,
        0x01, 0x00,
        0xFF, 0x00,
        0x00, 0x80,
        0x00, 0x83,
        0xFF, 0x7D,
        0x00, 0x40
    };

    fake_spi_queue_response(who_am_i, sizeof(who_am_i));
    fake_spi_queue_response(pwr_mgmt0, sizeof(pwr_mgmt0));
    fake_spi_queue_response(int_status, sizeof(int_status));
    fake_spi_queue_response(first_sample, sizeof(first_sample));
}

static icm40609_config_t make_test_config(void)
{
    icm40609_config_t config = ICM40609_CONFIG_DEFAULT;
    config.accel_fs = ICM40609_ACCEL_FS_8G;
    config.accel_odr = ICM40609_ODR_100HZ;
    config.gyro_fs = ICM40609_GYRO_FS_500DPS;
    config.gyro_odr = ICM40609_ODR_200HZ;
    config.accel_mode = ICM40609_ACCEL_MODE_LN;
    config.gyro_mode = ICM40609_GYRO_MODE_LN;
    config.int1_polarity = ICM40609_INT_ACTIVE_HIGH;
    config.int1_drive = ICM40609_INT_PUSH_PULL;
    config.int1_mode = ICM40609_INT_LATCHED;
    config.drdy_int1_en = true;
    return config;
}

static void reset_icm_globals(void)
{
    icm40609_ready = false;
    icm40609_dev = (icm40609_t){0};
    memset(&icm40609_sample_ring, 0, sizeof(icm40609_sample_ring));
    memset(&jobq_spi_2, 0, sizeof(jobq_spi_2));
}

static void reset_test_state(void)
{
    fake_stm32_reset();
    reset_icm_globals();
}

void test_icm40609_init_with_config_success(void)
{
    icm40609_t dev = {0};
    const icm40609_config_t config = make_test_config();

    reset_test_state();
    queue_init_success_responses(0x0F);

    TEST_ASSERT_EQUAL_UINT8(0, icm40609_init_with_config(&fake_spi_handle,
                                                         GPIOD,
                                                         GPIO_PIN_15,
                                                         &dev,
                                                         &config));
    TEST_ASSERT_TRUE(icm40609_ready);
    TEST_ASSERT_EQUAL_UINT8(8, fake_spi_tx_count);
    TEST_ASSERT_EQUAL_UINT8(4, fake_spi_txrx_count);
    TEST_ASSERT_EQUAL_UINT8(0x11, fake_spi_tx_calls[0].data[0]);
    TEST_ASSERT_EQUAL_UINT8(0x01, fake_spi_tx_calls[0].data[1]);
    TEST_ASSERT_EQUAL_UINT8(0x13, fake_spi_tx_calls[1].data[0]);
    TEST_ASSERT_EQUAL_UINT8(0x05, fake_spi_tx_calls[1].data[1]);
    TEST_ASSERT_EQUAL_UINT8(0xF5, fake_spi_txrx_calls[0].tx[0]);
    TEST_ASSERT_EQUAL_UINT8(0x4D, fake_spi_tx_calls[2].data[0]);
    TEST_ASSERT_EQUAL_UINT8(0x41, fake_spi_tx_calls[2].data[1]);
    TEST_ASSERT_EQUAL_UINT8(0x50, fake_spi_tx_calls[3].data[0]);
    TEST_ASSERT_EQUAL_UINT8(0x48, fake_spi_tx_calls[3].data[1]);
    TEST_ASSERT_EQUAL_UINT8(0x4F, fake_spi_tx_calls[4].data[0]);
    TEST_ASSERT_EQUAL_UINT8(0x47, fake_spi_tx_calls[4].data[1]);
    TEST_ASSERT_EQUAL_UINT8(0x4E, fake_spi_tx_calls[5].data[0]);
    TEST_ASSERT_EQUAL_UINT8(0x0F, fake_spi_tx_calls[5].data[1]);
    TEST_ASSERT_EQUAL_UINT8(0x14, fake_spi_tx_calls[6].data[0]);
    TEST_ASSERT_EQUAL_UINT8(0x07, fake_spi_tx_calls[6].data[1]);
    TEST_ASSERT_EQUAL_UINT8(0x65, fake_spi_tx_calls[7].data[0]);
    TEST_ASSERT_EQUAL_UINT8(0x08, fake_spi_tx_calls[7].data[1]);
    TEST_ASSERT_EQUAL_UINT8(ICM40609_ACCEL_FS_8G, dev.accel_fs_sel);
    TEST_ASSERT_EQUAL_UINT8(ICM40609_ODR_100HZ, dev.accel_odr);
    TEST_ASSERT_EQUAL_UINT16(100, dev.accel_odr_hz);
    TEST_ASSERT_EQUAL_UINT8(ICM40609_GYRO_FS_500DPS, dev.gyro_fs_sel);
    TEST_ASSERT_EQUAL_UINT8(ICM40609_ODR_200HZ, dev.gyro_odr);
    TEST_ASSERT_EQUAL_UINT16(200, dev.gyro_odr_hz);
}

void test_icm40609_init_with_config_rejects_wrong_who_am_i(void)
{
    icm40609_t dev = {0};
    const icm40609_config_t config = make_test_config();
    const uint8_t who_am_i[2] = {0x00, 0x00};

    reset_test_state();
    fake_spi_queue_response(who_am_i, sizeof(who_am_i));

    TEST_ASSERT_EQUAL_UINT8(1, icm40609_init_with_config(&fake_spi_handle,
                                                         GPIOD,
                                                         GPIO_PIN_15,
                                                         &dev,
                                                         &config));
    TEST_ASSERT_FALSE(icm40609_ready);
    TEST_ASSERT_EQUAL_UINT8(2, fake_spi_tx_count);
    TEST_ASSERT_EQUAL_UINT8(1, fake_spi_txrx_count);
}

void test_icm40609_init_with_config_rejects_bad_power_readback(void)
{
    icm40609_t dev = {0};
    const icm40609_config_t config = make_test_config();
    const uint8_t who_am_i[2] = {0x00, ICM40609_WHO_AM_I_VALUE};
    const uint8_t bad_pwr_mgmt0[2] = {0x00, 0x00};

    reset_test_state();
    fake_spi_queue_response(who_am_i, sizeof(who_am_i));
    fake_spi_queue_response(bad_pwr_mgmt0, sizeof(bad_pwr_mgmt0));

    TEST_ASSERT_EQUAL_UINT8(2, icm40609_init_with_config(&fake_spi_handle,
                                                         GPIOD,
                                                         GPIO_PIN_15,
                                                         &dev,
                                                         &config));
    TEST_ASSERT_FALSE(icm40609_ready);
    TEST_ASSERT_EQUAL_UINT8(6, fake_spi_tx_count);
    TEST_ASSERT_EQUAL_UINT8(2, fake_spi_txrx_count);
}

void test_icm40609_init_with_config_rejects_missing_data_ready(void)
{
    icm40609_t dev = {0};
    const icm40609_config_t config = make_test_config();
    const uint8_t who_am_i[2] = {0x00, ICM40609_WHO_AM_I_VALUE};
    const uint8_t pwr_mgmt0[2] = {0x00, 0x0F};
    const uint8_t int_status[2] = {0x00, 0x00};

    reset_test_state();
    fake_spi_queue_response(who_am_i, sizeof(who_am_i));
    fake_spi_queue_response(pwr_mgmt0, sizeof(pwr_mgmt0));
    fake_spi_queue_response(int_status, sizeof(int_status));

    TEST_ASSERT_EQUAL_UINT8(3, icm40609_init_with_config(&fake_spi_handle,
                                                         GPIOD,
                                                         GPIO_PIN_15,
                                                         &dev,
                                                         &config));
    TEST_ASSERT_FALSE(icm40609_ready);
    TEST_ASSERT_EQUAL_UINT8(8, fake_spi_tx_count);
    TEST_ASSERT_EQUAL_UINT8(3, fake_spi_txrx_count);
}

void test_icm40609_init_with_config_returns_parse_error_when_dev_null(void)
{
    const icm40609_config_t config = make_test_config();

    reset_test_state();
    queue_init_success_responses(0x0F);

    TEST_ASSERT_EQUAL_UINT8(4, icm40609_init_with_config(&fake_spi_handle,
                                                         GPIOD,
                                                         GPIO_PIN_15,
                                                         NULL,
                                                         &config));
    TEST_ASSERT_FALSE(icm40609_ready);
}

void test_icm40609_init_uses_default_config(void)
{
    icm40609_t dev = {0};

    reset_test_state();
    queue_init_success_responses(0x0F);

    TEST_ASSERT_EQUAL_UINT8(0, icm40609_init(&fake_spi_handle,
                                             GPIOD,
                                             GPIO_PIN_15,
                                             &dev));
    TEST_ASSERT_EQUAL_UINT8(0x26, fake_spi_tx_calls[3].data[1]);
    TEST_ASSERT_EQUAL_UINT8(0x06, fake_spi_tx_calls[4].data[1]);
    TEST_ASSERT_EQUAL_UINT8(ICM40609_ACCEL_FS_16G, dev.accel_fs_sel);
    TEST_ASSERT_EQUAL_UINT8(ICM40609_ODR_1KHZ, dev.accel_odr);
    TEST_ASSERT_EQUAL_UINT8(ICM40609_GYRO_FS_2000DPS, dev.gyro_fs_sel);
    TEST_ASSERT_EQUAL_UINT8(ICM40609_ODR_1KHZ, dev.gyro_odr);
}

void test_icm40609_data_ready_interrupt_submits_expected_job(void)
{
    reset_test_state();
    fake_set_timestamp_us(0x12345678u);

    icm40609_data_ready_interrupt();

    TEST_ASSERT_EQUAL_UINT8(1, fake_spi_submit_count);
    TEST_ASSERT_TRUE(fake_last_submit_queue == &jobq_spi_2);
    TEST_ASSERT_TRUE(fake_last_submitted_job.cs_port == GPIOD);
    TEST_ASSERT_EQUAL_HEX16(GPIO_PIN_15, fake_last_submitted_job.cs_pin);
    TEST_ASSERT_EQUAL_UINT16(15, fake_last_submitted_job.len);
    TEST_ASSERT_EQUAL_UINT32(0x12345678u, (uint32_t)fake_last_submitted_job.t_sample);
    TEST_ASSERT_EQUAL_INT(SPI_XFER_TXRX, fake_last_submitted_job.type);
    TEST_ASSERT_NOT_NULL(fake_last_submitted_job.done);
    TEST_ASSERT_EQUAL_INT(SENSOR_ID_ACCEL, fake_last_submitted_job.sensor);
    TEST_ASSERT_EQUAL_UINT32(ICM40609_SAMPLE_FLAG, fake_last_submitted_job.task_notification_flag);
    TEST_ASSERT_EQUAL_UINT8(0x9D, fake_last_submitted_job.tx[0]);
    for (size_t i = 1; i < fake_last_submitted_job.len; ++i) {
        TEST_ASSERT_EQUAL_UINT8(0x00, fake_last_submitted_job.tx[i]);
    }
}

void test_icm40609_done_enqueues_sample_from_captured_job(void)
{
    const uint8_t rx[15] = {
        0x00,
        0x00, 0x00,
        0x01, 0x00,
        0xFF, 0x00,
        0x00, 0x80,
        0x00, 0x83,
        0xFF, 0x7D,
        0x00, 0x40
    };
    icm40609_sample_t sample = {0};

    reset_test_state();
    icm40609_dev.accel_scale = 1.0f / 4096.0f;
    icm40609_dev.gyro_scale = 1.0f / 131.0f;
    fake_set_timestamp_us(0x10203u);

    icm40609_data_ready_interrupt();
    fake_last_submitted_job.done(&fake_last_submitted_job, rx, fake_last_submitted_job.done_arg);

    TEST_ASSERT_TRUE(icm40609_sample_dequeue(&icm40609_sample_ring, &sample));
    TEST_ASSERT_EQUAL_UINT64(0x10203u, sample.t_us);
    TEST_ASSERT_EQUAL_INT16(256, sample.ax_raw);
    TEST_ASSERT_EQUAL_INT16(-256, sample.ay_raw);
    TEST_ASSERT_EQUAL_INT16(128, sample.az_raw);
    TEST_ASSERT_EQUAL_INT16(131, sample.gx_raw);
    TEST_ASSERT_EQUAL_INT16(-131, sample.gy_raw);
    TEST_ASSERT_EQUAL_INT16(64, sample.gz_raw);
}
