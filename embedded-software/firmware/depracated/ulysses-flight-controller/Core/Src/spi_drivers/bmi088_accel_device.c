#include "SPI_device_interactions.h"
#include "main.h"

/* -------------------------------------------------------------------------- */
/* Accelerometer Initialization                                               */
/* -------------------------------------------------------------------------- */

uint8_t bmi088_accel_init_with_config(SPI_HandleTypeDef *hspi,
                                      GPIO_TypeDef *cs_port,
                                      uint16_t cs_pin,
                                      bmi088_accel_t *dev,
                                      const bmi088_accel_config_t *config)
{
    uint8_t tx[16], rx[16];

    /* --- 1. Soft reset --- */
    size_t n = bmi088_accel_build_softreset(tx);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    delay_us(1000);
    HAL_SPI_Transmit(hspi, tx, n, HAL_MAX_DELAY);
    delay_us(1000);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    delay_us(600000);

    /* --- 2. Verify chip ID --- */
    n = bmi088_accel_build_read_reg(BMI088_ACC_CHIP_ID_REG, tx);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    delay_us(1000);
    HAL_SPI_TransmitReceive(hspi, tx, rx, n, HAL_MAX_DELAY);
    delay_us(1000);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    delay_us(50000);

    if (rx[2] != BMI088_ACC_CHIP_ID_VALUE)
        return 1;

    /* --- 3. Power on accelerometer --- */
    n = bmi088_accel_build_power_conf(BMI088_ACC_PWR_ACTIVE, tx, dev);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    delay_us(1000);
    HAL_SPI_Transmit(hspi, tx, 2, HAL_MAX_DELAY);
    delay_us(1000);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    delay_us(50000);

    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    delay_us(1000);
    HAL_SPI_Transmit(hspi, &tx[2], 2, HAL_MAX_DELAY);
    delay_us(1000);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    delay_us(50000);
    
    n = bmi088_accel_build_read_reg(BMI088_ACC_PWR_CONF, tx);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    delay_us(1000);
    HAL_SPI_TransmitReceive(hspi, tx, rx, n, HAL_MAX_DELAY);
    delay_us(1000);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    delay_us(50000);

    if(rx[2] != 0)
        return 2;

    n = bmi088_accel_build_read_reg(BMI088_ACC_PWR_CTRL, tx);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    delay_us(1000);
    HAL_SPI_TransmitReceive(hspi, tx, rx, n, HAL_MAX_DELAY);
    delay_us(1000);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    delay_us(50000);

    if(rx[2] != 4)
        return 3;

    /* --- 4. Set measurement range from config --- */
    n = bmi088_accel_build_range(config->range, tx, dev);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    delay_us(1000);
    HAL_SPI_Transmit(hspi, tx, n, HAL_MAX_DELAY);
    delay_us(1000);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    delay_us(50000);

    /* --- 5. Configure ODR and bandwidth from config --- */
    n = bmi088_accel_build_odr_bw(config->odr, config->bandwidth, tx, dev);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    delay_us(1000);
    HAL_SPI_Transmit(hspi, tx, n, HAL_MAX_DELAY);
    delay_us(1000);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    delay_us(50000);

    n = bmi088_accel_build_read_reg(BMI088_ACC_CONF, tx);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    delay_us(1000);
    HAL_SPI_TransmitReceive(hspi, tx, rx, n, HAL_MAX_DELAY);
    delay_us(1000);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    delay_us(50000);

    n = bmi088_accel_build_read_reg(BMI088_ACC_RANGE, tx);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    delay_us(1000);
    HAL_SPI_TransmitReceive(hspi, tx, rx, 3, HAL_MAX_DELAY);
    delay_us(1000);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    delay_us(50000);

    /* --- 6. Configure interrupt pin from config --- */
    n = bmi088_accel_build_int_pin_conf(config->int_pin,
                                        config->int_drive,
                                        config->int_polarity,
                                        config->int_latched,
                                        tx);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    delay_us(1000);
    HAL_SPI_Transmit(hspi, tx, n, HAL_MAX_DELAY);
    delay_us(1000);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);

    /* --- 7. Map interrupt events from config --- */
    n = bmi088_accel_build_int_event_map(config->int_pin, config->int_events, tx);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    delay_us(1000);
    HAL_SPI_Transmit(hspi, tx, n, HAL_MAX_DELAY);
    delay_us(1000);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);

    /* --- 8. Verify data ready --- */
    n = bmi088_accel_build_read_reg(BMI088_ACC_STATUS_REG, tx);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    delay_us(1000);
    HAL_SPI_TransmitReceive(hspi, tx, rx, n, HAL_MAX_DELAY);
    delay_us(1000);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    delay_us(50000);

    if((rx[2] & 0x80) == 0)
        return 4;

    n = bmi088_accel_build_read(BMI088_ACC_READ_DATA_XYZ, tx);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    delay_us(1000);
    HAL_SPI_TransmitReceive(hspi, tx, rx, n, HAL_MAX_DELAY);
    delay_us(1000);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    delay_us(50000);

    bmi088_accel_sample_t sample;
    bmi088_accel_parse_data_xyz(&rx[2], &sample, dev);

    return 0;
}

uint8_t bmi088_accel_init(SPI_HandleTypeDef *hspi,
                          GPIO_TypeDef *cs_port,
                          uint16_t cs_pin,
                          bmi088_accel_t *dev)
{
    bmi088_accel_config_t default_config = BMI088_ACCEL_CONFIG_DEFAULT;
    return bmi088_accel_init_with_config(hspi, cs_port, cs_pin, dev, &default_config);
}

static void bmi088_accel_done(spi_job_t *job,
                              const uint8_t *rx_buf,
                              void *arg)
{
    (void)job;
    (void)arg;

    bmi088_accel_sample_t sample;
    if (bmi088_accel_parse_data_xyz(&rx_buf[2], &sample, &accel)) {
        sample.t_us = job->t_sample;
        bmi088_acc_sample_queue(&bmi088_acc_sample_ring,  &sample);
    }
}

void bmi088_accel_interrupt(void)
{
    const uint32_t now = timestamp_us();

    spi_job_t job;

    job.cs_port  = BMI_ACC_Chip_Select_GPIO_Port;
    job.cs_pin   = BMI_ACC_Chip_Select_Pin;

    job.len      = bmi088_accel_build_read(BMI088_ACC_READ_DATA_XYZ, job.tx); 
    job.t_sample = now;
    job.type     = SPI_XFER_TXRX;
    job.done     = bmi088_accel_done;
    job.done_arg = NULL;
    job.sensor   = SENSOR_ID_ACCEL;
    job.task_notification_flag = BMI088_ACCEL_SAMPLE_FLAG;

    spi_submit_job(job, &jobq_spi_2);
}
