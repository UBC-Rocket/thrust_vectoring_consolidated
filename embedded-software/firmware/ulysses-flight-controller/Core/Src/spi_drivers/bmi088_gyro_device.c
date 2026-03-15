#include "SPI_device_interactions.h"
#include "main.h"

/* -------------------------------------------------------------------------- */
/* SPI Helpers                                                                */
/* -------------------------------------------------------------------------- */

static void write_frame(SPI_HandleTypeDef *hspi,
                        GPIO_TypeDef *cs_port,
                        uint16_t cs_pin,
                        const uint8_t *tx,
                        size_t len)
{
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    delay_us(1000);
    HAL_SPI_Transmit(hspi, (uint8_t *)tx, len, HAL_MAX_DELAY);
    delay_us(1000);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    delay_us(1000);
}

static void write_sequence_split(SPI_HandleTypeDef *hspi,
                                 GPIO_TypeDef *cs_port,
                                 uint16_t cs_pin,
                                 const uint8_t *tx,
                                 size_t len)
{
    /* Write register/data pairs (2 bytes at a time) with independent CS strobes */
    for (size_t i = 0; i < len; i += 2) {
        write_frame(hspi, cs_port, cs_pin, &tx[i], 2);
    }
}

/* -------------------------------------------------------------------------- */
/* Gyroscope Initialization                                                   */
/* -------------------------------------------------------------------------- */

uint8_t bmi088_gyro_init_with_config(SPI_HandleTypeDef *hspi,
                                     GPIO_TypeDef *cs_port,
                                     uint16_t cs_pin,
                                     bmi088_gyro_t *dev,
                                     const bmi088_gyro_config_t *config)
{
    uint8_t tx[16], rx[16];
    size_t n;

    /* --- 1. Soft reset --- */
    n = bmi088_gyro_build_softreset(tx);
    write_frame(hspi, cs_port, cs_pin, tx, n);
    delay_us(30000); // per datasheet ≥30 ms

    /* --- 2. Verify chip ID --- */
    n = bmi088_gyro_build_read(BMI088_GYRO_READ_CHIP_ID, tx);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    delay_us(1000);
    HAL_SPI_TransmitReceive(hspi, tx, rx, n, HAL_MAX_DELAY);
    delay_us(1000);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    delay_us(5000);

    if (rx[1] != BMI088_GYRO_CHIP_ID_VALUE)
        return 1;

    /* --- 3. Power on gyroscope --- */
    n = bmi088_gyro_build_power_conf(BMI088_GYRO_PWR_NORMAL, tx, dev);
    write_frame(hspi, cs_port, cs_pin, tx, n);
    delay_us(80000); // PLL lock time ≥80 ms

    /* --- 4. Configure range from config --- */
    n = bmi088_gyro_build_range(config->range, tx, dev);
    write_frame(hspi, cs_port, cs_pin, tx, n);
    delay_us(5000);

    /* --- 5. Configure ODR from config --- */
    n = bmi088_gyro_build_odr(config->odr, tx, dev);
    write_frame(hspi, cs_port, cs_pin, tx, n);
    delay_us(5000);

    /* --- 6. Configure interrupt pin from config --- */
    n = bmi088_gyro_build_int_pin_conf(config->int_pin,
                                       config->int_drive,
                                       config->int_polarity,
                                       tx);
    write_frame(hspi, cs_port, cs_pin, tx, n);
    delay_us(5000);

    /* --- 7. Map interrupt events from config --- */
    n = bmi088_gyro_build_int_event_map(config->int_pin, config->int_events, tx);
    write_sequence_split(hspi, cs_port, cs_pin, tx, n);

    /* --- 8. Verify first reading --- */
    n = bmi088_gyro_build_read(BMI088_GYRO_READ_DATA_XYZ, tx);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    delay_us(1000);
    HAL_SPI_TransmitReceive(hspi, tx, rx, n, HAL_MAX_DELAY);
    delay_us(1000);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);

    bmi088_gyro_sample_t s;
    if (!bmi088_gyro_parse_data_xyz(rx, &s, dev))
        return 2;

    return 0;
}

uint8_t bmi088_gyro_init(SPI_HandleTypeDef *hspi,
                         GPIO_TypeDef *cs_port,
                         uint16_t cs_pin,
                         bmi088_gyro_t *dev)
{
    bmi088_gyro_config_t default_config = BMI088_GYRO_CONFIG_DEFAULT;
    return bmi088_gyro_init_with_config(hspi, cs_port, cs_pin, dev, &default_config);
}

static void bmi088_gyro_done(spi_job_t *job,
                             const uint8_t *rx_buf,
                             void *arg)
{
    (void)job;
    (void)arg;

    bmi088_gyro_sample_t s;
    if (bmi088_gyro_parse_data_xyz(&rx_buf[0], &s, &gyro)) {
        s.t_us = job->t_sample;
        bmi088_gyro_sample_q_push(&bmi088_gyro_sample_ring, &s);
    }
}

void bmi088_gyro_interrupt(void)
{
    const uint32_t now = timestamp_us();

    spi_job_t job;

    job.cs_port  = BMI_GYRO_Chip_Select_GPIO_Port;
    job.cs_pin   = BMI_GYRO_Chip_Select_Pin;

    job.len      = bmi088_gyro_build_read(BMI088_GYRO_READ_DATA_XYZ, job.tx); 
    job.t_sample = now;
    job.type     = SPI_XFER_TXRX;
    job.done     = bmi088_gyro_done;
    job.done_arg = NULL;
    job.sensor   = SENSOR_ID_GYRO;
    job.task_notification_flag = BMI088_GYRO_SAMPLE_FLAG;

    spi_submit_job(job, &jobq_spi_2);
}
