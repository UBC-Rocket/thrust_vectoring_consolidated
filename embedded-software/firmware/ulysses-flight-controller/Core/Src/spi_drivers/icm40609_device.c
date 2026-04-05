/*
 * ICM-40609-D device layer: HAL initialization, interrupt handler, DMA callback.
 *
 * This file bridges the bus-agnostic icm40609 library driver with the
 * STM32 HAL and the SPI job queue system.
 *
 * @ UBC Rocket, 2026
 */

#include "SPI_device_interactions.h"
#include "sensors/icm40609.h"
#include "main.h"

/* -------------------------------------------------------------------------- */
/* SPI helper: blocking single-register write with CS control                 */
/* -------------------------------------------------------------------------- */
static void icm40609_spi_write(SPI_HandleTypeDef *hspi,
                                GPIO_TypeDef *cs_port,
                                uint16_t cs_pin,
                                uint8_t *tx,
                                size_t len)
{
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    delay_us(1);
    HAL_SPI_Transmit(hspi, tx, len, HAL_MAX_DELAY);
    delay_us(1);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
}

/* -------------------------------------------------------------------------- */
/* SPI helper: blocking single-register read with CS control                  */
/* -------------------------------------------------------------------------- */
static void icm40609_spi_read(SPI_HandleTypeDef *hspi,
                               GPIO_TypeDef *cs_port,
                               uint16_t cs_pin,
                               uint8_t *tx,
                               uint8_t *rx,
                               size_t len)
{
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    delay_us(1);
    HAL_SPI_TransmitReceive(hspi, tx, rx, len, HAL_MAX_DELAY);
    delay_us(1);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
}

/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

uint8_t icm40609_init_with_config(SPI_HandleTypeDef *hspi,
                                   GPIO_TypeDef *cs_port,
                                   uint16_t cs_pin,
                                   icm40609_t *dev,
                                   const icm40609_config_t *config)
{
    uint8_t tx[16], rx[16];
    size_t n;

    icm40609_ready = false;

    /* --- 1. Soft reset ---
     * Write 1 to DEVICE_CONFIG.SOFT_RESET_CONFIG, then wait 1ms for reset
     * to complete before any other register access.
     */
    n = icm40609_build_soft_reset(tx);
    icm40609_spi_write(hspi, cs_port, cs_pin, tx, n);
    delay_us(1000);

    /* --- 2. Configure SPI slew rate ---
     * Per datasheet section 11.3: for SPI operation,
     * SPI_SLEW_RATE = 5, I2C_SLEW_RATE = 0.
     */
    n = icm40609_build_drive_config(5, tx);
    icm40609_spi_write(hspi, cs_port, cs_pin, tx, n);
    delay_us(100);

    /* --- 3. Verify WHO_AM_I --- */
    n = icm40609_build_read_reg(ICM40609_REG_WHO_AM_I, tx);
    icm40609_spi_read(hspi, cs_port, cs_pin, tx, rx, n);
    delay_us(100);

    if (rx[1] != ICM40609_WHO_AM_I_VALUE)
        return 1;

    /* --- 4. Configure clock source ---
     * INTF_CONFIG1: CLKSEL = 01 (auto-select PLL when available,
     * otherwise use RC oscillator). EN_TEST_MODE = 01 for normal operation.
     */
    n = icm40609_build_intf_config1(0x01, tx);
    icm40609_spi_write(hspi, cs_port, cs_pin, tx, n);
    delay_us(100);

    /* --- 5. Configure accelerometer FSR and ODR --- */
    n = icm40609_build_accel_config(config->accel_fs, config->accel_odr, tx, dev);
    icm40609_spi_write(hspi, cs_port, cs_pin, tx, n);
    delay_us(100);

    /* --- 6. Configure gyroscope FSR and ODR --- */
    n = icm40609_build_gyro_config(config->gyro_fs, config->gyro_odr, tx, dev);
    icm40609_spi_write(hspi, cs_port, cs_pin, tx, n);
    delay_us(100);

    /* --- 7. Power on sensors ---
     * Write PWR_MGMT0 to enable accel and gyro in the requested modes.
     * IMPORTANT: After writing PWR_MGMT0, wait at least 200us before any
     * other register writes (per datasheet).
     */
    n = icm40609_build_pwr_mgmt0(config->gyro_mode, config->accel_mode,
                                  false, tx);
    icm40609_spi_write(hspi, cs_port, cs_pin, tx, n);
    delay_us(300);  /* 200us minimum + margin */

    /* Verify power mode was applied */
    n = icm40609_build_read_reg(ICM40609_REG_PWR_MGMT0, tx);
    icm40609_spi_read(hspi, cs_port, cs_pin, tx, rx, n);
    delay_us(100);

    uint8_t expected_pwr = ((uint8_t)config->gyro_mode << 2) |
                           ((uint8_t)config->accel_mode);
    if ((rx[1] & 0x0F) != expected_pwr)
        return 2;

    /* --- 8. Wait for gyro startup ---
     * Gyroscope needs to be kept ON for a minimum of 45ms when transitioning
     * from OFF to any other mode (datasheet).
     */
    delay_us(45000);

    /* --- 9. Configure interrupts ---
     * INT_CONFIG: INT1 electrical configuration (polarity, drive, mode).
     * INT_SOURCE0: Enable DATA_RDY on INT1.
     */
    n = icm40609_build_int_config(config->int1_polarity,
                                   config->int1_drive,
                                   config->int1_mode, tx);
    icm40609_spi_write(hspi, cs_port, cs_pin, tx, n);
    delay_us(100);

    n = icm40609_build_int_source0(config->drdy_int1_en, false, false, tx);
    icm40609_spi_write(hspi, cs_port, cs_pin, tx, n);
    delay_us(100);

    /* --- 10. Configure FIFO (if enabled) --- */
    if (config->fifo_en) {
        n = icm40609_build_fifo_config(config->fifo_mode,
                                        config->fifo_accel_en,
                                        config->fifo_gyro_en,
                                        config->fifo_temp_en,
                                        config->fifo_watermark,
                                        tx);
        /* FIFO config is multiple register writes; send each 2-byte pair */
        for (size_t i = 0; i < n; i += 2) {
            icm40609_spi_write(hspi, cs_port, cs_pin, &tx[i], 2);
            delay_us(100);
        }
    }

    /* --- 11. Verify data ready ---
     * Read INT_STATUS and check if DATA_RDY_INT is set, indicating
     * that at least one sample is available.
     */
    delay_us(10000);  /* wait for first sample at configured ODR */

    n = icm40609_build_read_int_status(tx);
    icm40609_spi_read(hspi, cs_port, cs_pin, tx, rx, n);
    delay_us(100);

    if ((rx[1] & ICM40609_INT_STATUS_DATA_RDY) == 0)
        return 3;

    /* --- 12. Read first sample to verify data path --- */
    n = icm40609_build_read_accel_gyro(tx);
    icm40609_spi_read(hspi, cs_port, cs_pin, tx, rx, n);
    delay_us(100);

    icm40609_sample_t sample;
    if (!icm40609_parse_accel_gyro(&rx[1], &sample, dev))
        return 4;

    icm40609_ready = true;
    return 0;  /* success */
}

uint8_t icm40609_init(SPI_HandleTypeDef *hspi,
                       GPIO_TypeDef *cs_port,
                       uint16_t cs_pin,
                       icm40609_t *dev)
{
    icm40609_config_t default_config = ICM40609_CONFIG_DEFAULT;
    return icm40609_init_with_config(hspi, cs_port, cs_pin, dev, &default_config);
}

/* -------------------------------------------------------------------------- */
/* DMA completion callback                                                    */
/* -------------------------------------------------------------------------- */

static void icm40609_done(spi_job_t *job,
                           const uint8_t *rx_buf,
                           void *arg)
{
    (void)arg;

    icm40609_sample_t sample;
    /*
     * rx_buf[0] is the address byte echo; data starts at rx_buf[1].
     * The burst read covers 14 bytes: temp(2) + accel(6) + gyro(6).
     */
    if (icm40609_parse_accel_gyro(&rx_buf[1], &sample, &icm40609_dev)) {
        sample.t_us = job->t_sample;
        icm40609_sample_enqueue(&icm40609_sample_ring, &sample);
    }
}

/* -------------------------------------------------------------------------- */
/* Interrupt handler                                                          */
/* -------------------------------------------------------------------------- */

/*
 * Call this function from HAL_GPIO_EXTI_Rising_Callback (or Falling,
 * depending on INT1 polarity config) when the ICM-40609 DATA_RDY
 * interrupt fires.
 */
void icm40609_data_ready_interrupt(void)
{
    const uint64_t now = timestamp_us();

    spi_job_t job;

    job.cs_port  = ICM40609_CS_GPIO_Port;
    job.cs_pin   = ICM40609_CS_Pin;

    job.len      = icm40609_build_read_accel_gyro(job.tx);
    job.t_sample = now;
    job.type     = SPI_XFER_TXRX;
    job.done     = icm40609_done;
    job.done_arg = NULL;
    job.sensor   = SENSOR_ID_ACCEL;
    job.task_notification_flag = ICM40609_SAMPLE_FLAG;

    spi_submit_job(job, &jobq_spi_2);
}
