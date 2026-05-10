/*
 * ICM-40609-D device layer: HAL initialization, interrupt handler, DMA callback.
 *
 * Bridges the bus-agnostic icm40609 library driver to the STM32 HAL and the
 * SPI job queue. Blocking init runs polling SPI; the runtime data path is
 * interrupt-driven DMA.
 *
 * @ UBC Rocket
 */

#include "SPI_device_interactions.h"
#include "sensors/icm40609.h"
#include "main.h"
#include <math.h>

/* Finite SPI timeout for blocking init transactions (milliseconds).
 * Long enough to absorb any sane bus stall; short enough to surface a
 * wiring fault instead of hanging the firmware. */
#define ICM40609_SPI_TIMEOUT_MS  100

/* -------------------------------------------------------------------------- */
/* SPI helpers: blocking write/read with CS control. Return HAL_OK on         */
/* success so callers can fault-detect.                                       */
/* -------------------------------------------------------------------------- */
static HAL_StatusTypeDef icm40609_spi_write(SPI_HandleTypeDef *hspi,
                                            GPIO_TypeDef *cs_port,
                                            uint16_t cs_pin,
                                            uint8_t *tx,
                                            size_t len)
{
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    delay_us(1);
    HAL_StatusTypeDef st = HAL_SPI_Transmit(hspi, tx, len, ICM40609_SPI_TIMEOUT_MS);
    delay_us(1);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    return st;
}

static HAL_StatusTypeDef icm40609_spi_read(SPI_HandleTypeDef *hspi,
                                           GPIO_TypeDef *cs_port,
                                           uint16_t cs_pin,
                                           uint8_t *tx,
                                           uint8_t *rx,
                                           size_t len)
{
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    delay_us(1);
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(hspi, tx, rx, len, ICM40609_SPI_TIMEOUT_MS);
    delay_us(1);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    return st;
}

/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */
/*
 * Error codes (also documented in the header):
 *   0  - success
 *   1  - WHO_AM_I mismatch
 *   2  - PWR_MGMT0 readback mismatch
 *   3  - DATA_RDY never asserted after expected sample period
 *   4  - First-sample sanity check failed (accel magnitude or temp)
 *   5  - SPI bus error during init (HAL_SPI_* returned non-OK)
 *   6  - ACCEL_CONFIG0 readback mismatch
 *   7  - GYRO_CONFIG0 readback mismatch
 *   8  - INT_CONFIG readback mismatch
 *   9  - INT_SOURCE0 readback mismatch
 *  10  - NULL pointer argument
 *  11  - INT_CONFIG1 readback mismatch (INT_ASYNC_RESET not cleared)
 */

#define ICM40609_CHECK_SPI(st) do { if ((st) != HAL_OK) return 5; } while (0)

uint8_t icm40609_init_with_config(SPI_HandleTypeDef *hspi,
                                   GPIO_TypeDef *cs_port,
                                   uint16_t cs_pin,
                                   icm40609_t *dev,
                                   const icm40609_config_t *config)
{
    if (!hspi || !cs_port || !dev || !config) return 10;

    uint8_t tx[16], rx[16];
    size_t n;
    HAL_StatusTypeDef st;

    icm40609_ready = false;

    /* --- 1. Soft reset --- */
    n = icm40609_build_soft_reset(tx);
    st = icm40609_spi_write(hspi, cs_port, cs_pin, tx, n);
    ICM40609_CHECK_SPI(st);
    delay_us(1000);  /* DS-000330 §10.1.1: software reset takes >=1 ms */

    /* --- 2. Force Bank 0 (belt-and-suspenders; soft reset sets it to 0) --- */
    n = icm40609_build_set_bank(ICM40609_BANK_0, tx);
    st = icm40609_spi_write(hspi, cs_port, cs_pin, tx, n);
    ICM40609_CHECK_SPI(st);
    delay_us(100);

    /* --- 3. Configure SPI slew rate (DS-000330 §14.4) --- */
    n = icm40609_build_drive_config(5, tx);
    st = icm40609_spi_write(hspi, cs_port, cs_pin, tx, n);
    ICM40609_CHECK_SPI(st);
    delay_us(100);

    /* --- 4. Verify WHO_AM_I --- */
    n = icm40609_build_read_reg(ICM40609_REG_WHO_AM_I, tx);
    st = icm40609_spi_read(hspi, cs_port, cs_pin, tx, rx, n);
    ICM40609_CHECK_SPI(st);
    if (rx[1] != ICM40609_WHO_AM_I_VALUE) return 1;

    /* --- 5. Lock parser-side assumptions: big-endian sensor data + byte FIFO count --- */
    n = icm40609_build_intf_config0(ICM40609_INTF_CONFIG0_DEFAULT, tx);
    st = icm40609_spi_write(hspi, cs_port, cs_pin, tx, n);
    ICM40609_CHECK_SPI(st);
    delay_us(100);

    /* --- 6. Configure clock source (CLKSEL = 01 -> auto-select PLL) --- */
    n = icm40609_build_intf_config1(0x01, tx);
    st = icm40609_spi_write(hspi, cs_port, cs_pin, tx, n);
    ICM40609_CHECK_SPI(st);
    delay_us(100);

    /* --- 6a. INT_CONFIG1: clear INT_ASYNC_RESET (reset value 0x10) ---
     * DS-000330 §13.42 explicitly requires bit 4 to be cleared from its
     * reset value of 1 for proper INT1/INT2 pin operation. Done before
     * INT_CONFIG / INT_SOURCE0 so the routing config below latches into
     * a properly-initialized INT block.
     *
     * Pulse/de-assert mode depends on ODR: at ODR >= 4 kHz the 100µs
     * defaults exceed the sample interval and would jam the interrupt
     * line, so we switch to 8µs pulse + de-assert disabled (§13.42). */
    {
        const bool high_odr = (config->accel_odr == ICM40609_ODR_32KHZ
                            || config->accel_odr == ICM40609_ODR_16KHZ
                            || config->accel_odr == ICM40609_ODR_8KHZ
                            || config->accel_odr == ICM40609_ODR_4KHZ
                            || config->gyro_odr  == ICM40609_ODR_32KHZ
                            || config->gyro_odr  == ICM40609_ODR_16KHZ
                            || config->gyro_odr  == ICM40609_ODR_8KHZ
                            || config->gyro_odr  == ICM40609_ODR_4KHZ);
        const uint8_t int_cfg1_val = high_odr ? ICM40609_INT_CONFIG1_HIGH_ODR
                                              : ICM40609_INT_CONFIG1_DEFAULT;
        n = icm40609_build_int_config1(int_cfg1_val, tx);
        st = icm40609_spi_write(hspi, cs_port, cs_pin, tx, n);
        ICM40609_CHECK_SPI(st);
        delay_us(100);

        n = icm40609_build_read_reg(ICM40609_REG_INT_CONFIG1, tx);
        st = icm40609_spi_read(hspi, cs_port, cs_pin, tx, rx, n);
        ICM40609_CHECK_SPI(st);
        /* Verify the three driver-controlled bits (6:4); reserved bits are masked. */
        if ((rx[1] & 0x70) != (int_cfg1_val & 0x70)) return 11;
    }

    /* --- 7. Configure accelerometer FSR and ODR, then verify --- */
    uint8_t accel_cfg_expected = ((uint8_t)(config->accel_fs & 0x07) << 5)
                               | ((uint8_t)(config->accel_odr & 0x0F));
    n = icm40609_build_accel_config(config->accel_fs, config->accel_odr, tx, dev);
    st = icm40609_spi_write(hspi, cs_port, cs_pin, tx, n);
    ICM40609_CHECK_SPI(st);
    delay_us(100);

    n = icm40609_build_read_reg(ICM40609_REG_ACCEL_CONFIG0, tx);
    st = icm40609_spi_read(hspi, cs_port, cs_pin, tx, rx, n);
    ICM40609_CHECK_SPI(st);
    if (rx[1] != accel_cfg_expected) return 6;

    /* --- 8. Configure gyroscope FSR and ODR, then verify --- */
    uint8_t gyro_cfg_expected = ((uint8_t)(config->gyro_fs & 0x07) << 5)
                              | ((uint8_t)(config->gyro_odr & 0x0F));
    n = icm40609_build_gyro_config(config->gyro_fs, config->gyro_odr, tx, dev);
    st = icm40609_spi_write(hspi, cs_port, cs_pin, tx, n);
    ICM40609_CHECK_SPI(st);
    delay_us(100);

    n = icm40609_build_read_reg(ICM40609_REG_GYRO_CONFIG0, tx);
    st = icm40609_spi_read(hspi, cs_port, cs_pin, tx, rx, n);
    ICM40609_CHECK_SPI(st);
    if (rx[1] != gyro_cfg_expected) return 7;

    /* --- 9. Power on sensors (PWR_MGMT0) then verify --- */
    n = icm40609_build_pwr_mgmt0(config->gyro_mode, config->accel_mode, false, tx);
    st = icm40609_spi_write(hspi, cs_port, cs_pin, tx, n);
    ICM40609_CHECK_SPI(st);
    delay_us(300);  /* DS-000330: >=200 us after PWR_MGMT0 write */

    n = icm40609_build_read_reg(ICM40609_REG_PWR_MGMT0, tx);
    st = icm40609_spi_read(hspi, cs_port, cs_pin, tx, rx, n);
    ICM40609_CHECK_SPI(st);

    uint8_t expected_pwr = ((uint8_t)config->gyro_mode << 2)
                         | ((uint8_t)config->accel_mode);
    if ((rx[1] & 0x0F) != expected_pwr) return 2;

    /* --- 10. Wait for gyro startup (>=45 ms from OFF to any other mode) --- */
    delay_us(45000);

    /* --- 11. Configure INT1, then verify --- */
    uint8_t int_cfg_expected = (((uint8_t)config->int1_mode     & 0x01) << 2)
                             | (((uint8_t)config->int1_drive    & 0x01) << 1)
                             | (((uint8_t)config->int1_polarity & 0x01) << 0);
    n = icm40609_build_int_config(config->int1_polarity, config->int1_drive,
                                   config->int1_mode, tx);
    st = icm40609_spi_write(hspi, cs_port, cs_pin, tx, n);
    ICM40609_CHECK_SPI(st);
    delay_us(100);

    n = icm40609_build_read_reg(ICM40609_REG_INT_CONFIG, tx);
    st = icm40609_spi_read(hspi, cs_port, cs_pin, tx, rx, n);
    ICM40609_CHECK_SPI(st);
    /* Mask INT2 bits (5:3) to be tolerant of any prior INT2 wiring */
    if ((rx[1] & 0x07) != int_cfg_expected) return 8;

    /* --- 12. Configure INT_SOURCE0, then verify --- */
    uint8_t intsrc_expected = (config->drdy_int1_en ? (1u << 3) : 0);
    n = icm40609_build_int_source0(config->drdy_int1_en, false, false, tx);
    st = icm40609_spi_write(hspi, cs_port, cs_pin, tx, n);
    ICM40609_CHECK_SPI(st);
    delay_us(100);

    n = icm40609_build_read_reg(ICM40609_REG_INT_SOURCE0, tx);
    st = icm40609_spi_read(hspi, cs_port, cs_pin, tx, rx, n);
    ICM40609_CHECK_SPI(st);
    if ((rx[1] & 0x0E) != intsrc_expected) return 9;

    /* --- 13. Configure FIFO (if enabled) --- */
    if (config->fifo_en) {
        n = icm40609_build_fifo_config(config->fifo_mode,
                                        config->fifo_accel_en,
                                        config->fifo_gyro_en,
                                        config->fifo_temp_en,
                                        config->fifo_watermark,
                                        tx);
        /* FIFO config is a sequence of 2-byte (addr, value) writes. */
        for (size_t i = 0; i < n; i += 2) {
            st = icm40609_spi_write(hspi, cs_port, cs_pin, &tx[i], 2);
            ICM40609_CHECK_SPI(st);
            delay_us(100);
        }
    }

    /* --- 14. Wait for first sample, then check DATA_RDY ---
     * Allow at least two sample periods plus a 10 ms floor for the slowest
     * ODRs. dev->accel_period_us is set by icm40609_build_accel_config above. */
    uint32_t drdy_wait_us = (dev->accel_period_us > 0) ? 2u * dev->accel_period_us : 10000u;
    if (drdy_wait_us < 10000u) drdy_wait_us = 10000u;
    delay_us(drdy_wait_us);

    n = icm40609_build_read_int_status(tx);
    st = icm40609_spi_read(hspi, cs_port, cs_pin, tx, rx, n);
    ICM40609_CHECK_SPI(st);

    if ((rx[1] & ICM40609_INT_STATUS_DATA_RDY) == 0) return 3;

    /* --- 15. Burst-read first sample and sanity-check it --- */
    n = icm40609_build_read_accel_gyro(tx);
    st = icm40609_spi_read(hspi, cs_port, cs_pin, tx, rx, n);
    ICM40609_CHECK_SPI(st);

    icm40609_sample_t sample;
    if (!icm40609_parse_accel_gyro(&rx[1], &sample, dev)) return 4;

    /* First-sample sanity check.
     *
     * Goal: catch a totally-broken sensor (stuck at 0, stuck at full-scale)
     * without rejecting a healthy sensor on a board that's being handled,
     * vibrated, or carried during bring-up.
     *
     * Bounds:
     *   |a| in [0.5 g, 5 g]  -> 4.9 .. 49.05 m/s². Catches an all-zero or
     *                           rail-stuck accel; tolerates several g of
     *                           transient motion during init.
     *   temp_c in [-40, 85]  -> chip's operating range.
     */
    float a_mag = sqrtf(sample.ax * sample.ax
                       + sample.ay * sample.ay
                       + sample.az * sample.az);
    if (a_mag < 4.9f || a_mag > 49.05f) return 4;
    if (sample.temp_c < -40.0f || sample.temp_c > 85.0f) return 4;

    icm40609_ready = true;
    return 0;
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
static void icm40609_done(spi_job_t *job, const uint8_t *rx_buf, void *arg)
{
    icm40609_t *dev = (icm40609_t *)arg;
    if (!dev) return;

    icm40609_sample_t sample;
    /* rx_buf[0] is the address byte echo; data starts at rx_buf[1].
     * Burst length is 14 bytes: temp(2) + accel(6) + gyro(6). */
    if (icm40609_parse_accel_gyro(&rx_buf[1], &sample, dev)) {
        sample.t_us = job->t_sample;
        icm40609_sample_enqueue(&icm40609_sample_ring, &sample);
    }
}

/* -------------------------------------------------------------------------- */
/* Interrupt handler                                                          */
/* -------------------------------------------------------------------------- */
/*
 * Called from HAL_GPIO_EXTI_Rising_Callback (or Falling, depending on INT1
 * polarity) when the ICM-40609 DATA_RDY interrupt fires.
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
    job.done_arg = &icm40609_dev;
    job.sensor   = SENSOR_ID_ACCEL;
    job.task_notification_flag = ICM40609_SAMPLE_FLAG;

    /* New board wiring: ICM-40609 lives on SPI3, separate from the legacy
     * BMI088 IMU on SPI2. */
    spi_submit_job(job, &jobq_spi_3);
}
