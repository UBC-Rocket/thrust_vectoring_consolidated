#include "sensors/icm40609.h"
#include <math.h>

/* ---- Internal SPI command helpers ---- */
#define SPI_RD(a)   ((uint8_t)((a) | 0x80))
#define SPI_WR(a)   ((uint8_t)((a) & 0x7F))

/* ---- Scale factor lookup helpers ---- */

static void icm40609_update_accel_scale(icm40609_t *dev, icm40609_accel_fs_t fs)
{
    if (!dev) return;
    dev->accel_fs_sel = (uint8_t)fs;
    /* Sensitivity in LSB/g from datasheet Table 2 */
    switch (fs) {
        case ICM40609_ACCEL_FS_32G: dev->accel_scale = 1.0f / 1024.0f;  break;
        case ICM40609_ACCEL_FS_16G: dev->accel_scale = 1.0f / 2048.0f;  break;
        case ICM40609_ACCEL_FS_8G:  dev->accel_scale = 1.0f / 4096.0f;  break;
        case ICM40609_ACCEL_FS_4G:  dev->accel_scale = 1.0f / 8192.0f;  break;
    }
}

static void icm40609_update_gyro_scale(icm40609_t *dev, icm40609_gyro_fs_t fs)
{
    if (!dev) return;
    dev->gyro_fs_sel = (uint8_t)fs;
    /* Sensitivity in LSB/(deg/s) from datasheet Table 1 */
    switch (fs) {
        case ICM40609_GYRO_FS_2000DPS:    dev->gyro_scale = 1.0f / 16.4f;    break;
        case ICM40609_GYRO_FS_1000DPS:    dev->gyro_scale = 1.0f / 32.8f;    break;
        case ICM40609_GYRO_FS_500DPS:     dev->gyro_scale = 1.0f / 65.5f;    break;
        case ICM40609_GYRO_FS_250DPS:     dev->gyro_scale = 1.0f / 131.0f;   break;
        case ICM40609_GYRO_FS_125DPS:     dev->gyro_scale = 1.0f / 262.0f;   break;
        case ICM40609_GYRO_FS_62_5DPS:    dev->gyro_scale = 1.0f / 524.3f;   break;
        case ICM40609_GYRO_FS_31_25DPS:   dev->gyro_scale = 1.0f / 1048.6f;  break;
        case ICM40609_GYRO_FS_15_625DPS:  dev->gyro_scale = 1.0f / 2097.2f;  break;
    }
}

/* Sample period in microseconds, rounded to the nearest µs. Fractional
 * cases:
 *   32 kHz  -> 31.25 µs  -> 31
 *   16 kHz  -> 62.5  µs  -> 63
 *   12.5 / 6.25 / 3.125 / 1.5625 Hz round to exact integers anyway.
 * Rounding error is well below any timing tolerance the consumer cares
 * about; if a caller needs sub-µs precision, they should track sample
 * timestamps directly instead of computing from the nominal period. */
static uint32_t icm40609_odr_to_period_us(icm40609_odr_t odr)
{
    switch (odr) {
        case ICM40609_ODR_32KHZ:    return 31;       /* 31.25 us, rounded */
        case ICM40609_ODR_16KHZ:    return 63;       /* 62.5 us, rounded */
        case ICM40609_ODR_8KHZ:     return 125;
        case ICM40609_ODR_4KHZ:     return 250;
        case ICM40609_ODR_2KHZ:     return 500;
        case ICM40609_ODR_1KHZ:     return 1000;
        case ICM40609_ODR_500HZ:    return 2000;
        case ICM40609_ODR_200HZ:    return 5000;
        case ICM40609_ODR_100HZ:    return 10000;
        case ICM40609_ODR_50HZ:     return 20000;
        case ICM40609_ODR_25HZ:     return 40000;
        case ICM40609_ODR_12_5HZ:   return 80000;
        case ICM40609_ODR_6_25HZ:   return 160000;
        case ICM40609_ODR_3_125HZ:  return 320000;
        case ICM40609_ODR_1_5625HZ: return 640000;
        default:                    return 0;
    }
}

/* -------------------------------------------------------------------------- */
/* Soft reset                                                                 */
/* -------------------------------------------------------------------------- */
size_t icm40609_build_soft_reset(uint8_t *tx_buf)
{
    tx_buf[0] = SPI_WR(ICM40609_REG_DEVICE_CONFIG);
    tx_buf[1] = 0x01;  /* SOFT_RESET_CONFIG = 1 */
    return 2;
}

/* -------------------------------------------------------------------------- */
/* Bank selection                                                             */
/* -------------------------------------------------------------------------- */
size_t icm40609_build_set_bank(icm40609_bank_t bank, uint8_t *tx_buf)
{
    tx_buf[0] = SPI_WR(ICM40609_REG_REG_BANK_SEL);
    tx_buf[1] = (uint8_t)(bank & 0x07);
    return 2;
}

/* -------------------------------------------------------------------------- */
/* Single-register read                                                       */
/* -------------------------------------------------------------------------- */
size_t icm40609_build_read_reg(uint8_t reg, uint8_t *tx_buf)
{
    /*
     * ICM-40609 SPI read: first byte = reg | 0x80, then clock out data.
     * No dummy byte required (unlike BMI088 accelerometer).
     */
    tx_buf[0] = SPI_RD(reg);
    tx_buf[1] = 0x00;
    return 2;
}

/* -------------------------------------------------------------------------- */
/* Single-register write                                                      */
/* -------------------------------------------------------------------------- */
size_t icm40609_build_write_reg(uint8_t reg, uint8_t value, uint8_t *tx_buf)
{
    tx_buf[0] = SPI_WR(reg);
    tx_buf[1] = value;
    return 2;
}

/* -------------------------------------------------------------------------- */
/* Power management                                                           */
/* -------------------------------------------------------------------------- */
size_t icm40609_build_pwr_mgmt0(icm40609_gyro_mode_t gyro_mode,
                                 icm40609_accel_mode_t accel_mode,
                                 bool temp_dis,
                                 uint8_t *tx_buf)
{
    /*
     * PWR_MGMT0 (0x4E):
     *   bit 5: TEMP_DIS
     *   bit 4: IDLE (leave 0)
     *   bits 3:2: GYRO_MODE
     *   bits 1:0: ACCEL_MODE
     */
    uint8_t val = 0;
    if (temp_dis) val |= (1u << 5);
    val |= ((uint8_t)gyro_mode & 0x03) << 2;
    val |= ((uint8_t)accel_mode & 0x03);

    tx_buf[0] = SPI_WR(ICM40609_REG_PWR_MGMT0);
    tx_buf[1] = val;
    return 2;
}

/* -------------------------------------------------------------------------- */
/* Gyro configuration                                                         */
/* -------------------------------------------------------------------------- */
size_t icm40609_build_gyro_config(icm40609_gyro_fs_t fs_sel,
                                   icm40609_odr_t odr,
                                   uint8_t *tx_buf,
                                   icm40609_t *dev)
{
    /*
     * GYRO_CONFIG0 (0x4F):
     *   bits 7:5: GYRO_FS_SEL
     *   bit  4:   Reserved
     *   bits 3:0: GYRO_ODR
     */
    uint8_t val = ((uint8_t)(fs_sel & 0x07) << 5) | ((uint8_t)(odr & 0x0F));

    tx_buf[0] = SPI_WR(ICM40609_REG_GYRO_CONFIG0);
    tx_buf[1] = val;

    if (dev) {
        icm40609_update_gyro_scale(dev, fs_sel);
        dev->gyro_odr = (uint8_t)odr;
        dev->gyro_period_us = icm40609_odr_to_period_us(odr);
    }
    return 2;
}

/* -------------------------------------------------------------------------- */
/* Accel configuration                                                        */
/* -------------------------------------------------------------------------- */
size_t icm40609_build_accel_config(icm40609_accel_fs_t fs_sel,
                                    icm40609_odr_t odr,
                                    uint8_t *tx_buf,
                                    icm40609_t *dev)
{
    /*
     * ACCEL_CONFIG0 (0x50):
     *   bits 7:5: ACCEL_FS_SEL
     *   bit  4:   Reserved
     *   bits 3:0: ACCEL_ODR
     */
    uint8_t val = ((uint8_t)(fs_sel & 0x07) << 5) | ((uint8_t)(odr & 0x0F));

    tx_buf[0] = SPI_WR(ICM40609_REG_ACCEL_CONFIG0);
    tx_buf[1] = val;

    if (dev) {
        icm40609_update_accel_scale(dev, fs_sel);
        dev->accel_odr = (uint8_t)odr;
        dev->accel_period_us = icm40609_odr_to_period_us(odr);
    }
    return 2;
}

/* -------------------------------------------------------------------------- */
/* Interrupt configuration                                                    */
/* -------------------------------------------------------------------------- */
size_t icm40609_build_int_config(icm40609_int_polarity_t int1_polarity,
                                  icm40609_int_drive_t int1_drive,
                                  icm40609_int_mode_t int1_mode,
                                  uint8_t *tx_buf)
{
    /*
     * INT_CONFIG (0x14), reset value 0x00 per DS-000330 §13.3.
     *   bit 5: INT2_MODE
     *   bit 4: INT2_DRIVE_CIRCUIT
     *   bit 3: INT2_POLARITY
     *   bit 2: INT1_MODE
     *   bit 1: INT1_DRIVE_CIRCUIT
     *   bit 0: INT1_POLARITY
     *
     * We only configure INT1. INT2 bits are kept at their reset values (0),
     * which leaves INT2 pulsed / open-drain / active-low — matching the
     * tristate behaviour we want when INT2 isn't wired up.
     */
    uint8_t val = 0;
    val |= ((uint8_t)int1_mode     & 0x01) << 2;
    val |= ((uint8_t)int1_drive    & 0x01) << 1;
    val |= ((uint8_t)int1_polarity & 0x01) << 0;

    tx_buf[0] = SPI_WR(ICM40609_REG_INT_CONFIG);
    tx_buf[1] = val;
    return 2;
}

/* -------------------------------------------------------------------------- */
/* Interrupt source configuration                                             */
/* -------------------------------------------------------------------------- */
size_t icm40609_build_int_source0(bool drdy_en,
                                   bool fifo_ths_en,
                                   bool fifo_full_en,
                                   uint8_t *tx_buf)
{
    /*
     * INT_SOURCE0 (0x65):
     *   bit 3: UI_DRDY_INT1_EN
     *   bit 2: FIFO_THS_INT1_EN
     *   bit 1: FIFO_FULL_INT1_EN
     *   bit 0: UI_AGC_RDY_INT1_EN (leave 0)
     */
    uint8_t val = 0;
    if (drdy_en)     val |= (1u << 3);
    if (fifo_ths_en) val |= (1u << 2);
    if (fifo_full_en)val |= (1u << 1);

    tx_buf[0] = SPI_WR(ICM40609_REG_INT_SOURCE0);
    tx_buf[1] = val;
    return 2;
}

/* -------------------------------------------------------------------------- */
/* Drive configuration (SPI slew rate)                                        */
/* -------------------------------------------------------------------------- */
size_t icm40609_build_drive_config(uint8_t spi_slew_rate, uint8_t *tx_buf)
{
    /*
     * DRIVE_CONFIG (0x13), reset value 0x05:
     *   bits 5:3: I2C_SLEW_RATE
     *   bits 2:0: SPI_SLEW_RATE
     *
     * Per section 11.3: for SPI operation, set SPI_SLEW_RATE = 5,
     * I2C_SLEW_RATE = 0.
     */
    uint8_t val = (spi_slew_rate & 0x07);

    tx_buf[0] = SPI_WR(ICM40609_REG_DRIVE_CONFIG);
    tx_buf[1] = val;
    return 2;
}

/* -------------------------------------------------------------------------- */
/* Interface configuration (clock)                                            */
/* -------------------------------------------------------------------------- */
size_t icm40609_build_intf_config1(uint8_t clksel, uint8_t *tx_buf)
{
    /*
     * INTF_CONFIG1 (0x4D), reset value 0x91 per DS-000330 §13.28:
     *   bits 7:6: EN_TEST_MODE (must be 01 for normal operation)
     *   bits 5:4: Reserved (reset value: 01 — preserve)
     *   bit  3:   ACCEL_LP_CLK_SEL (0 = Wake Up oscillator, default)
     *   bit  2:   Reserved (reset: 0)
     *   bits 1:0: CLKSEL (01 = auto-select PLL)
     *
     * We preserve bits 5:4 = 01 per "do not modify reserved bits" guidance,
     * then set CLKSEL and EN_TEST_MODE.
     */
    uint8_t val = (0x01u << 6)        /* EN_TEST_MODE = 01 (normal) */
                | (0x01u << 4)        /* reserved, preserve reset state */
                | (clksel & 0x03u);   /* CLKSEL */

    tx_buf[0] = SPI_WR(ICM40609_REG_INTF_CONFIG1);
    tx_buf[1] = val;
    return 2;
}

/* -------------------------------------------------------------------------- */
/* Interface configuration (endianness, FIFO format)                          */
/* -------------------------------------------------------------------------- */
size_t icm40609_build_intf_config0(uint8_t value, uint8_t *tx_buf)
{
    /*
     * INTF_CONFIG0 (0x4C), reset value 0x30 per DS-000330 §13.27.
     *   bit 7: FIFO_HOLD_LAST_DATA_EN
     *   bit 6: FIFO_COUNT_REC      (0 = bytes, 1 = records)
     *   bit 5: FIFO_COUNT_ENDIAN   (1 = big-endian)
     *   bit 4: SENSOR_DATA_ENDIAN  (1 = big-endian)
     *   bits 3:2: Reserved
     *   bits 1:0: UI_SIFS_CFG
     *
     * The parsers in this driver assume:
     *   - sensor data big-endian        (bit 4 = 1)
     *   - FIFO count big-endian         (bit 5 = 1)
     *   - FIFO count in bytes           (bit 6 = 0)
     * Callers should pass ICM40609_INTF_CONFIG0_DEFAULT (0x30) unless they
     * really mean to deviate from any of those.
     *
     * INT pulse duration lives in INT_CONFIG1, not here — see
     * icm40609_build_int_config1().
     */
    tx_buf[0] = SPI_WR(ICM40609_REG_INTF_CONFIG0);
    tx_buf[1] = value;
    return 2;
}

/* -------------------------------------------------------------------------- */
/* INT_CONFIG1 (INT pulse / de-assert / async reset)                          */
/* -------------------------------------------------------------------------- */
size_t icm40609_build_int_config1(uint8_t value, uint8_t *tx_buf)
{
    /*
     * INT_CONFIG1 (0x64), reset value 0x10 per DS-000330 §13.42.
     *   bit 7:   Reserved
     *   bit 6:   INT_TPULSE_DURATION   (0 = 100µs for ODR<4kHz, default;
     *                                   1 = 8µs, required for ODR>=4kHz)
     *   bit 5:   INT_TDEASSERT_DISABLE (0 = 100µs min de-assert, default;
     *                                   1 = disabled, required for ODR>=4kHz)
     *   bit 4:   INT_ASYNC_RESET       (reset = 1; per the datasheet, "User
     *                                   should change setting to 0 from
     *                                   default setting of 1, for proper
     *                                   INT1 and INT2 pin operation")
     *   bits 3:0: Reserved
     *
     * Pass ICM40609_INT_CONFIG1_DEFAULT (0x00) for ODR < 4 kHz, or
     * ICM40609_INT_CONFIG1_HIGH_ODR (0x60) for ODR >= 4 kHz.
     */
    tx_buf[0] = SPI_WR(ICM40609_REG_INT_CONFIG1);
    tx_buf[1] = value;
    return 2;
}

/* -------------------------------------------------------------------------- */
/* Burst read: temp + accel + gyro                                            */
/* -------------------------------------------------------------------------- */
size_t icm40609_build_read_accel_gyro(uint8_t *tx_buf)
{
    /*
     * Burst read starting from TEMP_DATA1 (0x1D):
     *   TEMP_DATA1, TEMP_DATA0,
     *   ACCEL_DATA_X1, X0, Y1, Y0, Z1, Z0,
     *   GYRO_DATA_X1, X0, Y1, Y0, Z1, Z0
     * = 14 data bytes
     *
     * SPI frame: [addr|0x80] [14 x 0x00 to clock data out]
     */
    tx_buf[0] = SPI_RD(ICM40609_REG_TEMP_DATA1);
    for (size_t i = 0; i < ICM40609_BURST_READ_LEN; i++)
        tx_buf[1 + i] = 0x00;

    return 1 + ICM40609_BURST_READ_LEN;  /* 15 bytes */
}

/* -------------------------------------------------------------------------- */
/* Read INT_STATUS                                                            */
/* -------------------------------------------------------------------------- */
size_t icm40609_build_read_int_status(uint8_t *tx_buf)
{
    return icm40609_build_read_reg(ICM40609_REG_INT_STATUS, tx_buf);
}

/* -------------------------------------------------------------------------- */
/* FIFO configuration                                                         */
/* -------------------------------------------------------------------------- */
size_t icm40609_build_fifo_config(icm40609_fifo_mode_t mode,
                                   bool accel_en,
                                   bool gyro_en,
                                   bool temp_en,
                                   uint16_t watermark,
                                   uint8_t *tx_buf)
{
    uint8_t *p = tx_buf;

    /* FIFO_CONFIG (0x16): bits 7:6 = FIFO_MODE */
    *p++ = SPI_WR(ICM40609_REG_FIFO_CONFIG);
    *p++ = ((uint8_t)(mode & 0x03)) << 6;

    /* FIFO_CONFIG1 (0x5F):
     *   bit 0: FIFO_ACCEL_EN
     *   bit 1: FIFO_GYRO_EN
     *   bit 2: FIFO_TEMP_EN
     *   bit 3: FIFO_TMST_FSYNC_EN
     *   bit 4: FIFO_WM_GT_TH
     *   bit 5: FIFO_RESUME_PARTIAL_RD
     *
     * When both accel and gyro are enabled the device emits Packet 3
     * (16 bytes), which always includes a 2-byte timestamp slot. The
     * timestamp is only populated if FIFO_TMST_FSYNC_EN is set, otherwise
     * those two bytes are zero. We enable it so the slot is meaningful
     * and matches the Packet 3 layout the parser expects.
     */
    uint8_t cfg1 = 0;
    if (accel_en) cfg1 |= (1u << 0);
    if (gyro_en)  cfg1 |= (1u << 1);
    if (temp_en)  cfg1 |= (1u << 2);
    if (accel_en && gyro_en) cfg1 |= (1u << 3);  /* TMST_FSYNC_EN for Packet 3 */
    *p++ = SPI_WR(ICM40609_REG_FIFO_CONFIG1);
    *p++ = cfg1;

    /* FIFO_CONFIG2 (0x60): FIFO_WM[7:0] */
    *p++ = SPI_WR(ICM40609_REG_FIFO_CONFIG2);
    *p++ = (uint8_t)(watermark & 0xFF);

    /* FIFO_CONFIG3 (0x61): FIFO_WM[11:8] */
    *p++ = SPI_WR(ICM40609_REG_FIFO_CONFIG3);
    *p++ = (uint8_t)((watermark >> 8) & 0x0F);

    return (size_t)(p - tx_buf);
}

/* -------------------------------------------------------------------------- */
/* Read FIFO count                                                            */
/* -------------------------------------------------------------------------- */
size_t icm40609_build_read_fifo_count(uint8_t *tx_buf)
{
    /* Read 2 bytes: FIFO_COUNTH (0x2E) and FIFO_COUNTL (0x2F) */
    tx_buf[0] = SPI_RD(ICM40609_REG_FIFO_COUNTH);
    tx_buf[1] = 0x00;
    tx_buf[2] = 0x00;
    return 3;
}

/* -------------------------------------------------------------------------- */
/* Read FIFO data                                                             */
/* -------------------------------------------------------------------------- */
size_t icm40609_build_read_fifo_data(uint16_t byte_count,
                                      uint8_t *tx_buf,
                                      size_t tx_buf_capacity)
{
    /* Frame length = address byte + byte_count data bytes. Refuse if the
     * caller's buffer can't hold the full transaction. */
    if (!tx_buf) return 0;
    const size_t required = (size_t)byte_count + 1u;
    if (tx_buf_capacity < required) return 0;

    tx_buf[0] = SPI_RD(ICM40609_REG_FIFO_DATA);
    for (uint16_t i = 0; i < byte_count; i++)
        tx_buf[1 + i] = 0x00;
    return required;
}

/* -------------------------------------------------------------------------- */
/* FIFO flush                                                                 */
/* -------------------------------------------------------------------------- */
size_t icm40609_build_fifo_flush(uint8_t *tx_buf)
{
    /*
     * SIGNAL_PATH_RESET (0x4B):
     *   bit 1: FIFO_FLUSH = 1
     */
    tx_buf[0] = SPI_WR(ICM40609_REG_SIGNAL_PATH_RESET);
    tx_buf[1] = (1u << 1);
    return 2;
}

/* -------------------------------------------------------------------------- */
/* Parse burst read (temp + accel + gyro)                                     */
/* -------------------------------------------------------------------------- */
bool icm40609_parse_accel_gyro(const uint8_t *rx_buf,
                                icm40609_sample_t *sample,
                                const icm40609_t *dev)
{
    if (!rx_buf || !sample || !dev) return false;

    /*
     * rx_buf layout (14 bytes, big-endian by default):
     *   [0]  TEMP_DATA1 (high)
     *   [1]  TEMP_DATA0 (low)
     *   [2]  ACCEL_DATA_X1 (high)
     *   [3]  ACCEL_DATA_X0 (low)
     *   [4]  ACCEL_DATA_Y1 (high)
     *   [5]  ACCEL_DATA_Y0 (low)
     *   [6]  ACCEL_DATA_Z1 (high)
     *   [7]  ACCEL_DATA_Z0 (low)
     *   [8]  GYRO_DATA_X1 (high)
     *   [9]  GYRO_DATA_X0 (low)
     *   [10] GYRO_DATA_Y1 (high)
     *   [11] GYRO_DATA_Y0 (low)
     *   [12] GYRO_DATA_Z1 (high)
     *   [13] GYRO_DATA_Z0 (low)
     */

    /* Temperature (big-endian, 16-bit). DS-000330 §6.6: T[°C] = TEMP/132.48 + 25 */
    sample->temp_raw = (int16_t)((rx_buf[0] << 8) | rx_buf[1]);
    sample->temp_c   = (float)sample->temp_raw / 132.48f + 25.0f;

    /* Accelerometer (big-endian) */
    sample->ax_raw = (int16_t)((rx_buf[2]  << 8) | rx_buf[3]);
    sample->ay_raw = (int16_t)((rx_buf[4]  << 8) | rx_buf[5]);
    sample->az_raw = (int16_t)((rx_buf[6]  << 8) | rx_buf[7]);

    /* Gyroscope (big-endian) */
    sample->gx_raw = (int16_t)((rx_buf[8]  << 8) | rx_buf[9]);
    sample->gy_raw = (int16_t)((rx_buf[10] << 8) | rx_buf[11]);
    sample->gz_raw = (int16_t)((rx_buf[12] << 8) | rx_buf[13]);

    /* Convert accel to m/s^2: raw * (g/LSB) * 9.80665 */
    const float accel_mps2_per_lsb = dev->accel_scale * 9.80665f;
    sample->ax = (float)sample->ax_raw * accel_mps2_per_lsb;
    sample->ay = (float)sample->ay_raw * accel_mps2_per_lsb;
    sample->az = (float)sample->az_raw * accel_mps2_per_lsb;

    /* Convert gyro to rad/s: raw * (dps/LSB) * (pi/180) */
    const float gyro_rps_per_lsb = dev->gyro_scale * (3.14159265358979f / 180.0f);
    sample->gx = (float)sample->gx_raw * gyro_rps_per_lsb;
    sample->gy = (float)sample->gy_raw * gyro_rps_per_lsb;
    sample->gz = (float)sample->gz_raw * gyro_rps_per_lsb;

    return true;
}

/* -------------------------------------------------------------------------- */
/* Parse single FIFO packet                                                   */
/* -------------------------------------------------------------------------- */
bool icm40609_parse_fifo_packet(const uint8_t *rx_buf,
                                 size_t packet_size,
                                 icm40609_sample_t *sample,
                                 const icm40609_t *dev)
{
    if (!rx_buf || !sample || !dev) return false;

    uint8_t header = rx_buf[0];

    /* Check if FIFO is empty */
    if (header & ICM40609_FIFO_HEADER_EMPTY) return false;

    bool has_accel = (header & ICM40609_FIFO_HEADER_ACCEL) != 0;
    bool has_gyro  = (header & ICM40609_FIFO_HEADER_GYRO) != 0;

    const float accel_mps2_per_lsb = dev->accel_scale * 9.80665f;
    const float gyro_rps_per_lsb = dev->gyro_scale * (3.14159265358979f / 180.0f);

    /* FIFO temperature in Packet 1/2/3 is an 8-bit signed value:
     *   T[°C] = (TEMP_DATA / 2.07) + 25  (DS-000330 §6.6)
     * In Packet 3 the temperature byte sits at offset 13. */

    if (has_accel && has_gyro && packet_size >= 16) {
        /*
         * Packet 3 (16 bytes): header, accel(6), gyro(6), temp(1), timestamp(2)
         * Byte layout:
         *   [0]     Header
         *   [1..6]  Accel X_H, X_L, Y_H, Y_L, Z_H, Z_L
         *   [7..12] Gyro  X_H, X_L, Y_H, Y_L, Z_H, Z_L
         *   [13]    Temperature (8-bit signed)
         *   [14..15] Timestamp (zero unless FIFO_TMST_FSYNC_EN is set)
         */
        sample->ax_raw = (int16_t)((rx_buf[1] << 8) | rx_buf[2]);
        sample->ay_raw = (int16_t)((rx_buf[3] << 8) | rx_buf[4]);
        sample->az_raw = (int16_t)((rx_buf[5] << 8) | rx_buf[6]);

        sample->gx_raw = (int16_t)((rx_buf[7]  << 8) | rx_buf[8]);
        sample->gy_raw = (int16_t)((rx_buf[9]  << 8) | rx_buf[10]);
        sample->gz_raw = (int16_t)((rx_buf[11] << 8) | rx_buf[12]);

        sample->temp_raw = (int16_t)(int8_t)rx_buf[13];
        sample->temp_c   = (float)sample->temp_raw / 2.07f + 25.0f;

        sample->ax = (float)sample->ax_raw * accel_mps2_per_lsb;
        sample->ay = (float)sample->ay_raw * accel_mps2_per_lsb;
        sample->az = (float)sample->az_raw * accel_mps2_per_lsb;

        sample->gx = (float)sample->gx_raw * gyro_rps_per_lsb;
        sample->gy = (float)sample->gy_raw * gyro_rps_per_lsb;
        sample->gz = (float)sample->gz_raw * gyro_rps_per_lsb;

    } else if (has_accel && packet_size >= 8) {
        /* Packet 1 (8 bytes): header, accel(6), temp(1) */
        sample->ax_raw = (int16_t)((rx_buf[1] << 8) | rx_buf[2]);
        sample->ay_raw = (int16_t)((rx_buf[3] << 8) | rx_buf[4]);
        sample->az_raw = (int16_t)((rx_buf[5] << 8) | rx_buf[6]);

        sample->gx_raw = 0;
        sample->gy_raw = 0;
        sample->gz_raw = 0;

        sample->temp_raw = (int16_t)(int8_t)rx_buf[7];
        sample->temp_c   = (float)sample->temp_raw / 2.07f + 25.0f;

        sample->ax = (float)sample->ax_raw * accel_mps2_per_lsb;
        sample->ay = (float)sample->ay_raw * accel_mps2_per_lsb;
        sample->az = (float)sample->az_raw * accel_mps2_per_lsb;
        sample->gx = 0.0f;
        sample->gy = 0.0f;
        sample->gz = 0.0f;

    } else if (has_gyro && packet_size >= 8) {
        /* Packet 2 (8 bytes): header, gyro(6), temp(1) */
        sample->ax_raw = 0;
        sample->ay_raw = 0;
        sample->az_raw = 0;

        sample->gx_raw = (int16_t)((rx_buf[1] << 8) | rx_buf[2]);
        sample->gy_raw = (int16_t)((rx_buf[3] << 8) | rx_buf[4]);
        sample->gz_raw = (int16_t)((rx_buf[5] << 8) | rx_buf[6]);

        sample->temp_raw = (int16_t)(int8_t)rx_buf[7];
        sample->temp_c   = (float)sample->temp_raw / 2.07f + 25.0f;

        sample->ax = 0.0f;
        sample->ay = 0.0f;
        sample->az = 0.0f;
        sample->gx = (float)sample->gx_raw * gyro_rps_per_lsb;
        sample->gy = (float)sample->gy_raw * gyro_rps_per_lsb;
        sample->gz = (float)sample->gz_raw * gyro_rps_per_lsb;

    } else {
        return false;
    }

    return true;
}

/* -------------------------------------------------------------------------- */
/* Parse multiple FIFO packets                                                */
/* -------------------------------------------------------------------------- */
bool icm40609_parse_fifo(const uint8_t *rx_buf,
                          size_t len,
                          icm40609_sample_t *out_samples,
                          size_t *max_samples,
                          const icm40609_t *dev)
{
    if (!rx_buf || !out_samples || !max_samples || !dev) return false;

    size_t capacity = *max_samples;
    size_t count = 0;
    size_t i = 0;

    while (i < len && count < capacity) {
        uint8_t header = rx_buf[i];

        /* Empty FIFO marker */
        if (header & ICM40609_FIFO_HEADER_EMPTY) break;

        bool has_accel = (header & ICM40609_FIFO_HEADER_ACCEL) != 0;
        bool has_gyro  = (header & ICM40609_FIFO_HEADER_GYRO) != 0;
        bool has_tmst  = (header & ICM40609_FIFO_HEADER_TMST_FSYNC_MASK) != 0;

        /* Packet size depends on which fields are present in the header.
         *   Packet 1: accel only             -> 8 bytes
         *   Packet 2: gyro  only             -> 8 bytes
         *   Packet 3: accel + gyro           -> 16 bytes
         *               (always includes a timestamp slot, even when
         *                FIFO_TMST_FSYNC_EN is cleared and the slot reads 0)
         * The TMST_FSYNC header bits are informational for the parser to
         * know whether the timestamp slot is meaningful; they don't change
         * the byte length of Packet 3. */
        (void)has_tmst;

        size_t pkt_size;
        if (has_accel && has_gyro)
            pkt_size = 16;
        else if (has_accel || has_gyro)
            pkt_size = 8;
        else
            break;  /* unknown packet type */

        if (i + pkt_size > len) break;

        if (icm40609_parse_fifo_packet(&rx_buf[i], pkt_size,
                                        &out_samples[count], dev)) {
            count++;
        }
        i += pkt_size;
    }

    *max_samples = count;
    return (count > 0);
}

/* -------------------------------------------------------------------------- */
/* Parse FIFO count                                                           */
/* -------------------------------------------------------------------------- */
bool icm40609_parse_fifo_count(const uint8_t *rx_buf, uint16_t *count)
{
    if (!rx_buf || !count) return false;

    /*
     * FIFO count register pair is double-buffered: reading either register
     * snapshots the pair, so a single burst read starting at FIFO_COUNTH
     * (which the build function does) returns a coherent count. Output is
     * big-endian by default (matches BIG_ENDIAN bit in INTF_CONFIG0).
     */
    *count = ((uint16_t)rx_buf[0] << 8) | rx_buf[1];
    return true;
}
