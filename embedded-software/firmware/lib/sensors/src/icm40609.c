/**
 * @file icm40609.c
 * @brief Bus-agnostic driver for InvenSense ICM-40609-D — implementation.
 *
 * UBC Rocket, 2026
 */

#include "sensors/icm40609.h"

#include <math.h>

/* ---- Internal SPI command helpers ---- */
#define SPI_RD(a)   ((uint8_t)((a) | 0x80))
#define SPI_WR(a)   ((uint8_t)((a) & 0x7F))

#define GRAVITY_MPS2   9.80665f
#define DEG_TO_RAD     0.01745329251994329577f

/* -------------------------------------------------------------------------- */
/* Command builders                                                           */
/* -------------------------------------------------------------------------- */

size_t icm40609_build_softreset(uint8_t *tx_buf) {
    tx_buf[0] = SPI_WR(ICM40609_DEVICE_CONFIG);
    tx_buf[1] = 0x01;  /* SOFT_RESET = 1 */
    return 2;
}

size_t icm40609_build_bank_select(uint8_t bank, uint8_t *tx_buf) {
    tx_buf[0] = SPI_WR(ICM40609_REG_BANK_SEL);
    tx_buf[1] = (uint8_t)(bank & 0x07);
    return 2;
}

size_t icm40609_build_power(icm40609_power_t pwr, uint8_t *tx_buf, icm40609_t *dev) {
    tx_buf[0] = SPI_WR(ICM40609_PWR_MGMT0);
    tx_buf[1] = (uint8_t)pwr;
    if (dev) dev->pwr_mgmt0 = (uint8_t)pwr;
    return 2;
}

static float accel_scale_for(icm40609_accel_fs_t fs) {
    /* Datasheet: 32768 LSB = FS (g). Convert to m/s² per LSB. */
    float fs_g;
    switch (fs) {
    case ICM40609_ACCEL_FS_32G: fs_g = 32.0f; break;
    case ICM40609_ACCEL_FS_16G: fs_g = 16.0f; break;
    case ICM40609_ACCEL_FS_8G:  fs_g = 8.0f;  break;
    case ICM40609_ACCEL_FS_4G:  fs_g = 4.0f;  break;
    default:                    fs_g = 16.0f; break;
    }
    return (fs_g * GRAVITY_MPS2) / 32768.0f;
}

static float gyro_scale_for(icm40609_gyro_fs_t fs) {
    float fs_dps;
    switch (fs) {
    case ICM40609_GYRO_FS_4000DPS: fs_dps = 4000.0f; break;
    case ICM40609_GYRO_FS_2000DPS: fs_dps = 2000.0f; break;
    case ICM40609_GYRO_FS_1000DPS: fs_dps = 1000.0f; break;
    case ICM40609_GYRO_FS_500DPS:  fs_dps = 500.0f;  break;
    case ICM40609_GYRO_FS_250DPS:  fs_dps = 250.0f;  break;
    case ICM40609_GYRO_FS_125DPS:  fs_dps = 125.0f;  break;
    default:                       fs_dps = 2000.0f; break;
    }
    return (fs_dps * DEG_TO_RAD) / 32768.0f;
}

static uint16_t odr_to_hz(icm40609_odr_t odr) {
    switch (odr) {
    case ICM40609_ODR_8000_HZ: return 8000;
    case ICM40609_ODR_4000_HZ: return 4000;
    case ICM40609_ODR_2000_HZ: return 2000;
    case ICM40609_ODR_1000_HZ: return 1000;
    case ICM40609_ODR_200_HZ:  return 200;
    case ICM40609_ODR_100_HZ:  return 100;
    case ICM40609_ODR_50_HZ:   return 50;
    case ICM40609_ODR_25_HZ:   return 25;
    case ICM40609_ODR_12_5_HZ: return 12;
    default:                   return 0;
    }
}

size_t icm40609_build_accel_conf(icm40609_accel_fs_t fs,
                                 icm40609_odr_t odr,
                                 uint8_t *tx_buf,
                                 icm40609_t *dev) {
    /* ACCEL_CONFIG0: bits[7:5]=FS_SEL, bits[3:0]=ODR */
    uint8_t val = (uint8_t)(((fs & 0x07) << 5) | (odr & 0x0F));
    tx_buf[0] = SPI_WR(ICM40609_ACCEL_CONFIG0);
    tx_buf[1] = val;
    if (dev) {
        dev->accel_config0         = val;
        dev->accel_scale_mps2_lsb  = accel_scale_for(fs);
        dev->odr_hz                = odr_to_hz(odr);
    }
    return 2;
}

size_t icm40609_build_gyro_conf(icm40609_gyro_fs_t fs,
                                icm40609_odr_t odr,
                                uint8_t *tx_buf,
                                icm40609_t *dev) {
    uint8_t val = (uint8_t)(((fs & 0x07) << 5) | (odr & 0x0F));
    tx_buf[0] = SPI_WR(ICM40609_GYRO_CONFIG0);
    tx_buf[1] = val;
    if (dev) {
        dev->gyro_config0        = val;
        dev->gyro_scale_rads_lsb = gyro_scale_for(fs);
    }
    return 2;
}

size_t icm40609_build_int1_data_ready(uint8_t *tx_buf) {
    /* INT_CONFIG (0x14): INT1 push-pull, active-high, pulse mode.
     *   bit0=INT1_POLARITY (1=active high), bit1=INT1_DRIVE (1=push-pull),
     *   bit2=INT1_MODE (0=pulse).
     * INT_SOURCE0 (0x65): UI_DRDY_INT1_EN = bit3.
     * Two back-to-back writes. */
    size_t i = 0;
    tx_buf[i++] = SPI_WR(ICM40609_INT_CONFIG);
    tx_buf[i++] = 0x03;
    tx_buf[i++] = SPI_WR(ICM40609_INT_SOURCE0);
    tx_buf[i++] = 0x08;
    return i;
}

size_t icm40609_build_read(icm40609_read_t what, uint8_t *tx_buf) {
    uint8_t reg; size_t bytes;
    switch (what) {
    case ICM40609_READ_WHO_AM_I:           reg = ICM40609_WHO_AM_I;       bytes = 1; break;
    case ICM40609_READ_INT_STATUS:         reg = ICM40609_INT_STATUS;     bytes = 1; break;
    case ICM40609_READ_TEMP:               reg = ICM40609_TEMP_DATA_1;    bytes = 2; break;
    case ICM40609_READ_ACCEL_XYZ:          reg = ICM40609_ACCEL_DATA_X1;  bytes = 6; break;
    case ICM40609_READ_GYRO_XYZ:           reg = ICM40609_GYRO_DATA_X1;   bytes = 6; break;
    case ICM40609_READ_ACCEL_GYRO_XYZ:     reg = ICM40609_ACCEL_DATA_X1;  bytes = 12; break;
    case ICM40609_READ_FIFO_COUNT:         reg = ICM40609_FIFO_COUNTH;    bytes = 2; break;
    case ICM40609_READ_FIFO_DATA:          reg = ICM40609_FIFO_DATA;      bytes = 16; break;
    default:                               return 0;
    }
    tx_buf[0] = SPI_RD(reg);
    return 1 + bytes;
}

size_t icm40609_build_read_reg(uint8_t reg, uint8_t *tx_buf) {
    tx_buf[0] = SPI_RD(reg);
    return 2;
}

/* -------------------------------------------------------------------------- */
/* Parsers                                                                    */
/* -------------------------------------------------------------------------- */

static inline int16_t be16(const uint8_t *p) {
    return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}

bool icm40609_parse_accel(const uint8_t *rx_buf,
                          icm40609_sample_t *sample,
                          const icm40609_t *dev) {
    if (!rx_buf || !sample || !dev) return false;
    /* rx_buf[0] is the dummy byte clocked while CMD was on the wire. */
    sample->ax_raw = be16(&rx_buf[1]);
    sample->ay_raw = be16(&rx_buf[3]);
    sample->az_raw = be16(&rx_buf[5]);
    sample->ax = sample->ax_raw * dev->accel_scale_mps2_lsb;
    sample->ay = sample->ay_raw * dev->accel_scale_mps2_lsb;
    sample->az = sample->az_raw * dev->accel_scale_mps2_lsb;
    return true;
}

bool icm40609_parse_gyro(const uint8_t *rx_buf,
                         icm40609_sample_t *sample,
                         const icm40609_t *dev) {
    if (!rx_buf || !sample || !dev) return false;
    sample->gx_raw = be16(&rx_buf[1]);
    sample->gy_raw = be16(&rx_buf[3]);
    sample->gz_raw = be16(&rx_buf[5]);
    sample->gx = sample->gx_raw * dev->gyro_scale_rads_lsb;
    sample->gy = sample->gy_raw * dev->gyro_scale_rads_lsb;
    sample->gz = sample->gz_raw * dev->gyro_scale_rads_lsb;
    return true;
}

bool icm40609_parse_accel_gyro(const uint8_t *rx_buf,
                               icm40609_sample_t *sample,
                               const icm40609_t *dev) {
    if (!rx_buf || !sample || !dev) return false;
    sample->ax_raw = be16(&rx_buf[1]);
    sample->ay_raw = be16(&rx_buf[3]);
    sample->az_raw = be16(&rx_buf[5]);
    sample->gx_raw = be16(&rx_buf[7]);
    sample->gy_raw = be16(&rx_buf[9]);
    sample->gz_raw = be16(&rx_buf[11]);
    sample->ax = sample->ax_raw * dev->accel_scale_mps2_lsb;
    sample->ay = sample->ay_raw * dev->accel_scale_mps2_lsb;
    sample->az = sample->az_raw * dev->accel_scale_mps2_lsb;
    sample->gx = sample->gx_raw * dev->gyro_scale_rads_lsb;
    sample->gy = sample->gy_raw * dev->gyro_scale_rads_lsb;
    sample->gz = sample->gz_raw * dev->gyro_scale_rads_lsb;
    return true;
}

bool icm40609_parse_temp(const uint8_t *rx_buf, float *temp_c) {
    if (!rx_buf || !temp_c) return false;
    /* Datasheet: temp_c = raw / 132.48 + 25 */
    int16_t raw = be16(&rx_buf[1]);
    *temp_c = (raw / 132.48f) + 25.0f;
    return true;
}

bool icm40609_parse_who_am_i(const uint8_t *rx_buf, uint8_t *who) {
    if (!rx_buf || !who) return false;
    *who = rx_buf[1];
    return true;
}
