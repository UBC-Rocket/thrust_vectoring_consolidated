#include "BMI088_gyro.h"
#include <string.h>   // memset
#include <math.h>     // (optional) for M_PI if you prefer
#include "debug/log.h"

/* Bosch SPI convention used here:
   - Write: first byte = (addr & 0x7F), then data bytes
   - Read : first byte = (addr | 0x80), then N dummy 0x00 bytes to clock out N data bytes
   - Gyro side of BMI088 does NOT require a dummy byte between addr and data (unlike accel).
*/

/* -------------------------------------------------------------------------- */
/* Helpers                                                                    */
/* -------------------------------------------------------------------------- */

static inline uint8_t spi_wr(uint8_t reg) { return (uint8_t)(reg & 0x7F); }
static inline uint8_t spi_rd(uint8_t reg) { return (uint8_t)(reg | 0x80); }

/* Scale table in LSB per (deg/s) from datasheet.
   (These are powers-of-two friendly for Bosch gyro parts.)
   2000 dps: 16.384 LSB/deg/s
   1000 dps: 32.768
    500 dps: 65.536
    250 dps: 131.072
    125 dps: 262.144
*/
static inline float range_to_scale_lsb_per_dps(bmi088_gyro_range_t r)
{
    switch (r) {
        case BMI088_GYRO_RANGE_2000DPS: return 16.384f;
        case BMI088_GYRO_RANGE_1000DPS: return 32.768f;
        case BMI088_GYRO_RANGE_500DPS:  return 65.536f;
        case BMI088_GYRO_RANGE_250DPS:  return 131.072f;
        default: /* 125 dps */         return 262.144f;
    }
}

static inline uint16_t odr_code_to_hz(bmi088_gyro_odr_t odr)
{
    switch (odr) {
        case BMI088_GYRO_ODR_2000_HZ_BW_532: return 2000;
        case BMI088_GYRO_ODR_1000_HZ_BW_230: return 1000;
        case BMI088_GYRO_ODR_400_HZ_BW_116:  return 400;
        case BMI088_GYRO_ODR_200_HZ_BW_47:   return 200;
        case BMI088_GYRO_ODR_200_HZ_BW_12:   return 200; // same ODR, narrower BW
        case BMI088_GYRO_ODR_100_HZ_BW_23:   return 100;
        case BMI088_GYRO_ODR_100_HZ_BW_12:   return 100;
        case BMI088_GYRO_ODR_100_HZ_BW_8:    return 100;
        default: return 0;
    }
}

/* -------------------------------------------------------------------------- */
/* Soft reset                                                                 */
/* -------------------------------------------------------------------------- */

size_t bmi088_gyro_build_softreset(uint8_t *tx_buf)
{
    // Write 0xB6 to GYRO_SOFTRESET (0x14). Delay ~30 ms after issuing this write.
    tx_buf[0] = spi_wr(BMI088_GYRO_SOFTRESET);
    tx_buf[1] = 0xB6;
    return 2;
}

/* -------------------------------------------------------------------------- */
/* Power configuration                                                        */
/* -------------------------------------------------------------------------- */

size_t bmi088_gyro_build_power_conf(bmi088_gyro_power_t pwr,
                                    uint8_t *tx_buf,
                                    bmi088_gyro_t *dev)
{
    /* GYRO_LPM1 (0x11) power modes (commonly documented):
       - 0x00: normal
       - 0x80: suspend
       - 0x20: deep suspend (not exposed here)
    */
    uint8_t val = (pwr == BMI088_GYRO_PWR_SUSPEND) ? 0x80u : 0x00u;

    tx_buf[0] = spi_wr(BMI088_GYRO_PWR_CTRL);
    tx_buf[1] = val;

    if (dev) dev->pwr_ctrl_reg = val;
    return 2;
}

/* -------------------------------------------------------------------------- */
/* Range configuration                                                        */
/* -------------------------------------------------------------------------- */

size_t bmi088_gyro_build_range(bmi088_gyro_range_t range,
                               uint8_t *tx_buf,
                               bmi088_gyro_t *dev)
{
    // GYRO_RANGE (0x0F): write enum value (0x00..0x04)
    tx_buf[0] = spi_wr(BMI088_GYRO_RANGE);
    tx_buf[1] = (uint8_t)range;

    if (dev) {
        dev->range_reg    = (uint8_t)range;
        dev->scale_dps_lsb = range_to_scale_lsb_per_dps(range);
    }
    return 2;
}

/* -------------------------------------------------------------------------- */
/* ODR / bandwidth configuration                                              */
/* -------------------------------------------------------------------------- */

size_t bmi088_gyro_build_odr(bmi088_gyro_odr_t odr,
                             uint8_t *tx_buf,
                             bmi088_gyro_t *dev)
{
    /* GYRO_BANDWIDTH (0x10):
       - Valid codes are 0x00..0x07 (bit7 reads as 1, ignore on write).
       - We write only the lower 3 bits.
    */
    uint8_t code = ((uint8_t)odr) & 0x07u;

    tx_buf[0] = spi_wr(BMI088_GYRO_BANDWIDTH);
    tx_buf[1] = code;

    if (dev) {
        dev->conf_reg = code;
        dev->odr_hz   = odr_code_to_hz(odr);
    }
    return 2;
}

/* -------------------------------------------------------------------------- */
/* Read-command builder                                                       */
/* -------------------------------------------------------------------------- */

size_t bmi088_gyro_build_read(bmi088_gyro_read_t what, uint8_t *tx_buf)
{
    /* Gyro reads have NO dummy byte. We append N zeros to clock out N data bytes. */
    switch (what) {
        case BMI088_GYRO_READ_CHIP_ID:
            tx_buf[0] = spi_rd(BMI088_GYRO_CHIP_ID_REG);
            tx_buf[1] = 0x00;
            return 2;

        case BMI088_GYRO_READ_INT_STAT_1:
            tx_buf[0] = spi_rd(BMI088_GYRO_INT_STAT_1);
            tx_buf[1] = 0x00;
            return 2;

        case BMI088_GYRO_READ_DATA_XYZ:
            /* Read LSB first per axis, MSB locked after LSB read.
               We request 6 data bytes: X_L, X_H, Y_L, Y_H, Z_L, Z_H
            */
            tx_buf[0] = spi_rd(BMI088_GYRO_RATE_X_LSB);
            tx_buf[1] = 0x00; // X L
            tx_buf[2] = 0x00; // X H
            tx_buf[3] = 0x00; // Y L
            tx_buf[4] = 0x00; // Y H
            tx_buf[5] = 0x00; // Z L
            tx_buf[6] = 0x00; // Z H
            return 7;

        case BMI088_GYRO_READ_FIFO_STATUS:
            tx_buf[0] = spi_rd(BMI088_GYRO_FIFO_STATUS);
            tx_buf[1] = 0x00;
            return 2;

        case BMI088_GYRO_READ_FIFO_DATA:
            /* Caller should decide how many bytes to clock out (burst);
               here we only provide the header (address). */
            tx_buf[0] = spi_rd(BMI088_GYRO_FIFO_DATA);
            return 1;

        default:
            return 0;
    }
}

/* -------------------------------------------------------------------------- */
/* Parsing helpers                                                            */
/* -------------------------------------------------------------------------- */

bool bmi088_gyro_parse_data_xyz(const uint8_t *rx_buf,
                                bmi088_gyro_sample_t *sample,
                                const bmi088_gyro_t *dev)
{
    if (!rx_buf || !sample || !dev || dev->scale_dps_lsb <= 0.0f) return false;

    // rx_buf layout expected from build_read(DATA_XYZ): [addr][XL][XH][YL][YH][ZL][ZH]
    const int16_t x = (int16_t)((rx_buf[2] << 8) | rx_buf[1]);
    const int16_t y = (int16_t)((rx_buf[4] << 8) | rx_buf[3]);
    const int16_t z = (int16_t)((rx_buf[6] << 8) | rx_buf[5]);

    sample->x = x;
    sample->y = y;
    sample->z = z;

    // Convert to deg/s then to rad/s
    const float dps_x = ((float)x) / dev->scale_dps_lsb;
    const float dps_y = ((float)y) / dev->scale_dps_lsb;
    const float dps_z = ((float)z) / dev->scale_dps_lsb;

    // DLOG_PRINT("Peener\n");
    // DLOG_PRINT("%f %f %f\n", dps_x, dps_y, dps_z);

    const float kDeg2Rad = 0.017453292519943295769f; // pi/180
    sample->gx = dps_x * kDeg2Rad;
    sample->gy = dps_y * kDeg2Rad;
    sample->gz = dps_z * kDeg2Rad;

    return true;
}

/* FIFO status parse:
   FIFO_STATUS (0x0E)
   - [7] fifo_overrun (commonly named)
   - [6:0] fifo_frame_counter (0..127)
   If your doc uses slightly different bit names, adjust masks below.
*/
bool bmi088_gyro_parse_fifo_status(const uint8_t *rx_buf,
                                   uint8_t *fifo_frame_count,
                                   bool *fifo_overrun)
{
    if (!rx_buf || !fifo_frame_count || !fifo_overrun) return false;

    // rx_buf expected: [addr][status]
    const uint8_t st = rx_buf[1];
    *fifo_overrun     = ((st & 0x80u) != 0);
    *fifo_frame_count = (uint8_t)(st & 0x7Fu);
    return true;
}

/* -------------------------------------------------------------------------- */
/* Interrupt pin configuration                                                 */
/* -------------------------------------------------------------------------- */
/*
GYR_INT3_INT4_IO_CONF (0x16)
-----------------------------------
Bit layout (datasheet):
[7] INT4_od       0=push-pull, 1=open-drain
[6] INT3_od       0=push-pull, 1=open-drain
[5] INT4_lvl      0=active high, 1=active low
[4] INT3_lvl      0=active high, 1=active low
[3:0] reserved
*/
size_t bmi088_gyro_build_int_pin_conf(bmi088_gyro_int_pin_t pin,
                                      bmi088_gyro_int_drive_t drive,
                                      bmi088_gyro_int_polarity_t polarity,
                                      uint8_t *tx_buf)
{
    uint8_t regval = 0;

    if (pin == BMI088_GYRO_INT3) {
        if (drive == BMI088_GYRO_INT_OPEN_DRAIN) regval |= (1u << 6);
        if (polarity == BMI088_GYRO_INT_ACTIVE_LOW) regval |= (1u << 4);
    } else { /* INT4 */
        if (drive == BMI088_GYRO_INT_OPEN_DRAIN) regval |= (1u << 7);
        if (polarity == BMI088_GYRO_INT_ACTIVE_LOW) regval |= (1u << 5);
    }

    tx_buf[0] = spi_wr(BMI088_GYRO_INT3_INT4_IO_CONF);
    tx_buf[1] = regval;
    return 2;
}

/* -------------------------------------------------------------------------- */
/* Interrupt event mapping / enable                                            */
/* -------------------------------------------------------------------------- */
/*
GYR_INT_CTRL (0x15)
-----------------------------------
[7] drdy_en   → enable data-ready interrupt
[6] fifo_en   → enable FIFO interrupt
[5:0] reserved

GYR_INT3_INT4_IO_MAP (0x18)
-----------------------------------
[1] fifo_int3 / fifo_int4
[0] drdy_int3 / drdy_int4
Writing 1 routes corresponding interrupt to chosen pin.
*/
size_t bmi088_gyro_build_int_event_map(bmi088_gyro_int_pin_t pin,
                                       uint8_t events,
                                       uint8_t *tx_buf)
{
    uint8_t int_ctrl = 0;
    uint8_t int_map  = 0;

    /* Enable sources in INT_CTRL */
    if (events & BMI088_GYRO_INT_EVENT_DATA_READY)
        int_ctrl |= (1u << 7);
    if (events & (BMI088_GYRO_INT_EVENT_FIFO_WTM | BMI088_GYRO_INT_EVENT_FIFO_FULL))
        int_ctrl |= (1u << 6);

    /* Map to INT3/INT4 */
    if (pin == BMI088_GYRO_INT3) {
        if (events & BMI088_GYRO_INT_EVENT_DATA_READY) int_map |= (1u << 0);
        if (events & (BMI088_GYRO_INT_EVENT_FIFO_WTM | BMI088_GYRO_INT_EVENT_FIFO_FULL))
            int_map |= (1u << 1);
    } else { /* INT4 */
        if (events & BMI088_GYRO_INT_EVENT_DATA_READY) int_map |= (1u << 4);
        if (events & (BMI088_GYRO_INT_EVENT_FIFO_WTM | BMI088_GYRO_INT_EVENT_FIFO_FULL))
            int_map |= (1u << 5);
    }

    /* We write these in sequence: first INT_CTRL, then INT3_INT4_IO_MAP */
    tx_buf[0] = spi_wr(BMI088_GYRO_INT_CTRL);
    tx_buf[1] = int_ctrl;
    tx_buf[2] = spi_wr(BMI088_GYRO_INT3_INT4_IO_MAP);
    tx_buf[3] = int_map;

    return 4; /* total bytes written */
}

/* -------------------------------------------------------------------------- */
/* FIFO configuration                                                          */
/* -------------------------------------------------------------------------- */
/*
FIFO_CONFIG_0 (0x3D): [6:0] watermark; writing clears FIFO
FIFO_CONFIG_1 (0x3E):
  [7] stream mode enable
  [6] fifo mode enable
  others reserved
*/
size_t bmi088_gyro_build_fifo_conf(bmi088_gyro_fifo_mode_t mode,
                                   bmi088_gyro_fifo_source_t src,
                                   uint8_t watermark,
                                   uint8_t *tx_buf)
{
    (void)src; /* Only XYZ supported; sensortime not available on gyro side */

    uint8_t cfg0 = (uint8_t)(watermark & 0x7Fu);
    uint8_t cfg1 = (uint8_t)mode;  /* already carries bit pattern (0x40/0x80/0x00) */

    tx_buf[0] = spi_wr(BMI088_GYRO_FIFO_CONFIG_0);
    tx_buf[1] = cfg0;
    tx_buf[2] = spi_wr(BMI088_GYRO_FIFO_CONFIG_1);
    tx_buf[3] = cfg1;

    return 4;
}

/* -------------------------------------------------------------------------- */
/* FIFO data parser                                                            */
/* -------------------------------------------------------------------------- */
/*
Each gyro FIFO frame (header disabled) = 6 bytes: X_L, X_H, Y_L, Y_H, Z_L, Z_H
Total bytes = 6 * frame_count
*/
bool bmi088_gyro_parse_fifo(const uint8_t *rx_buf, size_t len,
                            bmi088_gyro_sample_t *out_samples,
                            size_t *max_samples,
                            const bmi088_gyro_t *dev)
{
    if (!rx_buf || !out_samples || !max_samples || !dev ||
        dev->scale_dps_lsb <= 0.0f)
        return false;

    const size_t frame_size = 6;
    size_t n_frames = len / frame_size;
    if (n_frames > *max_samples) n_frames = *max_samples;

    const float kDeg2Rad = 0.017453292519943295769f;

    for (size_t i = 0; i < n_frames; ++i) {
        const uint8_t *p = &rx_buf[i * frame_size];
        const int16_t x = (int16_t)((p[1] << 8) | p[0]);
        const int16_t y = (int16_t)((p[3] << 8) | p[2]);
        const int16_t z = (int16_t)((p[5] << 8) | p[4]);

        bmi088_gyro_sample_t *s = &out_samples[i];
        s->x = x; s->y = y; s->z = z;
        const float dps_x = ((float)x) / dev->scale_dps_lsb;
        const float dps_y = ((float)y) / dev->scale_dps_lsb;
        const float dps_z = ((float)z) / dev->scale_dps_lsb;
        s->gx = dps_x * kDeg2Rad;
        s->gy = dps_y * kDeg2Rad;
        s->gz = dps_z * kDeg2Rad;
    }

    *max_samples = n_frames;
    return true;
}
