/*
    Driver for the ICM-40609-D 6-axis IMU (3-axis gyro + 3-axis accelerometer).
    This driver is bus-agnostic: it only builds SPI command frames and parses
    responses. No hardware dependencies.

    Datasheet: DS-000330-ICM-40609-D-v1.2

    @ UBC Rocket, 2026
*/

#ifndef ICM40609_H
#define ICM40609_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "sensors/spsc_queue.h"

/* -------------------------------------------------------------------------- */
/* Register map — User Bank 0                                                 */
/* -------------------------------------------------------------------------- */
#define ICM40609_REG_DEVICE_CONFIG      0x11
#define ICM40609_REG_DRIVE_CONFIG       0x13
#define ICM40609_REG_INT_CONFIG         0x14
#define ICM40609_REG_FIFO_CONFIG        0x16

#define ICM40609_REG_TEMP_DATA1         0x1D
#define ICM40609_REG_TEMP_DATA0         0x1E
#define ICM40609_REG_ACCEL_DATA_X1      0x1F
#define ICM40609_REG_ACCEL_DATA_X0      0x20
#define ICM40609_REG_ACCEL_DATA_Y1      0x21
#define ICM40609_REG_ACCEL_DATA_Y0      0x22
#define ICM40609_REG_ACCEL_DATA_Z1      0x23
#define ICM40609_REG_ACCEL_DATA_Z0      0x24
#define ICM40609_REG_GYRO_DATA_X1       0x25
#define ICM40609_REG_GYRO_DATA_X0       0x26
#define ICM40609_REG_GYRO_DATA_Y1       0x27
#define ICM40609_REG_GYRO_DATA_Y0       0x28
#define ICM40609_REG_GYRO_DATA_Z1       0x29
#define ICM40609_REG_GYRO_DATA_Z0       0x2A

#define ICM40609_REG_INT_STATUS         0x2D
#define ICM40609_REG_FIFO_COUNTH        0x2E
#define ICM40609_REG_FIFO_COUNTL        0x2F
#define ICM40609_REG_FIFO_DATA          0x30

#define ICM40609_REG_INT_STATUS2        0x37
#define ICM40609_REG_SIGNAL_PATH_RESET  0x4B
#define ICM40609_REG_INTF_CONFIG0       0x4C
#define ICM40609_REG_INTF_CONFIG1       0x4D
#define ICM40609_REG_PWR_MGMT0         0x4E
#define ICM40609_REG_GYRO_CONFIG0       0x4F
#define ICM40609_REG_ACCEL_CONFIG0      0x50
#define ICM40609_REG_GYRO_CONFIG1       0x51
#define ICM40609_REG_GYRO_ACCEL_CONFIG0 0x52
#define ICM40609_REG_ACCEL_CONFIG1      0x53
#define ICM40609_REG_TMST_CONFIG        0x54
#define ICM40609_REG_WOM_CONFIG         0x57

#define ICM40609_REG_FIFO_CONFIG1       0x5F
#define ICM40609_REG_FIFO_CONFIG2       0x60
#define ICM40609_REG_FIFO_CONFIG3       0x61

#define ICM40609_REG_FSYNC_CONFIG       0x62
#define ICM40609_REG_INT_CONFIG0        0x63
#define ICM40609_REG_INT_CONFIG1        0x64
#define ICM40609_REG_INT_SOURCE0        0x65
#define ICM40609_REG_INT_SOURCE1        0x66
#define ICM40609_REG_INT_SOURCE3        0x68
#define ICM40609_REG_INT_SOURCE4        0x69

#define ICM40609_REG_SELF_TEST_CONFIG   0x70
#define ICM40609_REG_WHO_AM_I           0x75
#define ICM40609_REG_REG_BANK_SEL       0x76

/* -------------------------------------------------------------------------- */
/* Register map — User Bank 1 (selected registers)                            */
/* -------------------------------------------------------------------------- */
#define ICM40609_B1_REG_SENSOR_CONFIG0      0x03
#define ICM40609_B1_REG_GYRO_CONFIG_STATIC2 0x0B
#define ICM40609_B1_REG_INTF_CONFIG4        0x7A
#define ICM40609_B1_REG_INTF_CONFIG5        0x7B

/* -------------------------------------------------------------------------- */
/* Constants                                                                   */
/* -------------------------------------------------------------------------- */
#define ICM40609_WHO_AM_I_VALUE     0x3B

#define ICM40609_SAMPLE_Q_SIZE      32

/* INTF_CONFIG0 (0x4C). Per DS-000330 §13.27:
 *   bit 7: FIFO_HOLD_LAST_DATA_EN     (0 = empty on read out, default)
 *   bit 6: FIFO_COUNT_REC             (0 = bytes, 1 = records)
 *   bit 5: FIFO_COUNT_ENDIAN          (1 = big-endian)
 *   bit 4: SENSOR_DATA_ENDIAN         (1 = big-endian)
 *   bits 3:2: Reserved
 *   bits 1:0: UI_SIFS_CFG
 *
 * The reset/default value 0x30 sets bits 5 and 4 (big-endian for both FIFO
 * count and sensor data) and leaves FIFO_COUNT_REC=0 (byte count). All
 * parsers in this driver depend on those three bits. */
#define ICM40609_INTF_CONFIG0_SENSOR_BIG_ENDIAN (1u << 4)
#define ICM40609_INTF_CONFIG0_COUNT_BIG_ENDIAN  (1u << 5)
#define ICM40609_INTF_CONFIG0_COUNT_RECORDS     (1u << 6)
#define ICM40609_INTF_CONFIG0_DEFAULT           0x30

/* INT_CONFIG1 (0x64). Per DS-000330 §13.42:
 *   bit 6: INT_TPULSE_DURATION    (0 = 100us for ODR<4kHz, default; 1 = 8us)
 *   bit 5: INT_TDEASSERT_DISABLE  (0 = 100us min de-assert; 1 = disabled;
 *                                  required for ODR>=4kHz)
 *   bit 4: INT_ASYNC_RESET        (MUST be cleared from default 1 to 0 for
 *                                  proper INT pin operation — InvenSense
 *                                  erratum shared across ICM-426xx/40609)
 *   bits 3:0: Reserved
 *
 * ICM40609_INT_CONFIG1_DEFAULT (0x00) clears INT_ASYNC_RESET and leaves the
 * pulse/de-assert fields at their use-case defaults; correct for ODR<4kHz.
 * For ODR>=4kHz use ICM40609_INT_CONFIG1_HIGH_ODR (0x60). */
#define ICM40609_INT_CONFIG1_DEFAULT            0x00
#define ICM40609_INT_CONFIG1_HIGH_ODR           0x60

/* INT_STATUS register bit masks */
#define ICM40609_INT_STATUS_DATA_RDY    (1u << 3)
#define ICM40609_INT_STATUS_FIFO_THS    (1u << 2)
#define ICM40609_INT_STATUS_FIFO_FULL   (1u << 1)

/* FIFO header bit masks */
#define ICM40609_FIFO_HEADER_EMPTY      (1u << 7)
#define ICM40609_FIFO_HEADER_ACCEL      (1u << 6)
#define ICM40609_FIFO_HEADER_GYRO       (1u << 5)
#define ICM40609_FIFO_HEADER_TMST_FSYNC_MASK  0x0C
#define ICM40609_FIFO_HEADER_ODR_ACCEL  (1u << 1)
#define ICM40609_FIFO_HEADER_ODR_GYRO   (1u << 0)

/* Burst read: TEMP(2) + ACCEL(6) + GYRO(6) = 14 data bytes */
#define ICM40609_BURST_READ_LEN         14

/* -------------------------------------------------------------------------- */
/* Enums                                                                       */
/* -------------------------------------------------------------------------- */

/* --- Accelerometer full-scale range (ACCEL_CONFIG0 bits 7:5) --- */
typedef enum {
    ICM40609_ACCEL_FS_32G = 0x00,
    ICM40609_ACCEL_FS_16G = 0x01,
    ICM40609_ACCEL_FS_8G  = 0x02,
    ICM40609_ACCEL_FS_4G  = 0x03
} icm40609_accel_fs_t;

/* --- Gyroscope full-scale range (GYRO_CONFIG0 bits 7:5) --- */
typedef enum {
    ICM40609_GYRO_FS_2000DPS    = 0x00,
    ICM40609_GYRO_FS_1000DPS    = 0x01,
    ICM40609_GYRO_FS_500DPS     = 0x02,
    ICM40609_GYRO_FS_250DPS     = 0x03,
    ICM40609_GYRO_FS_125DPS     = 0x04,
    ICM40609_GYRO_FS_62_5DPS    = 0x05,
    ICM40609_GYRO_FS_31_25DPS   = 0x06,
    ICM40609_GYRO_FS_15_625DPS  = 0x07
} icm40609_gyro_fs_t;

/* --- ODR selection (ACCEL_CONFIG0 / GYRO_CONFIG0 bits 3:0) --- */
typedef enum {
    ICM40609_ODR_32KHZ   = 0x01,
    ICM40609_ODR_16KHZ   = 0x02,
    ICM40609_ODR_8KHZ    = 0x03,
    ICM40609_ODR_4KHZ    = 0x04,
    ICM40609_ODR_2KHZ    = 0x05,
    ICM40609_ODR_1KHZ    = 0x06,
    ICM40609_ODR_200HZ   = 0x07,
    ICM40609_ODR_100HZ   = 0x08,
    ICM40609_ODR_50HZ    = 0x09,
    ICM40609_ODR_25HZ    = 0x0A,
    ICM40609_ODR_12_5HZ  = 0x0B,
    ICM40609_ODR_6_25HZ  = 0x0C,   /* accel LP mode only */
    ICM40609_ODR_3_125HZ = 0x0D,   /* accel LP mode only */
    ICM40609_ODR_1_5625HZ= 0x0E,   /* accel LP mode only */
    ICM40609_ODR_500HZ   = 0x0F
} icm40609_odr_t;

/* --- Accelerometer power mode (PWR_MGMT0 bits 1:0) --- */
typedef enum {
    ICM40609_ACCEL_MODE_OFF = 0x00,
    ICM40609_ACCEL_MODE_LP  = 0x02,
    ICM40609_ACCEL_MODE_LN  = 0x03
} icm40609_accel_mode_t;

/* --- Gyroscope power mode (PWR_MGMT0 bits 3:2) --- */
typedef enum {
    ICM40609_GYRO_MODE_OFF     = 0x00,
    ICM40609_GYRO_MODE_STANDBY = 0x01,
    ICM40609_GYRO_MODE_LN      = 0x03
} icm40609_gyro_mode_t;

/* --- Interrupt polarity (INT_CONFIG bits 0, 3) --- */
typedef enum {
    ICM40609_INT_ACTIVE_LOW  = 0,
    ICM40609_INT_ACTIVE_HIGH = 1
} icm40609_int_polarity_t;

/* --- Interrupt drive circuit (INT_CONFIG bits 1, 4) --- */
typedef enum {
    ICM40609_INT_OPEN_DRAIN = 0,
    ICM40609_INT_PUSH_PULL  = 1
} icm40609_int_drive_t;

/* --- Interrupt mode (INT_CONFIG bits 2, 5) --- */
typedef enum {
    ICM40609_INT_PULSED  = 0,
    ICM40609_INT_LATCHED = 1
} icm40609_int_mode_t;

/* --- FIFO mode (FIFO_CONFIG bits 7:6) --- */
typedef enum {
    ICM40609_FIFO_BYPASS       = 0x00,
    ICM40609_FIFO_STREAM       = 0x01,
    ICM40609_FIFO_STOP_ON_FULL = 0x02
} icm40609_fifo_mode_t;

/* --- UI filter order (GYRO_CONFIG1 bits 3:2, ACCEL_CONFIG1 bits 4:3) --- */
typedef enum {
    ICM40609_FILT_ORD_1ST = 0x00,
    ICM40609_FILT_ORD_2ND = 0x01,
    ICM40609_FILT_ORD_3RD = 0x02
} icm40609_filt_ord_t;

/* --- Register bank selection --- */
typedef enum {
    ICM40609_BANK_0 = 0,
    ICM40609_BANK_1 = 1,
    ICM40609_BANK_2 = 2,
    ICM40609_BANK_4 = 4
} icm40609_bank_t;

/* -------------------------------------------------------------------------- */
/* Device state struct                                                        */
/* -------------------------------------------------------------------------- */
typedef struct {
    uint8_t  accel_fs_sel;
    uint8_t  gyro_fs_sel;
    uint8_t  accel_odr;
    uint8_t  gyro_odr;
    float    accel_scale;       ///< g per LSB
    float    gyro_scale;        ///< dps per LSB
    uint32_t accel_period_us;   ///< Sample period in microseconds (rounded to nearest µs)
    uint32_t gyro_period_us;    ///< Sample period in microseconds (rounded to nearest µs)
} icm40609_t;

/* -------------------------------------------------------------------------- */
/* Configuration struct                                                       */
/* -------------------------------------------------------------------------- */
typedef struct {
    icm40609_accel_fs_t    accel_fs;
    icm40609_odr_t         accel_odr;
    icm40609_gyro_fs_t     gyro_fs;
    icm40609_odr_t         gyro_odr;
    icm40609_accel_mode_t  accel_mode;
    icm40609_gyro_mode_t   gyro_mode;
    /* Interrupt configuration */
    icm40609_int_polarity_t  int1_polarity;
    icm40609_int_drive_t     int1_drive;
    icm40609_int_mode_t      int1_mode;
    bool                     drdy_int1_en;
    /* FIFO configuration */
    bool                     fifo_en;
    bool                     fifo_accel_en;
    bool                     fifo_gyro_en;
    bool                     fifo_temp_en;
    icm40609_fifo_mode_t     fifo_mode;
    uint16_t                 fifo_watermark;
} icm40609_config_t;

#define ICM40609_CONFIG_DEFAULT {                         \
    .accel_fs       = ICM40609_ACCEL_FS_16G,             \
    .accel_odr      = ICM40609_ODR_1KHZ,                 \
    .gyro_fs        = ICM40609_GYRO_FS_2000DPS,          \
    .gyro_odr       = ICM40609_ODR_1KHZ,                 \
    .accel_mode     = ICM40609_ACCEL_MODE_LN,            \
    .gyro_mode      = ICM40609_GYRO_MODE_LN,             \
    .int1_polarity  = ICM40609_INT_ACTIVE_HIGH,          \
    .int1_drive     = ICM40609_INT_PUSH_PULL,            \
    .int1_mode      = ICM40609_INT_PULSED,               \
    .drdy_int1_en   = true,                              \
    .fifo_en        = false,                             \
    .fifo_accel_en  = false,                             \
    .fifo_gyro_en   = false,                             \
    .fifo_temp_en   = false,                             \
    .fifo_mode      = ICM40609_FIFO_BYPASS,              \
    .fifo_watermark = 0                                  \
}

/* -------------------------------------------------------------------------- */
/* Sample struct (combined accel + gyro + temperature)                        */
/* -------------------------------------------------------------------------- */
typedef struct {
    uint64_t t_us;
    int16_t  ax_raw, ay_raw, az_raw;
    int16_t  gx_raw, gy_raw, gz_raw;
    int16_t  temp_raw;
    float    ax, ay, az;   ///< acceleration in m/s^2
    float    gx, gy, gz;   ///< angular rate in rad/s
    float    temp_c;       ///< die temperature in °C
} icm40609_sample_t;

/* -------------------------------------------------------------------------- */
/* Command builder functions (bus-agnostic)                                    */
/* -------------------------------------------------------------------------- */

/**
 * @brief Build command to perform a soft reset.
 * @param[out] tx_buf Buffer for SPI command.
 * @return Number of bytes to send.
 */
size_t icm40609_build_soft_reset(uint8_t *tx_buf);

/**
 * @brief Build command to select a register bank.
 * @param bank Target bank (0, 1, 2, or 4).
 * @param[out] tx_buf Buffer for SPI command.
 * @return Number of bytes to send.
 */
size_t icm40609_build_set_bank(icm40609_bank_t bank, uint8_t *tx_buf);

/**
 * @brief Build a single-register SPI read command.
 * @param reg Register address (7-bit).
 * @param[out] tx_buf Buffer for SPI command.
 * @return Number of bytes in the full SPI transaction (addr + data).
 */
size_t icm40609_build_read_reg(uint8_t reg, uint8_t *tx_buf);

/**
 * @brief Build command to write a single register.
 * @param reg Register address (7-bit).
 * @param value Value to write.
 * @param[out] tx_buf Buffer for SPI command.
 * @return Number of bytes to send.
 */
size_t icm40609_build_write_reg(uint8_t reg, uint8_t value, uint8_t *tx_buf);

/**
 * @brief Build command to configure PWR_MGMT0 (power modes).
 * @param gyro_mode Gyroscope power mode.
 * @param accel_mode Accelerometer power mode.
 * @param temp_dis True to disable temperature sensor.
 * @param[out] tx_buf Buffer for SPI command.
 * @return Number of bytes to send.
 */
size_t icm40609_build_pwr_mgmt0(icm40609_gyro_mode_t gyro_mode,
                                 icm40609_accel_mode_t accel_mode,
                                 bool temp_dis,
                                 uint8_t *tx_buf);

/**
 * @brief Build command to configure gyroscope FSR and ODR.
 * @param fs_sel Full-scale range.
 * @param odr Output data rate.
 * @param[out] tx_buf Buffer for SPI command.
 * @param[in,out] dev Device state updated with new settings.
 * @return Number of bytes to send.
 */
size_t icm40609_build_gyro_config(icm40609_gyro_fs_t fs_sel,
                                   icm40609_odr_t odr,
                                   uint8_t *tx_buf,
                                   icm40609_t *dev);

/**
 * @brief Build command to configure accelerometer FSR and ODR.
 * @param fs_sel Full-scale range.
 * @param odr Output data rate.
 * @param[out] tx_buf Buffer for SPI command.
 * @param[in,out] dev Device state updated with new settings.
 * @return Number of bytes to send.
 */
size_t icm40609_build_accel_config(icm40609_accel_fs_t fs_sel,
                                    icm40609_odr_t odr,
                                    uint8_t *tx_buf,
                                    icm40609_t *dev);

/**
 * @brief Build command to configure INT_CONFIG register (INT1/INT2 electrical).
 * @param int1_polarity INT1 polarity.
 * @param int1_drive INT1 drive circuit.
 * @param int1_mode INT1 mode (pulsed/latched).
 * @param[out] tx_buf Buffer for SPI command.
 * @return Number of bytes to send.
 */
size_t icm40609_build_int_config(icm40609_int_polarity_t int1_polarity,
                                  icm40609_int_drive_t int1_drive,
                                  icm40609_int_mode_t int1_mode,
                                  uint8_t *tx_buf);

/**
 * @brief Build command to configure INT_SOURCE0 (INT1 interrupt sources).
 * @param drdy_en Enable data-ready interrupt on INT1.
 * @param fifo_ths_en Enable FIFO threshold interrupt on INT1.
 * @param fifo_full_en Enable FIFO full interrupt on INT1.
 * @param[out] tx_buf Buffer for SPI command.
 * @return Number of bytes to send.
 */
size_t icm40609_build_int_source0(bool drdy_en,
                                   bool fifo_ths_en,
                                   bool fifo_full_en,
                                   uint8_t *tx_buf);

/**
 * @brief Build command to configure DRIVE_CONFIG (SPI slew rate).
 * @param spi_slew_rate SPI slew rate setting (0-5, see datasheet).
 * @param[out] tx_buf Buffer for SPI command.
 * @return Number of bytes to send.
 */
size_t icm40609_build_drive_config(uint8_t spi_slew_rate, uint8_t *tx_buf);

/**
 * @brief Build command to configure INTF_CONFIG1 (clock selection).
 * @param clksel Clock source selection (0=RC, 1=PLL auto-select).
 * @param[out] tx_buf Buffer for SPI command.
 * @return Number of bytes to send.
 */
size_t icm40609_build_intf_config1(uint8_t clksel, uint8_t *tx_buf);

/**
 * @brief Build command to configure INTF_CONFIG0 (endianness, FIFO format).
 *
 * Writes @p value verbatim. Pass @c ICM40609_INTF_CONFIG0_DEFAULT to keep
 * the documented defaults (big-endian sensor data, big-endian FIFO count,
 * FIFO count in bytes) that the parsers in this driver depend on.
 *
 * Note: this register does NOT control INT pulse duration — that lives in
 * INT_CONFIG1; see icm40609_build_int_config1().
 *
 * @param value Raw register value to write.
 * @param[out] tx_buf Buffer for SPI command.
 * @return Number of bytes to send.
 */
size_t icm40609_build_intf_config0(uint8_t value, uint8_t *tx_buf);

/**
 * @brief Build command to configure INT_CONFIG1 (INT pulse / de-assert / async reset).
 *
 * This register MUST be written during init: per DS-000330 §13.42 the reset
 * value has INT_ASYNC_RESET = 1, which the datasheet explicitly says must
 * be cleared for proper INT pin operation (the InvenSense ICM-426xx/40609
 * "INT lines don't work out of reset" erratum).
 *
 * Pass @c ICM40609_INT_CONFIG1_DEFAULT (0x00) for any ODR < 4 kHz, or
 * @c ICM40609_INT_CONFIG1_HIGH_ODR (0x60) for ODR >= 4 kHz.
 *
 * @param value Raw register value to write.
 * @param[out] tx_buf Buffer for SPI command.
 * @return Number of bytes to send.
 */
size_t icm40609_build_int_config1(uint8_t value, uint8_t *tx_buf);

/**
 * @brief Build burst read command for temp + accel + gyro data registers.
 *
 * Reads 14 bytes starting from TEMP_DATA1 (0x1D) through GYRO_DATA_Z0 (0x2A).
 * Total SPI transaction: 1 address byte + 14 data bytes = 15 bytes.
 *
 * @param[out] tx_buf Buffer for SPI command.
 * @return Number of bytes in the full SPI transaction.
 */
size_t icm40609_build_read_accel_gyro(uint8_t *tx_buf);

/**
 * @brief Build command to read INT_STATUS register.
 * @param[out] tx_buf Buffer for SPI command.
 * @return Number of bytes in the full SPI transaction.
 */
size_t icm40609_build_read_int_status(uint8_t *tx_buf);

/**
 * @brief Build commands to configure FIFO.
 * @param mode FIFO operating mode.
 * @param accel_en Enable accelerometer data in FIFO.
 * @param gyro_en Enable gyroscope data in FIFO.
 * @param temp_en Enable temperature data in FIFO.
 * @param watermark FIFO watermark threshold in bytes.
 * @param[out] tx_buf Buffer for SPI commands (multiple register writes).
 * @return Number of bytes to send.
 */
size_t icm40609_build_fifo_config(icm40609_fifo_mode_t mode,
                                   bool accel_en,
                                   bool gyro_en,
                                   bool temp_en,
                                   uint16_t watermark,
                                   uint8_t *tx_buf);

/**
 * @brief Build command to read FIFO byte count (FIFO_COUNTH + FIFO_COUNTL).
 * @param[out] tx_buf Buffer for SPI command.
 * @return Number of bytes in the full SPI transaction.
 */
size_t icm40609_build_read_fifo_count(uint8_t *tx_buf);

/**
 * @brief Build command to read FIFO data.
 *
 * The SPI transaction length is `1 + byte_count`. The caller must provide
 * a tx buffer large enough; this function will refuse and return 0 if the
 * declared capacity is insufficient (no partial write).
 *
 * Typical caller bound: `spi_job_t.tx` is 16 bytes wide in this project,
 * which limits a single chunk to 15 FIFO bytes. Larger reads should be
 * split into multiple jobs.
 *
 * @param byte_count       Number of FIFO bytes to read.
 * @param[out] tx_buf      Buffer for SPI command.
 * @param tx_buf_capacity  Size of tx_buf in bytes.
 * @return Number of bytes in the full SPI transaction (1 + byte_count), or
 *         0 if @p tx_buf is NULL or @p tx_buf_capacity is too small.
 */
size_t icm40609_build_read_fifo_data(uint16_t byte_count,
                                      uint8_t *tx_buf,
                                      size_t tx_buf_capacity);

/**
 * @brief Build command to flush the FIFO.
 * @param[out] tx_buf Buffer for SPI command.
 * @return Number of bytes to send.
 */
size_t icm40609_build_fifo_flush(uint8_t *tx_buf);

/* -------------------------------------------------------------------------- */
/* Parsing helpers                                                            */
/* -------------------------------------------------------------------------- */

/**
 * @brief Parse burst-read data (14 bytes: temp + accel XYZ + gyro XYZ).
 *
 * Expects rx_buf to point at the first data byte (after the address byte),
 * i.e. rx_buf[0] = TEMP_DATA1, rx_buf[1] = TEMP_DATA0, etc.
 * Data is in big-endian format (default ICM-40609 configuration).
 *
 * @param[in] rx_buf Raw received data (14 bytes).
 * @param[out] sample Parsed sample with raw + converted values.
 * @param[in] dev Device state (for scale factors).
 * @return True if parsing successful.
 */
bool icm40609_parse_accel_gyro(const uint8_t *rx_buf,
                                icm40609_sample_t *sample,
                                const icm40609_t *dev);

/**
 * @brief Parse a single FIFO packet.
 *
 * @param[in] rx_buf Raw FIFO packet data (starts with header byte).
 * @param packet_size Size of this FIFO packet (8 or 16 bytes).
 * @param[out] sample Parsed sample.
 * @param[in] dev Device state (for scale factors).
 * @return True if parsing successful, false if header indicates empty FIFO.
 */
bool icm40609_parse_fifo_packet(const uint8_t *rx_buf,
                                 size_t packet_size,
                                 icm40609_sample_t *sample,
                                 const icm40609_t *dev);

/**
 * @brief Parse multiple FIFO packets from a burst read.
 *
 * @param[in] rx_buf Raw FIFO data buffer.
 * @param len Length of rx_buf in bytes.
 * @param[out] out_samples Array of decoded samples.
 * @param[in,out] max_samples Input: capacity; Output: number filled.
 * @param[in] dev Device state (for scale factors).
 * @return True if at least one packet parsed successfully.
 */
bool icm40609_parse_fifo(const uint8_t *rx_buf,
                          size_t len,
                          icm40609_sample_t *out_samples,
                          size_t *max_samples,
                          const icm40609_t *dev);

/**
 * @brief Parse FIFO count registers.
 *
 * Input byte order matches `icm40609_build_read_fifo_count`, which reads
 * starting from FIFO_COUNTH (auto-increment):
 *   rx_buf[0] = FIFO_COUNTH (high)
 *   rx_buf[1] = FIFO_COUNTL (low)
 *
 * The unit (bytes vs. records) is determined by FIFO_COUNT_REC in
 * INTF_CONFIG0; this driver's default leaves that bit cleared, so the
 * count is in bytes.
 *
 * @param[in] rx_buf Raw received data (2 bytes: COUNTH, COUNTL).
 * @param[out] count FIFO byte count.
 * @return True if parsing successful.
 */
bool icm40609_parse_fifo_count(const uint8_t *rx_buf, uint16_t *count);

/* -------------------------------------------------------------------------- */
/* SPSC ring buffer for IMU samples                                           */
/* -------------------------------------------------------------------------- */

typedef struct {
    icm40609_sample_t samples[ICM40609_SAMPLE_Q_SIZE];
    volatile uint8_t  head;
    volatile uint8_t  tail;
    volatile uint32_t dropped;  ///< Count of enqueue calls that hit a full queue
} icm40609_sample_queue_t;

static inline bool icm40609_sample_queue_empty(icm40609_sample_queue_t *q) {
    return q->head == q->tail;
}

static inline bool icm40609_sample_queue_full(icm40609_sample_queue_t *q) {
    return ((q->head + 1) % ICM40609_SAMPLE_Q_SIZE) == q->tail;
}

static inline bool icm40609_sample_enqueue(icm40609_sample_queue_t *q,
                                            const icm40609_sample_t *sample) {
    if (icm40609_sample_queue_full(q)) {
        q->dropped++;
        return false;
    }
    q->samples[q->head] = *sample;
    SENSORS_MEMORY_BARRIER();
    q->head = (q->head + 1) % ICM40609_SAMPLE_Q_SIZE;
    return true;
}

static inline bool icm40609_sample_dequeue(icm40609_sample_queue_t *q,
                                            icm40609_sample_t *sample) {
    if (icm40609_sample_queue_empty(q)) return false;
    SENSORS_MEMORY_BARRIER();
    *sample = q->samples[q->tail];
    SENSORS_MEMORY_BARRIER();
    q->tail = (q->tail + 1) % ICM40609_SAMPLE_Q_SIZE;
    return true;
}

#endif /* ICM40609_H */
