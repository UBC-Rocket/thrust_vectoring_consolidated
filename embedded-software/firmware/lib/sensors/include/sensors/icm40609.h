/**
 * @file icm40609.h
 * @brief Bus-agnostic driver for the InvenSense ICM-40609-D IMU.
 *
 * Mirrors the structure of the BMI088 driver: command builders write SPI
 * frames into a caller-supplied buffer; parsers decode responses into typed
 * samples. The IO layer owns the actual SPI transport.
 *
 * UBC Rocket, 2026
 */

#ifndef ICM40609_H
#define ICM40609_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "sensors/spsc_queue.h"

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* Register map — Bank 0 (unless explicitly noted)                            */
/* -------------------------------------------------------------------------- */
#define ICM40609_DEVICE_CONFIG          0x11   ///< soft reset, SPI mode
#define ICM40609_DRIVE_CONFIG           0x13
#define ICM40609_INT_CONFIG             0x14
#define ICM40609_FIFO_CONFIG            0x16
#define ICM40609_TEMP_DATA_1            0x1D   ///< TEMP_DATA_1, TEMP_DATA_0
#define ICM40609_ACCEL_DATA_X1          0x1F   ///< ACCEL_DATA_X1 .. ACCEL_DATA_Z0 (6 bytes)
#define ICM40609_GYRO_DATA_X1           0x25   ///< GYRO_DATA_X1  .. GYRO_DATA_Z0  (6 bytes)
#define ICM40609_TMST_FSYNCH            0x2B
#define ICM40609_INT_STATUS             0x2D
#define ICM40609_FIFO_COUNTH            0x2E
#define ICM40609_FIFO_DATA              0x30
#define ICM40609_INTF_CONFIG0           0x4C
#define ICM40609_INTF_CONFIG1           0x4D
#define ICM40609_PWR_MGMT0              0x4E
#define ICM40609_GYRO_CONFIG0           0x4F
#define ICM40609_ACCEL_CONFIG0          0x50
#define ICM40609_GYRO_CONFIG1           0x51
#define ICM40609_GYRO_ACCEL_CONFIG0     0x52
#define ICM40609_ACCEL_CONFIG1          0x53
#define ICM40609_TMST_CONFIG            0x54
#define ICM40609_INT_CONFIG0            0x63
#define ICM40609_INT_CONFIG1            0x64
#define ICM40609_INT_SOURCE0            0x65
#define ICM40609_INT_SOURCE1            0x66
#define ICM40609_INT_SOURCE3            0x68   ///< INT2 source enables (UI_DRDY_INT2_EN = bit3)
#define ICM40609_WHO_AM_I               0x75
#define ICM40609_REG_BANK_SEL           0x76

#define ICM40609_WHO_AM_I_VAL           0x3B   ///< expected WHO_AM_I value (DS-000330 §13.66)

#define ICM40609_SAMPLE_Q_SIZE          15

/* -------------------------------------------------------------------------- */
/* Init-time register values (datasheet-derived)                              */
/* -------------------------------------------------------------------------- */
/* INTF_CONFIG0: big-endian sensor data + byte-mode FIFO count. The parsers in
 * this file assume big-endian (be16), so we lock that assumption at bring-up
 * rather than trust the reset default. */
#define ICM40609_INTF_CONFIG0_BE        0x30
/* INT_CONFIG1 (0x64): bit4 INT_ASYNC_RESET must be cleared from its reset
 * value of 1 for proper INT1 operation (DS-000330 §13.42). 0x00 for ODR<4kHz;
 * 0x60 enables the 8µs pulse / de-assert-disabled path required at ODR>=4kHz. */
#define ICM40609_INT_CONFIG1_DEFAULT    0x00
#define ICM40609_INT_CONFIG1_HIGH_ODR   0x60
/* DRIVE_CONFIG (0x13): SPI slew-rate setting (DS-000330 §14.4). */
#define ICM40609_DRIVE_CONFIG_SLEW      0x05
/* INT_STATUS (0x2D): DATA_RDY_INT is bit3. */
#define ICM40609_INT_STATUS_DATA_RDY    0x08

/* -------------------------------------------------------------------------- */
/* Device state                                                               */
/* -------------------------------------------------------------------------- */
typedef struct {
    uint8_t pwr_mgmt0;
    uint8_t gyro_config0;
    uint8_t accel_config0;
    float   accel_scale_mps2_lsb;   ///< m/s² per LSB
    float   gyro_scale_rads_lsb;    ///< rad/s per LSB
    uint16_t odr_hz;
} icm40609_t;

/* -------------------------------------------------------------------------- */
/* Configuration enums                                                        */
/* -------------------------------------------------------------------------- */
typedef enum {
    ICM40609_ACCEL_FS_32G = 0x00,
    ICM40609_ACCEL_FS_16G = 0x01,
    ICM40609_ACCEL_FS_8G  = 0x02,
    ICM40609_ACCEL_FS_4G  = 0x03,
} icm40609_accel_fs_t;

/* GYRO_FS_SEL encoding for the ICM-40609-D (max ±2000 dps; DS-000330 §13.48).
 * Note: this part does NOT support ±4000 dps — that belongs to the ICM-42688. */
typedef enum {
    ICM40609_GYRO_FS_2000DPS   = 0x00,   ///< default
    ICM40609_GYRO_FS_1000DPS   = 0x01,
    ICM40609_GYRO_FS_500DPS    = 0x02,
    ICM40609_GYRO_FS_250DPS    = 0x03,
    ICM40609_GYRO_FS_125DPS    = 0x04,
    ICM40609_GYRO_FS_62_5DPS   = 0x05,
    ICM40609_GYRO_FS_31_25DPS  = 0x06,
    ICM40609_GYRO_FS_15_625DPS = 0x07,
} icm40609_gyro_fs_t;

/**
 * @brief Output data rate codes (apply to GYRO_CONFIG0 / ACCEL_CONFIG0).
 */
typedef enum {
    ICM40609_ODR_8000_HZ  = 0x03,
    ICM40609_ODR_4000_HZ  = 0x04,
    ICM40609_ODR_2000_HZ  = 0x05,
    ICM40609_ODR_1000_HZ  = 0x06,
    ICM40609_ODR_200_HZ   = 0x07,
    ICM40609_ODR_100_HZ   = 0x08,
    ICM40609_ODR_50_HZ    = 0x09,
    ICM40609_ODR_25_HZ    = 0x0A,
    ICM40609_ODR_12_5_HZ  = 0x0B,
} icm40609_odr_t;

typedef enum {
    ICM40609_PWR_OFF            = 0x00,  ///< both gyro+accel off
    ICM40609_PWR_LOW_NOISE      = 0x0F,  ///< gyro LN + accel LN
    ICM40609_PWR_ACCEL_ONLY_LN  = 0x03,
    ICM40609_PWR_GYRO_ONLY_LN   = 0x0C,
} icm40609_power_t;

/* -------------------------------------------------------------------------- */
/* Command builders                                                           */
/* -------------------------------------------------------------------------- */

/**
 * @brief Build a soft-reset command (writes 0x01 to DEVICE_CONFIG).
 * Datasheet specifies a ≥1 ms wait after the write before the part responds.
 */
size_t icm40609_build_softreset(uint8_t *tx_buf);

/**
 * @brief Build a bank-select write.
 */
size_t icm40609_build_bank_select(uint8_t bank, uint8_t *tx_buf);

/**
 * @brief Build a power-mode write (updates PWR_MGMT0).
 */
size_t icm40609_build_power(icm40609_power_t pwr, uint8_t *tx_buf, icm40609_t *dev);

/**
 * @brief Configure the accelerometer full-scale range + ODR (ACCEL_CONFIG0).
 */
size_t icm40609_build_accel_conf(icm40609_accel_fs_t fs,
                                 icm40609_odr_t odr,
                                 uint8_t *tx_buf,
                                 icm40609_t *dev);

/**
 * @brief Configure the gyro full-scale range + ODR (GYRO_CONFIG0).
 */
size_t icm40609_build_gyro_conf(icm40609_gyro_fs_t fs,
                                icm40609_odr_t odr,
                                uint8_t *tx_buf,
                                icm40609_t *dev);

/**
 * @brief Route the DATA_READY interrupt to INT1 (push-pull, active-high).
 */
size_t icm40609_build_int1_data_ready(uint8_t *tx_buf);

typedef enum {
    ICM40609_READ_WHO_AM_I,
    ICM40609_READ_INT_STATUS,
    ICM40609_READ_TEMP,
    ICM40609_READ_ACCEL_XYZ,
    ICM40609_READ_GYRO_XYZ,
    ICM40609_READ_ACCEL_GYRO_XYZ, ///< 12 contiguous bytes from ACCEL_DATA_X1
    ICM40609_READ_FIFO_COUNT,
    ICM40609_READ_FIFO_DATA,
} icm40609_read_t;

/**
 * @brief Build a read command; returns total TX length (cmd byte + dummy bytes
 *        to clock back the response).
 */
size_t icm40609_build_read(icm40609_read_t what, uint8_t *tx_buf);

size_t icm40609_build_read_reg(uint8_t reg, uint8_t *tx_buf);

/**
 * @brief Build a single-register write (reg, value). Returns 2.
 *        Generic escape hatch for bring-up registers without a dedicated
 *        builder (DRIVE_CONFIG, INTF_CONFIG0, INT_CONFIG1, ...).
 */
size_t icm40609_build_write_reg(uint8_t reg, uint8_t value, uint8_t *tx_buf);

/* -------------------------------------------------------------------------- */
/* Parsing                                                                    */
/* -------------------------------------------------------------------------- */
typedef struct {
    uint64_t t_us;
    int16_t  ax_raw, ay_raw, az_raw;
    int16_t  gx_raw, gy_raw, gz_raw;
    float    ax, ay, az;   ///< m/s²
    float    gx, gy, gz;   ///< rad/s
    float    temp_c;
} icm40609_sample_t;

/**
 * @brief Parse ACCEL+GYRO big-endian 12-byte block (ACCEL_X1..GYRO_Z0).
 */
bool icm40609_parse_accel_gyro(const uint8_t *rx_buf,
                               icm40609_sample_t *sample,
                               const icm40609_t *dev);

bool icm40609_parse_accel(const uint8_t *rx_buf,
                          icm40609_sample_t *sample,
                          const icm40609_t *dev);

bool icm40609_parse_gyro(const uint8_t *rx_buf,
                         icm40609_sample_t *sample,
                         const icm40609_t *dev);

bool icm40609_parse_temp(const uint8_t *rx_buf, float *temp_c);

bool icm40609_parse_who_am_i(const uint8_t *rx_buf, uint8_t *who);

/* -------------------------------------------------------------------------- */
/* Ring queue (SPSC)                                                          */
/* -------------------------------------------------------------------------- */
typedef struct {
    icm40609_sample_t samples[ICM40609_SAMPLE_Q_SIZE];
    volatile uint8_t  head;
    volatile uint8_t  tail;
} icm40609_sample_queue_t;

static inline bool icm40609_queue_empty(icm40609_sample_queue_t *q) {
    return q->head == q->tail;
}

static inline bool icm40609_queue_full(icm40609_sample_queue_t *q) {
    return ((q->head + 1) % ICM40609_SAMPLE_Q_SIZE) == q->tail;
}

static inline bool icm40609_queue(icm40609_sample_queue_t *q,
                                  const icm40609_sample_t *s) {
    if (icm40609_queue_full(q)) return false;
    q->samples[q->head] = *s;
    SENSORS_MEMORY_BARRIER();
    q->head = (q->head + 1) % ICM40609_SAMPLE_Q_SIZE;
    return true;
}

static inline bool icm40609_dequeue(icm40609_sample_queue_t *q,
                                    icm40609_sample_t *s) {
    if (icm40609_queue_empty(q)) return false;
    SENSORS_MEMORY_BARRIER();
    *s = q->samples[q->tail];
    SENSORS_MEMORY_BARRIER();
    q->tail = (q->tail + 1) % ICM40609_SAMPLE_Q_SIZE;
    return true;
}

#ifdef __cplusplus
}
#endif

#endif /* ICM40609_H */
