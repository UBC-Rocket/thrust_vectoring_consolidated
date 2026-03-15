/*
    driver for the BMI088 IMU accelerometer. this driver is only used to parse, and create packages.
    it is completley bus agnostic.

    the core functionalities of the device are implemented, as required by the TVR team,
    but some more specific functionalities are not.

    @ UBC Rocket, Sept 29th 2025, Benedikt Howard
*/

#ifndef BMI088_ACCEL_H
#define BMI088_ACCEL_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <collections/spsc_ring.h>

/* -------------------------------------------------------------------------- */
/* Register map (accelerometer side of BMI088)                                */
/* -------------------------------------------------------------------------- */
#define BMI088_ACC_CHIP_ID_REG       0x00
#define BMI088_ACC_ERR_REG           0x02
#define BMI088_ACC_STATUS_REG        0x03
#define BMI088_ACC_DATA_START        0x12  // X_LSB..Z_MSB
#define BMI088_ACC_SENSORTIME_0      0x18
#define BMI088_ACC_SENSORTIME_1      0x19
#define BMI088_ACC_SENSORTIME_2      0x1A
#define BMI088_ACC_INT_STAT_1        0x1D
#define BMI088_ACC_TEMP_MSB          0x22
#define BMI088_ACC_CONF              0x40
#define BMI088_ACC_RANGE             0x41
#define BMI088_ACC_FIFO_LENGTH_0     0x24
#define BMI088_ACC_FIFO_LENGTH_1     0x25
#define BMI088_ACC_FIFO_DATA         0x26
#define BMI088_ACC_FIFO_CONFIG_0     0x48
#define BMI088_ACC_FIFO_CONFIG_1     0x49
#define BMI088_ACC_FIFO_WTM_0        0x46
#define BMI088_ACC_FIFO_WTM_1        0x47
#define BMI088_ACC_FIFO_DOWNS        0x4A
#define BMI088_ACC_FIFO_FILTER       0x4B
#define BMI088_ACC_INT1_IO_CONF      0x53
#define BMI088_ACC_INT2_IO_CONF      0x54
#define BMI088_ACC_INT1_INT2_MAP     0x58
#define BMI088_ACC_SELF_TEST         0x6D
#define BMI088_ACC_PWR_CONF          0x7C
#define BMI088_ACC_PWR_CTRL          0x7D
#define BMI088_ACC_SOFTRESET         0x7E

#define BMI088_ACC_SAMPLE_Q_SIZE     15

/* -------------------------------------------------------------------------- */
/* Device state struct                                                        */
/* -------------------------------------------------------------------------- */
typedef struct {
    uint8_t range_reg;     ///< last written value for ACC_RANGE
    uint8_t conf_reg;      ///< last written value for ACC_CONF (ODR/bw)
    uint8_t pwr_conf_reg;  ///< last written ACC_PWR_CONF
    uint8_t pwr_ctrl_reg;  ///< last written ACC_PWR_CTRL
    uint8_t fifo_conf0;    ///< last written FIFO_CONFIG_0
    uint8_t fifo_conf1;    ///< last written FIFO_CONFIG_1

    float scale_mg_lsb;    ///< mg/LSB scale factor (depends on range)
    uint16_t odr_hz;       ///< effective output data rate in Hz
} bmi088_accel_t;

/* -------------------------------------------------------------------------- */
/* Function headers and their enums grouped by purpose                        */
/* -------------------------------------------------------------------------- */

/* --- Soft reset ----------------------------------------------------------- */
/**
 * @brief Build command to soft-reset the accelerometer.
 * @param[out] tx_buf Buffer where SPI command is written.
 * @return Number of bytes to send (always 2).
 */
size_t bmi088_accel_build_softreset(uint8_t *tx_buf);

/* --- Power configuration -------------------------------------------------- */
typedef enum {
    BMI088_ACC_PWR_ACTIVE,   ///< Normal measurement mode
    BMI088_ACC_PWR_SUSPEND   ///< Suspend mode (lowest power)
} bmi088_accel_power_t;

/**
 * @brief Build command to set power mode of accelerometer.
 * @param pwr Desired power mode.
 * @param[out] tx_buf Buffer where SPI command is written.
 * @param[in,out] dev Device state struct updated with new mode.
 * @return Number of bytes to send.
 */
size_t bmi088_accel_build_power_conf(bmi088_accel_power_t pwr, uint8_t *tx_buf,
                                     bmi088_accel_t *dev);

/* --- Range configuration -------------------------------------------------- */
typedef enum {
    BMI088_ACC_RANGE_3G  = 0x00, ///< ±3g
    BMI088_ACC_RANGE_6G  = 0x01, ///< ±6g
    BMI088_ACC_RANGE_12G = 0x02, ///< ±12g
    BMI088_ACC_RANGE_24G = 0x03  ///< ±24g
} bmi088_accel_range_t;

/**
 * @brief Build command to configure measurement range.
 * @param range One of the supported g-ranges.
 * @param[out] tx_buf Buffer where SPI command is written.
 * @param[in,out] dev Device state struct updated with new range & scale factor.
 * @return Number of bytes to send.
 */
size_t bmi088_accel_build_range(bmi088_accel_range_t range, uint8_t *tx_buf,
                                bmi088_accel_t *dev);

/* --- ODR / bandwidth configuration --------------------------------------- */
typedef enum {
    BMI088_ACC_ODR_12_5_HZ = 0x05,
    BMI088_ACC_ODR_25_HZ   = 0x06,
    BMI088_ACC_ODR_50_HZ   = 0x07,
    BMI088_ACC_ODR_100_HZ  = 0x08,
    BMI088_ACC_ODR_200_HZ  = 0x09,
    BMI088_ACC_ODR_400_HZ  = 0x0A,
    BMI088_ACC_ODR_800_HZ  = 0x0B,
    BMI088_ACC_ODR_1600_HZ = 0x0C
} bmi088_accel_odr_t;

/**
 * @brief Accelerometer bandwidth / performance filter settings.
 *
 * These map directly to acc_bwp field in ACC_CONF register (bits [7:4]).
 * See datasheet, Table 20.
 */
typedef enum {
    BMI088_ACC_BWP_OSR4   = 0x00,  ///< Oversampling rate 4, highest bandwidth, highest noise
    BMI088_ACC_BWP_OSR2   = 0x01,  ///< Oversampling rate 2
    BMI088_ACC_BWP_NORMAL = 0x0A   ///< Normal mode (recommended tradeoff)
    // Note: Datasheet defines more codes (0x02..0x09, 0x0B..0x0F),
    // but these three are the most common. Extend as needed.
} bmi088_accel_bwp_t;

/**
 * @brief Build command to configure output data rate (ODR) and bandwidth.
 * @param odr   Desired output data rate (lower 4 bits).
 * @param bwp   Bandwidth / performance filter setting (upper 4 bits).
 * @param[out] tx_buf Buffer where SPI command is written.
 * @param[in,out] dev Device state struct updated with ODR and register value.
 * @return Number of bytes to send.
 */
size_t bmi088_accel_build_odr_bw(bmi088_accel_odr_t odr,
                                 bmi088_accel_bwp_t bwp,
                                 uint8_t *tx_buf,
                                 bmi088_accel_t *dev);

/* --- Interrupt pin configuration ------------------------------------------ */
typedef enum {
    BMI088_ACC_INT1 = 0,
    BMI088_ACC_INT2 = 1
} bmi088_accel_int_pin_t;

typedef enum {
    BMI088_ACC_INT_PUSH_PULL = 0,
    BMI088_ACC_INT_OPEN_DRAIN = 1
} bmi088_accel_int_drive_t;

typedef enum {
    BMI088_ACC_INT_ACTIVE_LOW = 0,
    BMI088_ACC_INT_ACTIVE_HIGH = 1
} bmi088_accel_int_polarity_t;

typedef enum {
    BMI088_ACC_INT_EVENT_NONE        = 0x00,
    BMI088_ACC_INT_EVENT_DATA_READY  = 0x01,
    BMI088_ACC_INT_EVENT_FIFO_WTM    = 0x02,
    BMI088_ACC_INT_EVENT_FIFO_FULL   = 0x04
    // others (orientation, motion) can be added later
} bmi088_accel_int_event_t;

/**
 * @brief Build command to configure an interrupt pin (INT1/INT2).
 * @param pin Which interrupt pin to configure.
 * @param drive Push-pull or open-drain.
 * @param polarity Active high or low.
 * @param latch If true, interrupt latches until cleared; else pulses.
 * @param[out] tx_buf Buffer where SPI command is written.
 * @return Number of bytes to send.
 */
/**
 * @brief Build command to configure the electrical behavior of an interrupt pin.
 *        This sets polarity, drive mode, and latching, but not which events appear.
 */
size_t bmi088_accel_build_int_pin_conf(bmi088_accel_int_pin_t pin,
                                       bmi088_accel_int_drive_t drive,
                                       bmi088_accel_int_polarity_t polarity,
                                       bool latch,
                                       uint8_t *tx_buf);

/**
 * @brief Build command to map specific interrupt events to an interrupt pin.
 * @param pin    Which pin (INT1 or INT2).
 * @param events Bitwise OR of bmi088_accel_int_event_t.
 * @param[out] tx_buf Buffer where SPI command is written.
 * @return Number of bytes to send.
 */
size_t bmi088_accel_build_int_event_map(bmi088_accel_int_pin_t pin,
                                        uint8_t events,
                                        uint8_t *tx_buf);

/* --- FIFO configuration --------------------------------------------------- */
typedef enum {
    BMI088_ACC_FIFO_BYPASS = 0x00,
    BMI088_ACC_FIFO_FIFO   = 0x01,
    BMI088_ACC_FIFO_STREAM = 0x02
} bmi088_accel_fifo_mode_t;

typedef enum {
    BMI088_ACC_FIFO_NONE      = 0x00,
    BMI088_ACC_FIFO_XYZ       = 0x01, ///< store accel XYZ data
    BMI088_ACC_FIFO_SENSORTIME= 0x02  ///< store sensor time
} bmi088_accel_fifo_source_t;

/**
 * @brief Build commands to configure FIFO behavior.
 * @param mode FIFO mode: bypass, FIFO, or stream.
 * @param src Which data to store in FIFO (XYZ, sensortime, both).
 * @param watermark Watermark level in samples (1..1024).
 * @param[out] tx_buf Buffer where SPI command(s) are written.
 * @return Number of bytes to send.
 */
size_t bmi088_accel_build_fifo_conf(bmi088_accel_fifo_mode_t mode,
                                    bmi088_accel_fifo_source_t src,
                                    uint16_t watermark,
                                    uint8_t *tx_buf);

/* --- Self-test ------------------------------------------------------------ */
/**
 * @brief Build command to trigger self-test.
 * @param axis_mask Which axes to test (bitfield).
 * @param enable True to enable self-test, false to disable.
 * @param[out] tx_buf Buffer where SPI command is written.
 * @return Number of bytes to send.
 */
size_t bmi088_accel_build_selftest(uint8_t axis_mask, bool enable,
                                   uint8_t *tx_buf);

/*---Read-command------------------------------------------------------------*/
typedef enum {
    BMI088_ACC_READ_INT_STATUS,
    BMI088_ACC_READ_DATA_XYZ,
    BMI088_ACC_READ_TEMP,
    BMI088_ACC_READ_SENSORTIME,
    BMI088_ACC_READ_FIFO_STATUS,
    BMI088_ACC_READ_FIFO_DATA
} bmi088_accel_read_t;


/**
 * @brief Build an SPI read command for a given sensor read type.
 * @param what Which data block to read.
 * @param[out] tx_buf Buffer for the command (first byte = reg addr | 0x80).
 * @return Number of bytes to transmit (address + dummy for rx).
 */
size_t bmi088_accel_build_read(bmi088_accel_read_t what, uint8_t *tx_buf);

size_t bmi088_accel_build_read_reg(uint8_t reg, uint8_t *tx_buf);

/* -------------------------------------------------------------------------- */
/* Parsing helpers                                                            */
/* -------------------------------------------------------------------------- */

typedef struct {
    uint64_t t_us;   ///< timestamp if sensortime was enabled
    int16_t x, y, z; ///< raw accelerometer counts
    float ax, ay, az;///< converted values in m/s^2
} bmi088_accel_sample_t;

/**
 * @brief Parse raw XYZ acceleration data (6 bytes).
 * @param[in] rx_buf Raw buffer read from device.
 * @param[out] sample sample decoded into raw signed, m/s^2.
 * @param[in] dev Device state (for mg/LSB scale factor).
 * @return True if parsing successful, false otherwise.
 */
bool bmi088_accel_parse_data_xyz(const uint8_t *rx_buf,
                                 bmi088_accel_sample_t *sample,
                                 const bmi088_accel_t *dev);

/**
 * @brief Parse raw temperature data (2 bytes).
 * @param[in] rx_buf Raw buffer from device.
 * @param[out] temp_c Temperature in °C.
 * @return True if parsing successful.
 */
bool bmi088_accel_parse_temp(const uint8_t *rx_buf, float *temp_c);

/**
 * @brief Parse sensor time counter (3 bytes).
 * @param[in] rx_buf Raw buffer from device.
 * @param[out] sensortime_us Sensor time in microseconds.
 * @return True if parsing successful.
 */
bool bmi088_accel_parse_sensortime(const uint8_t *rx_buf, uint32_t *sensortime_us);

/**
 * @brief Parse FIFO length registers (2 bytes).
 * @param[in] rx_buf Raw buffer.
 * @param[out] fifo_bytes Current FIFO fill level in bytes.
 * @return True if parsing successful.
 */
bool bmi088_accel_parse_fifo_length(const uint8_t *rx_buf, uint16_t *fifo_bytes);

/* --- FIFO parsing is more complex; provide a dedicated API ----------------- */

/**
 * @brief Parse FIFO data frames into samples.
 * @param[in] rx_buf Raw FIFO data read from device.
 * @param[in] len Length of rx_buf in bytes.
 * @param[out] out_samples Array of decoded samples.
 * @param[in,out] max_samples Input: capacity of out_samples, Output: number filled.
 * @param[in] dev Device state for scaling.
 * @return True if at least one frame parsed successfully.
 */
bool bmi088_accel_parse_fifo(const uint8_t *rx_buf, size_t len,
                             bmi088_accel_sample_t *out_samples,
                             size_t *max_samples,
                             const bmi088_accel_t *dev);

/* --- Ring buffer for accelerometer samples -------------------------------- */
SPSC_RING_DECLARE(bmi088_acc_sample_q, bmi088_accel_sample_t, BMI088_ACC_SAMPLE_Q_SIZE)

#endif /* BMI088_ACCEL_H */
