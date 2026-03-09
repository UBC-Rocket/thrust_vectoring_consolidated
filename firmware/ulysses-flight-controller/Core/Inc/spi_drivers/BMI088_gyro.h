/*
    Driver for the BMI088 gyroscope. This driver is only used to parse and create
    register access packages. It is completely bus-agnostic.

    The core functionalities of the device are implemented, as required by the TVR team,
    but some more specific functionalities are not.

    @ UBC Rocket, Sept 29th 2025, Benedikt Howard
*/

#ifndef BMI088_GYRO_H
#define BMI088_GYRO_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "sync.h"

/* -------------------------------------------------------------------------- */
/* Register map (gyroscope side of BMI088)                                    */
/* -------------------------------------------------------------------------- */
// memory barrier (:
#define BMI088_GYRO_CHIP_ID_REG         0x00
#define BMI088_GYRO_CHIP_ID_VAL         0x0F  ///< Expected chip ID

#define BMI088_GYRO_RATE_X_LSB          0x02
#define BMI088_GYRO_RATE_X_MSB          0x03
#define BMI088_GYRO_RATE_Y_LSB          0x04
#define BMI088_GYRO_RATE_Y_MSB          0x05
#define BMI088_GYRO_RATE_Z_LSB          0x06
#define BMI088_GYRO_RATE_Z_MSB          0x07

#define BMI088_GYRO_INT_STAT_1          0x0A

#define BMI088_GYRO_RANGE               0x0F
#define BMI088_GYRO_BANDWIDTH           0x10  ///< ODR + bandwidth (CONF)
#define BMI088_GYRO_CONF                0x10  ///< Alias of BANDWIDTH

#define BMI088_GYRO_PWR_CTRL            0x11  ///< Power control (LPM1)
#define BMI088_GYRO_SOFTRESET           0x14  ///< Write 0xB6 to reset

#define BMI088_GYRO_INT_CTRL            0x15  ///< Interrupt enable (DRDY/FIFO)
#define BMI088_GYRO_INT3_INT4_IO_CONF   0x16  ///< INT pin electrical config
#define BMI088_GYRO_INT3_INT4_IO_MAP    0x18  ///< INT routing (DRDY/FIFO -> INT3/4)

#define BMI088_GYRO_FIFO_STATUS         0x0E  ///< FIFO frame counter / flags
#define BMI088_GYRO_FIFO_CONFIG_0       0x3D  ///< FIFO watermark (7-bit). Write clears FIFO.
#define BMI088_GYRO_FIFO_CONFIG_1       0x3E  ///< FIFO mode/config bits
#define BMI088_GYRO_FIFO_DATA           0x3F  ///< FIFO data port (read)

#define BMI088_GYRO_SAMPLE_Q_SIZE       32

/* -------------------------------------------------------------------------- */
/* Device state struct                                                        */
/* -------------------------------------------------------------------------- */
typedef struct {
    uint8_t range_reg;       ///< last written value for GYRO_RANGE
    uint8_t conf_reg;        ///< last written value for GYRO_BANDWIDTH (ODR/bw)
    uint8_t pwr_ctrl_reg;    ///< last written GYRO_PWR_CTRL (LPM1)
    uint8_t fifo_conf0;      ///< last written FIFO_CONFIG_0 (watermark)
    uint8_t fifo_conf1;      ///< last written FIFO_CONFIG_1 (mode)

    float   scale_dps_lsb;   ///< deg/s per LSB (depends on range)
    uint16_t odr_hz;         ///< effective output data rate (for bookkeeping)
} bmi088_gyro_t;

/* -------------------------------------------------------------------------- */
/* Function headers and their enums grouped by purpose                        */
/* -------------------------------------------------------------------------- */

/* --- Soft reset ----------------------------------------------------------- */
/**
 * @brief Build command to soft-reset the gyroscope (write 0xB6 to 0x14).
 */
size_t bmi088_gyro_build_softreset(uint8_t *tx_buf);

/* --- Power configuration -------------------------------------------------- */
typedef enum {
    BMI088_GYRO_PWR_NORMAL,   ///< Normal measurement mode
    BMI088_GYRO_PWR_SUSPEND   ///< Suspend mode
} bmi088_gyro_power_t;

/**
 * @brief Build command to set power mode of gyroscope.
 */
size_t bmi088_gyro_build_power_conf(bmi088_gyro_power_t pwr,
                                    uint8_t *tx_buf,
                                    bmi088_gyro_t *dev);

/* --- Range configuration -------------------------------------------------- */
typedef enum {
    BMI088_GYRO_RANGE_2000DPS = 0x00,
    BMI088_GYRO_RANGE_1000DPS = 0x01,
    BMI088_GYRO_RANGE_500DPS  = 0x02,
    BMI088_GYRO_RANGE_250DPS  = 0x03,
    BMI088_GYRO_RANGE_125DPS  = 0x04
} bmi088_gyro_range_t;

/**
 * @brief Build command to configure measurement range.
 */
size_t bmi088_gyro_build_range(bmi088_gyro_range_t range,
                               uint8_t *tx_buf,
                               bmi088_gyro_t *dev);

/* --- ODR / bandwidth configuration --------------------------------------- */
/* Per datasheet: 
   0x00: 2000 Hz / 532 Hz
   0x01: 1000 Hz / 230 Hz
   0x02:  400 Hz / 116 Hz
   0x03:  200 Hz /  47 Hz
   0x04:  100 Hz /  23 Hz
   0x05:  200 Hz /  12 Hz
   0x06:  100 Hz /  12 Hz
   0x07:  100 Hz /   8 Hz
*/
typedef enum {
    BMI088_GYRO_ODR_2000_HZ_BW_532 = 0x00,
    BMI088_GYRO_ODR_1000_HZ_BW_230 = 0x01,
    BMI088_GYRO_ODR_400_HZ_BW_116  = 0x02,
    BMI088_GYRO_ODR_200_HZ_BW_47   = 0x03,
    BMI088_GYRO_ODR_100_HZ_BW_23   = 0x04,
    BMI088_GYRO_ODR_200_HZ_BW_12   = 0x05,
    BMI088_GYRO_ODR_100_HZ_BW_12   = 0x06,
    BMI088_GYRO_ODR_100_HZ_BW_8    = 0x07
} bmi088_gyro_odr_t;

/**
 * @brief Build command to configure output data rate and bandwidth.
 */
size_t bmi088_gyro_build_odr(bmi088_gyro_odr_t odr,
                             uint8_t *tx_buf,
                             bmi088_gyro_t *dev);

/* --- Interrupt configuration --------------------------------------------- */
typedef enum {
    BMI088_GYRO_INT3 = 0,
    BMI088_GYRO_INT4 = 1
} bmi088_gyro_int_pin_t;

typedef enum {
    BMI088_GYRO_INT_PUSH_PULL = 0,
    BMI088_GYRO_INT_OPEN_DRAIN = 1
} bmi088_gyro_int_drive_t;

typedef enum {
    BMI088_GYRO_INT_ACTIVE_LOW = 0,
    BMI088_GYRO_INT_ACTIVE_HIGH = 1
} bmi088_gyro_int_polarity_t;

typedef enum {
    BMI088_GYRO_INT_EVENT_NONE       = 0x00,
    BMI088_GYRO_INT_EVENT_DATA_READY = 0x01,
    BMI088_GYRO_INT_EVENT_FIFO_WTM   = 0x02,
    BMI088_GYRO_INT_EVENT_FIFO_FULL  = 0x04
} bmi088_gyro_int_event_t;

/**
 * @brief Build command to configure the electrical behavior of an interrupt pin.
 * Note: Gyro side supports push-pull/open-drain and active level; no latch parameter here.
 */
size_t bmi088_gyro_build_int_pin_conf(bmi088_gyro_int_pin_t pin,
                                      bmi088_gyro_int_drive_t drive,
                                      bmi088_gyro_int_polarity_t polarity,
                                      uint8_t *tx_buf);

/**
 * @brief Build command to map specific interrupt events to an interrupt pin.
 * The 'events' parameter is a bitmask of bmi088_gyro_int_event_t.
 */
size_t bmi088_gyro_build_int_event_map(bmi088_gyro_int_pin_t pin,
                                       uint8_t events,
                                       uint8_t *tx_buf);

/* --- FIFO configuration --------------------------------------------------- */
typedef enum {
    BMI088_GYRO_FIFO_BYPASS = 0x00,  ///< fifo_mode bits cleared
    BMI088_GYRO_FIFO_FIFO   = 0x40,  ///< fifo_mode == FIFO
    BMI088_GYRO_FIFO_STREAM = 0x80   ///< fifo_mode == STREAM
} bmi088_gyro_fifo_mode_t;

typedef enum {
    BMI088_GYRO_FIFO_NONE = 0x00,
    BMI088_GYRO_FIFO_XYZ  = 0x01
    /* Note: sensortime frames are not produced by the gyro FIFO */
} bmi088_gyro_fifo_source_t;

/**
 * @brief Build commands to configure FIFO behavior.
 * @param watermark 7-bit value (0..127). Writing FIFO_CONFIG_0 clears FIFO.
 */
size_t bmi088_gyro_build_fifo_conf(bmi088_gyro_fifo_mode_t mode,
                                   bmi088_gyro_fifo_source_t src,
                                   uint8_t watermark,
                                   uint8_t *tx_buf);

/* --- Read-command builder ------------------------------------------------- */
typedef enum {
    BMI088_GYRO_READ_CHIP_ID,
    BMI088_GYRO_READ_INT_STAT_1,
    BMI088_GYRO_READ_DATA_XYZ,
    BMI088_GYRO_READ_FIFO_STATUS,
    BMI088_GYRO_READ_FIFO_DATA
} bmi088_gyro_read_t;

/**
 * @brief Build an SPI read command for a given sensor read type.
 * (Gyro SPI reads do not require a dummy byte.)
 */
size_t bmi088_gyro_build_read(bmi088_gyro_read_t what, uint8_t *tx_buf);

/* -------------------------------------------------------------------------- */
/* Parsing helpers                                                            */
/* -------------------------------------------------------------------------- */
typedef struct {
    uint32_t t_us;     ///< optional timestamp from host; not provided by gyro FIFO
    int16_t  x, y, z;  ///< raw gyro counts
    float    gx, gy, gz; ///< converted values in rad/s
} bmi088_gyro_sample_t;

/**
 * @brief Parse raw XYZ gyro data (6 bytes).
 */
bool bmi088_gyro_parse_data_xyz(const uint8_t *rx_buf,
                                bmi088_gyro_sample_t *sample,
                                const bmi088_gyro_t *dev);

/**
 * @brief Parse FIFO status register (0x0E).
 * @param fifo_frame_count out: 7-bit FIFO frame counter (0..127)
 * @param fifo_overrun out: true if overrun flag is set
 */
bool bmi088_gyro_parse_fifo_status(const uint8_t *rx_buf,
                                   uint8_t *fifo_frame_count,
                                   bool *fifo_overrun);

/**
 * @brief Parse FIFO data frames into samples.
 * Expects a buffer read from FIFO_DATA (0x3F) and decodes gyro XYZ frames.
 */
bool bmi088_gyro_parse_fifo(const uint8_t *rx_buf, size_t len,
                            bmi088_gyro_sample_t *out_samples,
                            size_t *max_samples,
                            const bmi088_gyro_t *dev);

/* --- Ring buffer for gyroscope samples ----*/

/**
 * @brief SPSC ring buffer for gyroscope samples.
 *
 * Thread Safety:
 * - Single Producer (ISR/DMA callback) writes samples via bmi088_gyro_sample_queue()
 * - Single Consumer (state estimation task) reads via bmi088_gyro_sample_dequeue()
 * - Memory barriers ensure proper ordering between producer and consumer.
 */
typedef struct {
    bmi088_gyro_sample_t samples[BMI088_GYRO_SAMPLE_Q_SIZE];
    volatile uint8_t head;  /**< Written by producer only */
    volatile uint8_t tail;  /**< Written by consumer only */
} bmi088_gyro_sample_queue_t;

/**
 * @brief Check if sample queue is empty.
 */
static inline bool bmi088_gyro_sample_queue_empty(bmi088_gyro_sample_queue_t *q) {
    return q->head == q->tail;
}

/**
 * @brief Check if sample queue is full.
 */
static inline bool bmi088_gyro_sample_queue_full(bmi088_gyro_sample_queue_t *q) {
    return ((q->head + 1) % BMI088_GYRO_SAMPLE_Q_SIZE) == q->tail;
}

/**
 * @brief Enqueue a new gyroscope sample (producer side - ISR context).
 * @param q Pointer to the sample queue.
 * @param sample Pointer to sample to enqueue.
 * @return True if successful, false if queue full.
 * @note Memory barrier ensures sample data is visible before head update.
 */
static inline bool bmi088_gyro_sample_queue(bmi088_gyro_sample_queue_t *q,
                                            const bmi088_gyro_sample_t *sample) {
    if (bmi088_gyro_sample_queue_full(q)) return false;
    q->samples[q->head] = *sample;
    SYNC_DMB();  /* Ensure sample data written before head update is visible */
    q->head = (q->head + 1) % BMI088_GYRO_SAMPLE_Q_SIZE;
    return true;
}

/**
 * @brief Dequeue the next gyroscope sample (consumer side - task context).
 * @param q Pointer to the sample queue.
 * @param sample Pointer to receive dequeued sample.
 * @return True if successful, false if queue empty.
 * @note Memory barriers ensure proper ordering with producer.
 */
static inline bool bmi088_gyro_sample_dequeue(bmi088_gyro_sample_queue_t *q,
                                              bmi088_gyro_sample_t *sample) {
    if (bmi088_gyro_sample_queue_empty(q)) return false;
    SYNC_DMB();  /* Ensure we see sample data written before head was updated */
    *sample = q->samples[q->tail];
    SYNC_DMB();  /* Ensure sample read completes before tail update */
    q->tail = (q->tail + 1) % BMI088_GYRO_SAMPLE_Q_SIZE;
    return true;
}

#endif /* BMI088_GYRO_H */
