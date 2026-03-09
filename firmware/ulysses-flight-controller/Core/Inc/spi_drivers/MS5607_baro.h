/**
 * @file MS5607_baro.h
 * @brief Driver for the MS5607-02BA03 barometer.
 *
 * Bus-agnostic implementation: this driver only builds and parses SPI command frames.
 * Sampling is done via polling (no interrupt pin on device).
 *
 * Note: MS5607 uses different compensation formulas than MS5611!
 * Do not mix implementations.
 *
 * References:
 * - TE Connectivity MS5607-02BA03 Datasheet
 *
 * @ UBC Rocket, Jan 2026
 */

#ifndef MS5607_BARO_H
#define MS5607_BARO_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "sync.h"

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* SPI Commands (see datasheet "Command Set")                                 */
/* -------------------------------------------------------------------------- */
/*
   Reset:                   0x1E
   Conversion D1 (Pressure): 0x40 | osr_bits
   Conversion D2 (Temp):     0x50 | osr_bits
   ADC Read (24-bit result): 0x00
   PROM Read (16-bit word):  0xA0 | (addr << 1), where addr = 0..7
*/
#define MS5607_CMD_RESET        0x1E
#define MS5607_CMD_ADC_READ     0x00
#define MS5607_CMD_CONVERT_D1   0x40
#define MS5607_CMD_CONVERT_D2   0x50
#define MS5607_CMD_PROM_READ    0xA0

/* -------------------------------------------------------------------------- */
/* Oversampling settings (OSR)                                                */
/* -------------------------------------------------------------------------- */
/*
 * Datasheet page 3 (ADC table) and page 2 (resolution):
 *
 * OSR  | Conversion Time (max) | Pressure Resolution (RMS)
 * -----|----------------------|-------------------------
 * 256  | 0.60 ms              | 0.130 mbar
 * 512  | 1.17 ms              | 0.084 mbar
 * 1024 | 2.28 ms              | 0.054 mbar
 * 2048 | 4.54 ms              | 0.036 mbar
 * 4096 | 9.04 ms              | 0.024 mbar
 *
 * Enum values are the actual bits OR'd into convert commands.
 */
typedef enum {
    MS5607_OSR_256  = 0x00,  ///< typ 0.54 ms, fastest, lowest resolution
    MS5607_OSR_512  = 0x02,  ///< typ 1.06 ms
    MS5607_OSR_1024 = 0x04,  ///< typ 2.08 ms
    MS5607_OSR_2048 = 0x06,  ///< typ 4.13 ms
    MS5607_OSR_4096 = 0x08   ///< typ 8.22 ms, slowest, highest resolution
} ms5607_osr_t;

/* -------------------------------------------------------------------------- */
/* Device state structure                                                     */
/* -------------------------------------------------------------------------- */
/**
 * @brief MS5607 device state.
 *
 * Holds PROM calibration data and current measurement values.
 *
 * PROM layout (datasheet):
 *   C[0]: Factory data and setup (reserved)
 *   C[1]: Pressure sensitivity (SENS_T1) - unsigned 16-bit
 *   C[2]: Pressure offset (OFF_T1) - unsigned 16-bit
 *   C[3]: Temp coefficient of pressure sensitivity (TCS) - unsigned 16-bit
 *   C[4]: Temp coefficient of pressure offset (TCO) - unsigned 16-bit
 *   C[5]: Reference temperature (T_REF) - unsigned 16-bit
 *   C[6]: Temp coefficient of temperature (TEMPSENS) - unsigned 16-bit
 *   C[7]: Serial code and CRC-4 (CRC in bits [3:0])
 */
typedef struct {
    uint16_t C[8];          ///< PROM coefficients - indices match datasheet addresses
    uint32_t D1_raw;        ///< Raw pressure ADC (24-bit, 0 to 16777216)
    uint32_t D2_raw;        ///< Raw temperature ADC (24-bit, 0 to 16777216)
    int32_t  TEMP_centi;    ///< Temperature in 0.01 °C (e.g., 2000 = 20.00°C)
    int32_t  P_centi_mbar;  ///< Pressure in 0.01 mbar (e.g., 101325 = 1013.25 mbar)
} ms5607_t;

/* -------------------------------------------------------------------------- */
/* Command builders                                                           */
/* -------------------------------------------------------------------------- */

/**
 * @brief Build command to reset the MS5607 (0x1E).
 * @param[out] tx_buf Buffer where SPI command is written.
 * @return Number of bytes to send (always 1).
 * @note After issuing reset, wait ≥2.8 ms for PROM reload.
 */
size_t ms5607_build_reset(uint8_t *tx_buf);

/**
 * @brief Build command to start pressure conversion (D1).
 * @param osr Oversampling ratio (MS5607_OSR_xxx)
 * @param[out] tx_buf Buffer where SPI command is written.
 * @return Number of bytes to send (always 1).
 */
size_t ms5607_build_convert_d1(ms5607_osr_t osr, uint8_t *tx_buf);

/**
 * @brief Build command to start temperature conversion (D2).
 * @param osr Oversampling ratio (MS5607_OSR_xxx)
 * @param[out] tx_buf Buffer where SPI command is written.
 * @return Number of bytes to send (always 1).
 */
size_t ms5607_build_convert_d2(ms5607_osr_t osr, uint8_t *tx_buf);

/**
 * @brief Build command to read the ADC (24-bit result, MSB first).
 * @param[out] tx_buf Buffer where SPI command is written (4 bytes).
 * @return Number of bytes in frame (always 4).
 * @note Pass &rx_buf[1] to ms5607_parse_adc_result().
 */
size_t ms5607_build_adc_read(uint8_t *tx_buf);

/**
 * @brief Build command to read a PROM word (16-bit).
 * @param index PROM address 0..7
 * @param[out] tx_buf Buffer where SPI command is written (3 bytes).
 * @return Number of bytes in frame (always 3).
 * @note Pass &rx_buf[1] to ms5607_parse_prom_word().
 */
size_t ms5607_build_prom_read(uint8_t index, uint8_t *tx_buf);

/* -------------------------------------------------------------------------- */
/* Parsing helpers                                                            */
/* -------------------------------------------------------------------------- */

/**
 * @brief Parse raw PROM word (2 bytes MSB first).
 * @param[in] rx_buf Pointer to first data byte (skip command response byte).
 * @param[out] out_word Destination for parsed 16-bit value.
 * @return true on success, false if rx_buf or out_word is NULL.
 */
bool ms5607_parse_prom_word(const uint8_t *rx_buf, uint16_t *out_word);

/**
 * @brief Parse ADC result (3 bytes MSB first → 24-bit value).
 * @param[in] rx_buf Pointer to first data byte (skip command response byte).
 * @param[out] out_val Destination for parsed 24-bit value.
 * @return true on success, false if rx_buf or out_val is NULL.
 */
bool ms5607_parse_adc_result(const uint8_t *rx_buf, uint32_t *out_val);

/**
 * @brief Verify PROM CRC-4 according to datasheet algorithm.
 * @param[in] C Array of 8 PROM words (C[0] through C[7]).
 * @return true if CRC matches, false on mismatch or NULL input.
 */
bool ms5607_check_crc(const uint16_t *C);

/* -------------------------------------------------------------------------- */
/* Computation helpers                                                        */
/* -------------------------------------------------------------------------- */

/**
 * @brief Compute compensated temperature and pressure from raw ADC values.
 *
 * Implements the MS5607-specific compensation algorithm including
 * first-order and second-order temperature correction.
 *
 * MS5607 formulas (DIFFERENT from MS5611!):
 *   OFF  = C2 × 2^17 + (C4 × dT) / 2^6
 *   SENS = C1 × 2^16 + (C3 × dT) / 2^7
 *
 * @param[in,out] dev Device structure with valid PROM coefficients and raw ADC values.
 * @return true on success, false if dev is NULL.
 */
bool ms5607_compute(ms5607_t *dev);

/* -------------------------------------------------------------------------- */
/* Ring buffer for barometer samples                                          */
/* -------------------------------------------------------------------------- */

#ifndef MS5607_SAMPLE_RING_SIZE
#define MS5607_SAMPLE_RING_SIZE 16
#endif

/**
 * @brief Single barometer sample for ring buffer.
 */
typedef struct {
    uint32_t t_us;            ///< host timestamp when result ready
    uint32_t d1_raw;          ///< raw pressure conversion (24-bit)
    uint32_t d2_raw;          ///< raw temperature conversion (24-bit)
    int32_t  temp_centi;      ///< temperature in 0.01 °C
    int32_t  pressure_centi;  ///< pressure in 0.01 mbar
    uint32_t seq;             ///< monotonically increasing sequence
} ms5607_sample_t;

/**
 * @brief SPSC ring buffer for barometer samples.
 *
 * Thread Safety:
 * - Single Producer (ISR/DMA callback) writes samples via ms5607_sample_queue()
 * - Single Consumer (state estimation task) reads via ms5607_sample_dequeue()
 * - Memory barriers ensure proper ordering between producer and consumer.
 */
typedef struct {
    ms5607_sample_t samples[MS5607_SAMPLE_RING_SIZE];
    volatile uint8_t head;  /**< Written by producer only */
    volatile uint8_t tail;  /**< Written by consumer only */
} ms5607_sample_queue_t;

/**
 * @brief Check if sample queue is empty.
 */
static inline bool ms5607_sample_queue_empty(ms5607_sample_queue_t *q) {
    return q->head == q->tail;
}

/**
 * @brief Check if sample queue is full.
 */
static inline bool ms5607_sample_queue_full(ms5607_sample_queue_t *q) {
    return ((q->head + 1U) % MS5607_SAMPLE_RING_SIZE) == q->tail;
}

/**
 * @brief Enqueue a new barometer sample (producer side - ISR/callback context).
 * @param q Pointer to the sample queue.
 * @param sample Pointer to sample to enqueue.
 * @return True if successful, false if queue full.
 * @note Memory barrier ensures sample data is visible before head update.
 */
static inline bool ms5607_sample_queue(ms5607_sample_queue_t *q,
                                       const ms5607_sample_t *sample) {
    if (ms5607_sample_queue_full(q)) return false;
    q->samples[q->head] = *sample;
    SYNC_DMB();  /* Ensure sample data written before head update is visible */
    q->head = (uint8_t)((q->head + 1U) % MS5607_SAMPLE_RING_SIZE);
    return true;
}

/**
 * @brief Dequeue the next barometer sample (consumer side - task context).
 * @param q Pointer to the sample queue.
 * @param sample Pointer to receive dequeued sample.
 * @return True if successful, false if queue empty.
 * @note Memory barriers ensure proper ordering with producer.
 */
static inline bool ms5607_sample_dequeue(ms5607_sample_queue_t *q,
                                         ms5607_sample_t *sample) {
    if (ms5607_sample_queue_empty(q)) return false;
    SYNC_DMB();  /* Ensure we see sample data written before head was updated */
    *sample = q->samples[q->tail];
    SYNC_DMB();  /* Ensure sample read completes before tail update */
    q->tail = (uint8_t)((q->tail + 1U) % MS5607_SAMPLE_RING_SIZE);
    return true;
}

/* -------------------------------------------------------------------------- */
/* Interface constraints                                                      */
/* -------------------------------------------------------------------------- */
/*
 - No interrupt pin: polling-based sampling required.
 - Max SPI clock: 20 MHz (datasheet electrical characteristics).
 - All SPI commands are single-byte followed by readback (16 or 24 bits MSB-first).
*/

#ifdef __cplusplus
}
#endif

#endif /* MS5607_BARO_H */
