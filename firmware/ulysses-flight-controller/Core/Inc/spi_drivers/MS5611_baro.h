/*
    Driver for the MS5611-01BA03 barometer.
    Bus-agnostic implementation: this driver only builds and parses SPI command frames.
    Sampling is done via polling (no interrupt pin on device).

    References:
    - TE Connectivity MS5611-01BA03 Datasheet (ENG_DS_MS5611-01BA03_B3)
      Verified Nov 2025.

    @ UBC Rocket, Benedikt Howard
*/

#ifndef MS5611_BARO_H
#define MS5611_BARO_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "sync.h"

/* -------------------------------------------------------------------------- */
/* SPI Commands (see datasheet Table “Command Set”)                           */
/* -------------------------------------------------------------------------- */
/*
   Reset:                   0x1E
   Conversion D1 (Pressure): 0x40 | osr_bits
   Conversion D2 (Temp):     0x50 | osr_bits
   ADC Read (24-bit result): 0x00
   PROM Read (16-bit word):  0xA0 | (addr << 1), where addr = 0..7
*/
#define MS5611_CMD_RESET        0x1E
#define MS5611_CMD_ADC_READ     0x00
#define MS5611_CMD_CONVERT_D1   0x40
#define MS5611_CMD_CONVERT_D2   0x50
#define MS5611_CMD_PROM_READ    0xA0

/* -------------------------------------------------------------------------- */
/* Oversampling settings (OSR)                                                */
/* -------------------------------------------------------------------------- */
/*
Typical conversion times tc (ms) from datasheet Table 1:
 OSR   | Command bits |  Min | Typ | Max | RMS res (mbar)
 ------|---------------|------|------|------|--------------
 256   |   +0x00       | 0.48 | 0.54 | 0.60 | 0.065
 512   |   +0x02       | 0.95 | 1.06 | 1.17 | 0.042
 1024  |   +0x04       | 1.88 | 2.08 | 2.28 | 0.027
 2048  |   +0x06       | 3.72 | 4.13 | 4.54 | 0.018
 4096  |   +0x08       | 7.40 | 8.22 | 9.04 | 0.012
*/
typedef enum {
    MS5611_OSR_256  = 0x00, ///< typ 0.54 ms
    MS5611_OSR_512  = 0x02, ///< typ 1.06 ms
    MS5611_OSR_1024 = 0x04, ///< typ 2.08 ms
    MS5611_OSR_2048 = 0x06, ///< typ 4.13 ms
    MS5611_OSR_4096 = 0x08  ///< typ 8.22 ms
} ms5611_osr_t;

/* -------------------------------------------------------------------------- */
/* Device state structure                                                     */
/* -------------------------------------------------------------------------- */
typedef struct {
    uint16_t C[8];       ///< Calibration PROM (C0..C7): C1–C6 coeffs, C0 factory data, C7 serial+CRC
    uint32_t D1_raw;     ///< Uncompensated pressure (24-bit)
    uint32_t D2_raw;     ///< Uncompensated temperature (24-bit)
    int32_t  TEMP_centi; ///< Temperature in 0.01 °C
    int32_t  P_centi_mbar; ///< Pressure in 0.01 mbar (datasheet pressure unit is mbar/hPa)
} ms5611_t;

/* -------------------------------------------------------------------------- */
/* Command builders                                                           */
/* -------------------------------------------------------------------------- */

/**
 * @brief Build command to reset the MS5611 (0x1E).
 * @note After issuing reset, wait ≥2.8 ms for PROM reload (datasheet section “Reset Sequence”).
 */
size_t ms5611_build_reset(uint8_t *tx_buf);

/**
 * @brief Build command to start pressure conversion (D1).
 * @param osr Oversampling ratio (MS5611_OSR_xxx)
 */
size_t ms5611_build_convert_d1(ms5611_osr_t osr, uint8_t *tx_buf);

/**
 * @brief Build command to start temperature conversion (D2).
 * @param osr Oversampling ratio (MS5611_OSR_xxx)
 */
size_t ms5611_build_convert_d2(ms5611_osr_t osr, uint8_t *tx_buf);

/**
 * @brief Build command to read the ADC (24-bit result, MSB first).
 */
size_t ms5611_build_adc_read(uint8_t *tx_buf);

/**
 * @brief Build command to read a PROM word (16-bit).
 * @param index PROM address 0..7
 */
size_t ms5611_build_prom_read(uint8_t index, uint8_t *tx_buf);

/* -------------------------------------------------------------------------- */
/* Parsing helpers                                                            */
/* -------------------------------------------------------------------------- */

/**
 * @brief Parse raw PROM word (2 bytes MSB first).
 */
bool ms5611_parse_prom_word(const uint8_t *rx_buf, uint16_t *out_word);

/**
 * @brief Parse ADC result (3 bytes MSB first → 24-bit value).
 */
bool ms5611_parse_adc_result(const uint8_t *rx_buf, uint32_t *out_val);

/**
 * @brief Verify PROM CRC-4 according to datasheet algorithm.
 */
bool ms5611_check_crc(const uint16_t *coeff);

/* -------------------------------------------------------------------------- */
/* Computation helpers                                                        */
/* -------------------------------------------------------------------------- */
/**
 * @brief Compute compensated temperature and pressure using first- and
 *        second-order correction (as per datasheet section “Pressure and
 *        Temperature Calculation Example”).
 *
 *  Equations:
 *   dT   = D2 - C5 * 2^8
 *   TEMP = 2000 + dT * C6 / 2^23
 *   OFF  = C2 * 2^16 + (C4 * dT) / 2^7
 *   SENS = C1 * 2^15 + (C3 * dT) / 2^8
 *   P    = (D1 * SENS / 2^21 - OFF) / 2^15
 *
 *  If TEMP < 20°C, apply second-order compensation per datasheet Table 2:
 *   TEMP2, OFF2, SENS2 terms adjust TEMP/OFF/SENS.
 */
bool ms5611_compute(ms5611_t *dev);

/* --- Ring buffer for barometer samples ------------------------------------ */
#ifndef MS5611_SAMPLE_Q_SIZE
#define MS5611_SAMPLE_Q_SIZE 16
#endif

typedef struct {
    uint32_t t_us;            ///< host timestamp when result ready
    uint32_t d1_raw;          ///< raw pressure conversion (24-bit)
    uint32_t d2_raw;          ///< raw temperature conversion (24-bit)
    int32_t  temp_centi;      ///< temperature in 0.01 °C
    int32_t  pressure_centi;  ///< pressure in 0.01 mbar
    uint32_t seq;             ///< monotonically increasing sequence
} ms5611_sample_t;

/**
 * @brief SPSC ring buffer for barometer samples.
 *
 * Thread Safety:
 * - Single Producer (ISR/DMA callback) writes samples via ms5611_sample_queue()
 * - Single Consumer (state estimation task) reads via ms5611_sample_dequeue()
 * - Memory barriers ensure proper ordering between producer and consumer.
 */
typedef struct {
    ms5611_sample_t samples[MS5611_SAMPLE_Q_SIZE];
    volatile uint8_t head;  /**< Written by producer only */
    volatile uint8_t tail;  /**< Written by consumer only */
} ms5611_sample_queue_t;

/**
 * @brief Check if sample queue is empty.
 */
static inline bool ms5611_sample_queue_empty(ms5611_sample_queue_t *q) {
    return q->head == q->tail;
}

/**
 * @brief Check if sample queue is full.
 */
static inline bool ms5611_sample_queue_full(ms5611_sample_queue_t *q) {
    return ((q->head + 1U) % MS5611_SAMPLE_Q_SIZE) == q->tail;
}

/**
 * @brief Enqueue a new barometer sample (producer side - ISR/callback context).
 * @param q Pointer to the sample queue.
 * @param sample Pointer to sample to enqueue.
 * @return True if successful, false if queue full.
 * @note Memory barrier ensures sample data is visible before head update.
 */
static inline bool ms5611_sample_queue(ms5611_sample_queue_t *q,
                                       const ms5611_sample_t *sample) {
    if (ms5611_sample_queue_full(q)) return false;
    q->samples[q->head] = *sample;
    SYNC_DMB();  /* Ensure sample data written before head update is visible */
    q->head = (uint8_t)((q->head + 1U) % MS5611_SAMPLE_Q_SIZE);
    return true;
}

/**
 * @brief Dequeue the next barometer sample (consumer side - task context).
 * @param q Pointer to the sample queue.
 * @param sample Pointer to receive dequeued sample.
 * @return True if successful, false if queue empty.
 * @note Memory barriers ensure proper ordering with producer.
 */
static inline bool ms5611_sample_dequeue(ms5611_sample_queue_t *q,
                                         ms5611_sample_t *sample) {
    if (ms5611_sample_queue_empty(q)) return false;
    SYNC_DMB();  /* Ensure we see sample data written before head was updated */
    *sample = q->samples[q->tail];
    SYNC_DMB();  /* Ensure sample read completes before tail update */
    q->tail = (uint8_t)((q->tail + 1U) % MS5611_SAMPLE_Q_SIZE);
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
#endif /* MS5611_BARO_H */
