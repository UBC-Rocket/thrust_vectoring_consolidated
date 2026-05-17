#ifndef LOG_FRAME_H
#define LOG_FRAME_H

#include <stddef.h>
#include <stdint.h>

/**
 * @brief Update CRC-16/CCITT with a single byte.
 */
uint16_t log_crc16_ccitt_update(uint16_t crc, uint8_t data);

/**
 * @brief Compute CRC-16/CCITT over a buffer (initial value 0xFFFF).
 */
uint16_t log_crc16_ccitt_compute(const uint8_t *data, size_t len);

/**
 * @brief Continue CRC-16/CCITT accumulation over additional data.
 */
uint16_t log_crc16_ccitt_accumulate(uint16_t crc, const uint8_t *data, size_t len);

#endif /* LOG_FRAME_H */
