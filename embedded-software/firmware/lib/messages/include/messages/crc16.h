/**
 * @file crc16.h
 * @brief CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, no xorout).
 *
 * Test vector: ASCII "123456789" -> 0x29B1 (Catalogue of CRC algorithms).
 *
 * @ UBC Rocket, 2026
 */

#ifndef MESSAGES_CRC16_H
#define MESSAGES_CRC16_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Initial CRC value (CCITT-FALSE). */
#define MESSAGES_CRC16_INIT 0xFFFFU

/**
 * @brief Compute CRC-16/CCITT-FALSE over a buffer using the precomputed table.
 *
 * @param data Pointer to input bytes.
 * @param len  Number of bytes.
 * @return Computed CRC, initial value 0xFFFF.
 */
uint16_t messages_crc16(const uint8_t *data, size_t len);

/**
 * @brief Continue an in-progress CRC over additional bytes.
 *
 * @param crc  Previous CRC value (start with MESSAGES_CRC16_INIT).
 * @param data Pointer to bytes.
 * @param len  Number of bytes.
 * @return Updated CRC.
 */
uint16_t messages_crc16_update(uint16_t crc, const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* MESSAGES_CRC16_H */
