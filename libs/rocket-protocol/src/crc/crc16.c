#include "rp/crc/crc.h"

#include <stdint.h>

/**
 * Computes the checksum of the data using the CRC-16/KERMIT (CCITT) algorithm.
 *
 * The computation is performed 1 byte at a time according to the algorithm described
 * in https://en.wikipedia.org/wiki/Computation_of_cyclic_redundancy_checks#Examples_for_sparse_polynomials.
 *
 * @param data Buffer of bytes to compute the checksum for
 * @param length Number of bytes in the data buffer
 * @return uint16_t
 */
uint16_t crc16_ccitt(const uint8_t *data, size_t length)
{
    uint16_t crc = CRC16_CCITT_INITIAL_VALUE;
    uint8_t e;
    uint8_t f;

    if (data == NULL) {
        return crc;
    }

    for (; length > 0; length--) {
        e = crc ^ *data;
        data++;
        f = e ^ (e << 4);
        crc = (crc >> 8) ^ ((uint16_t)f << 8) ^ ((uint16_t)f << 3) ^ ((uint16_t)f >> 4);
    }

    return crc;
}
