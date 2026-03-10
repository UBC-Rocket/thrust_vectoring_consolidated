#ifndef RP_CRC_H
#define RP_CRC_H

#include <stddef.h>
#include <stdint.h>

#define CRC16_CCITT_INITIAL_VALUE (0x0000)
#define CRC16_CCITT_RESIDUE (0x0000)

uint16_t crc16_ccitt(const uint8_t *data, size_t length);

#endif // RP_CRC_H
