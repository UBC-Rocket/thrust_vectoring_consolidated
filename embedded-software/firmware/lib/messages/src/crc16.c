/**
 * @file crc16.c
 * @brief Table-driven CRC-16/CCITT-FALSE.
 *
 * Table is built on first use (single-threaded init expected during
 * messages_init or first messages_crc16 call). Worst case the table is
 * computed twice if two threads race on the very first call — that produces
 * the same bytes, so it's harmless. We do not protect with a lock.
 *
 * @ UBC Rocket, 2026
 */

#include "messages/crc16.h"

#define CRC16_POLY 0x1021U

static uint16_t s_table[256];
static int s_table_ready = 0; /* 0 = not built, 1 = built */

static void build_table(void)
{
    for (unsigned int i = 0U; i < 256U; ++i) {
        uint16_t crc = (uint16_t)(i << 8);
        for (unsigned int b = 0U; b < 8U; ++b) {
            if ((crc & 0x8000U) != 0U) {
                crc = (uint16_t)((crc << 1) ^ CRC16_POLY);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
        s_table[i] = crc;
    }
    s_table_ready = 1;
}

uint16_t messages_crc16_update(uint16_t crc, const uint8_t *data, size_t len)
{
    if (s_table_ready == 0) {
        build_table();
    }

    if (data == NULL) {
        return crc;
    }

    for (size_t i = 0U; i < len; ++i) {
        uint8_t idx = (uint8_t)((crc >> 8) ^ data[i]);
        crc = (uint16_t)((crc << 8) ^ s_table[idx]);
    }
    return crc;
}

uint16_t messages_crc16(const uint8_t *data, size_t len)
{
    return messages_crc16_update(MESSAGES_CRC16_INIT, data, len);
}
