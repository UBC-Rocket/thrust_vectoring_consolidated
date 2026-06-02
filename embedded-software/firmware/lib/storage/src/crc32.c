/**
 * @file crc32.c
 * @brief CRC-32 (IEEE 802.3 / zlib polynomial 0xEDB88320, reflected,
 *        init 0xFFFFFFFF, final xor 0xFFFFFFFF) — matches Python's
 *        binascii.crc32 byte-for-byte. Small ~1 kB lookup table.
 *
 * @ UBC Rocket, 2026
 */
#include "storage_crc32.h"

static uint32_t s_table[256];
static bool     s_table_ready;

static void build_table(void) {
    for (uint32_t i = 0; i < 256; ++i) {
        uint32_t c = i;
        for (int j = 0; j < 8; ++j) {
            c = (c & 1U) ? (0xEDB88320U ^ (c >> 1)) : (c >> 1);
        }
        s_table[i] = c;
    }
    s_table_ready = true;
}

uint32_t storage_crc32(const void *data, size_t len) {
    if (!s_table_ready) build_table();
    const uint8_t *p = (const uint8_t *)data;
    uint32_t c = 0xFFFFFFFFU;
    while (len--) {
        c = s_table[(c ^ *p++) & 0xFFU] ^ (c >> 8);
    }
    return c ^ 0xFFFFFFFFU;
}
