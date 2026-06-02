#ifndef STORAGE_CRC32_PRIV_H
#define STORAGE_CRC32_PRIV_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Matches Python's binascii.crc32 byte-for-byte. */
uint32_t storage_crc32(const void *data, size_t len);

#endif
