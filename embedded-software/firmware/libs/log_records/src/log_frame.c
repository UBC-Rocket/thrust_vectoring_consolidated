#include "log_records/log_frame.h"

uint16_t log_crc16_ccitt_update(uint16_t crc, uint8_t data)
{
    crc ^= (uint16_t)data << 8;
    for (uint8_t i = 0; i < 8U; ++i) {
        if (crc & 0x8000U) {
            crc = (crc << 1U) ^ 0x1021U;
        } else {
            crc <<= 1U;
        }
    }
    return crc;
}

uint16_t log_crc16_ccitt_compute(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFU;
    return log_crc16_ccitt_accumulate(crc, data, len);
}

uint16_t log_crc16_ccitt_accumulate(uint16_t crc, const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; ++i) {
        crc = log_crc16_ccitt_update(crc, data[i]);
    }
    return crc;
}
