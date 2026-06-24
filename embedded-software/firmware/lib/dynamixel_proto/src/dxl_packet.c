/**
 * @file    dxl_packet.c
 * @brief   Dynamixel Protocol 2.0 packet encode/decode.
 *
 * UBC Rocket, 2026
 */
#include "dynamixel/dxl_packet.h"

#include <string.h>

uint16_t dxl_crc16_update(uint16_t crc, uint8_t byte) {
    crc ^= (uint16_t)byte;
    for (int i = 0; i < 8; ++i) {
        if (crc & 1u) {
            crc = (uint16_t)((crc >> 1) ^ 0xA001u);
        } else {
            crc = (uint16_t)(crc >> 1);
        }
    }
    return crc;
}

uint16_t dxl_crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0;
    for (size_t i = 0; i < len; ++i) {
        crc = dxl_crc16_update(crc, data[i]);
    }
    return crc;
}

static size_t pkt_finalize(uint8_t *buf, size_t cap, uint8_t id,
                           uint8_t inst, const uint8_t *params, uint16_t param_len) {
    /* LENGTH = instruction + parameters + CRC(2) = param_len + 3 */
    const uint16_t wire_len = (uint16_t)(param_len + 3u);
    const size_t total = (size_t)DXL_PKT_HDR_SIZE + 1u + 2u + wire_len;
    if (cap < total) return 0;

    size_t i = 0;
    buf[i++] = DXL_HDR0;
    buf[i++] = DXL_HDR1;
    buf[i++] = DXL_HDR2;
    buf[i++] = id;
    buf[i++] = (uint8_t)(wire_len & 0xFFu);
    buf[i++] = (uint8_t)(wire_len >> 8);
    buf[i++] = inst;
    if (param_len > 0u && params != NULL) {
        memcpy(&buf[i], params, param_len);
        i += param_len;
    }

    const uint16_t crc = dxl_crc16(&buf[3], i - 3u);
    buf[i++] = (uint8_t)(crc & 0xFFu);
    buf[i++] = (uint8_t)(crc >> 8);
    return i;
}

size_t dxl_pkt_build_ping(uint8_t *buf, size_t cap, uint8_t id) {
    return pkt_finalize(buf, cap, id, DXL_INST_PING, NULL, 0);
}

size_t dxl_pkt_build_read(uint8_t *buf, size_t cap, uint8_t id,
                          uint16_t addr, uint16_t len) {
    uint8_t params[4];
    params[0] = (uint8_t)(addr & 0xFFu);
    params[1] = (uint8_t)(addr >> 8);
    params[2] = (uint8_t)(len & 0xFFu);
    params[3] = (uint8_t)(len >> 8);
    return pkt_finalize(buf, cap, id, DXL_INST_READ, params, 4);
}

size_t dxl_pkt_build_write(uint8_t *buf, size_t cap, uint8_t id,
                           uint16_t addr, const uint8_t *data, uint16_t data_len) {
    uint8_t params[2u + 256u];
    if (data_len > 256u) return 0;
    params[0] = (uint8_t)(addr & 0xFFu);
    params[1] = (uint8_t)(addr >> 8);
    if (data_len > 0u && data != NULL) {
        memcpy(&params[2], data, data_len);
    }
    return pkt_finalize(buf, cap, id, DXL_INST_WRITE, params, (uint16_t)(2u + data_len));
}

static bool verify_header(const uint8_t *rx, size_t rx_len) {
    return rx_len >= DXL_PKT_MIN_SIZE
        && rx[0] == DXL_HDR0
        && rx[1] == DXL_HDR1
        && rx[2] == DXL_HDR2;
}

bool dxl_pkt_parse_status(const uint8_t *rx, size_t rx_len, dxl_status_t *out) {
    if (out == NULL || !verify_header(rx, rx_len)) return false;

    const uint8_t id = rx[3];
    const uint16_t wire_len = (uint16_t)rx[4] | ((uint16_t)rx[5] << 8);
    const size_t pkt_len = (size_t)DXL_PKT_HDR_SIZE + 1u + 2u + wire_len;
    if (rx_len < pkt_len || wire_len < 3u) return false;

    const uint16_t crc_rx = (uint16_t)rx[pkt_len - 2u] | ((uint16_t)rx[pkt_len - 1u] << 8);
    const uint16_t crc_calc = dxl_crc16(&rx[3], pkt_len - 3u - 2u);
    if (crc_rx != crc_calc) return false;

    out->id = id;
    out->instruction = rx[6];
    out->error = rx[7];
    out->param_len = (size_t)wire_len - 4u; /* inst + error + crc(2) */
    if (out->param_len > sizeof(out->params)) return false;
    if (out->param_len > 0u) {
        memcpy(out->params, &rx[8], out->param_len);
    }
    return true;
}
