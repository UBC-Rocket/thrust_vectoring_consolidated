/**
 * @file    dxl_packet.c
 * @brief   Dynamixel Protocol 2.0 packet encode/decode.
 *
 * UBC Rocket, 2026
 */
#include "dynamixel/dxl_packet.h"

#include <string.h>

/* CRC16 lookup table (Robotis Protocol 2.0). */
static const uint16_t s_crc_table[256] = {
    0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011, 0x8033,
    0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022, 0x8063, 0x0066,
    0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072, 0x0050, 0x8055, 0x805F,
    0x005A, 0x804B, 0x004E, 0x0044, 0x8041, 0x80C3, 0x00C6, 0x00CC, 0x80C9,
    0x00D8, 0x80DD, 0x80D7, 0x00D2, 0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB,
    0x00EE, 0x00E4, 0x80E1, 0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE,
    0x00B4, 0x80B1, 0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087,
    0x0082, 0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
    0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1, 0x01E0,
    0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1, 0x81D3, 0x01D6,
    0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2, 0x0140, 0x8145, 0x814F,
    0x014A, 0x815B, 0x015E, 0x0154, 0x8151, 0x8173, 0x0176, 0x017C, 0x8179,
    0x0168, 0x816D, 0x8167, 0x0162, 0x8123, 0x0126, 0x012C, 0x8129, 0x0138,
    0x813D, 0x8137, 0x0132, 0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E,
    0x0104, 0x8101, 0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317,
    0x0312, 0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
    0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371, 0x8353,
    0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342, 0x03C0, 0x83C5,
    0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1, 0x83F3, 0x03F6, 0x03FC,
    0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2, 0x83A3, 0x03A6, 0x03AC, 0x83A9,
    0x03B8, 0x83BD, 0x83B7, 0x03B2, 0x0390, 0x8395, 0x839F, 0x039A, 0x838B,
    0x038E, 0x0384, 0x8381, 0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E,
    0x0294, 0x8291, 0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7,
    0x02A2, 0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
    0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1, 0x8243,
    0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252, 0x0270, 0x8275,
    0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261, 0x0220, 0x8225, 0x822F,
    0x022A, 0x823B, 0x023E, 0x0234, 0x8231, 0x8213, 0x0216, 0x021C, 0x8219,
    0x0208, 0x820D, 0x8207, 0x0202,
};

uint16_t dxl_crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0;
    for (size_t i = 0; i < len; ++i) {
        const uint16_t idx = ((uint16_t)(crc >> 8) ^ data[i]) & 0xFFu;
        crc = (uint16_t)((crc << 8) ^ s_crc_table[idx]);
    }
    return crc;
}

static void write_header(uint8_t *buf, uint8_t id, uint16_t len_field) {
    buf[0] = DXL_HDR0;
    buf[1] = DXL_HDR1;
    buf[2] = DXL_HDR2;
    buf[3] = DXL_RESERVED;
    buf[4] = id;
    buf[5] = (uint8_t)(len_field & 0xFFu);
    buf[6] = (uint8_t)(len_field >> 8);
}

static bool apply_byte_stuffing(uint8_t *buf, size_t *len_io, size_t cap) {
    uint8_t tmp[DXL_MAX_PACKET_SIZE];
    const size_t src_len = *len_io;

    if (src_len < 6u || src_len > cap) {
        return false;
    }

    memcpy(tmp, buf, 3u);

    size_t di = 3u;
    for (size_t si = 3u; si < src_len; ++si) {
        if (di >= sizeof(tmp)) {
            return false;
        }
        tmp[di++] = buf[si];

        if (si >= 5u && buf[si - 2u] == 0xFFu && buf[si - 1u] == 0xFFu && buf[si] == 0xFDu) {
            if (di >= sizeof(tmp)) {
                return false;
            }
            tmp[di++] = 0xFDu;
        }
    }

    if (di > cap) {
        return false;
    }

    memcpy(buf, tmp, di);
    *len_io = di;
    return true;
}

static size_t remove_byte_stuffing(uint8_t *dst, const uint8_t *src, size_t src_len) {
    if (src_len < 6u) {
        if (dst != src) {
            memcpy(dst, src, src_len);
        }
        return src_len;
    }

    memcpy(dst, src, 3u);

    size_t di = 3u;
    for (size_t si = 3u; si < src_len; ++si) {
        dst[di++] = src[si];
        if (si >= 5u && src[si - 2u] == 0xFFu && src[si - 1u] == 0xFFu &&
            src[si] == 0xFDu && (si + 1u) < src_len && src[si + 1u] == 0xFDu) {
            ++si;
        }
    }

    return di;
}

static size_t finalize_packet(uint8_t *buf, size_t cap, size_t total_before_crc) {
    if (total_before_crc + 2u > cap) {
        return 0;
    }

    const uint16_t crc = dxl_crc16(buf, total_before_crc);
    buf[total_before_crc]     = (uint8_t)(crc & 0xFFu);
    buf[total_before_crc + 1u] = (uint8_t)(crc >> 8);

    size_t len = total_before_crc + 2u;
    if (!apply_byte_stuffing(buf, &len, cap)) {
        return 0;
    }
    return len;
}

size_t dxl_pkt_build_ping(uint8_t *buf, size_t cap, uint8_t id) {
    if (cap < 10u) {
        return 0;
    }

    write_header(buf, id, 3u);
    buf[7] = DXL_INST_PING;
    return finalize_packet(buf, cap, 8u);
}

size_t dxl_pkt_build_reboot(uint8_t *buf, size_t cap, uint8_t id) {
    if (cap < 10u) {
        return 0;
    }

    write_header(buf, id, 3u);
    buf[7] = DXL_INST_REBOOT;
    return finalize_packet(buf, cap, 8u);
}

size_t dxl_pkt_build_read(uint8_t *buf, size_t cap, uint8_t id,
                          uint16_t addr, uint16_t len) {
    if (cap < 14u) {
        return 0;
    }

    write_header(buf, id, 7u);
    buf[7] = DXL_INST_READ;
    buf[8] = (uint8_t)(addr & 0xFFu);
    buf[9] = (uint8_t)(addr >> 8);
    buf[10] = (uint8_t)(len & 0xFFu);
    buf[11] = (uint8_t)(len >> 8);
    return finalize_packet(buf, cap, 12u);
}

size_t dxl_pkt_build_write(uint8_t *buf, size_t cap, uint8_t id,
                           uint16_t addr, const uint8_t *data, uint16_t data_len) {
    const size_t total = 12u + (size_t)data_len;
    if (data_len > 256u || cap < total) {
        return 0;
    }

    write_header(buf, id, (uint16_t)(5u + data_len));
    buf[7] = DXL_INST_WRITE;
    buf[8] = (uint8_t)(addr & 0xFFu);
    buf[9] = (uint8_t)(addr >> 8);
    if (data_len > 0u && data != NULL) {
        memcpy(&buf[10], data, data_len);
    }
    return finalize_packet(buf, cap, 10u + (size_t)data_len);
}

size_t dxl_pkt_expected_read_rx(uint16_t data_len) {
    return (size_t)DXL_PKT_META_SIZE + 4u + (size_t)data_len;
}

bool dxl_pkt_parse_status(const uint8_t *rx, size_t rx_len, dxl_status_t *out) {
    if (out == NULL || rx_len == 0u || rx_len > DXL_MAX_PACKET_SIZE) {
        return false;
    }

    uint8_t pkt[DXL_MAX_PACKET_SIZE];
    const size_t pkt_len = remove_byte_stuffing(pkt, rx, rx_len);

    if (pkt_len < DXL_STATUS_MIN_LEN) {
        return false;
    }
    if (pkt[0] != DXL_HDR0 || pkt[1] != DXL_HDR1 || pkt[2] != DXL_HDR2 ||
        pkt[3] != DXL_RESERVED) {
        return false;
    }
    if (pkt[7] != DXL_INST_STATUS) {
        return false;
    }

    const uint16_t declared_len = (uint16_t)pkt[5] | ((uint16_t)pkt[6] << 8);
    if ((size_t)declared_len + DXL_PKT_META_SIZE != pkt_len) {
        return false;
    }

    const uint16_t crc_rx = (uint16_t)pkt[pkt_len - 2u] |
                            ((uint16_t)pkt[pkt_len - 1u] << 8);
    if (dxl_crc16(pkt, pkt_len - 2u) != crc_rx) {
        return false;
    }

    out->id = pkt[4];
    out->instruction = pkt[7];
    out->error = pkt[8];

    const uint16_t param_len = (declared_len > 4u) ? (uint16_t)(declared_len - 4u) : 0u;
    out->param_len = param_len;
    if (param_len > sizeof(out->params)) {
        return false;
    }
    if (param_len > 0u) {
        memcpy(out->params, &pkt[9], param_len);
    }

    return true;
}
