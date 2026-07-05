/**
 * @file    dxl_packet.h
 * @brief   Dynamixel Protocol 2.0 packet encode/decode (pure C, no HAL).
 *
 * Packet layout matches Robotis Protocol 2.0 and the Cedar reference driver:
 *   FF FF FD 00 | ID | LEN_L LEN_H | INST | PARAMS... | CRC_L CRC_H
 *
 * UBC Rocket, 2026
 */
#ifndef DXL_PACKET_H
#define DXL_PACKET_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DXL_HDR0              0xFFu
#define DXL_HDR1              0xFFu
#define DXL_HDR2              0xFDu
#define DXL_RESERVED          0x00u
#define DXL_BROADCAST_ID      0xFEu

#define DXL_INST_PING         0x01u
#define DXL_INST_READ         0x02u
#define DXL_INST_WRITE        0x03u
#define DXL_INST_REBOOT       0x08u
#define DXL_INST_STATUS       0x55u

#define DXL_PKT_HDR_SIZE      3u   /* FF FF FD */
#define DXL_PKT_META_SIZE     7u   /* header + reserved + id + len(2) */
#define DXL_STATUS_MIN_LEN    11u  /* status with no params */
#define DXL_PING_RESPONSE_LEN 14u  /* status + model(2) + firmware(1) */
#define DXL_MAX_PACKET_SIZE   128u

typedef struct {
    uint8_t  id;
    uint8_t  instruction;
    uint8_t  error;
    uint8_t  params[256];
    size_t   param_len;
} dxl_status_t;

uint16_t dxl_crc16(const uint8_t *data, size_t len);

size_t dxl_pkt_build_ping(uint8_t *buf, size_t cap, uint8_t id);
size_t dxl_pkt_build_reboot(uint8_t *buf, size_t cap, uint8_t id);
size_t dxl_pkt_build_read(uint8_t *buf, size_t cap, uint8_t id,
                          uint16_t addr, uint16_t len);
size_t dxl_pkt_build_write(uint8_t *buf, size_t cap, uint8_t id,
                           uint16_t addr, const uint8_t *data, uint16_t data_len);

/** Expected status length for a read of @p data_len bytes. */
size_t dxl_pkt_expected_read_rx(uint16_t data_len);

bool dxl_pkt_parse_status(const uint8_t *rx, size_t rx_len, dxl_status_t *out);

#ifdef __cplusplus
}
#endif

#endif /* DXL_PACKET_H */
