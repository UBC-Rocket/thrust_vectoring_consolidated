/**
 * @file    dxl_packet.h
 * @brief   Dynamixel Protocol 2.0 packet encode/decode (pure C, no HAL).
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

#define DXL_HDR0           0xFFu
#define DXL_HDR1           0xFFu
#define DXL_HDR2           0xFDu
#define DXL_BROADCAST_ID   0xFEu

#define DXL_INST_PING      0x01u
#define DXL_INST_READ      0x02u
#define DXL_INST_WRITE     0x03u
#define DXL_INST_STATUS    0x55u

#define DXL_PKT_HDR_SIZE   3u
#define DXL_PKT_MIN_SIZE   7u   /* header + id + len(2) + inst */

typedef struct {
    uint8_t  id;
    uint8_t  instruction;
    uint8_t  error;
    uint8_t  params[256];
    size_t   param_len;
} dxl_status_t;

uint16_t dxl_crc16_update(uint16_t crc, uint8_t byte);
uint16_t dxl_crc16(const uint8_t *data, size_t len);

size_t dxl_pkt_build_ping(uint8_t *buf, size_t cap, uint8_t id);
size_t dxl_pkt_build_read(uint8_t *buf, size_t cap, uint8_t id,
                          uint16_t addr, uint16_t len);
size_t dxl_pkt_build_write(uint8_t *buf, size_t cap, uint8_t id,
                           uint16_t addr, const uint8_t *data, uint16_t data_len);

bool dxl_pkt_parse_status(const uint8_t *rx, size_t rx_len, dxl_status_t *out);

#ifdef __cplusplus
}
#endif

#endif /* DXL_PACKET_H */
