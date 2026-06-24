/**
 * @file    test_dxl_packet.c
 * @brief   Host unit tests for Dynamixel Protocol 2.0 packet layer.
 */
#include "dynamixel/dxl_packet.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static void test_crc16_ping_golden(void) {
    const uint8_t body[] = {0x01, 0x03, 0x00, 0x01};
    assert(dxl_crc16(body, sizeof(body)) == 0x3C30u);
}

static void test_build_ping(void) {
    uint8_t tx[16];
    const size_t n = dxl_pkt_build_ping(tx, sizeof(tx), 1);
    assert(n == 9);
    assert(tx[0] == 0xFF && tx[1] == 0xFF && tx[2] == 0xFD);
    assert(tx[3] == 1 && tx[4] == 3 && tx[6] == DXL_INST_PING);
    const uint16_t crc = dxl_crc16(&tx[3], 4);
    assert(tx[7] == (uint8_t)(crc & 0xFFu));
    assert(tx[8] == (uint8_t)(crc >> 8));
}

static void test_parse_status_read_response(void) {
    uint8_t rx[32];
    const uint8_t body[] = {0x01, 0x06, 0x00, 0x55, 0x00, 0x06, 0x04};
    const uint16_t crc = dxl_crc16(body, sizeof(body));
    rx[0] = 0xFF; rx[1] = 0xFF; rx[2] = 0xFD;
    memcpy(&rx[3], body, sizeof(body));
    rx[10] = (uint8_t)(crc & 0xFFu);
    rx[11] = (uint8_t)(crc >> 8);
    const size_t total = 3u + 1u + 2u + 6u;

    dxl_status_t st;
    assert(dxl_pkt_parse_status(rx, total, &st));
    assert(st.id == 1);
    assert(st.instruction == DXL_INST_STATUS);
    assert(st.error == 0);
    assert(st.param_len == 2);
    assert(st.params[0] == 0x06 && st.params[1] == 0x04);
}

static void test_build_read(void) {
    uint8_t tx[32];
    const size_t n = dxl_pkt_build_read(tx, sizeof(tx), 1, 132, 4);
    assert(n > 0);
    assert(tx[6] == DXL_INST_READ);
    assert(tx[7] == 132);
    assert(tx[8] == 0);
    assert(tx[9] == 4);
    assert(tx[10] == 0);
}

static void test_build_write_position(void) {
    uint8_t tx[32];
    const uint8_t data[4] = {0x00, 0x08, 0x00, 0x00}; /* 2048 LE */
    const size_t n = dxl_pkt_build_write(tx, sizeof(tx), 1, 116, data, 4);
    assert(n > 0);
    assert(tx[6] == DXL_INST_WRITE);
    assert(tx[7] == 116);
    assert(tx[8] == 0);
    assert(memcmp(&tx[9], data, 4) == 0);
}

int main(void) {
    test_crc16_ping_golden();
    test_build_ping();
    test_parse_status_read_response();
    test_build_read();
    test_build_write_position();
    printf("dynamixel_proto tests passed\n");
    return 0;
}
