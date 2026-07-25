/**
 * @file    test_dxl_packet.c
 * @brief   Host unit tests for Dynamixel Protocol 2.0 packet layer.
 */
#include "dynamixel/dxl_packet.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static void test_build_ping(void) {
    uint8_t tx[16];
    const size_t n = dxl_pkt_build_ping(tx, sizeof(tx), 1);
    assert(n == 10);
    assert(tx[0] == 0xFF && tx[1] == 0xFF && tx[2] == 0xFD && tx[3] == 0x00);
    assert(tx[4] == 1 && tx[5] == 3 && tx[7] == DXL_INST_PING);
}

static void test_parse_status_read_response(void) {
    uint8_t pkt[32];
    pkt[0] = 0xFF; pkt[1] = 0xFF; pkt[2] = 0xFD; pkt[3] = 0x00;
    pkt[4] = 0x01; pkt[5] = 0x06; pkt[6] = 0x00;
    pkt[7] = 0x55; pkt[8] = 0x00; pkt[9] = 0x06; pkt[10] = 0x04;
    const uint16_t crc = dxl_crc16(pkt, 11u);
    pkt[11] = (uint8_t)(crc & 0xFFu);
    pkt[12] = (uint8_t)(crc >> 8);
    const size_t total = 13u;

    dxl_status_t st;
    assert(dxl_pkt_parse_status(pkt, total, &st));
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
    assert(tx[3] == 0x00);
    assert(tx[7] == DXL_INST_READ);
    assert(tx[8] == 132);
    assert(tx[9] == 0);
    assert(tx[10] == 4);
    assert(tx[11] == 0);
}

static void test_build_write_position(void) {
    uint8_t tx[32];
    const uint8_t data[4] = {0x00, 0x08, 0x00, 0x00};
    const size_t n = dxl_pkt_build_write(tx, sizeof(tx), 1, 116, data, 4);
    assert(n > 0);
    assert(tx[7] == DXL_INST_WRITE);
    assert(tx[8] == 116);
    assert(tx[9] == 0);
    assert(memcmp(&tx[10], data, 4) == 0);
}

static void test_expected_read_rx(void) {
    assert(dxl_pkt_expected_read_rx(2) == 13u);
    assert(dxl_pkt_expected_read_rx(4) == 15u);
}

int main(void) {
    test_build_ping();
    test_parse_status_read_response();
    test_build_read();
    test_build_write_position();
    test_expected_read_rx();
    printf("dynamixel_proto tests passed\n");
    return 0;
}
