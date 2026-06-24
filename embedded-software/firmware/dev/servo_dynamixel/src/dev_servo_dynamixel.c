/**
 * @file    dev_servo_dynamixel.c
 * @brief   Robotis Dynamixel XM430-W210-T driver (Protocol 2.0).
 *
 * UBC Rocket, 2026
 */
#include "dev_servo_dynamixel.h"

#include "dxl_hal.h"
#include "io_sys/io_test_hooks.h"

#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

#define SERVO_ID              1u
#define MODEL_NUMBER_XM430    1030u
#define BUS_TIMEOUT_MS        10u

#define REG_MODEL_NUMBER      0u
#define REG_OPERATING_MODE    11u
#define REG_TORQUE_ENABLE     64u
#define REG_GOAL_POSITION     116u
#define REG_PRESENT_POSITION  132u

#define OP_MODE_POSITION      3u

struct servo_dynamixel {
    bool ready;
};

static struct servo_dynamixel s_self;

IO_TEST_HOOK_RW(s_self, struct servo_dynamixel, dev_servo_dynamixel_self)

static bool write_u8(uint8_t addr, uint8_t value) {
    uint8_t tx[32];
    const size_t n = dxl_pkt_build_write(tx, sizeof(tx), SERVO_ID, addr, &value, 1);
    if (n == 0u) return false;

    uint8_t rx[32];
    dxl_status_t st;
    return dxl_hal_txrx(tx, n, rx, sizeof(rx), &st, BUS_TIMEOUT_MS) == DXL_OK;
}

static bool write_i32(uint8_t addr, int32_t value) {
    uint8_t data[4];
    data[0] = (uint8_t)(value & 0xFF);
    data[1] = (uint8_t)((value >> 8) & 0xFF);
    data[2] = (uint8_t)((value >> 16) & 0xFF);
    data[3] = (uint8_t)((value >> 24) & 0xFF);

    uint8_t tx[32];
    const size_t n = dxl_pkt_build_write(tx, sizeof(tx), SERVO_ID, addr, data, 4);
    if (n == 0u) return false;

    uint8_t rx[32];
    dxl_status_t st;
    return dxl_hal_txrx(tx, n, rx, sizeof(rx), &st, BUS_TIMEOUT_MS) == DXL_OK;
}

static bool read_bytes(uint8_t addr, uint16_t len, uint8_t *out) {
    uint8_t tx[32];
    const size_t n = dxl_pkt_build_read(tx, sizeof(tx), SERVO_ID, addr, len);
    if (n == 0u) return false;

    uint8_t rx[32];
    dxl_status_t st;
    if (dxl_hal_txrx(tx, n, rx, sizeof(rx), &st, BUS_TIMEOUT_MS) != DXL_OK) {
        return false;
    }
    if (st.param_len < len) return false;
    memcpy(out, st.params, len);
    return true;
}

static bool read_u16(uint8_t addr, uint16_t *out) {
    uint8_t data[2];
    if (!read_bytes(addr, 2, data)) return false;
    *out = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    return true;
}

static bool read_i32(uint8_t addr, int32_t *out) {
    uint8_t data[4];
    if (!read_bytes(addr, 4, data)) return false;
    *out = (int32_t)((uint32_t)data[0]
           | ((uint32_t)data[1] << 8)
           | ((uint32_t)data[2] << 16)
           | ((uint32_t)data[3] << 24));
    return true;
}

bool servo_dynamixel_ping(servo_dynamixel_t *d) {
    (void)d;
    uint8_t tx[16];
    const size_t n = dxl_pkt_build_ping(tx, sizeof(tx), SERVO_ID);
    if (n == 0u) return false;

    uint8_t rx[32];
    dxl_status_t st;
    return dxl_hal_txrx(tx, n, rx, sizeof(rx), &st, BUS_TIMEOUT_MS) == DXL_OK;
}

bool servo_dynamixel_init(void) {
    memset(&s_self, 0, sizeof(s_self));

    if (!dxl_hal_init()) return false;
    if (!servo_dynamixel_ping(&s_self)) return false;

    uint16_t model = 0;
    if (!read_u16(REG_MODEL_NUMBER, &model)) return false;
    if (model != MODEL_NUMBER_XM430) return false;

    if (!write_u8(REG_OPERATING_MODE, OP_MODE_POSITION)) return false;
    if (!write_u8(REG_TORQUE_ENABLE, 1u)) return false;

    s_self.ready = true;
    return true;
}

servo_dynamixel_t *servo_dynamixel_get(void) {
    return s_self.ready ? &s_self : NULL;
}

bool servo_dynamixel_set_goal_position(servo_dynamixel_t *d, int32_t pos) {
    if (d == NULL || !d->ready) return false;
    return write_i32(REG_GOAL_POSITION, pos);
}

bool servo_dynamixel_get_present_position(servo_dynamixel_t *d, int32_t *pos) {
    if (d == NULL || !d->ready || pos == NULL) return false;
    return read_i32(REG_PRESENT_POSITION, pos);
}

static void move_and_wait(servo_dynamixel_t *d, int32_t goal, uint32_t hold_ms) {
    (void)servo_dynamixel_set_goal_position(d, goal);
    vTaskDelay(pdMS_TO_TICKS(hold_ms));
    int32_t present = 0;
    (void)servo_dynamixel_get_present_position(d, &present);
    (void)present;
}

void servo_dynamixel_run_position_test(servo_dynamixel_t *d) {
    if (d == NULL || !d->ready) return;

    move_and_wait(d, 2048, 1000);
    move_and_wait(d, 1024, 1000);
    move_and_wait(d, 3072, 1000);
    move_and_wait(d, 2048, 1000);
}
