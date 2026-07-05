/**
 * @file    dev_servo_dynamixel.c
 * @brief   Robotis Dynamixel XM430-W210-T driver (Protocol 2.0).
 *
 * UBC Rocket, 2026
 */
#include "dev_servo_dynamixel.h"

#include "dxl_hal.h"
#include "io_sys/io_debug.h"
#include "io_sys/io_test_hooks.h"

#include "stm32h7xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

#define SERVO_ID_X            1u
#define SERVO_ID_Y            2u
#define MODEL_NUMBER_XM430    1030u
#define BUS_TIMEOUT_MS        50u
#define SETUP_STEP_MS         50u

#define REG_MODEL_NUMBER            0u
#define REG_OPERATING_MODE          11u
#define REG_TORQUE_ENABLE           64u
#define REG_HARDWARE_ERROR_STATUS   70u
#define REG_GOAL_VELOCITY           104u
#define REG_PRESENT_VELOCITY        128u
#define REG_GOAL_POSITION           116u
#define REG_PRESENT_POSITION        132u

#define OP_MODE_VELOCITY            1u

/* XM430: goal-velocity unit = 0.229 rpm.  30 deg/s ~= 5 rpm ~= 22 units. */
#define GOAL_VEL_30DPS              22
#define SWEEP_HALF_PERIOD_MS        1000u   /* 30 deg at 30 deg/s per half-cycle */
#define REBOOT_DELAY_MS             500u
#define INIT_RETRY_COUNT            3u

#define DXL_TX_BUF_SIZE       DXL_MAX_PACKET_SIZE
#define DXL_RX_BUF_SIZE       DXL_MAX_PACKET_SIZE

struct servo_dynamixel {
    bool ready;
    bool id_x_ready;
    bool id_y_ready;
};

static struct servo_dynamixel s_self;

IO_TEST_HOOK_RW(s_self, struct servo_dynamixel, dev_servo_dynamixel_self)

static dxl_status_code_t bus_txrx(uint8_t id, const uint8_t *tx, size_t tx_len,
                                size_t expected_rx, dxl_status_t *st) {
    (void)id;
    uint8_t rx[DXL_RX_BUF_SIZE];
    memset(st, 0, sizeof(*st));
    return dxl_hal_txrx(tx, tx_len, rx, sizeof(rx), expected_rx, st,
                          BUS_TIMEOUT_MS);
}

static bool bus_ok(dxl_status_code_t rc) {
    return rc == DXL_OK;
}

static bool bus_responded(dxl_status_code_t rc) {
    return rc == DXL_OK || rc == DXL_ERR_STATUS;
}

static bool write_u8(uint8_t id, uint8_t addr, uint8_t value) {
    uint8_t tx[DXL_TX_BUF_SIZE];
    const size_t n = dxl_pkt_build_write(tx, sizeof(tx), id, addr, &value, 1);
    if (n == 0u) return false;

    dxl_status_t st;
    return bus_ok(bus_txrx(id, tx, n, DXL_STATUS_MIN_LEN, &st));
}

static bool write_i32(uint8_t id, uint8_t addr, int32_t value) {
    const uint8_t data[4] = {
        (uint8_t)(value & 0xFF),
        (uint8_t)((value >> 8) & 0xFF),
        (uint8_t)((value >> 16) & 0xFF),
        (uint8_t)((value >> 24) & 0xFF),
    };

    uint8_t tx[DXL_TX_BUF_SIZE];
    const size_t n = dxl_pkt_build_write(tx, sizeof(tx), id, addr, data, 4);
    if (n == 0u) return false;

    dxl_status_t st;
    return bus_ok(bus_txrx(id, tx, n, DXL_STATUS_MIN_LEN, &st));
}

static bool read_bytes(uint8_t id, uint8_t addr, uint16_t len, uint8_t *out) {
    uint8_t tx[DXL_TX_BUF_SIZE];
    const size_t n = dxl_pkt_build_read(tx, sizeof(tx), id, addr, len);
    if (n == 0u) return false;

    dxl_status_t st;
    if (!bus_ok(bus_txrx(id, tx, n, dxl_pkt_expected_read_rx(len), &st))) {
        return false;
    }
    if (st.param_len < len) return false;
    memcpy(out, st.params, len);
    return true;
}

static bool read_u16(uint8_t id, uint8_t addr, uint16_t *out) {
    uint8_t data[2];
    if (!read_bytes(id, addr, 2, data)) return false;
    *out = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    return true;
}

static bool read_i32(uint8_t id, uint8_t addr, int32_t *out) {
    uint8_t data[4];
    if (!read_bytes(id, addr, 4, data)) return false;
    *out = (int32_t)((uint32_t)data[0]
           | ((uint32_t)data[1] << 8)
           | ((uint32_t)data[2] << 16)
           | ((uint32_t)data[3] << 24));
    return true;
}

static void log_hw_error(uint8_t id) {
    uint8_t hw = 0;
    if (!read_bytes(id, REG_HARDWARE_ERROR_STATUS, 1, &hw)) {
        io_debug_printf("DXL id%u hw_err=READ FAIL\r\n", (unsigned)id);
        return;
    }
    io_debug_printf("DXL id%u hw_err=0x%02x\r\n", (unsigned)id, (unsigned)hw);
}

static void reboot_id(uint8_t id) {
    uint8_t tx[DXL_TX_BUF_SIZE];
    const size_t n = dxl_pkt_build_reboot(tx, sizeof(tx), id);
    if (n == 0u) {
        return;
    }

    dxl_status_t st;
    (void)bus_txrx(id, tx, n, DXL_STATUS_MIN_LEN, &st);
    HAL_Delay(REBOOT_DELAY_MS);
}

static bool ping_id(uint8_t id, uint8_t *pkt_error) {
    uint8_t tx[DXL_TX_BUF_SIZE];
    const size_t n = dxl_pkt_build_ping(tx, sizeof(tx), id);
    if (n == 0u) return false;

    dxl_status_t st;
    const dxl_status_code_t rc =
        bus_txrx(id, tx, n, DXL_PING_RESPONSE_LEN, &st);
    if (!bus_responded(rc)) {
        return false;
    }
    if (pkt_error != NULL) {
        *pkt_error = st.error;
    }
    return true;
}

static bool setup_velocity_mode(uint8_t id) {
    if (!write_u8(id, REG_TORQUE_ENABLE, 0u)) return false;
    HAL_Delay(SETUP_STEP_MS);
    if (!write_u8(id, REG_OPERATING_MODE, OP_MODE_VELOCITY)) return false;
    HAL_Delay(SETUP_STEP_MS);
    if (!write_i32(id, REG_GOAL_VELOCITY, 0)) return false;
    HAL_Delay(SETUP_STEP_MS);
    if (!write_u8(id, REG_TORQUE_ENABLE, 1u)) return false;
    HAL_Delay(SETUP_STEP_MS);
    return true;
}

static bool init_servo(uint8_t id) {
    for (unsigned attempt = 0; attempt < INIT_RETRY_COUNT; ++attempt) {
        uint8_t pkt_error = 0;
        if (!ping_id(id, &pkt_error)) {
            HAL_Delay(SETUP_STEP_MS);
            continue;
        }

        if (pkt_error != 0u) {
            io_debug_printf("DXL id%u pkt_err=0x%02x, rebooting\r\n",
                            (unsigned)id, (unsigned)pkt_error);
            log_hw_error(id);
            reboot_id(id);
            continue;
        }

        uint16_t model = 0;
        if (!read_u16(id, REG_MODEL_NUMBER, &model)) {
            reboot_id(id);
            continue;
        }
        if (model != MODEL_NUMBER_XM430) {
            io_debug_printf("DXL id%u model=%u (expected %u)\r\n",
                            (unsigned)id, (unsigned)model,
                            (unsigned)MODEL_NUMBER_XM430);
            return false;
        }

        if (setup_velocity_mode(id)) {
            return true;
        }

        log_hw_error(id);
        reboot_id(id);
    }

    return false;
}

static bool try_recover_servo(servo_dynamixel_t *d, uint8_t id, bool *ready) {
    if (*ready) {
        return true;
    }

    io_debug_printf("DXL: retry init id%u\r\n", (unsigned)id);
    *ready = init_servo(id);
    if (*ready) {
        io_debug_printf("DXL: id%u recovered\r\n", (unsigned)id);
    } else {
        log_hw_error(id);
    }
    return *ready;
}

bool servo_dynamixel_ping(servo_dynamixel_t *d) {
    (void)d;
    return ping_id(SERVO_ID_X, NULL) && ping_id(SERVO_ID_Y, NULL);
}

bool servo_dynamixel_init(void) {
    memset(&s_self, 0, sizeof(s_self));

    if (!dxl_hal_init()) {
        return false;
    }

    s_self.id_x_ready = init_servo(SERVO_ID_X);
    s_self.id_y_ready = init_servo(SERVO_ID_Y);

    if (!s_self.id_x_ready && !s_self.id_y_ready) {
        return false;
    }

    s_self.ready = true;
    return true;
}

void servo_dynamixel_log_status(void) {
    io_debug_printf("[init] DXL bus %s @ %lu baud\r\n",
                    dxl_hal_baud() ? "up" : "down",
                    (unsigned long)dxl_hal_baud());

    if (!s_self.ready) {
        io_debug_printf("[init] DXL servos FAIL (no response on id %u/%u)\r\n",
                        (unsigned)SERVO_ID_X, (unsigned)SERVO_ID_Y);
        return;
    }

    io_debug_printf("[init] DXL id%u=%s id%u=%s\r\n",
                    (unsigned)SERVO_ID_X, s_self.id_x_ready ? "OK" : "FAIL",
                    (unsigned)SERVO_ID_Y, s_self.id_y_ready ? "OK" : "FAIL");

    if (!s_self.id_x_ready) {
        log_hw_error(SERVO_ID_X);
    }
    if (!s_self.id_y_ready) {
        log_hw_error(SERVO_ID_Y);
    }
}

servo_dynamixel_t *servo_dynamixel_get(void) {
    return s_self.ready ? &s_self : NULL;
}

bool servo_dynamixel_set_goal_position(servo_dynamixel_t *d, uint8_t id, int32_t pos) {
    if (d == NULL || !d->ready) return false;
    if (id == SERVO_ID_X && !d->id_x_ready) return false;
    if (id == SERVO_ID_Y && !d->id_y_ready) return false;
    if (id != SERVO_ID_X && id != SERVO_ID_Y) return false;
    return write_i32(id, REG_GOAL_POSITION, pos);
}

bool servo_dynamixel_set_pair_goal_position(servo_dynamixel_t *d, int32_t pos) {
    if (d == NULL || !d->ready) return false;
    bool ok = true;
    if (d->id_x_ready) {
        ok = write_i32(SERVO_ID_X, REG_GOAL_POSITION, pos) && ok;
    }
    if (d->id_y_ready) {
        ok = write_i32(SERVO_ID_Y, REG_GOAL_POSITION, pos) && ok;
    }
    return ok;
}

bool servo_dynamixel_get_present_position(servo_dynamixel_t *d, uint8_t id, int32_t *pos) {
    if (d == NULL || !d->ready || pos == NULL) return false;
    if (id == SERVO_ID_X && !d->id_x_ready) return false;
    if (id == SERVO_ID_Y && !d->id_y_ready) return false;
    if (id != SERVO_ID_X && id != SERVO_ID_Y) return false;
    return read_i32(id, REG_PRESENT_POSITION, pos);
}

static bool set_pair_goal_velocity(servo_dynamixel_t *d, int32_t vel) {
    if (d == NULL || !d->ready) return false;
    bool ok = true;
    if (d->id_x_ready) {
        ok = write_i32(SERVO_ID_X, REG_GOAL_VELOCITY, vel) && ok;
    }
    if (d->id_y_ready) {
        ok = write_i32(SERVO_ID_Y, REG_GOAL_VELOCITY, vel) && ok;
    }
    return ok;
}

static void vel_pair_and_wait(servo_dynamixel_t *d, int32_t goal_vel, uint32_t hold_ms) {
    (void)set_pair_goal_velocity(d, goal_vel);
    vTaskDelay(pdMS_TO_TICKS(hold_ms));

    int32_t vel_x = 0;
    int32_t vel_y = 0;
    const bool ok_x = d->id_x_ready && read_i32(SERVO_ID_X, REG_PRESENT_VELOCITY, &vel_x);
    const bool ok_y = d->id_y_ready && read_i32(SERVO_ID_Y, REG_PRESENT_VELOCITY, &vel_y);

    io_debug_printf("DXL test: vel=%4ld id1=%s%4ld id2=%s%4ld\r\n",
                    (long)goal_vel,
                    ok_x ? "" : "FAIL/",
                    ok_x ? (long)vel_x : 0L,
                    ok_y ? "" : "FAIL/",
                    ok_y ? (long)vel_y : 0L);
}

void servo_dynamixel_run_position_test(servo_dynamixel_t *d) {
    if (d == NULL || !d->ready) {
        io_debug_printf("DXL test: skipped (servos not ready)\r\n");
        return;
    }

    io_debug_printf("DXL test: velocity +/-30deg/s (id%u=%s id%u=%s)\r\n",
                    (unsigned)SERVO_ID_X, d->id_x_ready ? "on" : "off",
                    (unsigned)SERVO_ID_Y, d->id_y_ready ? "on" : "off");

    for (;;) {
        (void)try_recover_servo(d, SERVO_ID_X, &d->id_x_ready);
        (void)try_recover_servo(d, SERVO_ID_Y, &d->id_y_ready);

        vel_pair_and_wait(d, GOAL_VEL_30DPS, SWEEP_HALF_PERIOD_MS);
        vel_pair_and_wait(d, -GOAL_VEL_30DPS, SWEEP_HALF_PERIOD_MS);
    }
}
