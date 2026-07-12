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

#include <math.h>
#include <string.h>

#define SERVO_ID_X            1u
#define SERVO_ID_Y            2u
#define MODEL_NUMBER_XM430    1030u
#define BUS_TIMEOUT_MS        20u
#define SETUP_STEP_MS         50u

#define REG_MODEL_NUMBER            0u
#define REG_OPERATING_MODE          11u
#define REG_TORQUE_ENABLE           64u
#define REG_HARDWARE_ERROR_STATUS   70u
#define REG_GOAL_VELOCITY           104u
#define REG_PRESENT_VELOCITY        128u
#define REG_PROFILE_ACCELERATION    108u
#define REG_PROFILE_VELOCITY        112u
#define REG_GOAL_POSITION           116u
#define REG_PRESENT_POSITION        132u

#define DXL_PROFILE_VELOCITY        200u   /* ~46 rpm; 0 = unlimited on XM430 */
#define DXL_PROFILE_ACCELERATION    75u    /* was 50; small bump for snappier moves */

#define OP_MODE_POSITION            3u
#define OP_MODE_EXTENDED_POSITION   4u

#define DXL_TICKS_PER_REV           4096
#define DXL_TEST_SWEEP_RANGE_DEG    90.0f
#define DXL_TEST_SWEEP_STEP_DEG     1.0f
#define DXL_TEST_SWEEP_STEP_MS      5U
#define DXL_TEST_CIRCLE_RADIUS_DEG  5.0f
#define DXL_TEST_CIRCLE_STEPS       72U
#define DXL_TEST_CIRCLE_STEP_MS     18U
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

static void reboot_id(uint8_t id);
static bool dxl_clear_hw_error_if_set(uint8_t id);
static void log_hw_error(uint8_t id);

static dxl_status_code_t bus_txrx(uint8_t id, const uint8_t *tx, size_t tx_len,
                                size_t expected_rx, dxl_status_t *st) {
    (void)id;
    uint8_t rx[DXL_RX_BUF_SIZE];
    memset(st, 0, sizeof(*st));
    return dxl_hal_txrx(tx, tx_len, rx, sizeof(rx), expected_rx, st,
                          BUS_TIMEOUT_MS);
}

void servo_dynamixel_bus_recover(void)
{
    dxl_hal_recover();
}

static bool bus_ok(dxl_status_code_t rc) {
    return rc == DXL_OK;
}

static bool bus_responded(dxl_status_code_t rc) {
    return rc == DXL_OK || rc == DXL_ERR_STATUS;
}

static void reboot_delay_ms(uint32_t ms)
{
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        vTaskDelay(pdMS_TO_TICKS(ms));
    } else {
        HAL_Delay(ms);
    }
}

static bool write_u8_inner(uint8_t id, uint8_t addr, uint8_t value, unsigned retries_left)
{
    uint8_t tx[DXL_TX_BUF_SIZE];
    const size_t n = dxl_pkt_build_write(tx, sizeof(tx), id, addr, &value, 1);
    if (n == 0u) return false;

    dxl_status_t st;
    const dxl_status_code_t rc = bus_txrx(id, tx, n, DXL_STATUS_MIN_LEN, &st);
    if (bus_ok(rc)) {
        return true;
    }
    if (retries_left > 0u && dxl_clear_hw_error_if_set(id)) {
        return write_u8_inner(id, addr, value, retries_left - 1u);
    }
    return false;
}

static bool write_u8(uint8_t id, uint8_t addr, uint8_t value) {
    return write_u8_inner(id, addr, value, 1u);
}

static bool write_i32_inner(uint8_t id, uint8_t addr, int32_t value, unsigned retries_left)
{
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
    const dxl_status_code_t rc = bus_txrx(id, tx, n, DXL_STATUS_MIN_LEN, &st);
    if (bus_ok(rc)) {
        return true;
    }
    if (retries_left > 0u && dxl_clear_hw_error_if_set(id)) {
        return write_i32_inner(id, addr, value, retries_left - 1u);
    }
    return false;
}

static bool write_i32(uint8_t id, uint8_t addr, int32_t value) {
    return write_i32_inner(id, addr, value, 1u);
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

/* Hardware Error Status (addr 70) is read-only; reboot clears latched faults. */
static bool read_hw_error(uint8_t id, uint8_t *hw_out)
{
    return read_bytes(id, REG_HARDWARE_ERROR_STATUS, 1, hw_out);
}

static bool dxl_clear_hw_error_if_set(uint8_t id)
{
    uint8_t hw = 0;
    if (!read_hw_error(id, &hw)) {
        return false;
    }
    if (hw == 0u) {
        return false;
    }

    io_debug_printf("DXL id%u hw_err=0x%02x, rebooting to clear\r\n",
                    (unsigned)id, (unsigned)hw);
    reboot_id(id);
    return true;
}

static void log_hw_error(uint8_t id) {
    uint8_t hw = 0;
    if (!read_hw_error(id, &hw)) {
        io_debug_printf("DXL id%u hw_err=READ FAIL\r\n", (unsigned)id);
        return;
    }
    io_debug_printf("DXL id%u hw_err=0x%02x\r\n", (unsigned)id, (unsigned)hw);
    if (hw != 0u) {
        reboot_id(id);
    }
}

static void reboot_id(uint8_t id) {
    uint8_t tx[DXL_TX_BUF_SIZE];
    const size_t n = dxl_pkt_build_reboot(tx, sizeof(tx), id);
    if (n == 0u) {
        return;
    }

    dxl_status_t st;
    (void)bus_txrx(id, tx, n, DXL_STATUS_MIN_LEN, &st);
    reboot_delay_ms(REBOOT_DELAY_MS);
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

static int32_t zero_ticks_for_id(uint8_t id)
{
    if (id == SERVO_ID_Y) {
        return DXL_ZERO_TICKS_Y;
    }
    return DXL_ZERO_TICKS_X;
}

static int32_t deg_to_ticks_x(float deg)
{
    return DXL_ZERO_TICKS_X +
           (int32_t)(deg * (float)DXL_TICKS_PER_REV / 360.0f);
}

static int32_t deg_to_ticks_y(float deg)
{
    return DXL_ZERO_TICKS_Y +
           (int32_t)(deg * (float)DXL_TICKS_PER_REV / 360.0f);
}

static float ticks_to_deg_x(int32_t ticks)
{
    return (float)(ticks - DXL_ZERO_TICKS_X) * 360.0f / (float)DXL_TICKS_PER_REV;
}

static float ticks_to_deg_y(int32_t ticks)
{
    return (float)(ticks - DXL_ZERO_TICKS_Y) * 360.0f / (float)DXL_TICKS_PER_REV;
}

static bool setup_position_mode(uint8_t id)
{
    if (!write_u8(id, REG_TORQUE_ENABLE, 0u)) return false;
    HAL_Delay(SETUP_STEP_MS);
    /* Mode 4 (extended) accepts negative goal ticks — required for calibrated
     * zero on Y (DXL_ZERO_TICKS_Y = -14). Mode 3 only allows 0..4095. */
    if (!write_u8(id, REG_OPERATING_MODE, OP_MODE_EXTENDED_POSITION)) return false;
    HAL_Delay(SETUP_STEP_MS);
    if (!write_i32(id, REG_PROFILE_VELOCITY, (int32_t)DXL_PROFILE_VELOCITY)) return false;
    HAL_Delay(SETUP_STEP_MS);
    if (!write_i32(id, REG_PROFILE_ACCELERATION, (int32_t)DXL_PROFILE_ACCELERATION)) return false;
    HAL_Delay(SETUP_STEP_MS);
    if (!write_i32(id, REG_GOAL_POSITION, zero_ticks_for_id(id))) return false;
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

        if (dxl_clear_hw_error_if_set(id)) {
            continue;
        }

        uint16_t model = 0;
        if (!read_u16(id, REG_MODEL_NUMBER, &model)) {
            if (dxl_clear_hw_error_if_set(id)) {
                continue;
            }
            reboot_id(id);
            continue;
        }
        if (model != MODEL_NUMBER_XM430) {
            io_debug_printf("DXL id%u model=%u (expected %u)\r\n",
                            (unsigned)id, (unsigned)model,
                            (unsigned)MODEL_NUMBER_XM430);
            return false;
        }

        if (setup_position_mode(id)) {
            return true;
        }

        log_hw_error(id);
        if (!dxl_clear_hw_error_if_set(id)) {
            reboot_id(id);
        }
    }

    return false;
}

static void init_servo_retry_if_failed(uint8_t id, bool *ready)
{
    if (*ready) {
        return;
    }

    if (ping_id(id, NULL)) {
        (void)dxl_clear_hw_error_if_set(id);
    }
    *ready = init_servo(id);
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

    init_servo_retry_if_failed(SERVO_ID_X, &s_self.id_x_ready);
    init_servo_retry_if_failed(SERVO_ID_Y, &s_self.id_y_ready);

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

    if (!s_self.id_x_ready) {
        log_hw_error(SERVO_ID_X);
        (void)try_recover_servo(&s_self, SERVO_ID_X, &s_self.id_x_ready);
    }
    if (!s_self.id_y_ready) {
        log_hw_error(SERVO_ID_Y);
        (void)try_recover_servo(&s_self, SERVO_ID_Y, &s_self.id_y_ready);
    }

    io_debug_printf("[init] DXL id%u=%s id%u=%s\r\n",
                    (unsigned)SERVO_ID_X, s_self.id_x_ready ? "OK" : "FAIL",
                    (unsigned)SERVO_ID_Y, s_self.id_y_ready ? "OK" : "FAIL");
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

static void bus_inter_gap_ms(void)
{
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        vTaskDelay(pdMS_TO_TICKS(3));
    } else {
        HAL_Delay(3);
    }
}

bool servo_dynamixel_set_pair_degrees(servo_dynamixel_t *d, float deg_x, float deg_y)
{
    if (d == NULL || !d->ready) {
        return false;
    }

    const int32_t ticks_x = deg_to_ticks_x(deg_x);
    const int32_t ticks_y = deg_to_ticks_y(deg_y);
    bool ok = true;

    if (d->id_x_ready) {
        ok = write_i32(SERVO_ID_X, REG_GOAL_POSITION, ticks_x) && ok;
    }
    if (d->id_y_ready) {
        if (d->id_x_ready) {
            bus_inter_gap_ms();
        }
        ok = write_i32(SERVO_ID_Y, REG_GOAL_POSITION, ticks_y) && ok;
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

bool servo_dynamixel_get_pair_degrees(servo_dynamixel_t *d, float *deg_x, float *deg_y)
{
    if (d == NULL || !d->ready) {
        return false;
    }

    int32_t tx = 0;
    int32_t ty = 0;
    bool ok = true;

    if (d->id_x_ready) {
        ok = read_i32(SERVO_ID_X, REG_PRESENT_POSITION, &tx) && ok;
        if (deg_x != NULL) {
            *deg_x = ticks_to_deg_x(tx);
        }
    } else if (deg_x != NULL) {
        *deg_x = 0.0f;
    }

    if (d->id_y_ready) {
        if (d->id_x_ready) {
            bus_inter_gap_ms();
        }
        ok = read_i32(SERVO_ID_Y, REG_PRESENT_POSITION, &ty) && ok;
        if (deg_y != NULL) {
            *deg_y = ticks_to_deg_y(ty);
        }
    } else if (deg_y != NULL) {
        *deg_y = 0.0f;
    }

    return ok;
}

bool servo_dynamixel_set_torque_enable(servo_dynamixel_t *d, uint8_t id, bool enable) {
    if (d == NULL || !d->ready) return false;
    if (id == SERVO_ID_X && !d->id_x_ready) return false;
    if (id == SERVO_ID_Y && !d->id_y_ready) return false;
    if (id != SERVO_ID_X && id != SERVO_ID_Y) return false;
    return write_u8(id, REG_TORQUE_ENABLE, enable ? 1u : 0u);
}

bool servo_dynamixel_disable_torque_pair(servo_dynamixel_t *d) {
    if (d == NULL || !d->ready) return false;
    bool ok = true;
    if (d->id_x_ready) {
        ok = servo_dynamixel_set_torque_enable(d, SERVO_ID_X, false) && ok;
    }
    if (d->id_y_ready) {
        ok = servo_dynamixel_set_torque_enable(d, SERVO_ID_Y, false) && ok;
    }
    return ok;
}

void servo_dynamixel_run_position_test(servo_dynamixel_t *d) {
    if (d == NULL || !d->ready) {
        io_debug_printf("DXL test: skipped (servos not ready)\r\n");
        return;
    }

    io_debug_printf("DXL test: position sweep (id%u=%s id%u=%s)\r\n",
                    (unsigned)SERVO_ID_X, d->id_x_ready ? "on" : "off",
                    (unsigned)SERVO_ID_Y, d->id_y_ready ? "on" : "off");

    (void)try_recover_servo(d, SERVO_ID_X, &d->id_x_ready);
    (void)try_recover_servo(d, SERVO_ID_Y, &d->id_y_ready);

    servo_dynamixel_set_pair_degrees(d, 0.0f, 0.0f);
    vTaskDelay(pdMS_TO_TICKS(100));

    for (float deg = 0.0f; deg <= DXL_TEST_SWEEP_RANGE_DEG;
         deg += DXL_TEST_SWEEP_STEP_DEG) {
        servo_dynamixel_set_pair_degrees(d, deg, deg);
        vTaskDelay(pdMS_TO_TICKS(DXL_TEST_SWEEP_STEP_MS));
    }
    for (float deg = DXL_TEST_SWEEP_RANGE_DEG;
         deg >= -DXL_TEST_SWEEP_RANGE_DEG;
         deg -= DXL_TEST_SWEEP_STEP_DEG) {
        servo_dynamixel_set_pair_degrees(d, deg, deg);
        vTaskDelay(pdMS_TO_TICKS(DXL_TEST_SWEEP_STEP_MS));
    }
    for (float deg = -DXL_TEST_SWEEP_RANGE_DEG; deg <= 0.0f;
         deg += DXL_TEST_SWEEP_STEP_DEG) {
        servo_dynamixel_set_pair_degrees(d, deg, deg);
        vTaskDelay(pdMS_TO_TICKS(DXL_TEST_SWEEP_STEP_MS));
    }

    for (unsigned i = 0; i < DXL_TEST_CIRCLE_STEPS; ++i) {
        const float ang = (2.0f * (float)M_PI * (float)i) /
                          (float)DXL_TEST_CIRCLE_STEPS;
        const float dx = DXL_TEST_CIRCLE_RADIUS_DEG * cosf(ang);
        const float dy = DXL_TEST_CIRCLE_RADIUS_DEG * sinf(ang);
        servo_dynamixel_set_pair_degrees(d, dx, dy);
        vTaskDelay(pdMS_TO_TICKS(DXL_TEST_CIRCLE_STEP_MS));
    }

    servo_dynamixel_set_pair_degrees(d, 0.0f, 0.0f);
}
