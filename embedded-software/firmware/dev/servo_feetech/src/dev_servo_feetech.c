/**
 * @file    dev_servo_feetech.c
 * @brief   Feetech STS/SCS driver. Opaque singleton; mechanical config baked in.
 *
 * UBC Rocket, 2026
 */
#include "dev_servo_feetech.h"

#include "io/io_uart.h"
#include "io_sys/io_test_hooks.h"

#include <string.h>

#define SERVO_ID_X     1
#define SERVO_ID_Y     2
#define POS_MIN        1024u
#define POS_MID        2048u
#define POS_MAX        3072u
#define DEG_RANGE      90.0f
#define BUS_TIMEOUT_MS 5u

#define FT_INST_PING        0x01
#define FT_INST_READ_DATA   0x02
#define FT_INST_WRITE_DATA  0x03

#define FT_REG_TORQUE_ENABLE  0x28
#define FT_REG_GOAL_POSITION  0x2A
#define FT_REG_PRESENT_POS    0x38

struct servo_feetech {
    bool enabled;
    bool ready;
};

static struct servo_feetech s_self;

/* Test-only access to module-private state. No-op in production builds. */
IO_TEST_HOOK_RW(s_self, struct servo_feetech, dev_servo_feetech_self)

static uint16_t deg_to_pos(float deg) {
    const float half = DEG_RANGE * 0.5f;
    if (deg >  half) deg =  half;
    if (deg < -half) deg = -half;
    if (deg >= 0.0f) {
        float t = deg / half;
        return (uint16_t)(POS_MID + t * (POS_MAX - POS_MID));
    } else {
        float t = -deg / half;
        return (uint16_t)(POS_MID - t * (POS_MID - POS_MIN));
    }
}

static float pos_to_deg(uint16_t pos) {
    if (pos >= POS_MID) {
        float t = (float)(pos - POS_MID) / (float)(POS_MAX - POS_MID);
        return t * (DEG_RANGE * 0.5f);
    } else {
        float t = (float)(POS_MID - pos) / (float)(POS_MID - POS_MIN);
        return -t * (DEG_RANGE * 0.5f);
    }
}

static uint8_t checksum(const uint8_t *p, size_t len) {
    uint16_t s = 0;
    for (size_t i = 0; i < len; ++i) s += p[i];
    return (uint8_t)(~s & 0xFF);
}

static bool write_pos(uint8_t id, uint16_t pos) {
    uint8_t pkt[10];
    pkt[0] = 0xFF; pkt[1] = 0xFF;
    pkt[2] = id;     pkt[3] = 5;
    pkt[4] = FT_INST_WRITE_DATA;
    pkt[5] = FT_REG_GOAL_POSITION;
    pkt[6] = (uint8_t)(pos & 0xFF);
    pkt[7] = (uint8_t)(pos >> 8);
    pkt[8] = checksum(&pkt[2], 6);
    uint8_t resp[6];
    return io_uart_xfer(&IO_UART_SERVO_BUS, pkt, 9, resp, sizeof(resp), BUS_TIMEOUT_MS) == IO_OK;
}

static bool write_torque(uint8_t id, bool enable) {
    uint8_t pkt[9];
    pkt[0] = 0xFF; pkt[1] = 0xFF;
    pkt[2] = id;     pkt[3] = 4;
    pkt[4] = FT_INST_WRITE_DATA;
    pkt[5] = FT_REG_TORQUE_ENABLE;
    pkt[6] = enable ? 1 : 0;
    pkt[7] = checksum(&pkt[2], 5);
    uint8_t resp[6];
    return io_uart_xfer(&IO_UART_SERVO_BUS, pkt, 8, resp, sizeof(resp), BUS_TIMEOUT_MS) == IO_OK;
}

static bool read_pos(uint8_t id, uint16_t *out) {
    uint8_t pkt[8];
    pkt[0] = 0xFF; pkt[1] = 0xFF;
    pkt[2] = id;     pkt[3] = 4;
    pkt[4] = FT_INST_READ_DATA;
    pkt[5] = FT_REG_PRESENT_POS;
    pkt[6] = 2;
    pkt[7] = checksum(&pkt[2], 5);
    uint8_t resp[8];
    if (io_uart_xfer(&IO_UART_SERVO_BUS, pkt, 8, resp, sizeof(resp), BUS_TIMEOUT_MS) != IO_OK) return false;
    if (resp[0] != 0xFF || resp[1] != 0xFF || resp[2] != id) return false;
    *out = (uint16_t)resp[5] | ((uint16_t)resp[6] << 8);
    return true;
}

bool servo_feetech_init(void) {
    memset(&s_self, 0, sizeof(s_self));
    write_torque(SERVO_ID_X, true);
    write_torque(SERVO_ID_Y, true);
    s_self.enabled = true;
    s_self.ready   = true;
    return true;
}

servo_feetech_t *servo_feetech_get(void) {
    return s_self.ready ? &s_self : NULL;
}

/* Fire-and-forget write-position packet. Identical wire format to
 * write_pos() but uses io_uart_send_async — DMA-driven, no ACK read,
 * ISR-safe. The Feetech protocol allows the response to be discarded;
 * if you need to verify a servo is alive, use servo_feetech_get_pair_
 * degrees from a low-rate task (that one still does a verified xfer). */
static io_status_t write_pos_async(uint8_t id, uint16_t pos,
                                    uint8_t pkt[/*9*/]) {
    pkt[0] = 0xFF; pkt[1] = 0xFF;
    pkt[2] = id;     pkt[3] = 5;
    pkt[4] = FT_INST_WRITE_DATA;
    pkt[5] = FT_REG_GOAL_POSITION;
    pkt[6] = (uint8_t)(pos & 0xFF);
    pkt[7] = (uint8_t)(pos >> 8);
    pkt[8] = checksum(&pkt[2], 6);
    return io_uart_send_async(&IO_UART_SERVO_BUS, pkt, 9);
}

/* Alternating-axis state for ISR-driven updates. Two packet buffers so
 * the next cycle's kick has its own memory and never races a DMA still
 * reading the previous packet (DMA on 9 bytes at 1 Mbps takes ~80 µs;
 * ISR cycles are 1.25 ms so DMA is always done long before reuse — the
 * separate buffers just remove the timing assumption). */
typedef struct {
    uint8_t pkt_x[9];
    uint8_t pkt_y[9];
    bool    next_is_x;
} servo_pair_state_t;

static servo_pair_state_t s_pair = { .next_is_x = true };
IO_TEST_HOOK_RW(s_pair, servo_pair_state_t, dev_servo_feetech_pair_state)

/* Called from the 800 Hz controls ISR. Each call ships ONE servo
 * update over the half-duplex bus and alternates which axis goes
 * next cycle — X at 400 Hz, Y at 400 Hz. Caller passes the latest
 * target for both axes each cycle; only the alternator's pick is
 * actually shipped, so the un-shipped axis just waits one cycle for
 * its turn (no stale-command buildup — the LATEST value is used).
 *
 * 400 Hz per servo is ~8× the mechanical bandwidth of Feetech STS
 * servos; the rate decimation has no observable tracking effect.
 *
 * ISR cost: one deg-to-pos conversion + one packet build + one
 * io_uart_send_async kick ≈ 5 µs total. */
void servo_feetech_set_pair_degrees(servo_feetech_t *d, float deg_x, float deg_y) {
    if (!d->enabled) return;

    uint8_t  id;
    uint8_t *pkt;
    uint16_t pos;
    if (s_pair.next_is_x) {
        id  = SERVO_ID_X;
        pkt = s_pair.pkt_x;
        pos = deg_to_pos(deg_x);
    } else {
        id  = SERVO_ID_Y;
        pkt = s_pair.pkt_y;
        pos = deg_to_pos(deg_y);
    }
    /* Fire-and-forget. IO_ERR_BUSY (previous DMA still in flight) just
     * means we drop this cycle; the next ISR will try the other axis
     * with a fresh target. Should never trip at 800 Hz unless something
     * else is hogging the UART. */
    (void)write_pos_async(id, pos, pkt);
    s_pair.next_is_x = !s_pair.next_is_x;
}

void servo_feetech_enable(servo_feetech_t *d, bool enable) {
    d->enabled = enable;
    write_torque(SERVO_ID_X, enable);
    write_torque(SERVO_ID_Y, enable);
}

bool servo_feetech_get_pair_degrees(servo_feetech_t *d, float *deg_x_out, float *deg_y_out) {
    (void)d;
    uint16_t px, py;
    if (!read_pos(SERVO_ID_X, &px)) return false;
    if (!read_pos(SERVO_ID_Y, &py)) return false;
    if (deg_x_out) *deg_x_out = pos_to_deg(px);
    if (deg_y_out) *deg_y_out = pos_to_deg(py);
    return true;
}
