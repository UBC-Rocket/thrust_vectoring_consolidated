/**
 * @file    dev_esc_dshot.c
 * @brief   DShot ESC driver. Opaque singleton.
 *
 * UBC Rocket, 2026
 */
#include "dev_esc_dshot.h"

#include "io/io_dshot.h"
#include "io_sys/io_test_hooks.h"

#include <string.h>

#define DSHOT_THROTTLE_MIN   48
#define DSHOT_THROTTLE_MAX   2047
#define DSHOT_CMD_MOTOR_STOP 0

struct esc_dshot {
    bool armed;
    bool ready;
};

static struct esc_dshot s_self;

/* Test-only access to module-private state. No-op in production builds. */
IO_TEST_HOOK_RW(s_self, struct esc_dshot, dev_esc_dshot_self)

static uint16_t dshot_pack(uint16_t value, bool telemetry) {
    uint16_t packet = (uint16_t)(((value & 0x07FFu) << 1) | (telemetry ? 1u : 0u));
    uint16_t csum   = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0Fu;
    return (uint16_t)((packet << 4) | csum);
}

static uint16_t throttle_to_value(float t) {
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    return (uint16_t)(DSHOT_THROTTLE_MIN + t * (DSHOT_THROTTLE_MAX - DSHOT_THROTTLE_MIN));
}

bool esc_dshot_init(void) {
    memset(&s_self, 0, sizeof(s_self));
    s_self.ready = true;
    return true;
}

esc_dshot_t *esc_dshot_get(void) {
    return s_self.ready ? &s_self : NULL;
}

void esc_dshot_set_throttle(esc_dshot_t *d, float throttle_unit) {
    uint16_t v = d->armed ? throttle_to_value(throttle_unit) : DSHOT_CMD_MOTOR_STOP;
    uint16_t f = dshot_pack(v, false);
    io_dshot_send(&IO_DSHOT_ESC_UPPER, f);
    io_dshot_send(&IO_DSHOT_ESC_LOWER, f);
}

void esc_dshot_send_command(esc_dshot_t *d, uint16_t cmd) {
    (void)d;
    uint16_t f = dshot_pack(cmd & 0x07FFu, false);
    io_dshot_send(&IO_DSHOT_ESC_UPPER, f);
    io_dshot_send(&IO_DSHOT_ESC_LOWER, f);
}

void esc_dshot_arm(esc_dshot_t *d, bool armed) {
    d->armed = armed;
    if (!armed) esc_dshot_set_throttle(d, 0.0f);
}
