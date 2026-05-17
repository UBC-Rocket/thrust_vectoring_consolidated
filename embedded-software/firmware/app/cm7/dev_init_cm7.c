/**
 * @file    dev_init_cm7.c
 * @brief   DEV-layer phase for CM7. Each driver owns its own singleton
 *          storage; this file only triggers init and assembles the
 *          ergonomic view-struct.
 *
 * UBC Rocket, 2026
 */
#include "app/app_init.h"
#include "app/actuators_init.h"
#include "io_sys/io_test_hooks.h"

static actuators_t s_handles;
static bool        s_init_done;

/* Test-only access to module-private state. No-op in production builds. */
IO_TEST_HOOK_RW(s_handles,   actuators_t, dev_init_cm7_handles)
IO_TEST_HOOK_RW(s_init_done, bool,        dev_init_cm7_init_done)

void dev_init_cm7(void) {
    if (s_init_done) return;

    servo_feetech_init();
    esc_dshot_init();

    s_handles.servos = servo_feetech_get();
    s_handles.escs   = esc_dshot_get();
    s_init_done = true;
}

const actuators_t *actuators_handles(void) {
    return s_init_done ? &s_handles : NULL;
}
