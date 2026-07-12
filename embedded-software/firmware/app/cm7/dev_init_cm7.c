/**
 * @file    dev_init_cm7.c
 * @brief   DEV-layer phase for CM7. ESC/DShot only; servos live on CM4.
 *
 * UBC Rocket, 2026
 */
#include "app/app_init.h"
#include "esc_dshot/config.h"
#include "esc_dshot/io.h"
#include "io_sys/io_test_hooks.h"

#include "esc_dshot/bdshot.h"
#include <stddef.h>

void dev_init_cm7(void)
{
    static bool s_init_done;
    IO_TEST_HOOK_RW(s_init_done, bool, dev_init_cm7_init_done)

    if (s_init_done) return;

    static const esc_motor_config_t ESC_MOTOR_CONFIG_UPPER = { .pole_count = 14 };
    static const esc_motor_config_t ESC_MOTOR_CONFIG_LOWER = { .pole_count = 14 };

    esc_dshot_init();
    esc_dshot_motor_init(ESC_MOTOR_ID_UPPER, &ESC_MOTOR_CONFIG_UPPER, &IO_BDSHOT_ESC_UPPER);
    esc_dshot_motor_init(ESC_MOTOR_ID_LOWER, &ESC_MOTOR_CONFIG_LOWER, &IO_BDSHOT_ESC_LOWER);

    s_init_done = true;
}
