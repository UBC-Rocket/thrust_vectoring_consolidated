/**
 * @file    controls_task.c
 * @brief   CM7 task: 800 Hz control loop. Reads state, writes servo + ESC
 *          commands through driver instances.
 *
 * UBC Rocket, 2026
 */

#include "app/tasks.h"
#include "app/state_exchange.h"
#include "app/actuators_init.h"
#include "io_sys/io_timestamp.h"

void task_controls(void *arg) {
    (void)arg;
    /* TODO: 800 Hz tick; read state via state_exchange_get_state(); compute
     *       gimbal angles + throttle; push via actuators_handles()->servos
     *       / ->escs. DShot frames must be sent at the loop rate to keep
     *       the ESC armed. */
    for (;;) {
        /* placeholder */
    }
}
