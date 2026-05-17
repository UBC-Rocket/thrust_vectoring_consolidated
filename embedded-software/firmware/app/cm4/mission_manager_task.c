/**
 * @file    mission_manager_task.c
 * @brief   CM4 task: decode radio commands, run flight state machine.
 *
 * UBC Rocket, 2026
 */

#include "app/tasks.h"
#include "app/state_exchange.h"
#include "app/sensors_init.h"

void task_mission_manager(void *arg) {
    (void)arg;
    /* TODO: drain radio frames via sensors_handles()->radio,
     *       decode rocket-protocol, run mission state machine,
     *       publish flight_state + armed. */
    for (;;) {
        /* placeholder */
    }
}
