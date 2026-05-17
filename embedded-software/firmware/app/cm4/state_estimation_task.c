/**
 * @file    state_estimation_task.c
 * @brief   CM4 task: drain IMU/baro/GPS samples (via dev/), run EKF,
 *          publish state.
 *
 * UBC Rocket, 2026
 */

#include "app/tasks.h"
#include "app/state_exchange.h"
#include "app/sensors_init.h"
#include "io_sys/io_timestamp.h"

void task_state_estimation(void *arg) {
    (void)arg;
    /* TODO: bind notify target to this task; loop on notification bits;
     *       drain samples from sensors_handles(); run EKF; publish state. */
    for (;;) {
        /* placeholder */
    }
}
