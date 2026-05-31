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

#include "generated/messages/publish.h"
#include "generated/messages/types.h"

#include "FreeRTOS.h"
#include "task.h"

/* Phase-1 demo cadence. Real cadence comes from the IMU EXTI notification
 * once that path is wired; 10 Hz is enough to prove the publish pipeline
 * runs end-to-end without burning bandwidth. */
#define STATE_DEMO_PERIOD_MS  100U

void task_state_estimation(void *arg) {
    (void)arg;

    /* Phase-1 placeholder publisher. Real estimator produces a state_t every
     * IMU cycle (~800 Hz) and we'll copy its fields into the registry's
     * msg_state_estimation_state_estimate_t. For now, publish zeros so the
     * routing + envelope + sink path is exercised in flight. */
    const vec3_f32_t  zero3 = { 0.0f, 0.0f, 0.0f };
    const quat_wxyz_t identity = { 1.0f, 0.0f, 0.0f, 0.0f };

    for (;;) {
        const uint64_t t_us = io_timestamp_us();

        PUB_STATE_ESTIMATION_STATE_ESTIMATE(
            t_us, zero3, zero3, identity, zero3, zero3);

        vTaskDelay(pdMS_TO_TICKS(STATE_DEMO_PERIOD_MS));
    }
}
