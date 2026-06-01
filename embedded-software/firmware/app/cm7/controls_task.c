/**
 * @file    controls_task.c
 * @brief   CM7: hard-real-time control loop runs in the TIM16 ISR
 *          (800 Hz); this FreeRTOS task body is the SOLVER loop, which
 *          publishes reference setpoints the ISR consumes. Stub for now.
 *
 * ISR ↔ task split:
 *   - HAL_TIM_PeriodElapsedCallback (TIM16 update event, 800 Hz):
 *       reads state from state_exchange, runs flight_controller_run
 *       with the current reference, writes ESC + servo commands,
 *       publishes the control_output for CM4 SD logging.
 *   - task_controls (this file): the placeholder for the eventual
 *       trajectory / MPC / whatever-solver loop that produces
 *       references. Today: noop loop, the ISR uses a fixed default.
 *
 * Why split this way: hard-real-time math must not depend on the
 * FreeRTOS scheduler's context-switch jitter; the solver doesn't
 * need 800 Hz cadence and is free to take its time.
 *
 * UBC Rocket, 2026
 */

#include "app/tasks.h"
#include "app/state_exchange.h"
#include "app/actuators_init.h"
#include "io_sys/io_timestamp.h"

#include "controls/flight_controller.h"
#include "dev_servo_feetech.h"
#include "dev_esc_dshot.h"

#include "tim.h"  /* htim16 */

#include "FreeRTOS.h"
#include "task.h"

#include <math.h>

/* -------------------------------------------------------------------------- */
/* ISR-shared state                                                            */
/* -------------------------------------------------------------------------- */

#define CONTROLS_DT_S 0.00125f   /* 1 / 800 Hz */

/* Controller config. Mirrors the deprecated H5 defaults. Future: load
 * from flash, accept system.set_param overrides at runtime. */
static const flight_controller_config_t s_config = {
    .attitude = {
        .Kp = {{1.0f, 0, 0}, {0, 1.0f, 0}, {0, 0, 1.0f}},
        .Kd = {{0.1f, 0, 0}, {0, 0.1f, 0}, {0, 0, 0.1f}},
        .I  = {{0.01f, 0, 0}, {0, 0.01f, 0}, {0, 0, 0.01f}},
    },
    .allocation = { .t_hat = { 0.0f, 0.0f, -1.0f } },
    .gimbal = {
        .L         = 0.2f,
        .theta_min = -100.0f * (float)M_PI / 180.0f,
        .theta_max =  100.0f * (float)M_PI / 180.0f,
    },
    .thrust = {
        .m              = 1.2f,
        .g              = 9.8067f,
        .T_min          = 0.0f,
        .T_max          = 0.8f * 9.8067f,    /* 0.8 kg × g */
        .kp             = 2.0f,
        .ki             = 0.0f,
        .kd             = 0.0f,
        .integral_limit = 2.0f,
        .a_z_min        = -10.0f,
        .a_z_max        =  10.0f,
    },
};

/* Default reference until the solver task publishes one. Identity
 * attitude (upright), z setpoint at origin, no vz. */
static const flight_controller_ref_t s_default_ref = {
    .q_ref  = { .w = 1.0f, .x = 0.0f, .y = 0.0f, .z = 0.0f },
    .z_ref  = 0.0f,
    .vz_ref = 0.0f,
};

/* Snapshot of actuator handles, captured once at controls_init so the
 * ISR doesn't call into actuators_handles() (which gates on a static
 * init flag we don't need to re-check every cycle). */
static servo_feetech_t *s_servos;
static esc_dshot_t     *s_escs;
static volatile bool    s_isr_ready;

/* -------------------------------------------------------------------------- */
/* TIM16 ISR — the hard-real-time control loop                                 */
/* -------------------------------------------------------------------------- */

/* The CM7 HAL timebase TIM (TIM6) also lands on HAL_TIM_PeriodElapsed-
 * Callback, and CubeMX's main.c already defines that weak override.
 * Rather than fight CubeMX, we expose a named entry point and let
 * main.c's USER CODE block dispatch to us when the TIM16 update fires.
 * (Same pattern CM4 uses for io_timestamp_on_tim_period_elapsed.) */
void controls_on_tim_period_elapsed(void *htim_handle)
{
    TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)htim_handle;
    if (htim->Instance != TIM16) return;
    if (!s_isr_ready) return;

    state_t state;
    (void)state_exchange_get_state(&state);

    control_output_t out;
    flight_controller_run(&state, &s_default_ref, &s_config, &out, CONTROLS_DT_S);

    /* Convert gimbal angles to servo degrees (config min/max are radians). */
    float deg_x = out.theta_x_cmd * 180.0f / (float)M_PI;
    float deg_y = out.theta_y_cmd * 180.0f / (float)M_PI;
    servo_feetech_set_pair_degrees(s_servos, deg_x, deg_y);

    /* T_cmd → throttle in [0, 1]. */
    float throttle = (s_config.thrust.T_max > 0.0f)
                      ? (out.T_cmd / s_config.thrust.T_max) : 0.0f;
    if (throttle < 0.0f) throttle = 0.0f;
    if (throttle > 1.0f) throttle = 1.0f;
    esc_dshot_set_throttle(s_escs, throttle);

    /* Hand the control output back to CM4 for SD logging via state_exchange.
     * Seqlock publish from ISR context is safe — single writer, atomic
     * counter. */
    (void)state_exchange_publish_control_output(&out);
}

/* -------------------------------------------------------------------------- */
/* Bring-up — called from app_init_cm7 after dev_init_cm7                      */
/* -------------------------------------------------------------------------- */

void controls_isr_init(void)
{
    const actuators_t *a = actuators_handles();
    if (a == NULL) return;
    s_servos = a->servos;
    s_escs   = a->escs;

    /* Arm the ESC + enable servo torque so the ISR's first writes do
     * something useful. The ESC needs an arming gesture (idle DShot
     * frames at min throttle); doing it here keeps the ISR pure. */
    esc_dshot_arm(s_escs, true);
    servo_feetech_enable(s_servos, true);

    flight_controller_init(&s_config);

    s_isr_ready = true;

    /* Start the 800 Hz periodic interrupt that drives the ISR. */
    HAL_TIM_Base_Start_IT(&htim16);
}

/* -------------------------------------------------------------------------- */
/* Solver task body — STUB                                                     */
/* -------------------------------------------------------------------------- */

void task_controls(void *arg) {
    (void)arg;
    /* TODO(solver): trajectory / MPC / whatever produces the reference
     * setpoints that the ISR's flight_controller_run consumes. Today
     * the ISR uses s_default_ref (identity attitude, zero z). When this
     * task gains a body, it should publish references into a new
     * state_exchange slot the ISR reads. */
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
