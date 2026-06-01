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
 * Bring-up ordering (NEW — see controls_isr_init / task_controls below):
 *   1. controls_isr_init (pre-scheduler, from app_init_cm7):
 *        snapshots actuator handles, arms ESC, enables servo torque,
 *        inits the flight controller, sets s_isr_ready.
 *        It does NOT start TIM16 — leaving the ISR dormant so the
 *        startup self-test owns the servo bus + ESC exclusively.
 *   2. task_controls (post-scheduler):
 *        runs the pre-flight self-test (servo sweep + ESC ramp),
 *        then kicks HAL_TIM_Base_Start_IT(&htim16) to release the
 *        800 Hz ISR, then enters the (still-stub) solver loop.
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

    /* NOTE: TIM16 is intentionally NOT started here. task_controls owns
     * the servo bus + ESC during the pre-flight self-test; it kicks
     * HAL_TIM_Base_Start_IT(&htim16) after the self-test completes so
     * the ISR doesn't fight the test for the shared actuators. */
}

/* -------------------------------------------------------------------------- */
/* Pre-flight startup self-test                                                */
/* -------------------------------------------------------------------------- */

/* Servo sweep: 90° → -90° → 90° in 1° steps at 5 ms/step (~1.8 s total). */
#define SELFTEST_SWEEP_RANGE_DEG   90.0f
#define SELFTEST_SWEEP_STEP_DEG    1.0f
#define SELFTEST_SWEEP_STEP_MS     5U

/* Servo circle: 72 points (5° each) around a small radius at 18 ms/step
 * (~1.3 s total) — exercises coordinated XY motion. */
#define SELFTEST_CIRCLE_RADIUS_DEG 5.0f
#define SELFTEST_CIRCLE_STEPS      72U
#define SELFTEST_CIRCLE_STEP_MS    18U

/* ESC: settle after power-on, then arming delay, then ramp 0 → 0.1 over
 * 50 steps at 20 ms/step (~1 s ramp), hold, ramp back down. */
#define SELFTEST_ESC_SETTLE_MS     3000U
#define SELFTEST_ESC_ARM_MS        3000U
#define SELFTEST_ESC_RAMP_STEPS    50U
#define SELFTEST_ESC_RAMP_STEP_MS  20U
#define SELFTEST_ESC_PEAK_THROTTLE 0.10f
#define SELFTEST_ESC_HOLD_MS       500U

/* Run the pre-flight self-test on the actuators. Blocking; must run in
 * task context (vTaskDelay) and must complete before TIM16 is started. */
static void controls_run_startup_selftest(void)
{
    if (s_servos == NULL || s_escs == NULL) return;

    /* --- Servo sweep: center → +90 → -90 → center --------------------- */
    servo_feetech_set_pair_degrees(s_servos, 0.0f, 0.0f);
    vTaskDelay(pdMS_TO_TICKS(100));

    for (float deg = 0.0f; deg <= SELFTEST_SWEEP_RANGE_DEG;
         deg += SELFTEST_SWEEP_STEP_DEG) {
        servo_feetech_set_pair_degrees(s_servos, deg, deg);
        vTaskDelay(pdMS_TO_TICKS(SELFTEST_SWEEP_STEP_MS));
    }
    for (float deg = SELFTEST_SWEEP_RANGE_DEG;
         deg >= -SELFTEST_SWEEP_RANGE_DEG;
         deg -= SELFTEST_SWEEP_STEP_DEG) {
        servo_feetech_set_pair_degrees(s_servos, deg, deg);
        vTaskDelay(pdMS_TO_TICKS(SELFTEST_SWEEP_STEP_MS));
    }
    for (float deg = -SELFTEST_SWEEP_RANGE_DEG; deg <= 0.0f;
         deg += SELFTEST_SWEEP_STEP_DEG) {
        servo_feetech_set_pair_degrees(s_servos, deg, deg);
        vTaskDelay(pdMS_TO_TICKS(SELFTEST_SWEEP_STEP_MS));
    }

    /* --- Servo circle: 72 points at radius 5° ------------------------- */
    for (unsigned i = 0; i < SELFTEST_CIRCLE_STEPS; ++i) {
        float ang = (2.0f * (float)M_PI * (float)i) /
                    (float)SELFTEST_CIRCLE_STEPS;
        float dx = SELFTEST_CIRCLE_RADIUS_DEG * cosf(ang);
        float dy = SELFTEST_CIRCLE_RADIUS_DEG * sinf(ang);
        servo_feetech_set_pair_degrees(s_servos, dx, dy);
        vTaskDelay(pdMS_TO_TICKS(SELFTEST_CIRCLE_STEP_MS));
    }

    /* Return servos to neutral before motor test. */
    servo_feetech_set_pair_degrees(s_servos, 0.0f, 0.0f);

    /* --- ESC: settle, arm wait, ramp up, hold, ramp down -------------- */
    /* ESC was armed in controls_isr_init; give it time to see steady
     * idle DShot frames before commanding throttle. */
    esc_dshot_set_throttle(s_escs, 0.0f);
    vTaskDelay(pdMS_TO_TICKS(SELFTEST_ESC_SETTLE_MS));
    vTaskDelay(pdMS_TO_TICKS(SELFTEST_ESC_ARM_MS));

    for (unsigned i = 1; i <= SELFTEST_ESC_RAMP_STEPS; ++i) {
        float thr = (SELFTEST_ESC_PEAK_THROTTLE * (float)i) /
                    (float)SELFTEST_ESC_RAMP_STEPS;
        esc_dshot_set_throttle(s_escs, thr);
        vTaskDelay(pdMS_TO_TICKS(SELFTEST_ESC_RAMP_STEP_MS));
    }

    vTaskDelay(pdMS_TO_TICKS(SELFTEST_ESC_HOLD_MS));

    for (unsigned i = SELFTEST_ESC_RAMP_STEPS; i > 0; --i) {
        float thr = (SELFTEST_ESC_PEAK_THROTTLE * (float)(i - 1)) /
                    (float)SELFTEST_ESC_RAMP_STEPS;
        esc_dshot_set_throttle(s_escs, thr);
        vTaskDelay(pdMS_TO_TICKS(SELFTEST_ESC_RAMP_STEP_MS));
    }

    esc_dshot_set_throttle(s_escs, 0.0f);
}

/* -------------------------------------------------------------------------- */
/* Solver task body — STUB (with one-shot startup self-test)                   */
/* -------------------------------------------------------------------------- */

void task_controls(void *arg) {
    (void)arg;

    /* Run the pre-flight self-test ONCE, before the 800 Hz ISR is live.
     * controls_isr_init deliberately left TIM16 stopped so we own the
     * servo bus + ESC exclusively here. */
    controls_run_startup_selftest();

    /* Self-test done — release the hard-real-time loop. From here on the
     * TIM16 update ISR (controls_on_tim_period_elapsed) drives ESC + servo
     * commands at 800 Hz; this task must not touch the actuators directly. */
    HAL_TIM_Base_Start_IT(&htim16);

    /* TODO(solver): trajectory / MPC / whatever produces the reference
     * setpoints that the ISR's flight_controller_run consumes. Today
     * the ISR uses s_default_ref (identity attitude, zero z). When this
     * task gains a body, it should publish references into a new
     * state_exchange slot the ISR reads. */
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
