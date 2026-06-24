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
#ifdef USE_DYNAMIXEL_SERVO
#include "dev_servo_dynamixel.h"
#else
#include "dev_servo_feetech.h"
#endif
#include "dev_esc_dshot.h"

#include "tim.h"  /* htim16 */

#include "FreeRTOS.h"
#include "task.h"

#include <math.h>

/* -------------------------------------------------------------------------- */
/* ISR-shared state                                                            */
/* -------------------------------------------------------------------------- */

#define CONTROLS_DT_S 0.00125f   /* 1 / 800 Hz */

/* Controller config — mutable, seeded with the same defaults the deprecated
 * H5 build used. mission_manager forwards SetPidGains / SetConfig over
 * state_exchange; task_controls polls and merges into this struct. The
 * TIM16 ISR reads from it every cycle.
 *
 * Concurrency: the ISR fires at 800 Hz from TIM16, task_controls writes
 * from FreeRTOS context at ~20 Hz. We rely on word-aligned float writes
 * being atomic on M7 (they are — 32-bit aligned scalar stores complete in
 * one bus cycle with no tear) and accept that a multi-field merge can
 * straddle one ISR tick. Worst case the ISR sees half-old / half-new
 * gains for ≤1.25 ms, which is well below the controller's bandwidth. */
static flight_controller_config_t s_live_config = {
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

/* Live reference. Seeded with identity attitude / zero z. Replaced by
 * SetReference messages from the host (via mission_manager → state_exchange). */
static flight_controller_ref_t s_live_ref = {
    .q_ref  = { .w = 1.0f, .x = 0.0f, .y = 0.0f, .z = 0.0f },
    .z_ref  = 0.0f,
    .vz_ref = 0.0f,
};

/* Cached seq numbers for the IPC slots — task_controls only re-merges
 * when a publisher has actually moved the seq forward. */
static uint32_t s_last_pid_seq;
static uint32_t s_last_ref_seq;
static uint32_t s_last_cfg_seq;

/* Snapshot of actuator handles, captured once at controls_init so the
 * ISR doesn't call into actuators_handles() (which gates on a static
 * init flag we don't need to re-check every cycle). */
#ifdef USE_DYNAMIXEL_SERVO
static servo_dynamixel_t *s_servos;
#else
static servo_feetech_t   *s_servos;
#endif
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
    flight_controller_run(&state, &s_live_ref, &s_live_config, &out, CONTROLS_DT_S);

    /* Convert gimbal angles to servo degrees (config min/max are radians). */
#ifndef USE_DYNAMIXEL_SERVO
    float deg_x = out.theta_x_cmd * 180.0f / (float)M_PI;
    float deg_y = out.theta_y_cmd * 180.0f / (float)M_PI;
    servo_feetech_set_pair_degrees(s_servos, deg_x, deg_y);
#else
    (void)out;
#endif

    /* T_cmd → throttle in [0, 1]. */
    float throttle = (s_live_config.thrust.T_max > 0.0f)
                      ? (out.T_cmd / s_live_config.thrust.T_max) : 0.0f;
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
#ifndef USE_DYNAMIXEL_SERVO
    servo_feetech_enable(s_servos, true);
#endif

    flight_controller_init(&s_live_config);

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

#ifdef USE_DYNAMIXEL_SERVO
  servo_dynamixel_run_position_test(s_servos);
#else
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
#endif

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
/* Tunable merges — task context, run between ISR ticks                        */
/* -------------------------------------------------------------------------- */

/* SetPidGains carries fields one-by-one. "0" is the wire default for an
 * absent scalar; we read that as "unchanged" and skip the merge, which
 * lets the host send partial-update messages without needing to repeat
 * the full state every time. If 0 ever becomes a valid gain we'll add an
 * explicit valid bitmask to app_pid_gains_t. */
static inline void merge_pid_gains(const app_pid_gains_t *g) {
    for (int i = 0; i < 3; ++i) {
        if (g->attitude_kp[i] > 0.0f) s_live_config.attitude.Kp[i][i] = g->attitude_kp[i];
        if (g->attitude_kd[i] > 0.0f) s_live_config.attitude.Kd[i][i] = g->attitude_kd[i];
    }
    if (g->z_kp             > 0.0f) s_live_config.thrust.kp             = g->z_kp;
    if (g->z_ki             > 0.0f) s_live_config.thrust.ki             = g->z_ki;
    if (g->z_kd             > 0.0f) s_live_config.thrust.kd             = g->z_kd;
    if (g->z_integral_limit > 0.0f) s_live_config.thrust.integral_limit = g->z_integral_limit;
}

static inline void merge_reference(const app_reference_t *r) {
    s_live_ref.z_ref  = r->z_ref;
    s_live_ref.vz_ref = r->vz_ref;
    if (r->q_valid) {
        s_live_ref.q_ref.w = r->q_ref[0];
        s_live_ref.q_ref.x = r->q_ref[1];
        s_live_ref.q_ref.y = r->q_ref[2];
        s_live_ref.q_ref.z = r->q_ref[3];
    }
}

static inline void merge_vehicle_config(const app_vehicle_config_t *c) {
    if (c->mass_kg   > 0.0f) s_live_config.thrust.m     = c->mass_kg;
    if (c->T_max     > 0.0f) s_live_config.thrust.T_max = c->T_max;
    /* T_min and gimbal limits accept any value (zero is valid for T_min;
     * a host that wants to leave them unchanged simply omits the message). */
    s_live_config.thrust.T_min     = c->T_min;
    s_live_config.gimbal.theta_min = c->theta_min;
    s_live_config.gimbal.theta_max = c->theta_max;
}

/* Poll all three IPC slots; merge any whose seq has advanced since the
 * previous poll. Cheap — three slot reads + three compares. Runs at
 * TUNABLE_POLL_MS cadence, so a host SetPidGains lands in the live
 * config within that window. */
#define TUNABLE_POLL_MS  50U

static void poll_tunables(void) {
    uint32_t seq;

    app_pid_gains_t pid_gains;
    seq = state_exchange_get_pid_gains(&pid_gains);
    if (seq != s_last_pid_seq) {
        merge_pid_gains(&pid_gains);
        s_last_pid_seq = seq;
    }

    app_reference_t ref;
    seq = state_exchange_get_reference(&ref);
    if (seq != s_last_ref_seq) {
        merge_reference(&ref);
        s_last_ref_seq = seq;
    }

    app_vehicle_config_t cfg;
    seq = state_exchange_get_vehicle_config(&cfg);
    if (seq != s_last_cfg_seq) {
        merge_vehicle_config(&cfg);
        s_last_cfg_seq = seq;
    }
}

/* -------------------------------------------------------------------------- */
/* Solver task body — runs the self-test, then polls tunables                  */
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

    /* Poll cm4's tunable slots and merge any updates into s_live_config /
     * s_live_ref. The ISR reads from those on every tick. When the
     * trajectory/MPC solver lands, it'll write into s_live_ref directly
     * from this task — the polling and the solver are not mutually
     * exclusive. */
    for (;;) {
        poll_tunables();
        vTaskDelay(pdMS_TO_TICKS(TUNABLE_POLL_MS));
    }
}
