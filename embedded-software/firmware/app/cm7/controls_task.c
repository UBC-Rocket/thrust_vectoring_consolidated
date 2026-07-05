/**
 * @file    controls_task.c
 * @brief   CM7: hard-real-time control loop runs in the TIM16 ISR
 *          (800 Hz); this FreeRTOS task body is the SOLVER loop, which
 *          publishes reference setpoints the ISR consumes. Stub for now.
 *
 * Servo actuation lives on CM4 (UART8); the ISR publishes control_output_t
 * and CM4's actuator task drives the gimbal servos.
 *
 * UBC Rocket, 2026
 */

#include "app/tasks.h"
#include "app/state_exchange.h"

#include "controls/flight_controller.h"
#include "esc_dshot/bdshot.h"

#include "tim.h"

#include "FreeRTOS.h"
#include "task.h"

#include <math.h>

#ifdef DEBUG_TEXT_CONSOLE
#include "io_sys/io_debug.h"   /* bring-up: 1 Hz control-output trace */
#include <stdio.h>             /* snprintf for fixed-point formatting  */
#endif

#define CONTROLS_DT_S 0.00125f

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
        .T_max          = 0.8f * 9.8067f,
        .kp             = 2.0f,
        .ki             = 0.0f,
        .kd             = 0.0f,
        .integral_limit = 2.0f,
        .a_z_min        = -10.0f,
        .a_z_max        =  10.0f,
    },
};

static flight_controller_ref_t s_live_ref = {
    .q_ref  = { .w = 1.0f, .x = 0.0f, .y = 0.0f, .z = 0.0f },
    .z_ref  = 0.0f,
    .vz_ref = 0.0f,
};

static uint32_t s_last_pid_seq;
static uint32_t s_last_ref_seq;
static uint32_t s_last_cfg_seq;
static volatile bool s_isr_ready;

void controls_on_tim_period_elapsed(void *htim_handle)
{
    TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)htim_handle;
    if (htim->Instance != TIM16) return;
    if (!s_isr_ready) return;

    state_t state;
    (void)state_exchange_get_state(&state);

    control_output_t out;
    flight_controller_run(&state, &s_live_ref, &s_live_config, &out, CONTROLS_DT_S);

    float throttle = (s_live_config.thrust.T_max > 0.0f)
                      ? (out.T_cmd / s_live_config.thrust.T_max) : 0.0f;
    if (throttle < 0.0f) throttle = 0.0f;
    if (throttle > 1.0f) throttle = 1.0f;
    esc_dshot_motor_set_throttle(ESC_MOTOR_ID_LOWER, ESC_PERCENTAGE_TO_THROTTLE(throttle));
    esc_dshot_motor_set_throttle(ESC_MOTOR_ID_UPPER, ESC_PERCENTAGE_TO_THROTTLE(throttle));

    (void)state_exchange_publish_control_output(&out);
}

#define SELFTEST_ESC_SETTLE_MS     3000U
#define SELFTEST_ESC_ARM_MS        3000U
#define SELFTEST_ESC_RAMP_STEPS    50U
#define SELFTEST_ESC_RAMP_STEP_MS  20U
#define SELFTEST_ESC_PEAK_THROTTLE 0.10f
#define SELFTEST_ESC_HOLD_MS       500U

void controls_isr_init(void)
{
#ifndef BENCH_NO_MOTORS
    esc_dshot_set_armed(true);
#else
    /* BENCH_NO_MOTORS: leave the ESC driver disarmed. The ISR still runs the
     * full controller and publishes control_output (servos on CM4 do move),
     * but esc_dshot_motor_set_throttle() rejects every command while
     * disarmed, so no DShot pulses reach the motors. There is no armed-state
     * gating in the ISR yet and the controller commands hover-level thrust
     * (T_cmd ≈ m·g) whenever it runs — do NOT flip this off with props on. */
#endif
    flight_controller_init(&s_live_config);
    s_isr_ready = true;
}

#ifndef BENCH_NO_MOTORS
static void controls_run_startup_selftest(void)
{
    esc_dshot_motor_set_throttle(ESC_MOTOR_ID_LOWER, 0);
    esc_dshot_motor_set_throttle(ESC_MOTOR_ID_UPPER, 0);

    vTaskDelay(pdMS_TO_TICKS(SELFTEST_ESC_SETTLE_MS));
    vTaskDelay(pdMS_TO_TICKS(SELFTEST_ESC_ARM_MS));

    for (unsigned i = 1; i <= SELFTEST_ESC_RAMP_STEPS; ++i) {
        float thr = (SELFTEST_ESC_PEAK_THROTTLE * (float)i) /
                    (float)SELFTEST_ESC_RAMP_STEPS;
        esc_dshot_motor_set_throttle(ESC_MOTOR_ID_LOWER, ESC_PERCENTAGE_TO_THROTTLE(thr));
        esc_dshot_motor_set_throttle(ESC_MOTOR_ID_UPPER, ESC_PERCENTAGE_TO_THROTTLE(thr));
        vTaskDelay(pdMS_TO_TICKS(SELFTEST_ESC_RAMP_STEP_MS));
    }

    vTaskDelay(pdMS_TO_TICKS(SELFTEST_ESC_HOLD_MS));

    for (unsigned i = SELFTEST_ESC_RAMP_STEPS; i > 0; --i) {
        float thr = (SELFTEST_ESC_PEAK_THROTTLE * (float)(i - 1)) /
                    (float)SELFTEST_ESC_RAMP_STEPS;
        esc_dshot_motor_set_throttle(ESC_MOTOR_ID_LOWER, ESC_PERCENTAGE_TO_THROTTLE(thr));
        esc_dshot_motor_set_throttle(ESC_MOTOR_ID_UPPER, ESC_PERCENTAGE_TO_THROTTLE(thr));
        vTaskDelay(pdMS_TO_TICKS(SELFTEST_ESC_RAMP_STEP_MS));
    }

    esc_dshot_motor_set_throttle(ESC_MOTOR_ID_LOWER, 0);
    esc_dshot_motor_set_throttle(ESC_MOTOR_ID_UPPER, 0);
}
#endif /* !BENCH_NO_MOTORS */

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
    s_live_config.thrust.T_min     = c->T_min;
    s_live_config.gimbal.theta_min = c->theta_min;
    s_live_config.gimbal.theta_max = c->theta_max;
}

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

#ifdef DEBUG_TEXT_CONSOLE
/* Signed fixed-point formatter (3 decimals) — the linked printf has no %f.
 * Same shape as fmt_f3 in cm4/state_estimation_task.c. */
static void fmt_f3(char *buf, float v) {
    int neg = (v < 0.0f);
    if (neg) v = -v;
    if (v > 9999.0f) v = 9999.0f;
    unsigned long m = (unsigned long)(v * 1000.0f + 0.5f);
    (void)snprintf(buf, 16, "%s%lu.%03lu", neg ? "-" : "", m / 1000UL, m % 1000UL);
}
#endif

void task_controls(void *arg) {
    (void)arg;

#ifndef BENCH_NO_MOTORS
    controls_run_startup_selftest();
#endif
    HAL_TIM_Base_Start_IT(&htim16);

#ifdef DEBUG_TEXT_CONSOLE
    unsigned dbg_tick = 0;
#endif

    for (;;) {
        poll_tunables();

#ifdef DEBUG_TEXT_CONSOLE
        /* Bring-up: ~1 Hz trace of the ISR's latest published control output
         * (read back from the exchange slot — same data CM4's actuator task
         * consumes). Proves the CM7 loop is running on live EKF state. */
        if (++dbg_tick >= (1000U / TUNABLE_POLL_MS)) {
            dbg_tick = 0;
            control_output_t out;
            (void)state_exchange_get_control_output(&out);
            char t[16], tx[16], ty[16], px[16], py[16];
            fmt_f3(t,  out.T_cmd);
            fmt_f3(tx, out.theta_x_cmd);
            fmt_f3(ty, out.theta_y_cmd);
            fmt_f3(px, out.phi_x);
            fmt_f3(py, out.phi_y);
            io_debug_printf("[ctl] T=%s N  gim=%s,%s rad  phi=%s,%s\r\n",
                            t, tx, ty, px, py);
        }
#endif

        vTaskDelay(pdMS_TO_TICKS(TUNABLE_POLL_MS));
    }
}
