/**
 * @file    controls_task.c
 * @brief   CM7: hard-real-time control loop runs in the TIM16 ISR
 *          (800 Hz); this FreeRTOS task gates ESC PWM on flight state and
 *          polls radio tunables.
 *
 * Servo actuation: Dynamixel gimbals are driven on CM4 (tilt KF + local PD in
 * actuator_task). CM7 publishes control_output_t for ESC / Feetech / telemetry.
 *
 * Motor actuation: standard 1000-2000 us / 400 Hz PWM (ported from the
 * deprecated ulysses-flight-controller's motor_drivers/pwm_output.c) on the
 * same TIM4_CH1 (PD12, lower) / TIM4_CH2 (PD13, upper) pins DShot used to
 * drive — see esc_init.h / dev_init_cm7.c for the wiring and tim.c's
 * MX_TIM4_Init for the timer reconfiguration. DShot's bidirectional
 * telemetry (RPM readback) has no equivalent on plain PWM; that capability
 * is gone, not just unported. dev/esc_dshot/ is left on disk (unlinked) if
 * bidirectional DShot needs to come back.
 *
 * UBC Rocket, 2026
 */

#include "app/tasks.h"
#include "app/state_exchange.h"
#include "app/esc_init.h"

#include "controls/flight_controller.h"
#include "controls/pwm.h"
#include "cmsis_os2.h"

#include "tim.h"

#include "FreeRTOS.h"
#include "task.h"

#include <math.h>
#include <stdbool.h>

#ifdef DEBUG_TEXT_CONSOLE
#include "io_sys/io_debug.h"   /* bring-up: 1 Hz control-output trace */
#include <stdio.h>             /* snprintf for fixed-point formatting  */
#endif

#define CONTROLS_DT_S 0.00125f

#define ESC_PWM_MIN_US 1000U
#define ESC_PWM_MAX_US 2000U

#define ESC_LAUNCH_THROTTLE 0.65f

static inline uint16_t throttle_to_us(float throttle) {
    if (throttle < 0.0f) throttle = 0.0f;
    if (throttle > 1.0f) throttle = 1.0f;
    return (uint16_t)(ESC_PWM_MIN_US +
                      throttle * (float)(ESC_PWM_MAX_US - ESC_PWM_MIN_US));
}

static inline void esc_set_us(const pwm_output_t *pwm, uint16_t us) {
    pwm_set_compare(pwm, pwm_clamp_ticks(pwm, pwm_us_to_ticks(pwm, us)));
}

static flight_controller_config_t s_live_config = {
    .attitude = {
        .Kp = {{1.0f, 0, 0}, {0, 1.0f, 0}, {0, 0, 1.0f}},
        .Kd = {{0.1f, 0, 0}, {0, 0.1f, 0}, {0, 0, 0.1f}},
        .I  = {{0.01f, 0, 0}, {0, 0.01f, 0}, {0, 0, 0.01f}},
    },
    .allocation = { .t_hat = { 0.0f, 0.0f, -1.0f } },
    .gimbal = {
        .L         = 0.2f,
        .theta_min = -15.0f * (float)M_PI / 180.0f,
        .theta_max =  15.0f * (float)M_PI / 180.0f,
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

    /* ESC PWM is driven by task_controls() from flight state. */

    (void)state_exchange_publish_control_output(&out);
}

void controls_isr_init(void)
{
    flight_controller_init(&s_live_config);
    s_isr_ready = true;
    HAL_TIM_Base_Start_IT(&htim16);
}

static inline bool controls_was_in_flight(app_flight_state_t fs)
{
    return fs == APP_FLIGHT_ARMED   ||
           fs == APP_FLIGHT_RISE    ||
           fs == APP_FLIGHT_DESCENT;
}

static inline bool controls_is_abort_state(app_flight_state_t fs)
{
    return fs == APP_FLIGHT_IDLE || fs == APP_FLIGHT_ESTOP;
}

#ifndef BENCH_NO_MOTORS
static uint16_t controls_esc_us_for_state(app_flight_state_t fs)
{
    switch (fs) {
        case APP_FLIGHT_RISE:
        case APP_FLIGHT_DESCENT:
            return throttle_to_us(ESC_LAUNCH_THROTTLE);
        case APP_FLIGHT_ARMED:
        case APP_FLIGHT_IDLE:
        case APP_FLIGHT_ESTOP:
        case APP_FLIGHT_LANDED:
        default:
            return ESC_PWM_MIN_US;
    }
}

static void controls_esc_apply(const escs_t *escs, uint16_t us)
{
    if (escs == NULL) {
        return;
    }
    esc_set_us(&escs->lower, us);
    esc_set_us(&escs->upper, us);
}

static void controls_esc_start_pwm(const escs_t *escs)
{
    if (escs == NULL) {
        return;
    }
    esc_set_us(&escs->lower, ESC_PWM_MIN_US);
    esc_set_us(&escs->upper, ESC_PWM_MIN_US);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
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
    s_live_config.thrust.T_min = c->T_min;
    /* Ignore unset (zero) limits — CM4 seeds cfg_defaults as {0}, which
     * would otherwise clamp every gimbal command to 0 rad. */
    if (c->theta_max > 0.0f) {
        s_live_config.gimbal.theta_min = c->theta_min;
        s_live_config.gimbal.theta_max = c->theta_max;
    }
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

static const char *controls_state_name(app_flight_state_t fs)
{
    switch (fs) {
        case APP_FLIGHT_IDLE:    return "IDLE";
        case APP_FLIGHT_ARMED:   return "ARMED";
        case APP_FLIGHT_RISE:    return "RISE";
        case APP_FLIGHT_DESCENT: return "DESCENT";
        case APP_FLIGHT_LANDED:  return "LANDED";
        case APP_FLIGHT_ESTOP:   return "ESTOP";
        default:                 return "?";
    }
}
#endif

void task_controls(void *arg) {
    (void)arg;

    app_flight_state_t prev_fs = APP_FLIGHT_IDLE;
    app_flight_state_t fs      = APP_FLIGHT_IDLE;
#ifndef BENCH_NO_MOTORS
    const escs_t *escs = esc_handles();
    bool pwm_started = false;
    uint16_t esc_us  = ESC_PWM_MIN_US;
#endif

#ifdef DEBUG_TEXT_CONSOLE
    unsigned dbg_tick = 0;
#endif

    for (;;) {
        poll_tunables();

        (void)state_exchange_get_flight_state(&fs);

        if (controls_is_abort_state(fs) && controls_was_in_flight(prev_fs)) {
            flight_controller_init(&s_live_config);
        }

#ifndef BENCH_NO_MOTORS
        if (fs == APP_FLIGHT_ARMED && !pwm_started) {
            controls_esc_start_pwm(escs);
            pwm_started = true;
        }

        if (pwm_started) {
            const uint16_t desired_us = controls_esc_us_for_state(fs);
            if (desired_us != esc_us) {
                esc_us = desired_us;
                controls_esc_apply(escs, esc_us);
            }
        }
#endif

#ifdef DEBUG_TEXT_CONSOLE
        if (++dbg_tick >= (1000U / TUNABLE_POLL_MS)) {
            dbg_tick = 0;
            control_output_t out;
            (void)state_exchange_get_control_output(&out);
            char t[16], tx[16], ty[16], zi[16];
            fmt_f3(t,  out.T_cmd);
            fmt_f3(tx, out.theta_x_cmd);
            fmt_f3(ty, out.theta_y_cmd);
            fmt_f3(zi, out.z_pid_integral);
            io_debug_printf("[ctl] state=%s esc=%u us  T=%s  gim=%s,%s  z_i=%s\r\n",
                            controls_state_name(fs),
#ifndef BENCH_NO_MOTORS
                            (unsigned)esc_us,
#else
                            0U,
#endif
                            t, tx, ty, zi);
        }
#endif

        prev_fs = fs;
        vTaskDelay(pdMS_TO_TICKS(TUNABLE_POLL_MS));
    }
}
