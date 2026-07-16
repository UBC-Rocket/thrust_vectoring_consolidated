/**
 * @file    controls_task.c
 * @brief   CM7: hard-real-time control loop runs in the TIM16 ISR
 *          (800 Hz); this FreeRTOS task arms ESC PWM on boot and polls
 *          radio tunables.
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

#ifdef DEBUG_TEXT_CONSOLE
#include "io_sys/io_debug.h"   /* bring-up: 1 Hz control-output trace */
#include <stdio.h>             /* snprintf for fixed-point formatting  */
#endif

#define CONTROLS_DT_S 0.00125f

#define ESC_PWM_MIN_US 1000U
#define ESC_PWM_MAX_US 2000U

/* Bring-up: fixed throttle after ESC arm delay. Change here only — the debug
 * log and esc_set_us() both use this (the old log hardcoded 0.25f and lied).
 * After editing: rebuild + flash CM7 (./build.sh && ./flash.sh Debug cm7). */
#define BRINGUP_ESC_THROTTLE       0.65f
#define BRINGUP_ESC_BUILD_TAG      "esc-bringup-v2"

static inline uint16_t throttle_to_us(float throttle) {
    if (throttle < 0.0f) throttle = 0.0f;
    if (throttle > 1.0f) throttle = 1.0f;
    return (uint16_t)(ESC_PWM_MIN_US +
                      throttle * (float)(ESC_PWM_MAX_US - ESC_PWM_MIN_US));
}

static inline void esc_set_us(const pwm_output_t *pwm, uint16_t us) {
    pwm_set_compare(pwm, pwm_clamp_ticks(pwm, pwm_us_to_ticks(pwm, us)));
}

static inline uint16_t esc_get_us(const pwm_output_t *pwm) {
    if (pwm == NULL || pwm->htim == NULL) {
        return 0U;
    }
    return (uint16_t)__HAL_TIM_GET_COMPARE(pwm->htim, pwm->channel);
}

static inline void esc_apply_bringup_throttle(const escs_t *escs, uint16_t us) {
    if (escs == NULL) {
        return;
    }
    esc_set_us(&escs->lower, us);
    esc_set_us(&escs->upper, us);
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
static volatile bool s_esc_bringup_armed;
static const escs_t *s_escs;
static uint16_t s_bringup_esc_us;

void controls_on_tim_period_elapsed(void *htim_handle)
{
    TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)htim_handle;
    if (htim->Instance != TIM16) return;
    if (!s_isr_ready) return;

    state_t state;
    (void)state_exchange_get_state(&state);

    control_output_t out;
    flight_controller_run(&state, &s_live_ref, &s_live_config, &out, CONTROLS_DT_S);

    /* Bring-up: pin ESC PWM from the 800 Hz ISR so nothing else can stomp CCR.
     * T_cmd / pwm_setpoint_from_forces() is NOT wired yet — ignore T= in logs. */
    if (s_esc_bringup_armed && s_escs != NULL) {
        esc_apply_bringup_throttle(s_escs, s_bringup_esc_us);
    }

    (void)state_exchange_publish_control_output(&out);
}

void controls_isr_init(void)
{
    s_escs = esc_handles();
#ifndef BENCH_NO_MOTORS

#else
    /* BENCH_NO_MOTORS: intended to leave the ESC PWM channels unarmed
     * (never HAL_TIM_PWM_Start'd) so no pulses reach the motors. NOTE:
     * task_controls() currently calls HAL_TIM_PWM_Start()/esc_set_us()
     * unconditionally — this ifdef does not actually gate that. */
#endif
    flight_controller_init(&s_live_config);
    s_isr_ready = true;
}

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
#endif

void task_controls(void *arg) {
    (void)arg;

    /* Arming sequence — mirrors the deprecated ulysses-flight-controller's
     * esc_pair_set_force(0, 0) -> esc_pair_set_armed(true) ordering: force
     * the pulse to minimum BEFORE enabling the PWM channel output, so the
     * very first pulse the ESC ever sees is a safe minimum, not whatever
     * the control loop happens to be computing. Hold minimum for 6 s (ESCs
     * need a sustained min-throttle signal to complete their own arm
     * sequence) before the 800 Hz control-loop timer starts — so there's no
     * window where the ISR could write a live (non-minimum) compare value
     * before or during arming. */
    if (s_escs != NULL) {
        esc_apply_bringup_throttle(s_escs, ESC_PWM_MIN_US);
    }
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);

    osDelay(pdMS_TO_TICKS(6000));

    /* Bring-up: confirm the motors actually spin post-arm before handing
     * CCR over to the live controller. */
    s_bringup_esc_us = throttle_to_us(BRINGUP_ESC_THROTTLE);
    esc_apply_bringup_throttle(s_escs, s_bringup_esc_us);
    s_esc_bringup_armed = true;

#ifdef DEBUG_TEXT_CONSOLE
    io_debug_printf("[ctl] %s armed esc=%u us (%u%%)\r\n",
                    BRINGUP_ESC_BUILD_TAG,
                    (unsigned)s_bringup_esc_us,
                    (unsigned)(BRINGUP_ESC_THROTTLE * 100.0f + 0.5f));
#endif

    HAL_TIM_Base_Start_IT(&htim16);

#ifdef DEBUG_TEXT_CONSOLE
    unsigned dbg_tick = 0;
#endif

    for (;;) {
        poll_tunables();

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
            if (s_escs != NULL && s_esc_bringup_armed) {
                io_debug_printf("[ctl] esc lo/hi/cmd=%u/%u/%u us  T=%s  gim=%s,%s  z_i=%s\r\n",
                                (unsigned)esc_get_us(&s_escs->lower),
                                (unsigned)esc_get_us(&s_escs->upper),
                                (unsigned)s_bringup_esc_us,
                                t, tx, ty, zi);
            } else if (s_escs != NULL) {
                io_debug_printf("[ctl] esc arming lo/hi=%u/%u us  T=%s\r\n",
                                (unsigned)esc_get_us(&s_escs->lower),
                                (unsigned)esc_get_us(&s_escs->upper),
                                t);
            } else {
                io_debug_printf("[ctl] esc=N/A  T=%s  gim=%s,%s  z_i=%s\r\n",
                                t, tx, ty, zi);
            }
        }
#endif

        vTaskDelay(pdMS_TO_TICKS(TUNABLE_POLL_MS));
    }
}
