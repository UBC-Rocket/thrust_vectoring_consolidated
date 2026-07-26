/**
 * @file    controls_task.c
 * @brief   CM7: hard-real-time control loop runs in the TIM16 ISR
 *          (800 Hz); this FreeRTOS task arms ESC PWM on boot and polls
 *          radio tunables.
 *
 * Servo actuation: Dynamixel gimbals are driven on CM4 (tilt KF + local PD in
 * actuator_task). CM7 publishes control_output_t for ESC / Feetech / telemetry.
 *
 * UBC Rocket, 2026
 */

#include "app/tasks.h"
#include "app/state_exchange.h"

#include "cmsis_os2.h"
#include "controls/flight_controller.h"
#include "esc_dshot/bdshot.h"
#include "esc_dshot/config.h"

#include "lib/timestamp/include/timestamp/timestamp.h"
#include "projdefs.h"
#include "tim.h"

#include "FreeRTOS.h"
#include "task.h"

#include <math.h>
#include <stdint.h>

#ifdef DEBUG_TEXT_CONSOLE
#include "io_sys/io_debug.h"   /* bring-up: 1 Hz control-output trace */
#include <stdio.h>             /* snprintf for fixed-point formatting  */
#endif

#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))

#define S_TO_US(s) (s * 1000000)

#define CONTROLS_DT_S 0.00125f

#define BRINGUP_ESC_BUILD_TAG      "esc-bringup-10pct-v3"

#define ESC_THROTTLE_ZERO  ESC_PERCENTAGE_TO_THROTTLE(0.00f)
#define ESC_THROTTLE_INITIAL ESC_PERCENTAGE_TO_THROTTLE(0.10f)

#define ESC_THROTTLE_MAX ESC_PERCENTAGE_TO_THROTTLE(0.85f)

#define ESC_RPM_DEADBAND (50) // TODO: adjust based on how fast RPM changes with higher throttles
#define ESC_RPM_THROTTLE_ADJUSTMENT (1)

#define ESC_RPM_DESIRED_RISE (16350) // TODO: find actual value
#define ESC_RPM_DESIRED_LAND (8000) // TODO: find actual value

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
static uint32_t s_last_throttle_seq;
static volatile bool s_isr_ready;

/* Operator throttle from the GCS (SetThrottle), in DShot units, one value
 * per motor (legacy single-float sends mirror lower into upper on CM4).
 * Written by poll_tunables; read in the RISE/DESCENT path. A non-zero value
 * overrides the autonomous RPM-closed-loop throttle for that motor (manual
 * bench testing); 0 — the default, and what mission_manager publishes on
 * every disarm — falls through to autonomous, so a disarm can never zero out
 * an autonomous RISE. */
static volatile uint16_t s_operator_throttle_lower;
static volatile uint16_t s_operator_throttle_upper;

void controls_on_tim_period_elapsed(void *htim_handle)
{
    TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)htim_handle;
    if (htim->Instance != TIM16) return;
    if (!s_isr_ready) return;

    state_t state;
    (void)state_exchange_get_state(&state);

    control_output_t out;
    flight_controller_run(&state, &s_live_ref, &s_live_config, &out, CONTROLS_DT_S);

    esc_dshot_update();

    (void)state_exchange_publish_control_output(&out);
}

void controls_isr_init(void)
{
    flight_controller_init(&s_live_config);
    HAL_TIM_Base_Start_IT(&htim16);
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

#define TUNABLE_POLL_MS  5U

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

    /* Manual-throttle uplink (SetThrottle), converted to DShot units. */
    app_throttle_cmd_t thr;
    seq = state_exchange_get_throttle_cmd(&thr);
    if (seq != s_last_throttle_seq) {
        s_operator_throttle_lower = ESC_PERCENTAGE_TO_THROTTLE(thr.throttle_lower);
        s_operator_throttle_upper = ESC_PERCENTAGE_TO_THROTTLE(thr.throttle_upper);
        s_last_throttle_seq = seq;
    }
}

/* Motor RPM downlink to the GCS, sourced from the DShot driver's telemetry.
 * get_telemetry is a peek, so this is safe alongside the RISE/DESCENT
 * closed-loop read. valid=false when neither motor has fresh telemetry (not
 * spinning) so the GCS renders "—" rather than a stale value. */
static void publish_motor_rpm(void)
{
    esc_motor_telemetry_t lo, up;
    bool lo_ok = esc_dshot_motor_get_telemetry(ESC_MOTOR_ID_LOWER, &lo);
    bool up_ok = esc_dshot_motor_get_telemetry(ESC_MOTOR_ID_UPPER, &up);
    app_motor_rpm_t rpm = {
        .rpm_lower = lo_ok ? lo.rpm : 0.0f,
        .rpm_upper = up_ok ? up.rpm : 0.0f,
        .valid     = lo_ok || up_ok,
    };
    (void)state_exchange_publish_motor_rpm(&rpm);
}

#ifdef DEBUG_TEXT_CONSOLE
/* Signed fixed-point formatter (3 decimals) — the linked printf has no %f.
 * Same shape as fmt_f3 in cm4/state_estimation_task.c. */
static void fmt_f3(char *buf, float v) {
    int neg = (v < 0.0f);
    if (neg) v = -v;
    unsigned long m = (unsigned long)(v * 1000.0f + 0.5f);
    (void)snprintf(buf, 16, "%s%lu.%03lu", neg ? "-" : "", m / 1000UL, m % 1000UL);
}
#endif

static uint16_t esc_get_rpm_adjusted_throttle(float desired_rpm, float actual_rpm,
                                              uint16_t current_throttle)
{
    int32_t new_throttle; // TODO: don't use signed integer for this...

    if (actual_rpm > (desired_rpm + ESC_RPM_DEADBAND)) {
        new_throttle = current_throttle - ESC_RPM_THROTTLE_ADJUSTMENT;
    } else if (actual_rpm < (desired_rpm - ESC_RPM_DEADBAND)) {
        new_throttle = current_throttle + ESC_RPM_THROTTLE_ADJUSTMENT;
    } else {
        new_throttle = current_throttle;
    }

    return (uint16_t)MAX(0, new_throttle);
}

void task_controls(void *arg) {
    (void)arg;

    app_flight_state_t last_flight_state = APP_FLIGHT_IDLE;

    uint16_t motor_rpm_desired = 0;
    uint16_t motor_throttle_lower = 0;
    uint16_t motor_throttle_upper = 0;

    uint64_t rise_start_time = timestamp_us64();

#ifdef DEBUG_TEXT_CONSOLE
    unsigned dbg_tick = 0;
#endif

    for (;;) {
        poll_tunables();
        publish_motor_rpm();

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
            io_debug_printf("[ctl] T=%s gim=%s,%s z_i=%s\r\n", t, tx, ty, zi);
        }
#endif

        bool is_state_updated = false;
        app_flight_state_t flight_state = APP_FLIGHT_IDLE;
        (void)state_exchange_get_flight_state(&flight_state);

        if (flight_state != last_flight_state) {
            last_flight_state = flight_state;
            is_state_updated = true;
        }

        switch (flight_state) {
        case APP_FLIGHT_ARMED: {
            if (is_state_updated) {
                esc_dshot_set_armed(true);
            }
            break;
        }
        case APP_FLIGHT_RISE: {
            if (is_state_updated) {
                rise_start_time = timestamp_us64();
                motor_rpm_desired = ESC_RPM_DESIRED_RISE;
                motor_throttle_lower = ESC_THROTTLE_INITIAL;
                motor_throttle_upper = ESC_THROTTLE_INITIAL;
            }
            break;
        }
        case APP_FLIGHT_DESCENT: {
            if (is_state_updated) {
                motor_rpm_desired = ESC_RPM_DESIRED_LAND;
            }
            break;
        }
        default: {
            motor_rpm_desired = 0;
            motor_throttle_lower = 0;
            motor_throttle_upper = 0;
            esc_dshot_motor_set_throttle(ESC_MOTOR_ID_LOWER, ESC_THROTTLE_ZERO);
            esc_dshot_motor_set_throttle(ESC_MOTOR_ID_UPPER, ESC_THROTTLE_ZERO);
            esc_dshot_set_armed(false);
            break;
        }
        }

        if (flight_state == APP_FLIGHT_RISE || flight_state == APP_FLIGHT_DESCENT) {
            /* Manual throttle: a non-zero operator command overrides the
             * autonomous RPM-closed-loop value for that motor; 0 (default /
             * disarm) keeps the autonomous throttle. The closed loop keeps
             * adjusting motor_throttle_* from telemetry either way, so
             * clearing the override falls back without a step. */
            uint16_t cmd_lower =
                (s_operator_throttle_lower > 0U) ? s_operator_throttle_lower : motor_throttle_lower;
            uint16_t cmd_upper =
                (s_operator_throttle_upper > 0U) ? s_operator_throttle_upper : motor_throttle_upper;
            cmd_lower = MIN(cmd_lower, ESC_THROTTLE_MAX);
            cmd_upper = MIN(cmd_upper, ESC_THROTTLE_MAX);
            esc_dshot_motor_set_throttle(ESC_MOTOR_ID_LOWER, cmd_lower);
            esc_dshot_motor_set_throttle(ESC_MOTOR_ID_UPPER, cmd_upper);

            esc_motor_telemetry_t lower_motor_telemetry;
            esc_motor_telemetry_t upper_motor_telemetry;

            if (esc_dshot_motor_get_telemetry(ESC_MOTOR_ID_LOWER, &lower_motor_telemetry)) {
                char rpm[16];
                char throttle[16];
                fmt_f3(rpm, lower_motor_telemetry.rpm);
                fmt_f3(throttle, motor_throttle_lower);

                io_debug_printf("[ctl] esc-lower rpm=%s throttle=%s\r\n", rpm, throttle);

                motor_throttle_lower = esc_get_rpm_adjusted_throttle(
                    motor_rpm_desired, lower_motor_telemetry.rpm, motor_throttle_lower);
            }

            if (esc_dshot_motor_get_telemetry(ESC_MOTOR_ID_UPPER, &upper_motor_telemetry)) {
                char rpm[16];
                char throttle[16];
                fmt_f3(rpm, upper_motor_telemetry.rpm);
                fmt_f3(throttle, motor_throttle_upper);

                io_debug_printf("[ctl] esc-upper rpm=%s throttle=%s\r\n", rpm, throttle);

                motor_throttle_upper = esc_get_rpm_adjusted_throttle(
                    motor_rpm_desired, upper_motor_telemetry.rpm, motor_throttle_upper);
            }

            motor_throttle_lower = MIN(motor_throttle_lower, ESC_THROTTLE_MAX);
            motor_throttle_upper = MIN(motor_throttle_upper, ESC_THROTTLE_MAX);
        }

        vTaskDelay(pdMS_TO_TICKS(TUNABLE_POLL_MS));
    }
}
