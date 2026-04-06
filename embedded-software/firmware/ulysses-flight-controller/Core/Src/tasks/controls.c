/**
 * @file    controls.c
 * @brief   Controls task: runs flight controller at 800 Hz, publishes control output.
 *
 * On startup, runs a cinematic actuator test sequence (servos then motors)
 * before entering the main loop.  The main loop gates all actuator output
 * on the armed flag — set by the mission manager after CMD_ARM is received.
 */
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "cmsis_os2.h"
#include "command.pb.h"
#include "common.pb.h"
#include "projdefs.h"
#include "state_exchange.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "state_estimation/state.h"
#include "mission_manager/mission_manager.h"
#include "controls/flight_controller.h"
#include "motor_drivers/servo_driver.h"
#include "motor_drivers/esc_driver.h"
#include "debug/log.h"
#include "SD_logging/log_service.h"
#include "timestamp.h"

#define RAD_TO_DEG (180.0f / 3.14159265f)

#define CONTROLS_DT_S 0.00125f /**< Control period [s] (800 Hz via TIM4 CH2). */
#define STALE_STATE_THRESHOLD_TICKS \
    100 /**< If state_seq unchanged for this many ticks, treat as stale and output safe (zero). */

/* ── Startup test parameters ────────────────────────────────────────────── */
#define SWEEP_RANGE_DEG   90.0f         /**< Servo sweep half-range (degrees). */
#define SWEEP_STEP_DEG    1.0f          /**< Degrees per step. */
#define SWEEP_STEP_MS     5             /**< Milliseconds between steps. */
#define CIRCLE_RADIUS_DEG 72.0f         /**< Servo circle sweep radius (degrees). */
#define CIRCLE_STEPS      72            /**< Steps per full revolution (5 deg each). */
#define CIRCLE_STEP_MS    18            /**< Milliseconds between circle steps. */
#define ESC_TEST_THRUST   (0.1f * 1200) /**< Motor test thrust (10%). */
#define ESC_RAMP_STEPS    50            /**< Steps to ramp up/down. */
#define ESC_RAMP_STEP_MS  10            /**< Milliseconds per ramp step. */
#define ESC_HOLD_MS       500           /**< Hold at peak thrust (ms). */

/* Empirical times for the ESC power on sequence*/
#define ESC_POWER_ON_TIME_MS (3000) /**< Delay before arming sequence begins */
#define ESC_ARM_TIME_MS      (3000) /**< Delay before PWM output */

/* Empirical values for a safe operational range of the gimbal */
#define SERVO_MAX_DEGREES (100)
#define SERVO_MIN_DEGREES (-100)

// TODO: remove me when we get empirical mech values, used for control testing
#define ROCKET_ESTIMATED_MASS_KG (1200.0 / 1000.0)
#define ROCKET_MAX_THRUST_KG     (800.0 / 1000.0)

static quaternion_t pb_to_fc_quaternion(tvr_Quaternion quaternion);
static inline float clampf(float value, float minimum, float maximum);

/** Fill config with default gains and limits (tune in use). */
static void init_default_config(flight_controller_config_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    /* Attitude: diagonal gains and identity-like inertia (tune in use) */
    cfg->attitude.Kp[0][0] = 1.0f;
    cfg->attitude.Kp[1][1] = 1.0f;
    cfg->attitude.Kp[2][2] = 1.0f;
    cfg->attitude.Kd[0][0] = 0.1f;
    cfg->attitude.Kd[1][1] = 0.1f;
    cfg->attitude.Kd[2][2] = 0.1f;
    cfg->attitude.I[0][0] = 0.01f;
    cfg->attitude.I[1][1] = 0.01f;
    cfg->attitude.I[2][2] = 0.01f;
    /* Allocation: thrust along -z body (z-up) */
    cfg->allocation.t_hat[0] = 0.0f;
    cfg->allocation.t_hat[1] = 0.0f;
    cfg->allocation.t_hat[2] = -1.0f;
    /* Gimbal */
    cfg->gimbal.L = 0.2f;
    cfg->gimbal.theta_min = SERVO_MIN_DEGREES / 180.0 * M_PI;
    cfg->gimbal.theta_max = SERVO_MAX_DEGREES / 180.0 * M_PI;
    /* Thrust */
    cfg->thrust.m = ROCKET_ESTIMATED_MASS_KG;
    cfg->thrust.g = 9.8067f;
    cfg->thrust.T_min = 0.0f;
    cfg->thrust.T_max = ROCKET_MAX_THRUST_KG * (double)cfg->thrust.g;
    cfg->thrust.kp = 2.0f;
    cfg->thrust.ki = 0.5f;
    cfg->thrust.kd = 1.0f;
    cfg->thrust.integral_limit = 2.0f;
    cfg->thrust.a_z_min = -20.0f;
    cfg->thrust.a_z_max = 20.0f;
}

/** Set ref to identity attitude and zero z setpoint (hover). */
static void init_default_ref(flight_controller_ref_t *ref)
{
    ref->q_ref.w = 1.0f;
    ref->q_ref.x = 0.0f;
    ref->q_ref.y = 0.0f;
    ref->q_ref.z = 0.0f;
    ref->z_ref = 0.0f;
    ref->vz_ref = 0.0f;
}

/* ── Task entry ───────────────────────────────────────────────────────── */

/**
 * @brief FreeRTOS entry: 1.25 ms period (800 Hz via TIM4 CH2), get state -> flight_controller_run -> publish control output.
 * @param argument Unused.
 */
void controls_task_start(void *argument)
{
    (void)argument;

    state_t current_state = {0};
    flight_state_t flight_state = IDLE;
    flight_controller_config_t config = {0};
    flight_controller_ref_t ref = {0};

    control_output_t control_output = {0};

    bool armed;
    uint32_t last_armed_seq = 0;

    uint8_t ctrl_log_div = 0;

    servo_pair_enable(false);
    esc_pair_set_armed(false);

    osDelay(pdMS_TO_TICKS(ESC_POWER_ON_TIME_MS));

    state_exchange_publish_startup_test_complete(true);

    for (;;) {
        /* Block until TIM4 CH2 output-compare ISR fires (see timing.c) */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        uint32_t armed_seq = state_exchange_get_armed(&armed);

        // Arming status has changed
        if (armed_seq != last_armed_seq) {
            last_armed_seq = armed_seq;

            DLOG_PRINT("[CTRL] ARM: begin %s sequence\r\n", armed ? "arm" : "disarm");

            if (armed) {
                init_default_config(&config);
                init_default_ref(&ref);

                tvr_SetPidGains pid_gains;
                tvr_SetReference flight_reference;
                tvr_SetConfig vehicle_config;

                uint32_t pid_gains_seq = state_exchange_get_pid_gains(&pid_gains);
                uint32_t flight_reference_seq =
                    state_exchange_get_flight_reference(&flight_reference);
                uint32_t vehicle_config_seq = state_exchange_get_vehicle_config(&vehicle_config);

                if (pid_gains_seq != 0) {
                    if (pid_gains.has_attitude_kp) {
                        config.attitude.Kp[0][0] = pid_gains.attitude_kp.x;
                        config.attitude.Kp[1][1] = pid_gains.attitude_kp.y;
                        config.attitude.Kp[2][2] = pid_gains.attitude_kp.z;
                    }

                    if (pid_gains.has_attitude_kd) {
                        config.attitude.Kd[0][0] = pid_gains.attitude_kd.x;
                        config.attitude.Kd[1][1] = pid_gains.attitude_kd.y;
                        config.attitude.Kd[2][2] = pid_gains.attitude_kd.z;
                    }

                    config.thrust.kp = pid_gains.z_kp;
                    config.thrust.ki = pid_gains.z_ki;
                    config.thrust.kd = pid_gains.z_kd;
                    config.thrust.integral_limit = pid_gains.z_integral_limit;
                }

                if (flight_reference_seq != 0) {
                    if (flight_reference.has_q_ref) {
                        ref.q_ref = pb_to_fc_quaternion(flight_reference.q_ref);
                    }

                    ref.z_ref = flight_reference.z_ref;
                    ref.vz_ref = flight_reference.vz_ref;
                }

                if (vehicle_config_seq != 0) {
                    config.thrust.m = vehicle_config.mass;
                    config.thrust.T_min = vehicle_config.T_min;
                    config.thrust.T_max = vehicle_config.T_max;
                    config.gimbal.theta_min = vehicle_config.theta_min;
                    config.gimbal.theta_max = vehicle_config.theta_max;
                }

                flight_controller_init(&config);
            }

            set_servo_pair_degrees(0, 0);
            servo_pair_enable(armed);

            esc_pair_set_force(0, 0);
            esc_pair_set_armed(armed);

            DLOG_PRINT("[CTRL] ARM: end %s sequence\r\n", armed ? "arm" : "disarm");

            log_service_log_event(&(log_record_event_t){
                .timestamp_us = timestamp_us(),
                .event_code = LOG_EVENT_CODE_ARM_STATE,
                .data_u16 = armed ? 1 : 0,
            });
        }

        uint32_t state_seq = state_exchange_get_state(&current_state);
        state_exchange_get_flight_state(&flight_state);

        if (state_seq != 0 && armed && flight_state == RISE) {
            flight_controller_run(&current_state, &ref, &config, &control_output, CONTROLS_DT_S);

            state_exchange_publish_control_output(&control_output);

            /* Log control output at 100 Hz (every 8th 800-Hz cycle). */
            if (++ctrl_log_div >= 8U) {
                ctrl_log_div = 0;
                log_service_log_control_output(&(log_record_control_output_t){
                    .timestamp_us = timestamp_us(),
                    .T_cmd = control_output.T_cmd,
                    .theta_x_cmd = control_output.theta_x_cmd,
                    .theta_y_cmd = control_output.theta_y_cmd,
                    .tau_gim_x = control_output.tau_gim[0],
                    .tau_gim_y = control_output.tau_gim[1],
                    .tau_gim_z = control_output.tau_gim[2],
                    .tau_thrust = control_output.tau_thrust,
                    .phi_x = control_output.phi_x,
                    .phi_y = control_output.phi_y,
                    .phi_z = control_output.phi_z,
                    .z_pid_integral = control_output.z_pid_integral,
                    .z_ref = ref.z_ref,
                    .vz_ref = ref.vz_ref,
                });
            }

            // FIXME: artificially limit the operational range of the gimbal since the propellers
            // could hit the landing legs in the current design
            float theta_x_cmd_safe = clampf((double)control_output.theta_x_cmd * 180.0 / M_PI,
                                            SERVO_MIN_DEGREES, SERVO_MAX_DEGREES);
            float theta_y_cmd_safe = clampf((double)control_output.theta_y_cmd * 180.0 / M_PI,
                                            SERVO_MIN_DEGREES, SERVO_MAX_DEGREES);

            set_servo_pair_degrees(theta_x_cmd_safe, -theta_y_cmd_safe);
            esc_pair_set_force(control_output.T_cmd, control_output.tau_thrust);
        }
    }
}

/**
 * Converts a protobuf quaternion to a flight controller quaternion.
 *
 * @param quaternion Protobuf quaternion
 * @return quaternion_t
 */
static quaternion_t pb_to_fc_quaternion(tvr_Quaternion quaternion)
{
    return (quaternion_t){
        .w = quaternion.w,
        .x = quaternion.x,
        .y = quaternion.y,
        .z = quaternion.z,
    };
}

static inline float clampf(float value, float minimum, float maximum)
{
    return fminf(fmaxf(value, minimum), maximum);
}
