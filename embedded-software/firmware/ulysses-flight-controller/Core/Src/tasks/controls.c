/**
 * @file    controls.c
 * @brief   Controls task: runs simple tilt-balance loop at 800 Hz, drives gimbal servos.
 *
 * Uses control_loop (two independent pitch/roll PIDs on body tilt angles
 * extracted from q_bn) to command the gimbal servos.  The ESCs are held at
 * a fixed constant thrust while armed and in RISE state.
 *
 * On startup, waits for the ESC power-on sequence before entering the main
 * loop.  All actuator output is gated on the armed flag.
 */
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "cmsis_os2.h"
#include "projdefs.h"
#include "state_exchange.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "state_estimation/state.h"
#include "mission_manager/mission_manager.h"
#include "controls/control_loop.h"
#include "motor_drivers/servo_driver.h"
#include "motor_drivers/esc_driver.h"
#include "debug/log.h"
#include "SD_logging/log_service.h"
#include "timestamp.h"

#define CONTROLS_DT_S 0.00125f /**< Control period [s] (800 Hz via TIM4 CH2). */

/* Empirical times for the ESC power on sequence */
#define ESC_POWER_ON_TIME_MS (3000) /**< Delay before arming sequence begins */

/* Empirical values for a safe operational range of the gimbal */
#define SERVO_MAX_DEGREES (100)
#define SERVO_MIN_DEGREES (-100)

/**
 * Constant thrust applied to both ESCs while the balancing loop is active.
 * Set this to the desired hover/test thrust in Newtons before flight.
 */
#define CONTROL_LOOP_CONST_THRUST_N (0.0f)

static inline float clampf(float value, float minimum, float maximum)
{
    return fminf(fmaxf(value, minimum), maximum);
}

/* ── Task entry ───────────────────────────────────────────────────────── */

/**
 * @brief FreeRTOS entry: 1.25 ms period (800 Hz via TIM4 CH2).
 *        Reads state → runs control_loop → drives servos and ESCs.
 * @param argument Unused.
 */
void controls_task_start(void *argument)
{
    (void)argument;

    state_t current_state = {0};
    flight_state_t flight_state = IDLE;

    control_loop_config_t cfg = {
        .x = {
            .kp             = 1.0f,
            .ki             = 0.0f,
            .kd             = 0.0f,
            .integral_limit = 2.0f,
            .theta_min_rad  = (float)(SERVO_MIN_DEGREES) * (float)M_PI / 180.0f,
            .theta_max_rad  = (float)(SERVO_MAX_DEGREES) * (float)M_PI / 180.0f,
            .sign           = -1.0f,   /* set to -1.0f to flip x-axis */
        },
        .y = {
            .kp             = 1.0f,
            .ki             = 0.0f,
            .kd             = 0.0f,
            .integral_limit = 2.0f,
            .theta_min_rad  = (float)(SERVO_MIN_DEGREES) * (float)M_PI / 180.0f,
            .theta_max_rad  = (float)(SERVO_MAX_DEGREES) * (float)M_PI / 180.0f,
            .sign           = 1.0f,   /* set to -1.0f to flip y-axis */
        },
    };

    bool armed;
    uint32_t last_armed_seq = 0;
    uint32_t last_pid_gains_seq = 0;

    uint8_t ctrl_log_div = 0;

    servo_pair_enable(false);
    esc_pair_set_armed(false);

    osDelay(pdMS_TO_TICKS(ESC_POWER_ON_TIME_MS));

    state_exchange_publish_startup_test_complete(true);

    for (;;) {
        /* Block until TIM4 CH2 output-compare ISR fires (see timing.c) */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        uint32_t armed_seq = state_exchange_get_armed(&armed);

        if (armed_seq != last_armed_seq) {
            last_armed_seq = armed_seq;

            DLOG_PRINT("[CTRL] ARM: begin %s sequence\r\n", armed ? "arm" : "disarm");

            if (armed) {
                control_loop_init(&cfg);
            }

            set_servo_pair_degrees(0, 0);
            servo_pair_enable(armed);

            esc_pair_set_force(0, 0);
            esc_pair_set_armed(armed);

            DLOG_PRINT("[CTRL] ARM: end %s sequence\r\n", armed ? "arm" : "disarm");

            log_service_log_event(&(log_record_event_t){
                .timestamp_us = timestamp_us(),
                .event_code   = LOG_EVENT_CODE_ARM_STATE,
                .data_u16     = armed ? 1 : 0,
            });
        }

        /* Check for updated PID gains from GCS. attitude_kp.x/y → kp for each
         * axis, attitude_kd.x/y → kd.  Ki is ignored for now. */
        tvr_SetPidGains pid_gains;
        uint32_t pid_gains_seq = state_exchange_get_pid_gains(&pid_gains);
        if (pid_gains_seq != 0 && pid_gains_seq != last_pid_gains_seq) {
            last_pid_gains_seq = pid_gains_seq;

            if (pid_gains.has_attitude_kp) {
                cfg.x.kp = pid_gains.attitude_kp.x;
                cfg.y.kp = pid_gains.attitude_kp.y;
            }
            if (pid_gains.has_attitude_kd) {
                cfg.x.kd = pid_gains.attitude_kd.x;
                cfg.y.kd = pid_gains.attitude_kd.y;
            }

            control_loop_init(&cfg);
            DLOG_PRINT("[CTRL] PID gains updated: kp=%.3f/%.3f kd=%.3f/%.3f\r\n",
                       cfg.x.kp, cfg.y.kp, cfg.x.kd, cfg.y.kd);
        }

        uint32_t state_seq = state_exchange_get_state(&current_state);
        state_exchange_get_flight_state(&flight_state);

        if (state_seq != 0 && armed && flight_state == RISE) {
            control_loop_output_t out;
            control_loop_run(&current_state, &cfg, &out, CONTROLS_DT_S);

            /* Log at 100 Hz (every 8th 800-Hz cycle).
             * Reuse the existing log schema: populate only the tilt/gimbal
             * fields; zero-fill the unused flight-controller fields. */
            if (++ctrl_log_div >= 8U) {
                ctrl_log_div = 0;
                log_service_log_control_output(&(log_record_control_output_t){
                    .timestamp_us   = timestamp_us(),
                    .T_cmd          = CONTROL_LOOP_CONST_THRUST_N,
                    .theta_x_cmd    = out.theta_x_cmd,
                    .theta_y_cmd    = out.theta_y_cmd,
                    .tau_gim_x      = 0.0f,
                    .tau_gim_y      = 0.0f,
                    .tau_gim_z      = 0.0f,
                    .tau_thrust     = 0.0f,
                    .phi_x          = out.phi_x,
                    .phi_y          = out.phi_y,
                    .phi_z          = 0.0f,
                    .z_pid_integral = 0.0f,
                    .z_ref          = 0.0f,
                    .vz_ref         = 0.0f,
                });
            }

            /* Rotate gimbal commands by 45° about z to convert from IMU body
             * frame to gimbal servo frame (servos are mounted 45° rotated). */
            #define GIMBAL_ROTATION_RAD (M_PI / 4.0f)
            float cos_r = cosf(GIMBAL_ROTATION_RAD);
            float sin_r = sinf(GIMBAL_ROTATION_RAD);
            float tx_rot = cos_r * out.theta_x_cmd - sin_r * out.theta_y_cmd;
            float ty_rot = sin_r * out.theta_x_cmd + cos_r * out.theta_y_cmd;

            float theta_x_cmd_safe = clampf(tx_rot * 180.0f / (float)M_PI,
                                            SERVO_MIN_DEGREES, SERVO_MAX_DEGREES);
            float theta_y_cmd_safe = clampf(ty_rot * 180.0f / (float)M_PI,
                                            SERVO_MIN_DEGREES, SERVO_MAX_DEGREES);

            set_servo_pair_degrees(theta_x_cmd_safe, -theta_y_cmd_safe);
            esc_pair_set_force(CONTROL_LOOP_CONST_THRUST_N, 0.0f);
        }
    }
}
