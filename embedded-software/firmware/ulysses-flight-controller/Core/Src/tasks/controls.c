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
#include "state_exchange.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32h5xx_hal.h"
#include "state_estimation/state.h"
#include "mission_manager/mission_manager.h"
#include "controls/flight_controller.h"
#include "motor_drivers/servo_driver.h"
#include "motor_drivers/esc_driver.h"
#include "debug/log.h"
#include "SD_logging/log_service.h"
#include "timestamp.h"

#define RAD_TO_DEG (180.0f / 3.14159265f)

#define CONTROLS_DT_S 0.00125f  /**< Control period [s] (800 Hz via TIM4 CH2). */
#define STALE_STATE_THRESHOLD_TICKS 100  /**< If state_seq unchanged for this many ticks, treat as stale and output safe (zero). */

/* ── Startup test parameters ────────────────────────────────────────────── */
#define SWEEP_RANGE_DEG   90.0f   /**< Servo sweep half-range (degrees). */
#define SWEEP_STEP_DEG    1.0f    /**< Degrees per step. */
#define SWEEP_STEP_MS     5       /**< Milliseconds between steps. */
#define CIRCLE_RADIUS_DEG 72.0f   /**< Servo circle sweep radius (degrees). */
#define CIRCLE_STEPS      72      /**< Steps per full revolution (5 deg each). */
#define CIRCLE_STEP_MS    18      /**< Milliseconds between circle steps. */
#define ESC_TEST_THRUST   0.10f   /**< Motor test thrust (10%). */
#define ESC_RAMP_STEPS    50      /**< Steps to ramp up/down. */
#define ESC_RAMP_STEP_MS  10      /**< Milliseconds per ramp step. */
#define ESC_HOLD_MS       500     /**< Hold at peak thrust (ms). */

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
    cfg->gimbal.theta_min = -0.05f;
    cfg->gimbal.theta_max = 0.05f;
    /* Thrust */
    cfg->thrust.m = 1.0f;
    cfg->thrust.g = 9.8067f;
    cfg->thrust.T_min = 0.0f;
    cfg->thrust.T_max = 50.0f;
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

/* ── Startup actuator test ────────────────────────────────────────────── */

/** Sweep a single servo through center → +max → center → -max → center. */
static void sweep_servo(int servo_index)
{
    const int steps = (int)(SWEEP_RANGE_DEG / SWEEP_STEP_DEG);

    /* center → +max */
    for (int i = 0; i <= steps; i++) {
        float angle = SWEEP_STEP_DEG * (float)i;
        if (servo_index == 0)
            set_servo_pair_degrees(angle, 0.0f);
        else
            set_servo_pair_degrees(0.0f, angle);
        osDelay(SWEEP_STEP_MS);
    }

    /* +max → center */
    for (int i = steps; i >= 0; i--) {
        float angle = SWEEP_STEP_DEG * (float)i;
        if (servo_index == 0)
            set_servo_pair_degrees(angle, 0.0f);
        else
            set_servo_pair_degrees(0.0f, angle);
        osDelay(SWEEP_STEP_MS);
    }

    /* center → -max */
    for (int i = 0; i <= steps; i++) {
        float angle = -SWEEP_STEP_DEG * (float)i;
        if (servo_index == 0)
            set_servo_pair_degrees(angle, 0.0f);
        else
            set_servo_pair_degrees(0.0f, angle);
        osDelay(SWEEP_STEP_MS);
    }

    /* -max → center */
    for (int i = steps; i >= 0; i--) {
        float angle = -SWEEP_STEP_DEG * (float)i;
        if (servo_index == 0)
            set_servo_pair_degrees(angle, 0.0f);
        else
            set_servo_pair_degrees(0.0f, angle);
        osDelay(SWEEP_STEP_MS);
    }
}

/** Trace one full clockwise circle with both servos simultaneously. */
static void sweep_circle(void)
{
    for (int i = 0; i <= CIRCLE_STEPS; i++) {
        float angle_rad = (2.0f * 3.14159265f * (float)i) / (float)CIRCLE_STEPS;
        float x = CIRCLE_RADIUS_DEG * cosf(angle_rad);
        float y = CIRCLE_RADIUS_DEG * sinf(angle_rad);
        set_servo_pair_degrees(x, y);
        osDelay(CIRCLE_STEP_MS);
    }
    /* Return to centre */
    set_servo_pair_degrees(0.0f, 0.0f);
    osDelay(50);
}

/** Smooth ramp a single motor: 0 → peak → hold → 0. */
static void ramp_motor(int motor_index)
{
    /* Ramp up */
    for (int i = 0; i <= ESC_RAMP_STEPS; i++) {
        float t = ESC_TEST_THRUST * (float)i / (float)ESC_RAMP_STEPS;
        if (motor_index == 0)
            ESC_set_pair_thrust(t, 0.0f);
        else
            ESC_set_pair_thrust(0.0f, t);
        osDelay(ESC_RAMP_STEP_MS);
    }

    /* Hold */
    osDelay(ESC_HOLD_MS);

    /* Ramp down */
    for (int i = ESC_RAMP_STEPS; i >= 0; i--) {
        float t = ESC_TEST_THRUST * (float)i / (float)ESC_RAMP_STEPS;
        if (motor_index == 0)
            ESC_set_pair_thrust(t, 0.0f);
        else
            ESC_set_pair_thrust(0.0f, t);
        osDelay(ESC_RAMP_STEP_MS);
    }
}

/**
 * @brief Run the startup actuator test sequence.
 *
 * Sweeps each servo through its full range one at a time, then briefly
 * spins each motor up to ESC_TEST_THRUST and back.  Total duration ~8 s.
 * The TIM4 ISR is already applying servo/ESC values in the background.
 */
static void run_startup_actuator_test(void)
{
    DLOG_PRINT("[CTRL] Startup actuator test begin\r\n");

    /* ── Servo test ── */
    servo_pair_enable(true);
    set_servo_pair_degrees(0.0f, 0.0f);

    DLOG_PRINT("[CTRL] Servo 1 sweep\r\n");
    sweep_servo(0);

    DLOG_PRINT("[CTRL] Servo 2 sweep\r\n");
    sweep_servo(1);

    DLOG_PRINT("[CTRL] Servo circle\r\n");
    sweep_circle();

    /* Return to center and disable */
    set_servo_pair_degrees(0.0f, 0.0f);
    osDelay(50);
    servo_pair_enable(false);

    /* ── Motor test ── */
    ESC_pair_arm();
    osDelay(7000);

    DLOG_PRINT("[CTRL] Motor 1 ramp\r\n");
    ramp_motor(0);

    DLOG_PRINT("[CTRL] Motor 2 ramp\r\n");
    ramp_motor(1);

    /* Shut down */
    ESC_set_pair_thrust(0.0f, 0.0f);
    osDelay(50);
    ESC_pair_disarm();

    DLOG_PRINT("[CTRL] Startup actuator test complete\r\n");
    state_exchange_publish_startup_test_complete(true);
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
    uint8_t config_done = 0;
    uint32_t last_state_seq = 0;
    uint32_t stale_tick_count = 0;
    bool esc_running = false;
    uint32_t esc_arm_tick = 0;
    bool last_armed = false;

    init_default_config(&config);
    init_default_ref(&ref);

    /* Run cinematic startup test before entering the control loop. */
    run_startup_actuator_test();

    for (;;) {
        /* Block until TIM4 CH2 output-compare ISR fires (see timing.c) */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* Re-arm request: safe actuators, redo startup test, then arm. */
        bool rearm = false;
        state_exchange_get_rearm_request(&rearm);
        if (rearm) {
            state_exchange_publish_rearm_request(false);
            DLOG_PRINT("[CTRL] Rearm: starting startup sequence\r\n");

            servo_pair_enable(false);
            ESC_set_pair_thrust(0.0f, 0.0f);
            ESC_pair_disarm();
            esc_running = false;
            esc_arm_tick = 0;

            run_startup_actuator_test();

            flight_controller_init(&config);
            config_done = 1;

            state_exchange_publish_armed(true);
            DLOG_PRINT("[CTRL] Rearm complete, armed\r\n");
            continue;
        }

        uint32_t state_seq = state_exchange_get_state(&current_state);
        state_exchange_get_flight_state(&flight_state);

        bool armed = false;
        state_exchange_get_armed(&armed);

        if (armed != last_armed) {
            log_service_log_event(LOG_EVENT_CODE_ARM_STATE,
                                  (uint16_t)(armed ? 1U : 0U),
                                  timestamp_us());
            last_armed = armed;
        }

        if (!config_done) {
            flight_controller_init(&config);
            config_done = 1;
        }

        /* Staleness: if state sequence has not changed for N ticks, treat as stale. */
        if (state_seq == last_state_seq) {
            stale_tick_count++;
        } else {
            last_state_seq = state_seq;
            stale_tick_count = 0;
        }
        //bool state_stale = (stale_tick_count >= STALE_STATE_THRESHOLD_TICKS);

        /* Do not run controller unless armed with valid state. */
        if (!armed || state_seq == 0 ){//|| state_stale) {
            memset(&control_output, 0, sizeof(control_output));
        } else {
            flight_controller_run(&current_state, &ref, &config, &control_output, CONTROLS_DT_S);
        }
        state_exchange_publish_control_output(&control_output);

        /* Log control output at 100 Hz (every 8th 800-Hz cycle). */
        static uint8_t ctrl_log_div = 0;
        if (++ctrl_log_div >= 8U) {
            ctrl_log_div = 0;
            log_service_log_control_output(
                timestamp_us(),
                control_output.T_cmd,
                control_output.theta_x_cmd,
                control_output.theta_y_cmd,
                control_output.tau_gim[0],
                control_output.tau_gim[1],
                control_output.tau_gim[2],
                control_output.tau_thrust,
                control_output.phi_x,
                control_output.phi_y,
                control_output.phi_z,
                control_output.z_pid_integral,
                ref.z_ref,
                ref.vz_ref
            );
        }

        /* Drive actuators only when armed. */
        if (armed) {
            /* Gimbal locked out post-startup: hold centre and keep disabled. */
            set_servo_pair_degrees(0.0f, 0.0f);
            servo_pair_enable(false);

            /* ESC: arm once on RISE entry, hold min throttle for the same
             * 7 s init window the startup sequence uses, then run at 10%.
             * Disarm once on exit. */
            if (flight_state == RISE) {
                if (!esc_running) {
                    ESC_pair_arm();
                    esc_arm_tick = HAL_GetTick();
                    esc_running = true;
                }
                if ((HAL_GetTick() - esc_arm_tick) >= 7000UL) {
                    ESC_set_pair_thrust(0.10f, 0.10f);
                }
            } else {
                if (esc_running) {
                    ESC_set_pair_thrust(0.0f, 0.0f);
                    ESC_pair_disarm();
                    esc_running = false;
                }
            }
        } else {
            set_servo_pair_degrees(0.0f, 0.0f);
            servo_pair_enable(false);
            if (esc_running) {
                ESC_set_pair_thrust(0.0f, 0.0f);
                ESC_pair_disarm();
                esc_running = false;
            }
        }
    }
}
