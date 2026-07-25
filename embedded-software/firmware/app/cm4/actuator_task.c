/**
 * @file    actuator_task.c
 * @brief   CM4: owns the UART8 servo bus and gimbal control loop.
 *
 * Dynamixel: tilt KF (state_estimation_task) -> tilt_state -> per-axis PID here.
 * Feetech: follows control_output_t from CM7 (full TVC path).
 *
 * UBC Rocket, 2026
 */

#include "app/tasks.h"
#include "app/state_exchange.h"
#include "app/actuators_init.h"
#include "tilt_state.h"

#include "io_sys/io_intercore.h"

#ifdef USE_DYNAMIXEL_SERVO
#include "dev_servo_dynamixel.h"
#else
#include "dev_servo_feetech.h"
#endif

#include "controls/flight_controller.h"
#include "controls/pid.h"

#include "FreeRTOS.h"
#include "task.h"

#include <math.h>
#include <stdbool.h>

#ifndef GIMBAL_CALIB_MODE
#define GIMBAL_CALIB_MODE          0   /* 1 = torque off, log position for zero calibration */
#endif

/* Single-axis ZN tuning: force one servo to 0° regardless of PID output. */
#ifndef GIMBAL_HOLD_X_AT_ZERO
#define GIMBAL_HOLD_X_AT_ZERO      0
#endif
#ifndef GIMBAL_HOLD_Y_AT_ZERO
#define GIMBAL_HOLD_Y_AT_ZERO      0
#endif

#if GIMBAL_CALIB_MODE || (defined(DEBUG_TEXT_CONSOLE) && !GIMBAL_CALIB_MODE)
#include "io_sys/io_debug.h"
#include <stdio.h>
#endif

#define GIMBAL_BUILD_TAG           "gimbal-pid-v1"

#define ACTUATOR_LOOP_MS           20U
#define ACTUATOR_LOOP_S            (ACTUATOR_LOOP_MS / 1000.0f)
#define GIMBAL_CALIB_LOG_MS        200U
#define ACTUATOR_MIN_WRITE_MS      20U   /* was 50 ms (~20 Hz); small step to ~25 Hz */
#define ACTUATOR_FAIL_BACKOFF_MS   250U
#define GIMBAL_CLAMP_DEG           10.0f
#define GIMBAL_DEFAULT_DEG_X       0.0f
#define GIMBAL_DEFAULT_DEG_Y       0.0f

/* Per-axis PID (KF y -> Dynamixel X, KF x -> Dynamixel Y). */
// NOTE: THESE VALUES WILL BE OVERWRITTEN, LEFT TEMPORARILY ONLY FOR TESTING
// delete if gc works
#define TILT_KI_X                  0.00f
#define TILT_KI_Y                  0.00f
#define TILT_INTEGRAL_LIMIT        1.0f   /* anti-windup [rad*s] */
#define GIMBAL_CLAMP_RAD           (GIMBAL_CLAMP_DEG * ((float)M_PI / 180.0f))

#define GIMBAL_CMD_DEADBAND_DEG    0.05f

#define RAD2DEG                    (180.0f / (float)M_PI)

#define DXL_SERVO_ID_X             1u
#define DXL_SERVO_ID_Y             2u

#if GIMBAL_CALIB_MODE || defined(DEBUG_TEXT_CONSOLE)
static void fmt_f3(char *buf, float v)
{
    const bool neg = (v < 0.0f);
    const uint32_t m = (uint32_t)(neg ? (-v * 1000.0f) : (v * 1000.0f));
    (void)snprintf(buf, 16, "%s%lu.%03lu", neg ? "-" : "", m / 1000UL, m % 1000UL);
}
#endif

static inline float actuator_clamp_deg(float deg)
{
    if (deg > GIMBAL_CLAMP_DEG) return GIMBAL_CLAMP_DEG;
    if (deg < -GIMBAL_CLAMP_DEG) return -GIMBAL_CLAMP_DEG;
    return deg;
}

static inline bool actuator_state_uses_default(app_flight_state_t state)
{
    return state == APP_FLIGHT_IDLE ||
           state == APP_FLIGHT_ARMED ||
           state == APP_FLIGHT_ESTOP;
}

#ifdef USE_DYNAMIXEL_SERVO
static pid_controller_t s_pid_x;
static pid_controller_t s_pid_y;

static void actuator_pid_init(void)
{
    /* X: negated gains preserve +Kp/Kd on KF y (inverted mount). */
    pid_init(&s_pid_x,
             0, -TILT_KI_X, 0,
             TILT_INTEGRAL_LIMIT,
             -GIMBAL_CLAMP_RAD, GIMBAL_CLAMP_RAD);
    pid_init(&s_pid_y,
             0, TILT_KI_Y, 0,
             TILT_INTEGRAL_LIMIT,
             -GIMBAL_CLAMP_RAD, GIMBAL_CLAMP_RAD);
}

static void updateConfiguration() {
    /* Static so a failed slot read (writer mid-update / not yet published)
     * keeps the last-applied gains instead of stomping them with whatever is
     * on the stack — the getter leaves *out untouched on failure. */
    static app_pid_gains_t pidGains = {0};
    state_exchange_get_pid_gains(&pidGains);

    s_pid_x.kd = -pidGains.attitude_kd[0];
    s_pid_y.kd = pidGains.attitude_kd[1];

    s_pid_x.kp = -pidGains.attitude_kp[0];
    s_pid_y.kp = pidGains.attitude_kp[1];

    // stupid chud ground control station doesn't send I apparently
    // s_pid_x.ki = pidGains.atti[0];
    // s_pid_y.ki = pidGains.attitude_kp[1];
}

/* Dynamixel X: PID on KF y (inverted mount). Dynamixel Y: PID on KF x. */
static inline float gimbal_cmd_x_rad(const tilt_state_t *tilt)
{
    return pid_compute(&s_pid_x, 0.0f, tilt->tilt_y_rad, ACTUATOR_LOOP_S);
}

static inline float gimbal_cmd_y_rad(const tilt_state_t *tilt)
{
    return pid_compute(&s_pid_y, 0.0f, tilt->tilt_x_rad, ACTUATOR_LOOP_S);
}

static inline void tilt_to_gimbal_deg(const tilt_state_t *tilt,
                                      float *deg_x, float *deg_y)
{
    *deg_x = actuator_clamp_deg(gimbal_cmd_x_rad(tilt) * RAD2DEG);
    *deg_y = actuator_clamp_deg(gimbal_cmd_y_rad(tilt) * RAD2DEG);
}

static float s_last_deg_x = GIMBAL_DEFAULT_DEG_X;
static float s_last_deg_y = GIMBAL_DEFAULT_DEG_Y;
static uint32_t s_consecutive_fail;
static TickType_t s_last_write_tick;
static TickType_t s_backoff_until_tick;

static bool actuator_bus_may_run(void)
{
    const TickType_t now = xTaskGetTickCount();
    if (s_consecutive_fail > 0U && now < s_backoff_until_tick) {
        return false;
    }
    if ((now - s_last_write_tick) < pdMS_TO_TICKS(ACTUATOR_MIN_WRITE_MS)) {
        return false;
    }
    return true;
}

static bool actuator_axis_changed(float deg, float last_deg)
{
    return fabsf(deg - last_deg) > GIMBAL_CMD_DEADBAND_DEG;
}

static void actuator_apply_default(servo_dynamixel_t *d)
{
    /* Do not carry PID history from a previous launch into the next one. */
    pid_reset(&s_pid_x);
    pid_reset(&s_pid_y);

    if ((!actuator_axis_changed(GIMBAL_DEFAULT_DEG_X, s_last_deg_x) &&
         !actuator_axis_changed(GIMBAL_DEFAULT_DEG_Y, s_last_deg_y)) ||
        !actuator_bus_may_run()) {
        return;
    }

    if (servo_dynamixel_set_pair_degrees(d,
                                         GIMBAL_DEFAULT_DEG_X,
                                         GIMBAL_DEFAULT_DEG_Y)) {
        s_last_deg_x = GIMBAL_DEFAULT_DEG_X;
        s_last_deg_y = GIMBAL_DEFAULT_DEG_Y;
        s_consecutive_fail = 0U;
        s_last_write_tick = xTaskGetTickCount();
    } else {
        s_consecutive_fail++;
        s_backoff_until_tick =
            xTaskGetTickCount() + pdMS_TO_TICKS(ACTUATOR_FAIL_BACKOFF_MS);
        servo_dynamixel_bus_recover();
    }
}

static void actuator_apply_tilt_pid(servo_dynamixel_t *d)
{
    tilt_state_t tilt;
    if (!tilt_state_get(&tilt)) {
        actuator_apply_default(d);
        return;
    }

    float deg_x;
    float deg_y;
    tilt_to_gimbal_deg(&tilt, &deg_x, &deg_y);

#if GIMBAL_HOLD_X_AT_ZERO
    deg_x = GIMBAL_DEFAULT_DEG_X;
#endif
#if GIMBAL_HOLD_Y_AT_ZERO
    deg_y = GIMBAL_DEFAULT_DEG_Y;
#endif

    if ((!actuator_axis_changed(deg_x, s_last_deg_x) &&
         !actuator_axis_changed(deg_y, s_last_deg_y)) ||
        !actuator_bus_may_run()) {
        return;
    }

    if (servo_dynamixel_set_pair_degrees(d, deg_x, deg_y)) {
        s_last_deg_x = deg_x;
        s_last_deg_y = deg_y;
        s_consecutive_fail = 0U;
        s_last_write_tick = xTaskGetTickCount();
    } else {
        s_consecutive_fail++;
        s_backoff_until_tick =
            xTaskGetTickCount() + pdMS_TO_TICKS(ACTUATOR_FAIL_BACKOFF_MS);
        servo_dynamixel_bus_recover();
    }
}
#else
static TaskHandle_t s_h_actuator;

static void on_control_output_ready(void)
{
    if (s_h_actuator == NULL) {
        return;
    }
    BaseType_t hpw = pdFALSE;
    vTaskNotifyGiveFromISR(s_h_actuator, &hpw);
    portYIELD_FROM_ISR(hpw);
}

static void actuator_apply_gimbal(servo_feetech_t *s)
{
    control_output_t out;
    (void)state_exchange_get_control_output(&out);
    const float deg_x = actuator_clamp_deg(out.theta_x_cmd * RAD2DEG);
    const float deg_y = actuator_clamp_deg(out.theta_y_cmd * RAD2DEG);
    servo_feetech_set_pair_degrees(s, deg_x, deg_y);
}
#endif

void task_actuator(void *arg)
{
    (void)arg;

    const actuators_t *a = actuators_handles();
    if (a == NULL || a->servos == NULL) {
        vTaskDelete(NULL);
        return;
    }

#ifdef USE_DYNAMIXEL_SERVO
    servo_dynamixel_t *dxl = a->servos;

#if GIMBAL_CALIB_MODE
    (void)servo_dynamixel_disable_torque_pair(dxl);
#ifdef DEBUG_TEXT_CONSOLE
    io_debug_printf("[gimbal] calib: torque OFF — move to mechanical zero\r\n");
    io_debug_printf("[gimbal] update DXL_ZERO_TICKS_X/Y in dev_servo_dynamixel.h\r\n");
#endif

    for (;;) {
        int32_t ticks_x = 0;
        int32_t ticks_y = 0;
        float deg_x = 0.0f;
        float deg_y = 0.0f;

        (void)servo_dynamixel_get_present_position(dxl, DXL_SERVO_ID_X, &ticks_x);
        (void)servo_dynamixel_get_present_position(dxl, DXL_SERVO_ID_Y, &ticks_y);
        (void)servo_dynamixel_get_pair_degrees(dxl, &deg_x, &deg_y);

#ifdef DEBUG_TEXT_CONSOLE
        char sx[16], sy[16];
        fmt_f3(sx, deg_x);
        fmt_f3(sy, deg_y);
        io_debug_printf("[gimbal] ticks x=%ld y=%ld  deg x=%s y=%s\r\n",
                        (long)ticks_x, (long)ticks_y, sx, sy);
#endif
        vTaskDelay(pdMS_TO_TICKS(GIMBAL_CALIB_LOG_MS));
    }
#else
    actuator_pid_init();
    (void)servo_dynamixel_set_pair_degrees(dxl, GIMBAL_DEFAULT_DEG_X, GIMBAL_DEFAULT_DEG_Y);
    s_last_deg_x = GIMBAL_DEFAULT_DEG_X;
    s_last_deg_y = GIMBAL_DEFAULT_DEG_Y;

#ifdef DEBUG_TEXT_CONSOLE
    io_debug_printf("[gimbal] %s clamp=%u deg  Kp x=%u y=%u  Ki x=%u y=%u  deadband=%u mdeg\r\n",
                    GIMBAL_BUILD_TAG,
                    (unsigned)(GIMBAL_CLAMP_DEG + 0.5f),
                    (unsigned)(s_pid_x.kd  * 1000.0f + 0.5f),
                    (unsigned)(s_pid_y.kd  * 1000.0f + 0.5f),
                    (unsigned)(s_pid_x.ki  * 1000.0f + 0.5f),
                    (unsigned)(s_pid_y.ki  * 1000.0f + 0.5f),
                    (unsigned)(GIMBAL_CMD_DEADBAND_DEG * 1000.0f + 0.5f));
    io_debug_printf("hello\n");
    io_debug_printf("[pid] kpx: %u, kpy: %u\n",
                (unsigned)(s_pid_x.kd),
                (unsigned)(s_pid_y.kd));
#endif
    app_flight_state_t current_state = APP_FLIGHT_IDLE;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(ACTUATOR_LOOP_MS));
        state_exchange_get_flight_state(&current_state);

        if (current_state == APP_FLIGHT_RISE || current_state == APP_FLIGHT_DESCENT) {
            actuator_apply_tilt_pid(dxl);
        }
        else {
            updateConfiguration();
            if (actuator_state_uses_default(current_state)) {
                actuator_apply_default(dxl);
            }
        }

#ifdef DEBUG_TEXT_CONSOLE
        {
            static uint32_t s_gimbal_dbg;
            if (++s_gimbal_dbg >= 50U) {
                s_gimbal_dbg = 0;
                tilt_state_t tdbg;
                if (tilt_state_get(&tdbg)) {
                    float cmd_x;
                    float cmd_y;
                    tilt_to_gimbal_deg(&tdbg, &cmd_x, &cmd_y);
#if GIMBAL_HOLD_X_AT_ZERO
                    cmd_x = GIMBAL_DEFAULT_DEG_X;
#endif
#if GIMBAL_HOLD_Y_AT_ZERO
                    cmd_y = GIMBAL_DEFAULT_DEG_Y;
#endif
                    char kx[16], ky[16], cx[16], cy[16], ox[16], oy[16];
                    fmt_f3(kx, tdbg.tilt_x_rad);
                    fmt_f3(ky, tdbg.tilt_y_rad);
                    fmt_f3(cx, cmd_x);
                    fmt_f3(cy, cmd_y);
                    fmt_f3(ox, s_last_deg_x);
                    fmt_f3(oy, s_last_deg_y);
                    io_debug_printf(
                        "[gimbal] kf x=%s y=%s rad  cmd x=%s y=%s deg  out x=%s y=%s\r\n",
                        kx, ky, cx, cy, ox, oy);
                }
            }
        }
#endif
    }
#endif
#else
    servo_feetech_t *s = a->servos;

    s_h_actuator = xTaskGetCurrentTaskHandle();
    io_intercore_register_handler(IO_IC_CONTROL_OUTPUT_READY, on_control_output_ready);

    servo_feetech_enable(s, true);
    servo_feetech_set_pair_degrees(s, GIMBAL_DEFAULT_DEG_X, GIMBAL_DEFAULT_DEG_Y);

    for (;;) {
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(ACTUATOR_LOOP_MS));
        app_flight_state_t current_state = APP_FLIGHT_IDLE;
        (void)state_exchange_get_flight_state(&current_state);
        if (actuator_state_uses_default(current_state)) {
            servo_feetech_set_pair_degrees(s,
                                           GIMBAL_DEFAULT_DEG_X,
                                           GIMBAL_DEFAULT_DEG_Y);
        } else {
            actuator_apply_gimbal(s);
        }
    }
#endif
}
