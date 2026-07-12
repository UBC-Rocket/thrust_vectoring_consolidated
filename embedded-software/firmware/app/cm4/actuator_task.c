/**
 * @file    actuator_task.c
 * @brief   CM4: owns the UART8 servo bus and gimbal control loop.
 *
 * Dynamixel: tilt KF (state_estimation_task) -> tilt_state -> per-axis PD here.
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

#include "FreeRTOS.h"
#include "task.h"

#include <math.h>
#include <stdbool.h>

#define ACTUATOR_LOOP_MS           20U
#define ACTUATOR_MIN_WRITE_MS      40U   /* was 50 ms (~20 Hz); small step to ~25 Hz */
#define ACTUATOR_FAIL_BACKOFF_MS   250U
#define GIMBAL_CLAMP_DEG           15.0f
#define GIMBAL_DEFAULT_DEG_X       0.0f
#define GIMBAL_DEFAULT_DEG_Y       0.0f

/* Per-axis PD (KF y -> Dynamixel X, KF x -> Dynamixel Y). */
#define TILT_KP_X                  1.0f
#define TILT_KD_X                  0.00f
#define TILT_KP_Y                  1.0f
#define TILT_KD_Y                  0.00f

#define GIMBAL_CMD_DEADBAND_DEG    0.05f

#define RAD2DEG                    (180.0f / (float)M_PI)

static inline float actuator_clamp_deg(float deg)
{
    if (deg > GIMBAL_CLAMP_DEG) return GIMBAL_CLAMP_DEG;
    if (deg < -GIMBAL_CLAMP_DEG) return -GIMBAL_CLAMP_DEG;
    return deg;
}

#ifdef USE_DYNAMIXEL_SERVO
/* Dynamixel X: +PD on KF y (inverted mount). Dynamixel Y: -PD on KF x. */
static inline float gimbal_cmd_x_rad(const tilt_state_t *tilt)
{
    return TILT_KP_X * tilt->tilt_y_rad + TILT_KD_X * tilt->rate_y_rad_s;
}

static inline float gimbal_cmd_y_rad(const tilt_state_t *tilt)
{
    return -(TILT_KP_Y * tilt->tilt_x_rad + TILT_KD_Y * tilt->rate_x_rad_s);
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

static void actuator_apply_tilt_pd(servo_dynamixel_t *d)
{
    tilt_state_t tilt;
    if (!tilt_state_get(&tilt)) {
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
        }
        return;
    }

    float deg_x;
    float deg_y;
    tilt_to_gimbal_deg(&tilt, &deg_x, &deg_y);

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
    (void)servo_dynamixel_set_pair_degrees(dxl, GIMBAL_DEFAULT_DEG_X, GIMBAL_DEFAULT_DEG_Y);
    s_last_deg_x = GIMBAL_DEFAULT_DEG_X;
    s_last_deg_y = GIMBAL_DEFAULT_DEG_Y;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(ACTUATOR_LOOP_MS));
        actuator_apply_tilt_pd(dxl);
    }
#else
    servo_feetech_t *s = a->servos;

    s_h_actuator = xTaskGetCurrentTaskHandle();
    io_intercore_register_handler(IO_IC_CONTROL_OUTPUT_READY, on_control_output_ready);

    servo_feetech_enable(s, true);
    servo_feetech_set_pair_degrees(s, GIMBAL_DEFAULT_DEG_X, GIMBAL_DEFAULT_DEG_Y);

    for (;;) {
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(ACTUATOR_LOOP_MS));
        actuator_apply_gimbal(s);
    }
#endif
}
