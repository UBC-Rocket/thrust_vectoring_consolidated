/**
 * @file    actuator_task.c
 * @brief   CM4: owns the UART8 servo bus.
 *
 * Dynamixel (USE_DYNAMIXEL_SERVO): reads present position at startup and
 * holds it (no PID). Re-asserts goal position at 50 Hz.
 *
 * Feetech: consumes control_output_t from CM7 via state_exchange.
 *
 * UBC Rocket, 2026
 */

#include "app/tasks.h"
#include "app/state_exchange.h"
#include "app/actuators_init.h"

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

#define SELFTEST_SWEEP_RANGE_DEG   90.0f
#define SELFTEST_SWEEP_STEP_DEG    1.0f
#define SELFTEST_SWEEP_STEP_MS     5U
#define SELFTEST_CIRCLE_RADIUS_DEG 5.0f
#define SELFTEST_CIRCLE_STEPS      72U
#define SELFTEST_CIRCLE_STEP_MS    18U

#ifdef USE_DYNAMIXEL_SERVO
#define ACTUATOR_LOOP_MS  20U
#define DXL_SERVO_ID_X    1u
#define DXL_SERVO_ID_Y    2u

static int32_t s_hold_ticks_x;
static int32_t s_hold_ticks_y;
static bool    s_hold_x_valid;
static bool    s_hold_y_valid;

static bool dynamixel_capture_and_hold(servo_dynamixel_t *d)
{
    s_hold_x_valid = servo_dynamixel_get_present_position(d, DXL_SERVO_ID_X,
                                                          &s_hold_ticks_x);
    s_hold_y_valid = servo_dynamixel_get_present_position(d, DXL_SERVO_ID_Y,
                                                          &s_hold_ticks_y);

    bool ok = false;
    if (s_hold_x_valid) {
        ok = servo_dynamixel_set_goal_position(d, DXL_SERVO_ID_X,
                                               s_hold_ticks_x) || ok;
    }
    if (s_hold_y_valid) {
        ok = servo_dynamixel_set_goal_position(d, DXL_SERVO_ID_Y,
                                               s_hold_ticks_y) || ok;
    }
    return ok;
}

static void dynamixel_maintain_hold(servo_dynamixel_t *d)
{
    if (s_hold_x_valid) {
        (void)servo_dynamixel_set_goal_position(d, DXL_SERVO_ID_X, s_hold_ticks_x);
    }
    if (s_hold_y_valid) {
        (void)servo_dynamixel_set_goal_position(d, DXL_SERVO_ID_Y, s_hold_ticks_y);
    }
}
#endif

#ifndef USE_DYNAMIXEL_SERVO
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
#endif

static void actuator_run_startup_selftest(void *servos)
{
    if (servos == NULL) {
        return;
    }

#ifndef USE_DYNAMIXEL_SERVO
    servo_feetech_t *s = (servo_feetech_t *)servos;

    servo_feetech_set_pair_degrees(s, 0.0f, 0.0f);
    vTaskDelay(pdMS_TO_TICKS(100));

    for (float deg = 0.0f; deg <= SELFTEST_SWEEP_RANGE_DEG;
         deg += SELFTEST_SWEEP_STEP_DEG) {
        servo_feetech_set_pair_degrees(s, deg, deg);
        vTaskDelay(pdMS_TO_TICKS(SELFTEST_SWEEP_STEP_MS));
    }
    for (float deg = SELFTEST_SWEEP_RANGE_DEG;
         deg >= -SELFTEST_SWEEP_RANGE_DEG;
         deg -= SELFTEST_SWEEP_STEP_DEG) {
        servo_feetech_set_pair_degrees(s, deg, deg);
        vTaskDelay(pdMS_TO_TICKS(SELFTEST_SWEEP_STEP_MS));
    }
    for (float deg = -SELFTEST_SWEEP_RANGE_DEG; deg <= 0.0f;
         deg += SELFTEST_SWEEP_STEP_DEG) {
        servo_feetech_set_pair_degrees(s, deg, deg);
        vTaskDelay(pdMS_TO_TICKS(SELFTEST_SWEEP_STEP_MS));
    }

    for (unsigned i = 0; i < SELFTEST_CIRCLE_STEPS; ++i) {
        float ang = (2.0f * (float)M_PI * (float)i) /
                    (float)SELFTEST_CIRCLE_STEPS;
        float dx = SELFTEST_CIRCLE_RADIUS_DEG * cosf(ang);
        float dy = SELFTEST_CIRCLE_RADIUS_DEG * sinf(ang);
        servo_feetech_set_pair_degrees(s, dx, dy);
        vTaskDelay(pdMS_TO_TICKS(SELFTEST_CIRCLE_STEP_MS));
    }

    servo_feetech_set_pair_degrees(s, 0.0f, 0.0f);
#endif
}

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
    (void)dynamixel_capture_and_hold(dxl);

    for (;;) {
        dynamixel_maintain_hold(dxl);
        vTaskDelay(pdMS_TO_TICKS(ACTUATOR_LOOP_MS));
    }
#else
    s_h_actuator = xTaskGetCurrentTaskHandle();

    servo_feetech_enable(a->servos, true);

    actuator_run_startup_selftest(a->servos);

    io_intercore_register_handler(IO_IC_CONTROL_OUTPUT_READY, on_control_output_ready);

    for (;;) {
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        control_output_t out;
        (void)state_exchange_get_control_output(&out);

        float deg_x = out.theta_x_cmd * 180.0f / (float)M_PI;
        float deg_y = out.theta_y_cmd * 180.0f / (float)M_PI;
        servo_feetech_set_pair_degrees(a->servos, deg_x, deg_y);
    }
#endif
}
