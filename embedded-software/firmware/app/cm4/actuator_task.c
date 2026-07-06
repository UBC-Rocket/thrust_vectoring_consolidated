/**
 * @file    actuator_task.c
 * @brief   CM4: owns the UART8 servo bus.
 *
 * Dynamixel (USE_DYNAMIXEL_SERVO): reads tilt_state from the state task,
 * runs simple x/y PIDs (setpoint = upright), drives goal position at 50 Hz.
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
#include "controls/pid.h"
#include "tilt_state.h"
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
#define RAD2DEG           (180.0f / 3.14159265f)

static pid_controller_t s_pid_x;
static pid_controller_t s_pid_y;

static void actuator_init_pid(void)
{
    pid_init(&s_pid_x, 2.0f, 0.0f, 0.1f, 10.0f, -90.0f, 90.0f);
    pid_init(&s_pid_y, 2.0f, 0.0f, 0.1f, 10.0f, -90.0f, 90.0f);
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

#ifdef USE_DYNAMIXEL_SERVO
    servo_dynamixel_run_position_test((servo_dynamixel_t *)servos);
#else
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
    actuator_run_startup_selftest(a->servos);
    actuator_init_pid();

    const float dt_s = (float)ACTUATOR_LOOP_MS / 1000.0f;

    for (;;) {
        tilt_state_t tilt;
        if (tilt_state_get(&tilt)) {
            const float deg_x_meas = tilt.tilt_x_rad * RAD2DEG;
            const float deg_y_meas = tilt.tilt_y_rad * RAD2DEG;
            const float cmd_x = pid_compute(&s_pid_x, 0.0f, deg_x_meas, dt_s);
            const float cmd_y = pid_compute(&s_pid_y, 0.0f, deg_y_meas, dt_s);
            (void)servo_dynamixel_set_pair_degrees(a->servos, cmd_x, cmd_y);
        }

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
