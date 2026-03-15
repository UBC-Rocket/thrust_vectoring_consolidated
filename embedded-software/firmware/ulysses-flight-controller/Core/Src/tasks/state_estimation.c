/**
 * @file state_estimation.c
 * @brief State estimation task - Event-driven EKF fusion of IMU, baro, and GPS.
 */

#include <stdbool.h>
#include <math.h>
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "spi_drivers/SPI_queue.h"
#include "spi_drivers/SPI_device_interactions.h"
#include "spi_drivers/ms5611_poller.h"
#include "spi_drivers/ms5607_poller.h"
#include "sensors_init.h"
#include "stm32h5xx_hal.h"
#include "main.h"
#include "state_estimation/ekf.h"
#include "state_estimation/body.h"
#include "state_estimation/quaternion.h"
#include "state_estimation/calibration.h"
#include "debug/log.h"
#include "state_exchange.h"
#include "state_estimation/state.h"
#include "mission_manager/mission_manager.h"
#include "SD_logging/log_service.h"
#include "spi_drivers/gnss_radio_master.h"
#include "utils/gnss_utils.h"
#include "timestamp.h"

#define GRAV 9.807f
#define FUSION_VECTOR_SAMPLE_SIZE       32
#define STARTUP_CALIBRATION_SAMPLES     2000U
#define GYRO_BIAS_TIME_CONSTANT_S       30.0f

static const float EXPECTED_GRAVITY[3] = {0.0f, 0.0f, 1.0f};

/* Persistent sensor buffers (static to avoid stack pressure) */
static bmi088_accel_sample_t s_accel_samples[FUSION_VECTOR_SAMPLE_SIZE];
static bmi088_gyro_sample_t  s_gyro_samples[FUSION_VECTOR_SAMPLE_SIZE];
static float s_baro1_heights[FUSION_VECTOR_SAMPLE_SIZE];
static float s_baro2_heights[FUSION_VECTOR_SAMPLE_SIZE];

/* Forward declarations */
static void se_drain_sensor_queues(uint8_t *num_accel, uint8_t *num_gyro,
                                   uint8_t *num_baro1, uint8_t *num_baro2);
static void se_process_gps(uint32_t isr_flags,
                           float gps_reference[3], bool *have_ref,
                           float pos_meters[3], bool *have_pos);

/* ========================================================================== */
/* Single IMU sample processing                                               */
/* ========================================================================== */

typedef struct {
    float    accel_bias[3];
    float    gyro_bias[3];
    uint32_t calibration_samples;
    uint32_t last_tick_us;
    float    delta_time;
    uint64_t ticks;
} se_imu_context_t;

static void se_run_imu_step(se_imu_context_t *ctx,
                            const bmi088_gyro_sample_t *gyro_s,
                            const bmi088_accel_sample_t *accel_s,
                            bool have_gps, const float pos_meters[3],
                            bool have_baro, float baro_altitude_m)
{
    const float g_data_raw[3] = {gyro_s->gx, gyro_s->gy, gyro_s->gz};
    const float a_data_raw[3] = {accel_s->ax / -GRAV,
                                  accel_s->ay / -GRAV,
                                  accel_s->az / -GRAV};
    const bool stationary = imu_is_stationary(g_data_raw, a_data_raw);

    /* Compute delta time */
    if (ctx->last_tick_us != 0U) {
        uint32_t dt_us = timestamp_elapsed_us(ctx->last_tick_us, gyro_s->t_us);
        if ((dt_us < 5U) || (dt_us > 5000U)) {
            ctx->last_tick_us = gyro_s->t_us;
            return;
        }
        ctx->delta_time = (float)dt_us / 1000000.0f;
    }
    ctx->last_tick_us = gyro_s->t_us;

    /* Startup calibration */
    if (ctx->calibration_samples < STARTUP_CALIBRATION_SAMPLES) {

        ctx->calibration_samples++;
        update_bias(ctx->gyro_bias, (float *)g_data_raw,
                    ctx->accel_bias, (float *)a_data_raw,
                    (float *)EXPECTED_GRAVITY, ctx->calibration_samples);
        
        return;
    }

    /* Online gyro bias adaptation while stationary */
    if (stationary && (ctx->delta_time > 0.0f)) {
        const float alpha = ctx->delta_time / (GYRO_BIAS_TIME_CONSTANT_S + ctx->delta_time);
        for (int axis = 0; axis < 3; axis++) {
            ctx->gyro_bias[axis] += alpha * (g_data_raw[axis] - ctx->gyro_bias[axis]);
        }
    }

    /* Bias-corrected sensor data */
    float g_data[3] = {g_data_raw[0] - ctx->gyro_bias[0],
                       g_data_raw[1] - ctx->gyro_bias[1],
                       g_data_raw[2] - ctx->gyro_bias[2]};
    float a_data[3] = {a_data_raw[0] - ctx->accel_bias[0],
                       a_data_raw[1] - ctx->accel_bias[1],
                       a_data_raw[2] - ctx->accel_bias[2]};

    if (stationary) {
        g_data[0] = 0.0f;
        g_data[1] = 0.0f;
        g_data[2] = 0.0f;
    }

    /* Orientation EKF: gyro predicts, accel corrects */
    tick_ekf_orientation(ctx->delta_time, g_data, a_data);

    /* Body EKF: rotate accel to nav frame, predict every cycle */
    float q[4], pos[3], vel[3];
    float a_nav[3];
    get_state(q, pos, vel);
    transform_accel_data(a_data, q, a_nav);

    predict_ekf_body(ctx->delta_time, a_nav);

    /* GPS update only when measurement is available */
    if (have_gps) {
        update_ekf_body((float *)pos_meters);
    }

    /* Baro altitude update */
    if (have_baro) {
        update_ekf_body_baro(baro_altitude_m);
    }

    /* Re-read state after body EKF */
    get_state(q, pos, vel);

    /* Publish state */
    state_t data = {
        .pos = {pos[0], pos[1], pos[2]},
        .vel = {vel[0], vel[1], vel[2]},
        .omega_b = {g_data[0], g_data[1], g_data[2]},
        .q_bn = {.w = q[0], .x = q[1], .y = q[2], .z = q[3]},
        .u_s = gyro_s->t_us
    };
    state_exchange_publish_state(&data);

    /* Logging */
    float e[3];
    quat_to_euler(q, e);

    flight_state_t flight_state;
    state_exchange_get_flight_state(&flight_state);
    log_service_log_state(&data, flight_state);

    if (ctx->ticks % 500 == 0) {
        DLOG_PRINT("[%f, %f, %f]deg\r\n", (double)e[0], (double)e[1], (double)e[2]);
    }
    ctx->ticks++;
}

/* ========================================================================== */
/* Task entry point                                                           */
/* ========================================================================== */

void state_estimation_task_start(void *argument)
{
    (void)argument;

    ms5611_poller_t *baro_poller = sensors_get_baro_poller();
    ms5607_poller_t *baro2_poller = sensors_get_baro2_poller();

    init_ekf(EXPECTED_GRAVITY);

    se_imu_context_t imu_ctx = {
        .accel_bias = {0},
        .gyro_bias = {0},
        .calibration_samples = 0,
        .last_tick_us = 0,
        .delta_time = 0.0f,
        .ticks = 0
    };

    float gps_reference_point[3] = {0};
    bool have_gps_reference_point = false;

    float baro_reference_alt = 0.0f;
    float baro_reference_accum = 0.0f;
    uint32_t baro_reference_count = 0;
    bool have_baro_reference = false;

    const TickType_t period_ticks = pdMS_TO_TICKS(1);
    uint32_t ISR_flags = 0;

    uint8_t num_accel_samples = 0;
    uint8_t num_gyro_samples = 0;
    uint8_t num_baro1_samples = 0;
    uint8_t num_baro2_samples = 0;
    float pos_meters[3] = {0};
    bool have_pos_meters_this_cycle = false;

    while (true) {
        xTaskNotifyWaitIndexed(0, 0, UINT32_MAX, &ISR_flags, period_ticks);

        /* Drain all sensor ring buffers */
        se_drain_sensor_queues(&num_accel_samples, &num_gyro_samples,
                               &num_baro1_samples, &num_baro2_samples);

        /* Process GPS fixes */
        se_process_gps(ISR_flags, gps_reference_point, &have_gps_reference_point,
                       pos_meters, &have_pos_meters_this_cycle);

        /* Process matched IMU pairs */
        const uint8_t imu_loops = (num_accel_samples < num_gyro_samples)
                                  ? num_accel_samples : num_gyro_samples;

        if (imu_loops > 0) {
            /* Average barometer heights from both sensors */
            float baro_alt = 0.0f;
            bool have_baro_this_cycle = false;
            uint8_t baro_count = 0;

            for (int i = 0; i < num_baro1_samples; i++) {
                baro_alt += s_baro1_heights[i];
                baro_count++;
            }
            for (int i = 0; i < num_baro2_samples; i++) {
                baro_alt += s_baro2_heights[i];
                baro_count++;
            }

            if (baro_count > 0) {
                baro_alt /= baro_count;

                /* Accumulate reference during calibration, use for updates after */
                if (!have_baro_reference) {
                    baro_reference_accum += baro_alt;
                    baro_reference_count++;
                    if (imu_ctx.calibration_samples >= STARTUP_CALIBRATION_SAMPLES) {
                        baro_reference_alt = baro_reference_accum / baro_reference_count;
                        have_baro_reference = true;
                    }
                } else {
                    baro_alt -= baro_reference_alt;
                    have_baro_this_cycle = true;
                }
            }

            for (uint8_t i = 0; i < imu_loops; i++) {
                se_run_imu_step(&imu_ctx, &s_gyro_samples[i], &s_accel_samples[i],
                                have_pos_meters_this_cycle, pos_meters,
                                have_baro_this_cycle, baro_alt);
            }

            num_accel_samples = 0;
            num_gyro_samples = 0;
            num_baro1_samples = 0;
            num_baro2_samples = 0;
            have_pos_meters_this_cycle = false;
        }

        ms5611_poller_tick(baro_poller);
        ms5607_poller_tick(baro2_poller);
    }
}

/* ========================================================================== */
/* Sensor dequeuing                                                           */
/* ========================================================================== */

static void se_drain_sensor_queues(uint8_t *num_accel, uint8_t *num_gyro,
                                   uint8_t *num_baro1, uint8_t *num_baro2)
{
    bmi088_accel_sample_t accel_sample;
    while (bmi088_acc_sample_dequeue(&bmi088_acc_sample_ring, &accel_sample)) {
        log_service_log_accel_sample((uint32_t)accel_sample.t_us,
                                     accel_sample.ax, accel_sample.ay, accel_sample.az);
        if (*num_accel < FUSION_VECTOR_SAMPLE_SIZE) {
            s_accel_samples[(*num_accel)++] = accel_sample;
        }
    }

    bmi088_gyro_sample_t gyro_sample;
    while (bmi088_gyro_sample_dequeue(&bmi088_gyro_sample_ring, &gyro_sample)) {
        log_service_log_gyro_sample(gyro_sample.t_us,
                                    gyro_sample.gx, gyro_sample.gy, gyro_sample.gz);
        if (*num_gyro < FUSION_VECTOR_SAMPLE_SIZE) {
            s_gyro_samples[(*num_gyro)++] = gyro_sample;
        }
    }

    ms5611_sample_t baro_sample;
    while (ms5611_sample_dequeue(&ms5611_sample_ring, &baro_sample)) {
        log_service_log_baro_sample(baro_sample.t_us, baro_sample.temp_centi,
                                    baro_sample.pressure_centi, baro_sample.seq);
        if (*num_baro1 < FUSION_VECTOR_SAMPLE_SIZE) {
            s_baro1_heights[(*num_baro1)++] = pressure_to_height(baro_sample.pressure_centi);
        }
    }

    ms5607_sample_t baro2_sample;
    while (ms5607_sample_dequeue(&ms5607_sample_ring, &baro2_sample)) {
        log_service_log_baro2_sample(baro2_sample.t_us, baro2_sample.temp_centi,
                                     baro2_sample.pressure_centi, baro2_sample.seq);
        if (*num_baro2 < FUSION_VECTOR_SAMPLE_SIZE) {
            s_baro2_heights[(*num_baro2)++] = pressure_to_height(baro2_sample.pressure_centi);
        }
    }
}

/* ========================================================================== */
/* GPS processing                                                             */
/* ========================================================================== */

static void se_process_gps(uint32_t isr_flags,
                           float gps_reference[3], bool *have_ref,
                           float pos_meters[3], bool *have_pos)
{
    if (!(isr_flags & GNSS_GPS_FIX_READY_FLAG))
        return;

    gnss_gps_fix_t gps_fix;
    while (gnss_gps_dequeue(&gps_fix)) {
        if (!*have_ref) {
            /* Sanity check: reject fixes outside North America */
            if (!(gps_fix.latitude > 30 && gps_fix.latitude < 60) ||
                !(gps_fix.longitude > -150 && gps_fix.longitude < -60)) {
                continue;
            }
            gps_reference[0] = gps_fix.latitude;
            gps_reference[1] = gps_fix.longitude;
            gps_reference[2] = (float)gps_fix.altitude_msl;
            *have_ref = true;
            continue;
        }

        longlat_to_meters(gps_reference, gps_fix.longitude,
                          gps_fix.latitude, gps_fix.altitude_msl, pos_meters);
        *have_pos = true;
    }
}
