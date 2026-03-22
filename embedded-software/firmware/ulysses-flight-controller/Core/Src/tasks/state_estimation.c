/**
 * @file state_estimation.c
 * @brief State estimation task — feeds sensor data to ESKF via batch API.
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
#include "debug/log.h"
#include "state_exchange.h"
#include "state_estimation/state.h"
#include "mission_manager/mission_manager.h"
#include "SD_logging/log_service.h"
#include "spi_drivers/gnss_radio_master.h"

#define GRAV 9.807f
#define FUSION_VECTOR_SAMPLE_SIZE 16
#define CALIBRATION_SAMPLES 3200  /* ~4s at 800 Hz — sensor warm-up */

/* ESKF tuning — continuous-time spectral densities (Q_c).
 * Library applies Q_d = Q_c * dt internally.
 * Values validated by 31 passing trajectory tests. */
#define ESKF_Q_THETA     0.01f     /* Attitude error [rad^2/s]           */
#define ESKF_Q_BIAS_G    1e-4f     /* Gyro bias drift [(rad/s)^2/s]      */
#define ESKF_Q_BIAS_A    1e-4f     /* Accel bias drift [g^2/s]           */
#define ESKF_Q_POS       5.0f      /* Position process noise [m^2/s^3]   */
#define ESKF_Q_VEL       2.0f      /* Velocity process noise [(m/s)^2/s^3] */

/* ---- Barometer EMA filter + reference ---- */

#define BARO_EMA_ALPHA 0.05f  /* EMA smoothing factor (~0.1s time constant at 200 Hz) */

static struct {
    float height_m;
    bool  set;
} baro_ref = {0};

static struct {
    float value;
    bool  init;
} baro_ema = {0};

/* ---- GPS reference (first valid fix becomes local origin) ---- */

#define WGS84_A  6378137.0        /* WGS84 semi-major axis [m] */
#define DEG2RAD  (M_PI / 180.0)

static struct {
    double lat_rad;
    double lon_rad;
    float  alt_m;
    bool   set;
} gps_ref = {0};

static void gps_to_local(const gnss_gps_fix_t *fix, float out[3])
{
    double dlat = (fix->latitude  * DEG2RAD) - gps_ref.lat_rad;
    double dlon = (fix->longitude * DEG2RAD) - gps_ref.lon_rad;

    out[0] = (float)(dlat * WGS84_A);                          /* North [m] */
    out[1] = (float)(dlon * WGS84_A * cos(gps_ref.lat_rad));   /* East  [m] */
    out[2] = fix->altitude_msl - gps_ref.alt_m;                /* Up    [m] */
}

/* ---- Task entry point ---- */

void state_estimation_task_start(void *argument)
{
    (void)argument;

    /* ---- ESKF initialization ---- */
    static eskf_t eskf;

    static eskf_sensor_noise_config_t sensor_configs[] = {
        { ESKF_SENSOR_ACCEL, 0, {{0.001f,0,0},{0,0.001f,0},{0,0,0.001f}} },
        { ESKF_SENSOR_GYRO,  0, {{0.01f,0,0},{0,0.01f,0},{0,0,0.01f}} },
        { ESKF_SENSOR_GPS,   0, {{15.0f,0,0},{0,15.0f,0},{0,0,15.0f}} },
        { ESKF_SENSOR_BARO,  0, {{0.1f,0,0},{0,0,0},{0,0,0}} },
        { ESKF_SENSOR_BARO,  1, {{0.1f,0,0},{0,0,0},{0,0,0}} },
    };

    eskf_config_t config = {0};
    config.num_imus = 1;

    /* Orientation process noise: θ (attitude), per-IMU gyro/accel bias */
    for (int i = 0; i < 3; i++)
        config.orientation_process_noise[i][i] = ESKF_Q_THETA;
    for (int k = 0; k < config.num_imus; k++) {
        for (int i = 0; i < 3; i++) {
            config.orientation_process_noise[3 + 6*k + i][3 + 6*k + i] = ESKF_Q_BIAS_G;
            config.orientation_process_noise[6 + 6*k + i][6 + 6*k + i] = ESKF_Q_BIAS_A;
        }
    }

    /* Body process noise: position, velocity */
    for (int i = 0; i < 3; i++) config.body_process_noise[i][i] = ESKF_Q_POS;
    for (int i = 3; i < 6; i++) config.body_process_noise[i][i] = ESKF_Q_VEL;

    config.sensors = sensor_configs;
    config.num_sensors = sizeof(sensor_configs) / sizeof(sensor_configs[0]);
    config.expected_g[0] = 0.0f;
    config.expected_g[1] = 0.0f;
    config.expected_g[2] = 1.0f;
    config.mag_ref[0] = 1.0f; /* North */
    config.mag_ref[1] = 0.0f;
    config.mag_ref[2] = 0.0f;
    config.calibration_samples = CALIBRATION_SAMPLES;

    eskf_init(&eskf, &config);

    /* ---- Sensor buffers ---- */
    static eskf_sample_t gyro_buf[FUSION_VECTOR_SAMPLE_SIZE];
    static eskf_sample_t accel_buf[FUSION_VECTOR_SAMPLE_SIZE];
    static eskf_sample_t baro1_buf[FUSION_VECTOR_SAMPLE_SIZE];
    static eskf_sample_t baro2_buf[FUSION_VECTOR_SAMPLE_SIZE];
    static eskf_sample_t gps_buf[4];

    uint8_t n_gyro = 0, n_accel = 0, n_baro1 = 0, n_baro2 = 0, n_gps = 0;
    static int32_t last_baro_p = 0;
    static uint32_t baro_total = 0;
    bool calibration_logged = false;
    uint8_t debug_phase = 0;
    uint32_t last_test_seq = 0;

    const TickType_t period_ticks = pdMS_TO_TICKS(1);
    uint32_t ISR_flags = 0;
    uint64_t ticks = 0;

    while (true) {
        xTaskNotifyWaitIndexed(0, 0, UINT32_MAX, &ISR_flags, period_ticks);

        /* Reset body state after actuator test to discard vibration-induced drift */
        {
            bool test_done = false;
            uint32_t seq = state_exchange_get_startup_test_complete(&test_done);
            if (test_done && seq != last_test_seq) {
                last_test_seq = seq;
                memset(eskf.body.position, 0, sizeof(eskf.body.position));
                memset(eskf.body.velocity, 0, sizeof(eskf.body.velocity));
                /* Reset covariance to initial uncertainty */
                memset(eskf.body.covar, 0, sizeof(eskf.body.covar));
                for (int i = 0; i < 3; i++) eskf.body.covar[i][i] = 10.0f;
                for (int i = 3; i < 6; i++) eskf.body.covar[i][i] = 1.0f;
                /* Reset baro reference so it re-establishes baseline */
                baro_ref.set = false;
                baro_ema.init = false;
            }
        }

        /* ---- Dequeue all available sensor data ---- */

        bmi088_accel_sample_t accel_sample;
        while (bmi088_acc_sample_dequeue(&bmi088_acc_sample_ring, &accel_sample)) {
            log_service_log_accel_sample((uint32_t)accel_sample.t_us,
                accel_sample.ax, accel_sample.ay, accel_sample.az);
            if (n_accel < FUSION_VECTOR_SAMPLE_SIZE) {
                accel_buf[n_accel].timestamp_us = accel_sample.t_us;
                /* BMI088 Z-axis points down on PCB; negate Z to match Z-up body frame */
                accel_buf[n_accel].data[0] =  accel_sample.ax / GRAV;
                accel_buf[n_accel].data[1] =  accel_sample.ay / GRAV;
                accel_buf[n_accel].data[2] = -accel_sample.az / GRAV;
                n_accel++;
            }
        }

        bmi088_gyro_sample_t gyro_sample;
        while (bmi088_gyro_sample_dequeue(&bmi088_gyro_sample_ring, &gyro_sample)) {
            log_service_log_gyro_sample(gyro_sample.t_us,
                gyro_sample.gx, gyro_sample.gy, gyro_sample.gz);
            if (n_gyro < FUSION_VECTOR_SAMPLE_SIZE) {
                gyro_buf[n_gyro].timestamp_us = gyro_sample.t_us;
                /* BMI088 Z-axis points down on PCB; negate Z to match Z-up */
                gyro_buf[n_gyro].data[0] =  gyro_sample.gx;
                gyro_buf[n_gyro].data[1] =  gyro_sample.gy;
                gyro_buf[n_gyro].data[2] = -gyro_sample.gz;
                n_gyro++;
            }
        }

        ms5611_sample_t baro_sample;
        while (ms5611_sample_dequeue(&ms5611_sample_ring, &baro_sample)) {
            log_service_log_baro_sample(baro_sample.t_us,
                baro_sample.temp_centi, baro_sample.pressure_centi, baro_sample.seq);
            if (n_baro1 < FUSION_VECTOR_SAMPLE_SIZE) {
                last_baro_p = baro_sample.pressure_centi;
                float h = pressure_to_height(baro_sample.pressure_centi);

                /* EMA filter — smooths sensor noise and warm-up transients */
                if (!baro_ema.init) { baro_ema.value = h; baro_ema.init = true; }
                else { baro_ema.value += BARO_EMA_ALPHA * (h - baro_ema.value); }
                h = baro_ema.value;

                if (!baro_ref.set && eskf_is_calibrated(&eskf)) {
                    baro_ref.height_m = h;
                    baro_ref.set = true;
                }
                if (!baro_ref.set) continue;
                baro_total++;
                baro1_buf[n_baro1].timestamp_us = baro_sample.t_us;
                baro1_buf[n_baro1].data[0] = h - baro_ref.height_m;
                baro1_buf[n_baro1].data[1] = 0.0f;
                baro1_buf[n_baro1].data[2] = 0.0f;
                n_baro1++;
            }
        }

        ms5607_sample_t baro2_sample;
        while (ms5607_sample_dequeue(&ms5607_sample_ring, &baro2_sample)) {
            log_service_log_baro2_sample(baro2_sample.t_us,
                baro2_sample.temp_centi, baro2_sample.pressure_centi, baro2_sample.seq);
            if (n_baro2 < FUSION_VECTOR_SAMPLE_SIZE) {
                float h = pressure_to_height(baro2_sample.pressure_centi);

                /* EMA filter — same as baro1 (shared filter state) */
                if (!baro_ema.init) { baro_ema.value = h; baro_ema.init = true; }
                else { baro_ema.value += BARO_EMA_ALPHA * (h - baro_ema.value); }
                h = baro_ema.value;

                if (!baro_ref.set && eskf_is_calibrated(&eskf)) {
                    baro_ref.height_m = h;
                    baro_ref.set = true;
                }
                if (!baro_ref.set) continue;
                baro_total++;
                baro2_buf[n_baro2].timestamp_us = baro2_sample.t_us;
                baro2_buf[n_baro2].data[0] = h - baro_ref.height_m;
                baro2_buf[n_baro2].data[1] = 0.0f;
                baro2_buf[n_baro2].data[2] = 0.0f;
                n_baro2++;
            }
        }

        if (ISR_flags & GNSS_GPS_FIX_READY_FLAG) {
            gnss_gps_fix_t gps_fix;
            while (gnss_gps_dequeue(&gps_fix)) {
                if (gps_fix.fix_quality == 0) continue;

                /* Capture first valid fix as local origin */
                if (!gps_ref.set) {
                    gps_ref.lat_rad = gps_fix.latitude * DEG2RAD;
                    gps_ref.lon_rad = gps_fix.longitude * DEG2RAD;
                    gps_ref.alt_m   = gps_fix.altitude_msl;
                    gps_ref.set     = true;
                }

                if (n_gps < 4) {
                    gps_buf[n_gps].timestamp_us = gps_fix.time_of_week_ms * 1000ULL;
                    gps_to_local(&gps_fix, gps_buf[n_gps].data);
                    n_gps++;
                }
            }
        }

        /* ---- Process when we have IMU data ---- */
        uint8_t n_imu = (n_accel < n_gyro) ? n_accel : n_gyro;
        if (n_imu > 0) {
            /* Build channel array */
            eskf_sensor_channel_t channels[5];
            size_t n_channels = 0;

            channels[n_channels++] = (eskf_sensor_channel_t){
                ESKF_SENSOR_GYRO, 0, gyro_buf, n_gyro };
            channels[n_channels++] = (eskf_sensor_channel_t){
                ESKF_SENSOR_ACCEL, 0, accel_buf, n_accel };
            if (n_baro1 > 0)
                channels[n_channels++] = (eskf_sensor_channel_t){
                    ESKF_SENSOR_BARO, 0, baro1_buf, n_baro1 };
            if (n_baro2 > 0)
                channels[n_channels++] = (eskf_sensor_channel_t){
                    ESKF_SENSOR_BARO, 1, baro2_buf, n_baro2 };
            if (n_gps > 0)
                channels[n_channels++] = (eskf_sensor_channel_t){
                    ESKF_SENSOR_GPS, 0, gps_buf, n_gps };

            eskf_input_t input = { channels, n_channels };
            eskf_process(&eskf, &input);

#ifdef DEBUG
            if (!calibration_logged && eskf_is_calibrated(&eskf)) {
                calibration_logged = true;
                DLOG_PRINT("[SE] Calibration complete\r\n");
            }
#endif

            /* ---- Publish state ---- */
            float q[4], pos[3], vel[3];
            eskf_get_state(&eskf, q, pos, vel);

            /* Bias-corrected body rates for controls feedback */
            state_t data = {
                .pos = {pos[0], pos[1], pos[2]},
                .vel = {vel[0], vel[1], vel[2]},
                .omega_b = {
                    gyro_buf[n_gyro - 1].data[0] - eskf.orientation.b_gyro[0][0],
                    gyro_buf[n_gyro - 1].data[1] - eskf.orientation.b_gyro[0][1],
                    gyro_buf[n_gyro - 1].data[2] - eskf.orientation.b_gyro[0][2]
                },
                .q_bn = {.w = q[0], .x = q[1], .y = q[2], .z = q[3]},
                .u_s = gyro_buf[n_gyro - 1].timestamp_us
            };

            state_exchange_publish_state(&data);

            flight_state_t flight_state;
            state_exchange_get_flight_state(&flight_state);
            log_service_log_state(&data, flight_state);

#ifdef DEBUG
            /* One DLOG_PRINT per cycle to avoid stream buffer partial writes.
             * Rotate through phases using a counter (not modular tick arithmetic). */
            if (ticks % 250 == 0) {
                switch (debug_phase) {
                case 0: {
                    float euler[3];
                    quat_to_euler(q, euler);
                    float qw = q[0], qz = q[3];
                    float ct = 2.0f*(qw*qw + qz*qz) - 1.0f;
                    if (ct > 1.0f) ct = 1.0f;
                    if (ct < -1.0f) ct = -1.0f;
                    DLOG_PRINT("ATT t%d r%d p%d y%d\r\n",
                        (int)(acosf(ct) * (180.0f/(float)M_PI)),
                        (int)euler[0], (int)euler[1], (int)euler[2]);
                    break;
                }
                case 1:
                    DLOG_PRINT("POS %d %d %d cm\r\n",
                        (int)(pos[0]*100), (int)(pos[1]*100),
                        (int)(pos[2]*100));
                    break;
                case 2:
                    DLOG_PRINT("VEL %d %d %d cm/s\r\n",
                        (int)(vel[0]*100), (int)(vel[1]*100),
                        (int)(vel[2]*100));
                    break;
                case 3: {
                    ms5611_poller_t *bp = sensors_get_baro_poller();
                    DLOG_PRINT("BARO %dcm t%d s%d d%d nxt%u\r\n",
                        baro_ref.set ? (int)((baro_ema.value - baro_ref.height_m) * 100) : 0,
                        (int)baro_total, (int)bp->state,
                        (int)bp->idle_delta,
                        (unsigned)bp->t_next_action_us);
                    break;
                }
                case 4:
                    DLOG_PRINT("BA %d/%d/%d mg\r\n",
                        (int)(eskf.orientation.b_accel[0][0]*1000),
                        (int)(eskf.orientation.b_accel[0][1]*1000),
                        (int)(eskf.orientation.b_accel[0][2]*1000));
                    break;
                }
                debug_phase = (debug_phase + 1) % 5;
            }
#endif
            ticks++;

            /* Reset counters */
            n_gyro = 0;
            n_accel = 0;
            n_baro1 = 0;
            n_baro2 = 0;
            n_gps = 0;
        }

    }
}
