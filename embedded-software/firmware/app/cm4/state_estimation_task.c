/**
 * @file    state_estimation_task.c
 * @brief   CM4 task: drain IMU/baro/GPS samples (via dev/), run ESKF,
 *          publish state to CM7 (state_exchange) and SD log (messages
 *          registry).
 *
 * Loop structure
 * --------------
 *   1. Initialise the ESKF (one IMU, ICM-40609 = primary).
 *   2. Block on xTaskNotifyWait with a 100 ms timeout so a silent sensor
 *      doesn't wedge the task.
 *   3. On each wake:
 *        a. Drain ICM-40609 (primary IMU)            — predict the EKF.
 *        b. Drain BMI088    (secondary IMU)          — kept for ring health
 *           and as a fallback if the primary goes silent (>50 ms).
 *        c. Drain MS5611    (barometer, z update).
 *        d. Drain GPS       (xy update, first fix sets local origin).
 *      Convert each driver's sample into the eskf_sample_t shape the
 *      library expects (accel in g, gyro in rad/s, baro in m relative
 *      to first calibrated reading, GPS in local ENU-ish metres).
 *   4. Call eskf_process(); it runs calibration accumulation until the
 *      configured target is hit, then transitions to predict/update.
 *   5. Pull the latest state with eskf_get_state, publish via
 *      state_exchange (CM7 controls reader) and PUB_STATE_ESTIMATION_
 *      STATE_ESTIMATE (SD-log routing path).
 *
 * IMU policy
 * ----------
 *   ICM-40609 is the primary (higher rate, drives predict). BMI088 is
 *   drained every tick to keep its ring from overflowing but is NOT fed
 *   into the EKF unless the primary has been silent for more than
 *   PRIMARY_IMU_FAILOVER_US microseconds, at which point we feed the
 *   secondary samples as if they were the primary IMU's. This keeps the
 *   EKF "num_imus" at 1 and means we never have to retune R/Q for the
 *   secondary — it just inherits the primary's noise model. Good enough
 *   for v1; a future revision can promote both IMUs to "active" with
 *   independent biases (the library supports up to ESKF_MAX_IMUS = 2).
 *
 * Calibration
 * -----------
 *   eskf_process() handles calibration internally: while
 *   eskf->cal.calibrated is false it sinks IMU samples into the bias
 *   accumulator instead of running predict/update. We still publish state
 *   during this window — the state will be zero pos/vel and identity
 *   quaternion — so the CM7 controls task never sees a stale reader.
 *
 * UBC Rocket, 2026
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "app/sensors_init.h"
#include "app/state_exchange.h"
#include "app/tasks.h"
#include "io_sys/io_timestamp.h"

#include "state_estimation/ekf.h"
#include "state_estimation/state.h"
#include "state_estimation/body.h"  /* pressure_to_height() */

#include "generated/messages/publish.h"
#include "generated/messages/types.h"

#include "FreeRTOS.h"
#include "task.h"

/* ---- Constants ------------------------------------------------------- */

#define GRAV_MPS2                 9.80665f
#define DEG2RAD                   (3.14159265358979323846 / 180.0)
#define WGS84_A                   6378137.0    /* semi-major axis [m] */

/* ~4 s at 1 kHz: same warm-up window as the H5 build at 800 Hz. */
#define CALIBRATION_SAMPLES       4000U

/* Per-cycle drain buffer. IMU EXTI fires ~1 kHz, our notify-wait runs at
 * IMU rate, so a single sample per cycle is the common case. 16 covers
 * burst catch-up if a higher-priority task starves us briefly. */
#define DRAIN_BATCH               16

/* If the primary IMU has been silent this long, fall back to feeding the
 * secondary IMU samples into the EKF. 50 ms ≈ 50 missed primary cycles. */
#define PRIMARY_IMU_FAILOVER_US   50000ULL

/* Notify wait timeout — bounds the loop's hot rate when no sensors fire. */
#define NOTIFY_TIMEOUT_MS         100U

/* ESKF tuning — same continuous-time spectral densities the deprecated
 * H5 build used and which the library's 31 trajectory tests validate. */
#define ESKF_Q_THETA              0.01f
#define ESKF_Q_BIAS_G             1e-4f
#define ESKF_Q_BIAS_A             1e-4f
#define ESKF_Q_POS                5.0f
#define ESKF_Q_VEL                2.0f

#define BARO_EMA_ALPHA            0.05f   /* ~0.1 s time const at 200 Hz  */

/* ---- File-local state ------------------------------------------------ */

static eskf_t s_eskf;

static eskf_sensor_noise_config_t s_sensor_configs[] = {
    { ESKF_SENSOR_ACCEL, 0, {{0.001f, 0, 0}, {0, 0.001f, 0}, {0, 0, 0.001f}} },
    { ESKF_SENSOR_GYRO,  0, {{0.01f,  0, 0}, {0, 0.01f,  0}, {0, 0, 0.01f }} },
    { ESKF_SENSOR_GPS,   0, {{15.0f,  0, 0}, {0, 15.0f,  0}, {0, 0, 15.0f }} },
    { ESKF_SENSOR_BARO,  0, {{0.1f,   0, 0}, {0,  0,    0}, {0, 0,  0    }} },
};

/* Per-cycle scratch — kept file-static (not on the task stack) because
 * 16 * eskf_sample_t * 4 channels is ~1 KB and the task stack is 8 KB. */
static eskf_sample_t s_accel_buf[DRAIN_BATCH];
static eskf_sample_t s_gyro_buf [DRAIN_BATCH];
static eskf_sample_t s_baro_buf [DRAIN_BATCH];
static eskf_sample_t s_gps_buf  [4];

static icm_sample_t  s_icm_raw  [DRAIN_BATCH];
static imu_sample_t  s_bmi_raw  [DRAIN_BATCH];
static baro_sample_t s_baro_raw [DRAIN_BATCH];
static gps_fix_t     s_gps_raw  [4];

static struct {
    float height_m;
    bool  set;
} s_baro_ref = {0};

static struct {
    float value;
    bool  init;
} s_baro_ema = {0};

static struct {
    double lat_rad;
    double lon_rad;
    float  alt_m;
    bool   set;
} s_gps_ref = {0};

/* ---- Helpers --------------------------------------------------------- */

static void eskf_setup(void)
{
    eskf_config_t config = {0};
    config.num_imus = 1;

    /* Orientation process noise. */
    for (int i = 0; i < 3; i++) {
        config.orientation_process_noise[i][i] = ESKF_Q_THETA;
    }
    for (int k = 0; k < config.num_imus; k++) {
        for (int i = 0; i < 3; i++) {
            config.orientation_process_noise[3 + 6 * k + i][3 + 6 * k + i] = ESKF_Q_BIAS_G;
            config.orientation_process_noise[6 + 6 * k + i][6 + 6 * k + i] = ESKF_Q_BIAS_A;
        }
    }

    /* Body process noise. */
    for (int i = 0; i < 3; i++) config.body_process_noise[i][i]     = ESKF_Q_POS;
    for (int i = 3; i < 6; i++) config.body_process_noise[i][i]     = ESKF_Q_VEL;

    config.sensors             = s_sensor_configs;
    config.num_sensors         = sizeof(s_sensor_configs) / sizeof(s_sensor_configs[0]);
    config.expected_g[0]       = 0.0f;
    config.expected_g[1]       = 0.0f;
    config.expected_g[2]       = 1.0f;   /* accel in g, z-up at rest */
    config.mag_ref[0]          = 1.0f;
    config.mag_ref[1]          = 0.0f;
    config.mag_ref[2]          = 0.0f;
    config.calibration_samples = CALIBRATION_SAMPLES;

    eskf_init(&s_eskf, &config);
}

/* Convert an IMU sample (m/s², rad/s) into two ESKF sample slots. */
static inline void imu_to_eskf(uint64_t t_us,
                               float ax, float ay, float az,
                               float gx, float gy, float gz,
                               eskf_sample_t *accel_out,
                               eskf_sample_t *gyro_out)
{
    accel_out->timestamp_us = t_us;
    accel_out->data[0] = ax / GRAV_MPS2;
    accel_out->data[1] = ay / GRAV_MPS2;
    accel_out->data[2] = az / GRAV_MPS2;

    gyro_out->timestamp_us = t_us;
    gyro_out->data[0] = gx;
    gyro_out->data[1] = gy;
    gyro_out->data[2] = gz;
}

/* WGS84 equirectangular projection — adequate for sub-km flight ranges. */
static void gps_to_local(const gps_fix_t *fix, float out[3])
{
    double dlat = (fix->latitude_deg  * DEG2RAD) - s_gps_ref.lat_rad;
    double dlon = (fix->longitude_deg * DEG2RAD) - s_gps_ref.lon_rad;

    out[0] = (float)(dlat * WGS84_A);                         /* North [m] */
    out[1] = (float)(dlon * WGS84_A * cos(s_gps_ref.lat_rad));/* East  [m] */
    out[2] = fix->altitude_m - s_gps_ref.alt_m;               /* Up    [m] */
}

/* ---- Task entry ------------------------------------------------------ */

void task_state_estimation(void *arg)
{
    (void)arg;

    eskf_setup();

    const sensors_t *sensors = sensors_handles();
    /* If dev_init never ran, fall back to the placeholder zero-publish
     * so the link path is still exercised. This shouldn't happen — the
     * init sequence is fixed in app_init_cm4 — but it keeps the task
     * defensive and avoids dereferencing NULL drain handles. */
    if (!sensors) {
        const vec3_f32_t  zero3   = {0.0f, 0.0f, 0.0f};
        const quat_wxyz_t identity = {1.0f, 0.0f, 0.0f, 0.0f};
        for (;;) {
            const uint64_t t_us = io_timestamp_us();
            PUB_STATE_ESTIMATION_STATE_ESTIMATE(
                t_us, zero3, zero3, identity, zero3, zero3);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    uint64_t last_primary_imu_us = 0;
    const TickType_t timeout = pdMS_TO_TICKS(NOTIFY_TIMEOUT_MS);

    for (;;) {
        uint32_t notify_bits = 0;
        /* Wait for any sensor to fire. Returning on timeout is fine —
         * we just continue the loop, which re-drains (likely no-op)
         * and lets us publish a heartbeat-y state even if everything is
         * dead. */
        (void)xTaskNotifyWait(0, UINT32_MAX, &notify_bits, timeout);

        size_t n_accel = 0, n_gyro = 0, n_baro = 0, n_gps = 0;

        /* ---- Primary IMU (ICM-40609) ---------------------------------
         * We drain unconditionally — the notify bit is only a hint; the
         * ring may have samples even on a spurious wake. */
        size_t n_icm = imu_icm40609_drain(sensors->icm40609,
                                          s_icm_raw, DRAIN_BATCH);
        for (size_t i = 0; i < n_icm && n_accel < DRAIN_BATCH; i++) {
            imu_to_eskf(s_icm_raw[i].t_us,
                        s_icm_raw[i].ax, s_icm_raw[i].ay, s_icm_raw[i].az,
                        s_icm_raw[i].gx, s_icm_raw[i].gy, s_icm_raw[i].gz,
                        &s_accel_buf[n_accel], &s_gyro_buf[n_gyro]);
            n_accel++;
            n_gyro++;
            last_primary_imu_us = s_icm_raw[i].t_us;
        }

        /* ---- Secondary IMU (BMI088) ----------------------------------
         * Always drain to keep the ring healthy. Only feed the EKF if
         * the primary IMU has gone silent for too long. */
        size_t n_bmi = imu_bmi088_drain(sensors->bmi088,
                                        s_bmi_raw, DRAIN_BATCH);
        const uint64_t now_us = io_timestamp_us();
        const bool primary_silent =
            (last_primary_imu_us == 0 && n_icm == 0) ||
            (last_primary_imu_us > 0 &&
             now_us - last_primary_imu_us > PRIMARY_IMU_FAILOVER_US);

        if (primary_silent) {
            for (size_t i = 0; i < n_bmi && n_accel < DRAIN_BATCH; i++) {
                imu_to_eskf(s_bmi_raw[i].t_us,
                            s_bmi_raw[i].ax, s_bmi_raw[i].ay, s_bmi_raw[i].az,
                            s_bmi_raw[i].gx, s_bmi_raw[i].gy, s_bmi_raw[i].gz,
                            &s_accel_buf[n_accel], &s_gyro_buf[n_gyro]);
                n_accel++;
                n_gyro++;
            }
        }
        /* else: BMI samples discarded after drain — ring is clear. */

        /* ---- Barometer ------------------------------------------------
         * Convert raw Pa to height via the same library helper the H5
         * build used (101325 Pa sea-level reference, ISA exponent).
         * Track an EMA to smooth sensor noise + warm-up, then capture
         * the first calibrated reading as a zero-altitude reference so
         * the EKF only sees relative-height innovations. */
        size_t n_br = baro_ms5611_drain(sensors->baro,
                                        s_baro_raw, DRAIN_BATCH);
        for (size_t i = 0; i < n_br && n_baro < DRAIN_BATCH; i++) {
            /* pressure_to_height takes Pa even though the param is
             * named *_centi — it divides by 101325.0f (Pa) internally. */
            float h = pressure_to_height((int32_t)s_baro_raw[i].pressure_pa);

            if (!s_baro_ema.init) {
                s_baro_ema.value = h;
                s_baro_ema.init  = true;
            } else {
                s_baro_ema.value += BARO_EMA_ALPHA * (h - s_baro_ema.value);
            }
            h = s_baro_ema.value;

            if (!s_baro_ref.set && eskf_is_calibrated(&s_eskf)) {
                s_baro_ref.height_m = h;
                s_baro_ref.set      = true;
            }
            if (!s_baro_ref.set) continue;

            s_baro_buf[n_baro].timestamp_us = s_baro_raw[i].t_us;
            s_baro_buf[n_baro].data[0]      = h - s_baro_ref.height_m;
            s_baro_buf[n_baro].data[1]      = 0.0f;
            s_baro_buf[n_baro].data[2]      = 0.0f;
            n_baro++;
        }

        /* ---- GPS ------------------------------------------------------
         * First valid fix sets the local-frame origin; subsequent fixes
         * become (north, east, up) metre deltas. */
        size_t n_gp = gps_nmea_drain_fixes(sensors->gps,
                                           s_gps_raw,
                                           sizeof(s_gps_raw) / sizeof(s_gps_raw[0]));
        for (size_t i = 0; i < n_gp && n_gps < (sizeof(s_gps_buf)/sizeof(s_gps_buf[0])); i++) {
            if (!s_gps_raw[i].valid || s_gps_raw[i].fix_type == 0) continue;

            if (!s_gps_ref.set) {
                s_gps_ref.lat_rad = s_gps_raw[i].latitude_deg  * DEG2RAD;
                s_gps_ref.lon_rad = s_gps_raw[i].longitude_deg * DEG2RAD;
                s_gps_ref.alt_m   = s_gps_raw[i].altitude_m;
                s_gps_ref.set     = true;
            }

            s_gps_buf[n_gps].timestamp_us = s_gps_raw[i].t_us;
            gps_to_local(&s_gps_raw[i], s_gps_buf[n_gps].data);
            n_gps++;
        }

        /* ---- Feed the ESKF -------------------------------------------
         * If no IMU data arrived we still publish below (the user wants
         * a heartbeat even if sensors are stuck). Skipping eskf_process
         * in that case is correct: there's nothing to predict on, and
         * baro/GPS without IMU would just sit in their channels until
         * the next wake. */
        if (n_accel > 0 || n_gyro > 0) {
            eskf_sensor_channel_t channels[4];
            size_t n_channels = 0;

            channels[n_channels++] = (eskf_sensor_channel_t){
                ESKF_SENSOR_GYRO, 0, s_gyro_buf, n_gyro };
            channels[n_channels++] = (eskf_sensor_channel_t){
                ESKF_SENSOR_ACCEL, 0, s_accel_buf, n_accel };
            if (n_baro > 0) {
                channels[n_channels++] = (eskf_sensor_channel_t){
                    ESKF_SENSOR_BARO, 0, s_baro_buf, n_baro };
            }
            if (n_gps > 0) {
                channels[n_channels++] = (eskf_sensor_channel_t){
                    ESKF_SENSOR_GPS, 0, s_gps_buf, n_gps };
            }

            eskf_input_t input = { channels, n_channels };
            eskf_process(&s_eskf, &input);
        }

        /* ---- Publish state -------------------------------------------
         * Even pre-calibration we publish identity attitude + zero pos /
         * vel so the CM7 controls task always has a reader. The bias
         * fields surface the EKF's current best estimate (zero pre-cal,
         * then the calibrated bias). */
        float q[4]   = {1.0f, 0.0f, 0.0f, 0.0f};
        float pos[3] = {0.0f, 0.0f, 0.0f};
        float vel[3] = {0.0f, 0.0f, 0.0f};
        eskf_get_state(&s_eskf, q, pos, vel);

        /* Use the latest IMU timestamp if we have one, else current time. */
        const uint64_t t_us = (n_gyro > 0)
            ? s_gyro_buf[n_gyro - 1].timestamp_us
            : io_timestamp_us();

        /* Bias-corrected body rates for controls feedback. The
         * orientation.b_gyro[0] entry is the primary IMU's bias estimate. */
        float omega_b[3] = {0.0f, 0.0f, 0.0f};
        if (n_gyro > 0) {
            omega_b[0] = s_gyro_buf[n_gyro - 1].data[0] - s_eskf.orientation.b_gyro[0][0];
            omega_b[1] = s_gyro_buf[n_gyro - 1].data[1] - s_eskf.orientation.b_gyro[0][1];
            omega_b[2] = s_gyro_buf[n_gyro - 1].data[2] - s_eskf.orientation.b_gyro[0][2];
        }

        state_t st = {
            .pos     = { pos[0], pos[1], pos[2] },
            .vel     = { vel[0], vel[1], vel[2] },
            .q_bn    = { .w = q[0], .x = q[1], .y = q[2], .z = q[3] },
            .omega_b = { omega_b[0], omega_b[1], omega_b[2] },
            .u_s     = t_us,
        };
        state_exchange_publish_state(&st);

        const vec3_f32_t  position    = { pos[0], pos[1], pos[2] };
        const vec3_f32_t  velocity    = { vel[0], vel[1], vel[2] };
        const quat_wxyz_t attitude    = { q[0],   q[1],   q[2], q[3] };
        const vec3_f32_t  gyro_bias   = {
            s_eskf.orientation.b_gyro[0][0],
            s_eskf.orientation.b_gyro[0][1],
            s_eskf.orientation.b_gyro[0][2],
        };
        const vec3_f32_t  accel_bias  = {
            s_eskf.orientation.b_accel[0][0],
            s_eskf.orientation.b_accel[0][1],
            s_eskf.orientation.b_accel[0][2],
        };
        PUB_STATE_ESTIMATION_STATE_ESTIMATE(
            t_us, position, velocity, attitude, gyro_bias, accel_bias);
    }
}
