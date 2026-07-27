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
#ifdef DEBUG_TEXT_CONSOLE
#include "io_sys/io_debug.h"        /* bring-up: stream IMU samples to console */
#include <stdio.h>                  /* snprintf for the fixed-point IMU formatter */
#endif

#include "state_estimation/ekf.h"
#include "state_estimation/state.h"
#include "state_estimation/body.h"  /* pressure_to_height() */
#include "state_estimation/tilt_kf.h"
#include "tilt_state.h"

#include "storage/storage.h"

#include "generated/messages/publish.h"
#include "generated/messages/types.h"

#include "FreeRTOS.h"
#include "task.h"

/* ---- Constants ------------------------------------------------------- */

/* State estimator selection. The full ESKF (15-state: attitude + per-IMU
 * gyro/accel bias + position/velocity, fed by IMU/baro/mag/GPS) is PRIMARY —
 * TILT_KF_ONLY defaults to 0. The lighter 2-axis tilt KF (attitude only) is
 * retained as a fallback and selected at build time with USE_TILT_KF=1
 * (build.sh) / -DUSE_TILT_KF=ON, which defines TILT_KF_ONLY=1 here. */
#ifndef TILT_KF_ONLY
#define TILT_KF_ONLY              1
#endif

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

static tilt_kf_t s_tilt_kf;

#if !TILT_KF_ONLY
static eskf_t s_eskf;

/* Mag noise: 0.05 on the unit-normalized direction vector. The mag is
 * fed normalized to the EKF, so noise is dimensionless. 0.05 is loose
 * enough to ignore short-lived field anomalies (ferrous structures,
 * motor currents) without de-tuning the yaw observability. */
static eskf_sensor_noise_config_t s_sensor_configs[] = {
    { ESKF_SENSOR_ACCEL, 0, {{0.001f, 0, 0}, {0, 0.001f, 0}, {0, 0, 0.001f}} },
    { ESKF_SENSOR_GYRO,  0, {{0.01f,  0, 0}, {0, 0.01f,  0}, {0, 0, 0.01f }} },
    { ESKF_SENSOR_GPS,   0, {{15.0f,  0, 0}, {0, 15.0f,  0}, {0, 0, 15.0f }} },
    { ESKF_SENSOR_BARO,  0, {{0.1f,   0, 0}, {0,  0,    0}, {0, 0,  0    }} },
    { ESKF_SENSOR_MAG,   0, {{0.05f,  0, 0}, {0, 0.05f,  0}, {0, 0, 0.05f }} },
};

/* Per-cycle scratch — kept file-static (not on the task stack) because
 * 16 * eskf_sample_t * 5 channels is ~1.3 KB and the task stack is 8 KB. */
static eskf_sample_t s_accel_buf[DRAIN_BATCH];
static eskf_sample_t s_gyro_buf [DRAIN_BATCH];
static eskf_sample_t s_baro_buf [DRAIN_BATCH];
static eskf_sample_t s_gps_buf  [4];
static eskf_sample_t s_mag_buf  [DRAIN_BATCH];

static imu_sample_t  s_bmi_raw  [DRAIN_BATCH];
static baro_sample_t s_baro_raw [DRAIN_BATCH];
static gps_fix_t     s_gps_raw  [4];
static mag_sample_t  s_mag_raw  [DRAIN_BATCH];

#ifdef DEBUG_TEXT_CONSOLE
/* Latest mag field (Gauss) fed to the ESKF, held for the [mag] console trace
 * so it prints a fresh value even on cycles where no mag sample drained. */
static float s_dbg_mag_last[3];
#endif

/* Tracks the EKF's calibration latch so we only persist the mag offset
 * once per boot. Set false at task start; flips true the first time
 * eskf_is_calibrated() returns true after a process call. */
static bool s_cal_persisted;

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

/* Latest GPS lock status, latched from drained fixes. "locked" clears if no
 * valid fix arrives within GPS_LOCK_TIMEOUT_US (a dropped fix shouldn't read
 * as locked forever). Feeds the readiness gate. */
#define GPS_LOCK_TIMEOUT_US   3000000ULL   /* 3 s */
static struct {
    bool     locked;
    uint8_t  sats;
    uint64_t last_fix_us;
} s_gps_stat = {0};
#endif

static icm_sample_t  s_icm_raw  [DRAIN_BATCH];

/* ---- Helpers --------------------------------------------------------- */

#ifdef DEBUG_TEXT_CONSOLE
/* Format a float as a signed fixed-point string with 3 decimals. The linked
 * printf has no %f (-u _printf_float not set), so this renders the parsed
 * physical value directly, e.g. -9.562, instead of a scaled integer. */
static void fmt_f3(char *buf, float v) {
    int neg = (v < 0.0f);
    if (neg) v = -v;
    if (v > 9999.0f) v = 9999.0f;          /* clamp; real samples are far smaller */
    unsigned long m = (unsigned long)(v * 1000.0f + 0.5f);   /* milli-units, rounded */
    (void)snprintf(buf, 16, "%s%lu.%03lu", neg ? "-" : "", m / 1000UL, m % 1000UL);
}
#endif

#if TILT_KF_ONLY
/* Build body-to-nav quaternion from 2-axis tilt: q = Ry(tilt_x) * Rx(tilt_y). */
static void tilt_to_state(const tilt_kf_t *kf,
                          const icm_sample_t *imu,
                          state_t *st_out)
{
    const float hx = kf->x.angle * 0.5f;
    const float hy = kf->y.angle * 0.5f;
    const float cy = cosf(hx);
    const float sy = sinf(hx);
    const float cx = cosf(hy);
    const float sx = sinf(hy);

    quaternion_t q = {
        .w = cy * cx,
        .x = cy * sx,
        .y = sy * cx,
        .z = -sy * sx,
    };

    const float n2 = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
    if (n2 > 1e-12f) {
        const float inv_n = 1.0f / sqrtf(n2);
        q.w *= inv_n;
        q.x *= inv_n;
        q.y *= inv_n;
        q.z *= inv_n;
    }

    st_out->pos[0]     = 0.0f;
    st_out->pos[1]     = 0.0f;
    st_out->pos[2]     = 0.0f;
    st_out->vel[0]     = 0.0f;
    st_out->vel[1]     = 0.0f;
    st_out->vel[2]     = 0.0f;
    st_out->q_bn       = q;
    st_out->omega_b[0] = imu->gx - kf->x.bias;
    st_out->omega_b[1] = imu->gy - kf->y.bias;
    st_out->omega_b[2] = imu->gz;
    st_out->u_s        = imu->t_us;
}

static void tilt_publish_state(const tilt_kf_t *kf, const icm_sample_t *imu)
{
    state_t st;
    tilt_to_state(kf, imu, &st);
    (void)state_exchange_publish_state(&st);
}
#endif

#if !TILT_KF_ONLY
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
#endif

/* ---- Task entry ------------------------------------------------------ */

void task_state_estimation(void *arg)
{
    (void)arg;

#if TILT_KF_ONLY
    tilt_kf_init(&s_tilt_kf);
#else
    eskf_setup();
#endif

    const sensors_t *sensors = sensors_handles();
    /* If dev_init never ran, fall back to the placeholder zero-publish
     * so the link path is still exercised. This shouldn't happen — the
     * init sequence is fixed in app_init_cm4 — but it keeps the task
     * defensive and avoids dereferencing NULL drain handles. */
    if (!sensors) {
#if TILT_KF_ONLY
        for (;;) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
#else
        const vec3_f32_t  zero3   = {0.0f, 0.0f, 0.0f};
        const quat_wxyz_t identity = {1.0f, 0.0f, 0.0f, 0.0f};
        for (;;) {
            const uint64_t t_us = io_timestamp_us();
            PUB_STATE_ESTIMATION_STATE_ESTIMATE(
                t_us, zero3, zero3, identity, zero3, zero3);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
#endif
    }

#if !TILT_KF_ONLY
    uint64_t last_primary_imu_us = 0;
#endif
    const TickType_t timeout = pdMS_TO_TICKS(NOTIFY_TIMEOUT_MS);

    for (;;) {
        uint32_t notify_bits = 0;
        /* Wait for any sensor to fire. Returning on timeout is fine —
         * we just continue the loop, which re-drains (likely no-op)
         * and lets us publish a heartbeat-y state even if everything is
         * dead. */
        (void)xTaskNotifyWait(0, UINT32_MAX, &notify_bits, timeout);

#if !TILT_KF_ONLY
        size_t n_accel = 0, n_gyro = 0, n_baro = 0, n_gps = 0, n_mag = 0;
#endif

        /* ---- Primary IMU (ICM-40609) ---------------------------------
         * We drain unconditionally — the notify bit is only a hint; the
         * ring may have samples even on a spurious wake. */
        size_t n_icm = 0;
        if (sensors->icm40609 != NULL) {
            n_icm = imu_icm40609_drain(sensors->icm40609,
                                       s_icm_raw, DRAIN_BATCH);
        }
        for (size_t i = 0; i < n_icm; i++) {
            /* Raw primary-IMU log record (SD only, all drained samples). */
            const vec3_f32_t icm_a = { s_icm_raw[i].ax, s_icm_raw[i].ay, s_icm_raw[i].az };
            const vec3_f32_t icm_g = { s_icm_raw[i].gx, s_icm_raw[i].gy, s_icm_raw[i].gz };
            PUB_IMU_SAMPLE(s_icm_raw[i].t_us, 0U, icm_a, icm_g, s_icm_raw[i].temp_c);
#if TILT_KF_ONLY
            tilt_kf_update(&s_tilt_kf,
                           s_icm_raw[i].t_us,
                           s_icm_raw[i].ax, s_icm_raw[i].ay, s_icm_raw[i].az,
                           s_icm_raw[i].gx, s_icm_raw[i].gy, s_icm_raw[i].gz);
#else
            if (n_accel >= DRAIN_BATCH) {
                break;
            }
            imu_to_eskf(s_icm_raw[i].t_us,
                        s_icm_raw[i].ax, s_icm_raw[i].ay, s_icm_raw[i].az,
                        s_icm_raw[i].gx, s_icm_raw[i].gy, s_icm_raw[i].gz,
                        &s_accel_buf[n_accel], &s_gyro_buf[n_gyro]);
            n_accel++;
            n_gyro++;
            last_primary_imu_us = s_icm_raw[i].t_us;
#endif
        }
#if TILT_KF_ONLY
        if (n_icm > 0U) {
            const icm_sample_t *latest = &s_icm_raw[n_icm - 1U];
            const float rate_x =
                TILT_KF_X_RATE_SIGN * latest->gz - s_tilt_kf.x.bias;
            const float rate_y =
                TILT_KF_Y_RATE_SIGN * latest->gy - s_tilt_kf.y.bias;
            tilt_state_publish(&s_tilt_kf, latest->t_us, rate_x, rate_y);
            tilt_publish_state(&s_tilt_kf, latest);
        }
#endif

#if !TILT_KF_ONLY
#ifdef DEBUG_TEXT_CONSOLE
        /* Bring-up: stream the latest ICM-40609 sample to the text console at
         * ~5 Hz. Integer milli-units only — the linked printf has no %f
         * (-u _printf_float is not set): accel is milli-m/s^2 (~9806 = 1 g on
         * the up axis at rest), gyro is milli-rad/s. n_icm == 0 means the
         * EXTI/queue path delivered nothing this window; the init=OK/FAIL tag
         * then distinguishes "chip never came up" from "up but not streaming".
         * io_debug_write blocks ~4 ms/line at 115200 — fine for a 5 Hz bring-up
         * trace; the whole block compiles out when DEBUG_TEXT_CONSOLE is OFF. */
        {
            static uint64_t s_imu_dbg_last_us;
            const uint64_t  t_dbg = io_timestamp_us();
            if (t_dbg - s_imu_dbg_last_us >= 200000ULL) {   /* 200 ms ~= 5 Hz */
                s_imu_dbg_last_us = t_dbg;
                if (n_icm > 0) {
                    const icm_sample_t *s = &s_icm_raw[n_icm - 1];
                    char ax[16], ay[16], az[16], gx[16], gy[16], gz[16];
                    fmt_f3(ax, s->ax); fmt_f3(ay, s->ay); fmt_f3(az, s->az);
                    fmt_f3(gx, s->gx); fmt_f3(gy, s->gy); fmt_f3(gz, s->gz);
                    io_debug_printf(
                        "[imu] n=%u a=%s,%s,%s m/s2  g=%s,%s,%s rad/s\r\n",
                        (unsigned)n_icm, ax, ay, az, gx, gy, gz);
                } else {
                    io_debug_printf("[imu] no samples (ICM init=%s)\r\n",
                                    (sensors->icm40609 != NULL) ? "OK" : "FAIL");
                }
            }
        }
#endif
#endif

#if TILT_KF_ONLY && defined(DEBUG_TEXT_CONSOLE)
        {
            // app_pid_gains_t pid = {0};
            // char pid_kp[16];
            // (void)state_exchange_get_pid_gains(&pid);
            // if (pid.attitude_kp[0] != 0.0f) {
            //     fmt_f3(pid_kp, pid.attitude_kp[0]);
            //     io_debug_printf("pid: %s\r\n", pid_kp);
            // }

            // app_flight_state_t b = APP_FLIGHT_IDLE;
            // state_exchange_get_flight_state(&b);

            // if (b==APP_FLIGHT_IDLE)io_debug_printf("idle\n");
            // if (b==APP_FLIGHT_RISE)io_debug_printf("rise\n");
            // if (b==APP_FLIGHT_DESCENT)io_debug_printf("drop\n");
            // if (b==APP_FLIGHT_LANDED)io_debug_printf("landed\n");
            // if (b==APP_FLIGHT_ESTOP)io_debug_printf("estop\n");
            // if (b==APP_FLIGHT_ARMED)io_debug_printf("armed\n");

            static uint64_t s_tilt_dbg_last_us;
            const uint64_t  t_dbg = io_timestamp_us();
            if (t_dbg - s_tilt_dbg_last_us >= 200000ULL && n_icm > 0) {
                s_tilt_dbg_last_us = t_dbg;
                char tx[16], ty[16];
                fmt_f3(tx, tilt_kf_angle_x_deg(&s_tilt_kf));
                fmt_f3(ty, tilt_kf_angle_y_deg(&s_tilt_kf));
                io_debug_printf("x=%s y=%s\r\n", tx, ty);
            }
        }

        /* Bring-up: drain the TIM7-paced baro ring and print the latest
         * pressure/temperature at ~1 Hz. In tilt-KF-only mode the baro isn't
         * fed to a filter yet, so this both exercises the drain path (keeps
         * the ring from staying full) and gives console visibility.
         * pressure_pa (~101 kPa) overflows fmt_f3's clamp, so print it as an
         * integer; temperature is small enough for fmt_f3. */
        if (sensors->baro != NULL) {
            baro_sample_t bs[8];
            size_t nb = baro_ms5611_drain(sensors->baro, bs, 8);
            static uint64_t s_baro_dbg_last_us;
            const uint64_t  t_dbg = io_timestamp_us();
            if (nb > 0 && (t_dbg - s_baro_dbg_last_us) >= 1000000ULL) {
                s_baro_dbg_last_us = t_dbg;
                char tc[16];
                fmt_f3(tc, bs[nb - 1].temp_c);
                io_debug_printf("[baro] P=%ld Pa  T=%s C\r\n",
                                (long)bs[nb - 1].pressure_pa, tc);
            }
        }
#endif

#if !TILT_KF_ONLY
        /* ---- Secondary IMU (BMI088) ----------------------------------
         * Always drain to keep the ring healthy. Only feed the EKF if
         * the primary IMU has gone silent for too long. */
        size_t n_bmi = imu_bmi088_drain(sensors->bmi088,
                                        s_bmi_raw, DRAIN_BATCH);
        /* Raw secondary-IMU log record (SD only) — every drained sample,
         * regardless of whether failover feeds it to the EKF. */
        for (size_t i = 0; i < n_bmi; i++) {
            const vec3_f32_t bmi_a = { s_bmi_raw[i].ax, s_bmi_raw[i].ay, s_bmi_raw[i].az };
            const vec3_f32_t bmi_g = { s_bmi_raw[i].gx, s_bmi_raw[i].gy, s_bmi_raw[i].gz };
            PUB_IMU_SAMPLE(s_bmi_raw[i].t_us, 1U, bmi_a, bmi_g, s_bmi_raw[i].temp_c);
        }
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
            /* Raw baro log record (SD only) — before the height/EMA/ref math. */
            PUB_BARO_SAMPLE(s_baro_raw[i].t_us, s_baro_raw[i].pressure_pa,
                            s_baro_raw[i].temp_c);
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

        /* ---- Magnetometer ---------------------------------------------
         * The driver's ring already carries SET/RESET-differenced field
         * samples — the (M_set + M_reset)/2 offset has been cancelled
         * by the differencing pair. We apply per-axis soft-iron scale
         * from storage (identity = 1,1,1 until ellipsoid-fit autocal
         * lands) and feed into the EKF. The EKF normalizes the field
         * internally for the unit-vector measurement model.
         *
         * mx/my/mz are in gauss; we publish both the field and the
         * driver's live hard-iron offset estimate to SD for
         * post-processing diagnostics. */
        size_t n_mr = mag_mmc5983_drain(sensors->mag,
                                        s_mag_raw, DRAIN_BATCH);
        for (size_t i = 0; i < n_mr && n_mag < DRAIN_BATCH; i++) {
            const float sx = g_storage.payload.calibration.mag_scale[0] > 0.0f
                              ? g_storage.payload.calibration.mag_scale[0] : 1.0f;
            const float sy = g_storage.payload.calibration.mag_scale[1] > 0.0f
                              ? g_storage.payload.calibration.mag_scale[1] : 1.0f;
            const float sz = g_storage.payload.calibration.mag_scale[2] > 0.0f
                              ? g_storage.payload.calibration.mag_scale[2] : 1.0f;

            s_mag_buf[n_mag].timestamp_us = s_mag_raw[i].t_us;
            s_mag_buf[n_mag].data[0]      = s_mag_raw[i].mx * sx;
            s_mag_buf[n_mag].data[1]      = s_mag_raw[i].my * sy;
            s_mag_buf[n_mag].data[2]      = s_mag_raw[i].mz * sz;
#ifdef DEBUG_TEXT_CONSOLE
            s_dbg_mag_last[0] = s_mag_buf[n_mag].data[0];
            s_dbg_mag_last[1] = s_mag_buf[n_mag].data[1];
            s_dbg_mag_last[2] = s_mag_buf[n_mag].data[2];
#endif
            n_mag++;

            /* Snapshot the driver's live offset estimate for the SD
             * record. Falls back to (0,0,0) if the driver hasn't
             * completed a SET/RESET pair yet — shouldn't happen in
             * steady state. */
            float live_offset[3] = {0.0f, 0.0f, 0.0f};
            (void)mag_mmc5983_get_offset(sensors->mag, live_offset);

            const vec3_f32_t field  = { s_mag_raw[i].mx, s_mag_raw[i].my, s_mag_raw[i].mz };
            const vec3_f32_t offset = { live_offset[0],  live_offset[1],  live_offset[2]  };
            PUB_MAG_SAMPLE(s_mag_raw[i].t_us, field, offset, s_mag_raw[i].temp_c);
        }

        /* ---- GPS ------------------------------------------------------
         * First valid fix sets the local-frame origin; subsequent fixes
         * become (north, east, up) metre deltas. */
        size_t n_gp = gps_nmea_drain_fixes(sensors->gps,
                                           s_gps_raw,
                                           sizeof(s_gps_raw) / sizeof(s_gps_raw[0]));
        for (size_t i = 0; i < n_gp && n_gps < (sizeof(s_gps_buf)/sizeof(s_gps_buf[0])); i++) {
            /* Raw GPS fix log record (SD only) — every drained fix, including
             * no-fix, so the log shows the receiver is alive even without lock. */
            PUB_GPS_FIX(s_gps_raw[i].t_us,
                        s_gps_raw[i].latitude_deg, s_gps_raw[i].longitude_deg,
                        s_gps_raw[i].altitude_m, s_gps_raw[i].ground_speed_mps,
                        s_gps_raw[i].course_deg, s_gps_raw[i].hdop,
                        s_gps_raw[i].fix_type, s_gps_raw[i].sats_used);
            if (!s_gps_raw[i].valid || s_gps_raw[i].fix_type == 0) continue;

            /* Latch GPS lock for the readiness gate. */
            s_gps_stat.locked      = true;
            s_gps_stat.sats        = s_gps_raw[i].sats_used;
            s_gps_stat.last_fix_us = s_gps_raw[i].t_us;

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
            eskf_sensor_channel_t channels[5];
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
            if (n_mag > 0) {
                channels[n_channels++] = (eskf_sensor_channel_t){
                    ESKF_SENSOR_MAG, 0, s_mag_buf, n_mag };
            }

            eskf_input_t input = { channels, n_channels };
            eskf_process(&s_eskf, &input);

            /* One-shot mag-offset persistence on cal-complete. The driver's
             * live SET/RESET offset estimate is already as good as we'll
             * get without a soft-iron sweep; snapshotting it to flash
             * means next boot's first samples have a known-recent offset
             * to compare against (diagnostic only — the runtime path keeps
             * differencing live). storage_save_async is fire-and-forget;
             * the storage task debounces + commits off the EKF hot path. */
            if (!s_cal_persisted && eskf_is_calibrated(&s_eskf)) {
                float live_offset[3];
                if (mag_mmc5983_get_offset(sensors->mag, live_offset)) {
                    storage_lock();
                    g_storage.payload.calibration.mag_offset[0] = live_offset[0];
                    g_storage.payload.calibration.mag_offset[1] = live_offset[1];
                    g_storage.payload.calibration.mag_offset[2] = live_offset[2];
                    storage_unlock();
                    storage_save_async();
                }
                s_cal_persisted = true;
            }
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

        /* ---- Sensor readiness gate ----------------------------------
         * The vehicle must not arm until its sensors have settled. Publish
         * per-sensor readiness for the mission manager (arming gate) and CM7.
         *   imu  — EKF stationary bias calibration complete
         *   mag  — hard-iron offset valid (status-only; SET/RESET is live)
         *   gps  — a valid fix seen recently (BENCH_FORCE_GPS_LOCK overrides
         *          for antenna-less bench testing) */
        {
            vehicle_readiness_t rdy;
            rdy.imu = eskf_is_calibrated(&s_eskf) ? SENSOR_CAL_READY
                                                  : SENSOR_CAL_SETTLING;

            float mo[3];
            rdy.mag = mag_mmc5983_get_offset(sensors->mag, mo) ? SENSOR_CAL_READY
                                                               : SENSOR_CAL_SETTLING;
#ifdef BENCH_FORCE_MAG_READY
            rdy.mag = SENSOR_CAL_READY;   /* bench: mag DRDY not firing yet */
#endif

            const uint64_t now2 = io_timestamp_us();
            if (s_gps_stat.locked &&
                (now2 - s_gps_stat.last_fix_us) > GPS_LOCK_TIMEOUT_US) {
                s_gps_stat.locked = false;
                s_gps_stat.sats   = 0;
            }
            rdy.gps_locked = s_gps_stat.locked;
            rdy.gps_sats   = s_gps_stat.sats;
#ifdef BENCH_FORCE_GPS_LOCK
            rdy.gps_locked = true;   /* bench: no antenna — force GPS lock */
#endif

            rdy.armable = (rdy.imu == SENSOR_CAL_READY) &&
                          (rdy.mag == SENSOR_CAL_READY) &&
                          rdy.gps_locked;

            (void)state_exchange_publish_readiness(&rdy);

#ifdef DEBUG_TEXT_CONSOLE
            static uint64_t s_rdy_dbg_last_us;
            if (now2 - s_rdy_dbg_last_us >= 1000000ULL) {   /* 1 Hz */
                s_rdy_dbg_last_us = now2;
                static const char *const cs[] = { "uncal", "settling", "READY" };
                io_debug_printf(
                    "[ready] imu=%s mag=%s gps=%s(%u)  ARMABLE=%d\r\n",
                    cs[rdy.imu], cs[rdy.mag],
                    rdy.gps_locked ? "lock" : "no", (unsigned)rdy.gps_sats,
                    (int)rdy.armable);
            }
#endif
        }

#ifdef DEBUG_TEXT_CONSOLE
        /* Bring-up: ESKF output at ~4 Hz. cal=0 during the ~4 s stationary
         * calibration window, then 1. rpy is attitude in degrees (roll/pitch
         * observable from accel+gyro; yaw from mag). alt is z position [m] —
         * ~0 at rest after the baro reference latches, tracks up/down motion.
         * xy will drift with no GPS fix (antenna off) — expected on the bench. */
        {
            static uint64_t s_eskf_dbg_last_us;
            const uint64_t  t_dbg = io_timestamp_us();
            if (t_dbg - s_eskf_dbg_last_us >= 250000ULL) {
                s_eskf_dbg_last_us = t_dbg;
                const float w = q[0], x = q[1], y = q[2], z = q[3];
                const float roll  = atan2f(2.0f*(w*x + y*z), 1.0f - 2.0f*(x*x + y*y)) * 57.29578f;
                float sp = 2.0f*(w*y - z*x);
                if (sp >  1.0f) sp =  1.0f;
                if (sp < -1.0f) sp = -1.0f;
                const float pitch = asinf(sp) * 57.29578f;
                const float yaw   = atan2f(2.0f*(w*z + x*y), 1.0f - 2.0f*(y*y + z*z)) * 57.29578f;
                char r[16], p[16], yw[16], alt[16];
                fmt_f3(r, roll); fmt_f3(p, pitch); fmt_f3(yw, yaw); fmt_f3(alt, pos[2]);
                io_debug_printf("[eskf] cal=%d rpy=%s,%s,%s deg  alt=%s m\r\n",
                                (int)eskf_is_calibrated(&s_eskf), r, p, yw, alt);
                char mgx[16], mgy[16], mgz[16];
                fmt_f3(mgx, s_dbg_mag_last[0]);
                fmt_f3(mgy, s_dbg_mag_last[1]);
                fmt_f3(mgz, s_dbg_mag_last[2]);
                io_debug_printf("[mag] field=%s,%s,%s G\r\n", mgx, mgy, mgz);
            }
        }
#endif
#endif
    }
}
