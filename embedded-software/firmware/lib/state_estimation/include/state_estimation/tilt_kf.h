/**
 * @file    tilt_kf.h
 * @brief   2-axis 1D Kalman inclination filter for self-balance applications.
 *
 * Fuses accelerometer tilt (atan2) with gyro rate per axis. Sensor-frame
 * inputs are remapped internally: body_z(down) = sensor_x, body_x = sensor_y,
 * body_y = sensor_z.
 *
 * UBC Rocket, 2026
 */
#ifndef TILT_KF_H
#define TILT_KF_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * Compile-Time Tunables
 * ====================================================================== */

#ifndef TILT_KF_Q_ANGLE
#define TILT_KF_Q_ANGLE       0.001f   /**< Process noise on angle [rad^2] */
#endif

#ifndef TILT_KF_Q_BIAS
#define TILT_KF_Q_BIAS        1e-6f    /**< Process noise on gyro bias [rad^2/s^2] */
#endif

#ifndef TILT_KF_R_ANGLE
#define TILT_KF_R_ANGLE       0.05f    /**< Measurement noise on accel tilt [rad^2] */
#endif

#ifndef TILT_KF_UPRIGHT_DEG
#define TILT_KF_UPRIGHT_DEG   5.0f     /**< Max |tilt| for upright flag [deg] */
#endif

#ifndef TILT_KF_HORIZ_ACCEL_G
#define TILT_KF_HORIZ_ACCEL_G 0.3f     /**< Max horizontal accel magnitude [g] */
#endif

#ifndef TILT_KF_X_RATE_SIGN
#define TILT_KF_X_RATE_SIGN   1.0f     /**< Sign: d(tilt_x)/dt vs gz */
#endif

#ifndef TILT_KF_Y_RATE_SIGN
#define TILT_KF_Y_RATE_SIGN   1.0f     /**< Sign: d(tilt_y)/dt vs gy */
#endif

#ifndef TILT_KF_DT_MIN_S
#define TILT_KF_DT_MIN_S      1e-4f
#endif

#ifndef TILT_KF_DT_MAX_S
#define TILT_KF_DT_MAX_S      0.1f
#endif

#ifndef TILT_KF_GRAV_MPS2
#define TILT_KF_GRAV_MPS2     9.80665f
#endif

/* ========================================================================
 * Types
 * ====================================================================== */

typedef struct {
    float    angle;          /**< Estimated tilt [rad] */
    float    bias;           /**< Estimated gyro bias [rad/s] */
    float    P[2][2];        /**< Covariance */
    uint64_t last_t_us;      /**< Previous sample timestamp */
    bool     initialised;
} tilt_kf_1d_t;

typedef struct {
    tilt_kf_1d_t x;          /**< Tilt about body_y (sensor_z rate) */
    tilt_kf_1d_t y;          /**< Tilt about body_x (sensor_y rate) */
    float        horiz_accel_g; /**< Last horizontal accel magnitude [g] */
} tilt_kf_t;

/* ========================================================================
 * API
 * ====================================================================== */

void  tilt_kf_init(tilt_kf_t *kf);

void  tilt_kf_update(tilt_kf_t *kf,
                     uint64_t t_us,
                     float ax, float ay, float az,
                     float gx, float gy, float gz);

float tilt_kf_angle_x_deg(const tilt_kf_t *kf);
float tilt_kf_angle_y_deg(const tilt_kf_t *kf);

bool  tilt_kf_is_upright(const tilt_kf_t *kf, float threshold_deg);

#ifdef __cplusplus
}
#endif

#endif /* TILT_KF_H */
