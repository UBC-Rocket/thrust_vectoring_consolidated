/**
 * @file    ekf.h
 * @brief   Error-State Kalman Filter (ESKF) with multi-sensor support.
 *
 * Standalone, instance-based API. Supports multiple sensors of each type
 * (accelerometer, gyroscope, GPS, magnetometer, barometer). Orientation
 * uses an error state [δθ, δb_g0, δb_a0, ..., δb_gN, δb_aN] with
 * per-IMU online bias estimation. Body (position/velocity) uses a
 * standard 6D Kalman filter.
 */
#ifndef EKF_H
#define EKF_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ========================================================================
 * Compile-Time Configuration
 * ====================================================================== */

#ifndef ESKF_MAX_IMUS
#define ESKF_MAX_IMUS 2
#endif

/** Error-state dimension: 3 (attitude) + 6 per IMU (gyro bias + accel bias) */
#define ESKF_ERR_DIM (3 + 6 * ESKF_MAX_IMUS)

/* ========================================================================
 * Sensor Input Types
 * ====================================================================== */

typedef enum {
    ESKF_SENSOR_ACCEL,  /**< 3-axis accelerometer, units: g (specific force) */
    ESKF_SENSOR_GYRO,   /**< 3-axis gyroscope, units: rad/s */
    ESKF_SENSOR_GPS,    /**< 3-axis position, units: m (local frame) */
    ESKF_SENSOR_MAG,    /**< 3-axis magnetometer, units: normalized direction */
    ESKF_SENSOR_BARO,   /**< Barometer, data[0] = height [m] */
    ESKF_SENSOR_COUNT
} eskf_sensor_type_t;

typedef struct {
    uint64_t timestamp_us;
    float data[3];
} eskf_sample_t;

typedef struct {
    eskf_sensor_type_t type;
    uint8_t id;              /**< Instance ID (e.g. IMU 0, IMU 1) */
    eskf_sample_t *samples;
    size_t count;
} eskf_sensor_channel_t;

typedef struct {
    eskf_sensor_channel_t *channels;
    size_t num_channels;
} eskf_input_t;

/* ========================================================================
 * Configuration
 * ====================================================================== */

#define ESKF_MAX_SENSORS 8

typedef struct {
    eskf_sensor_type_t type;
    uint8_t id;
    float noise[3][3];  /**< Measurement noise R for this sensor */
} eskf_sensor_noise_config_t;

typedef struct {
    /** Number of active IMUs (must be <= ESKF_MAX_IMUS) */
    uint8_t num_imus;

    /** Orientation process noise Q (ESKF_ERR_DIM x ESKF_ERR_DIM) */
    float orientation_process_noise[ESKF_ERR_DIM][ESKF_ERR_DIM];

    /** Body process noise Q (6x6) */
    float body_process_noise[6][6];

    /** Per-sensor measurement noise configs */
    eskf_sensor_noise_config_t *sensors;
    size_t num_sensors;

    /** Expected accelerometer reading at rest, normalized (e.g. [0,0,1]) */
    float expected_g[3];

    /** Magnetic field reference in nav frame (e.g. [1,0,0] for north) */
    float mag_ref[3];

    /** Number of initial IMU samples for stationary bias calibration (0 = skip) */
    uint32_t calibration_samples;
} eskf_config_t;

/* ========================================================================
 * Internal Filter State
 * ====================================================================== */

/**
 * Orientation ESKF state.
 * Error state: δx = [δθ(3), δb_g0(3), δb_a0(3), ..., δb_gN(3), δb_aN(3)]
 * Dimension: ESKF_ERR_DIM = 3 + 6 * ESKF_MAX_IMUS
 */
typedef struct {
    float q_nom[4];                        /**< Nominal quaternion [w,x,y,z] */
    float b_gyro[ESKF_MAX_IMUS][3];        /**< Per-IMU gyro bias [rad/s] */
    float b_accel[ESKF_MAX_IMUS][3];       /**< Per-IMU accel bias [g] */
    uint8_t num_imus;                      /**< Number of active IMUs */
    float covar[ESKF_ERR_DIM][ESKF_ERR_DIM]; /**< Error-state covariance P */
    float process[ESKF_ERR_DIM][ESKF_ERR_DIM]; /**< Process noise Q */
} orientation_eskf_state_t;

/**
 * Body state (position + velocity in nav frame).
 * Standard Kalman filter, 6D state [pos(3), vel(3)].
 */
typedef struct {
    float position[3];       /**< Position [m] nav frame */
    float velocity[3];       /**< Velocity [m/s] nav frame */
    float covar[6][6];       /**< Covariance P */
    float process[6][6];     /**< Process noise Q */
} body_state_t;

/**
 * Main ESKF instance — pass by pointer to all API functions.
 */
typedef struct {
    orientation_eskf_state_t orientation;
    body_state_t body;

    float expected_g[3];
    float mag_ref[3];

    struct {
        uint32_t calibration_target;
        uint32_t gyro_count[ESKF_MAX_IMUS];
        uint32_t accel_count[ESKF_MAX_IMUS];
        uint32_t mag_count;
        float gyro_accum[ESKF_MAX_IMUS][3];
        float accel_accum[ESKF_MAX_IMUS][3];
        float mag_accum[3];
        bool calibrated;
    } cal;

    eskf_sensor_noise_config_t sensor_configs[ESKF_MAX_SENSORS];
    size_t num_sensor_configs;

    uint64_t last_gyro_ts;     /**< Persists between eskf_process calls */
    uint64_t last_accel_ts;    /**< Persists between eskf_process calls */
} eskf_t;

/* ========================================================================
 * Public API
 * ====================================================================== */

void eskf_init(eskf_t *eskf, const eskf_config_t *config);
void eskf_process(eskf_t *eskf, const eskf_input_t *input);
void eskf_get_state(const eskf_t *eskf, float quat[4], float pos[3], float vel[3]);
bool eskf_is_calibrated(const eskf_t *eskf);

#endif /* EKF_H */
