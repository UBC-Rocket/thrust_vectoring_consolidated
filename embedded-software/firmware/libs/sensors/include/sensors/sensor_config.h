/**
 * @file sensor_config.h
 * @brief Sensor configuration structures for BMI088 and MS5611.
 *
 * Centralizes sensor configuration (range, ODR, bandwidth, interrupts)
 * to avoid hardcoded values in init functions and enable runtime tuning.
 *
 * Usage:
 *   bmi088_accel_config_t accel_cfg = BMI088_ACCEL_CONFIG_DEFAULT;
 *   accel_cfg.range = BMI088_ACC_RANGE_6G;  // Override if needed
 *   bmi088_accel_init_with_config(&hspi, cs_port, cs_pin, &dev, &accel_cfg);
 *
 * UBC Rocket, Benedikt Howard, 2025
 */

#ifndef SENSOR_CONFIG_H
#define SENSOR_CONFIG_H

#include "sensors/bmi088_accel.h"
#include "sensors/bmi088_gyro.h"
#include "sensors/ms5611_baro.h"
#include "sensors/ms5607_baro.h"

/* -------------------------------------------------------------------------- */
/* BMI088 Accelerometer Configuration                                         */
/* -------------------------------------------------------------------------- */

/**
 * @brief Configuration for BMI088 accelerometer.
 */
typedef struct {
    bmi088_accel_range_t range;      /**< Measurement range (±3g to ±24g) */
    bmi088_accel_odr_t   odr;        /**< Output data rate */
    bmi088_accel_bwp_t   bandwidth;  /**< Bandwidth / oversampling */

    /* Interrupt configuration */
    bmi088_accel_int_pin_t     int_pin;      /**< Which INT pin to use */
    bmi088_accel_int_drive_t   int_drive;    /**< Push-pull or open-drain */
    bmi088_accel_int_polarity_t int_polarity; /**< Active high or low */
    bool                       int_latched;  /**< Latched or pulsed interrupt */
    uint8_t                    int_events;   /**< Events to map (DATA_READY, etc.) */
} bmi088_accel_config_t;

/**
 * @brief Default accelerometer configuration.
 *
 * Range: ±3g (highest resolution for rocket applications)
 * ODR: 800 Hz (good balance of responsiveness and power)
 * Bandwidth: Normal (recommended default)
 * INT1: Push-pull, active high, pulsed, data-ready
 */
#define BMI088_ACCEL_CONFIG_DEFAULT {           \
    .range        = BMI088_ACC_RANGE_3G,        \
    .odr          = BMI088_ACC_ODR_800_HZ,      \
    .bandwidth    = BMI088_ACC_BWP_NORMAL,      \
    .int_pin      = BMI088_ACC_INT1,            \
    .int_drive    = BMI088_ACC_INT_PUSH_PULL,   \
    .int_polarity = BMI088_ACC_INT_ACTIVE_HIGH, \
    .int_latched  = false,                      \
    .int_events   = BMI088_ACC_INT_EVENT_DATA_READY \
}

/**
 * @brief High-G accelerometer configuration for high-thrust phases.
 *
 * Range: ±24g for high acceleration events
 * ODR: 1600 Hz maximum rate
 */
#define BMI088_ACCEL_CONFIG_HIGH_G {            \
    .range        = BMI088_ACC_RANGE_24G,       \
    .odr          = BMI088_ACC_ODR_1600_HZ,     \
    .bandwidth    = BMI088_ACC_BWP_NORMAL,      \
    .int_pin      = BMI088_ACC_INT1,            \
    .int_drive    = BMI088_ACC_INT_PUSH_PULL,   \
    .int_polarity = BMI088_ACC_INT_ACTIVE_HIGH, \
    .int_latched  = false,                      \
    .int_events   = BMI088_ACC_INT_EVENT_DATA_READY \
}

/* -------------------------------------------------------------------------- */
/* BMI088 Gyroscope Configuration                                             */
/* -------------------------------------------------------------------------- */

/**
 * @brief Configuration for BMI088 gyroscope.
 */
typedef struct {
    bmi088_gyro_range_t range;  /**< Measurement range (±125 to ±2000 dps) */
    bmi088_gyro_odr_t   odr;    /**< Output data rate and bandwidth */

    /* Interrupt configuration */
    bmi088_gyro_int_pin_t     int_pin;      /**< Which INT pin to use (INT3/INT4) */
    bmi088_gyro_int_drive_t   int_drive;    /**< Push-pull or open-drain */
    bmi088_gyro_int_polarity_t int_polarity; /**< Active high or low */
    uint8_t                   int_events;   /**< Events to map (DATA_READY, etc.) */
} bmi088_gyro_config_t;

/**
 * @brief Default gyroscope configuration.
 *
 * Range: ±500 dps (good for typical rocket angular rates)
 * ODR: 400 Hz / BW 116 Hz (good balance)
 * INT3: Push-pull, active high, data-ready
 */
#define BMI088_GYRO_CONFIG_DEFAULT {             \
    .range        = BMI088_GYRO_RANGE_500DPS,    \
    .odr          = BMI088_GYRO_ODR_400_HZ_BW_116, \
    .int_pin      = BMI088_GYRO_INT3,            \
    .int_drive    = BMI088_GYRO_INT_PUSH_PULL,   \
    .int_polarity = BMI088_GYRO_INT_ACTIVE_HIGH, \
    .int_events   = BMI088_GYRO_INT_EVENT_DATA_READY \
}

/**
 * @brief High-rate gyroscope configuration for fast rotation tracking.
 *
 * Range: ±2000 dps for tumble/spin recovery
 * ODR: 2000 Hz maximum
 */
#define BMI088_GYRO_CONFIG_HIGH_RATE {           \
    .range        = BMI088_GYRO_RANGE_2000DPS,   \
    .odr          = BMI088_GYRO_ODR_2000_HZ_BW_532, \
    .int_pin      = BMI088_GYRO_INT3,            \
    .int_drive    = BMI088_GYRO_INT_PUSH_PULL,   \
    .int_polarity = BMI088_GYRO_INT_ACTIVE_HIGH, \
    .int_events   = BMI088_GYRO_INT_EVENT_DATA_READY \
}

/* -------------------------------------------------------------------------- */
/* MS5611 Barometer Configuration                                             */
/* -------------------------------------------------------------------------- */

/**
 * @brief Configuration for MS5611 barometer.
 */
typedef struct {
    ms5611_osr_t osr;       /**< Oversampling ratio (256 to 4096) */
    uint32_t     odr_hz;    /**< Target output data rate in Hz */
} ms5611_config_t;

/**
 * @brief Default barometer configuration.
 *
 * OSR: 4096 (highest resolution, ~8.2ms conversion)
 * ODR: 50 Hz (suitable for altitude estimation)
 */
#define MS5611_CONFIG_DEFAULT {     \
    .osr    = MS5611_OSR_4096,      \
    .odr_hz = 50                    \
}

/**
 * @brief Fast barometer configuration for rapid descent.
 *
 * OSR: 1024 (faster conversion, ~2.3ms)
 * ODR: 100 Hz
 */
#define MS5611_CONFIG_FAST {        \
    .osr    = MS5611_OSR_1024,      \
    .odr_hz = 100                   \
}

/* -------------------------------------------------------------------------- */
/* MS5607 Barometer 2 Configuration                                           */
/* -------------------------------------------------------------------------- */

/**
 * @brief Configuration for MS5607 barometer (second barometer).
 */
typedef struct {
    ms5607_osr_t osr;       /**< Oversampling ratio (256 to 4096) */
    uint32_t     odr_hz;    /**< Target output data rate in Hz */
} ms5607_config_t;

/**
 * @brief Default barometer 2 configuration.
 *
 * OSR: 4096 (highest resolution, ~8.2ms conversion)
 * ODR: 50 Hz (suitable for altitude estimation)
 */
#define MS5607_CONFIG_DEFAULT {     \
    .osr    = MS5607_OSR_4096,      \
    .odr_hz = 50                    \
}

/**
 * @brief Fast barometer 2 configuration for rapid descent.
 *
 * OSR: 1024 (faster conversion, ~2.3ms)
 * ODR: 100 Hz
 */
#define MS5607_CONFIG_FAST {        \
    .osr    = MS5607_OSR_1024,      \
    .odr_hz = 100                   \
}

/* -------------------------------------------------------------------------- */
/* Complete System Configuration                                              */
/* -------------------------------------------------------------------------- */

/**
 * @brief Combined sensor configuration for the entire system.
 */
typedef struct {
    bmi088_accel_config_t accel;
    bmi088_gyro_config_t  gyro;
    ms5611_config_t       baro;
    ms5607_config_t       baro2;
} sensor_system_config_t;

/**
 * @brief Default system sensor configuration.
 */
#define SENSOR_SYSTEM_CONFIG_DEFAULT {          \
    .accel = BMI088_ACCEL_CONFIG_DEFAULT,       \
    .gyro  = BMI088_GYRO_CONFIG_DEFAULT,        \
    .baro  = MS5611_CONFIG_DEFAULT,             \
    .baro2 = MS5607_CONFIG_DEFAULT              \
}

#endif /* SENSOR_CONFIG_H */
