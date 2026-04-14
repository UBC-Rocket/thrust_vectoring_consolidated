#pragma once

#include <stdbool.h>
#include <stdint.h>

/*
 * AS5600 magnetic encoder driver
 *
 * Notes:
 * - I2C 7-bit address is fixed at 0x36.
 * - ANGLE and RAW ANGLE are 12-bit values in the range [0, 4095].
 * - DIR is a hardware pin, not an I2C register setting.
 */

#ifdef __cplusplus
extern "C" {
#endif

#define AS5600_I2C_ADDR_7BIT           (0x36u)
#define AS5600_I2C_ADDR_8BIT           (AS5600_I2C_ADDR_7BIT << 1)

#define AS5600_FULL_SCALE_COUNTS       (4096u)
#define AS5600_MAX_COUNT               (4095u)
#define AS5600_DEGREES_PER_COUNT       (360.0f / 4096.0f)

/* Register map */
#define AS5600_REG_ZMCO                (0x00u)
#define AS5600_REG_ZPOS_H              (0x01u)
#define AS5600_REG_ZPOS_L              (0x02u)
#define AS5600_REG_MPOS_H              (0x03u)
#define AS5600_REG_MPOS_L              (0x04u)
#define AS5600_REG_MANG_H              (0x05u)
#define AS5600_REG_MANG_L              (0x06u)
#define AS5600_REG_CONF_H              (0x07u)
#define AS5600_REG_CONF_L              (0x08u)
#define AS5600_REG_STATUS              (0x0Bu)
#define AS5600_REG_RAW_ANGLE_H         (0x0Cu)
#define AS5600_REG_RAW_ANGLE_L         (0x0Du)
#define AS5600_REG_ANGLE_H             (0x0Eu)
#define AS5600_REG_ANGLE_L             (0x0Fu)
#define AS5600_REG_AGC                 (0x1Au)
#define AS5600_REG_MAGNITUDE_H         (0x1Bu)
#define AS5600_REG_MAGNITUDE_L         (0x1Cu)
#define AS5600_REG_BURN                (0xFFu)

#define AS5600_BURN_ANGLE_CMD          (0x80u)
#define AS5600_BURN_SETTING_CMD        (0x40u)

/* STATUS bits */
#define AS5600_STATUS_MH               (1u << 3) /* magnet too strong */
#define AS5600_STATUS_ML               (1u << 4) /* magnet too weak   */
#define AS5600_STATUS_MD               (1u << 5) /* magnet detected   */

/* CONF fields */
typedef enum {
    AS5600_PM_NOM  = 0u,
    AS5600_PM_LPM1 = 1u,
    AS5600_PM_LPM2 = 2u,
    AS5600_PM_LPM3 = 3u,
} as5600_power_mode_t;

typedef enum {
    AS5600_HYST_OFF  = 0u,
    AS5600_HYST_1LSB = 1u,
    AS5600_HYST_2LSB = 2u,
    AS5600_HYST_3LSB = 3u,
} as5600_hysteresis_t;

typedef enum {
    AS5600_OUT_ANALOG_FULL    = 0u,
    AS5600_OUT_ANALOG_REDUCED = 1u,
    AS5600_OUT_PWM            = 2u,
} as5600_output_stage_t;

typedef enum {
    AS5600_PWM_115HZ = 0u,
    AS5600_PWM_230HZ = 1u,
    AS5600_PWM_460HZ = 2u,
    AS5600_PWM_920HZ = 3u,
} as5600_pwm_freq_t;

typedef enum {
    AS5600_SF_16X = 0u,
    AS5600_SF_8X  = 1u,
    AS5600_SF_4X  = 2u,
    AS5600_SF_2X  = 3u,
} as5600_slow_filter_t;

typedef enum {
    AS5600_FTH_SLOW_ONLY = 0u,
    AS5600_FTH_6_LSB     = 1u,
    AS5600_FTH_7_LSB     = 2u,
    AS5600_FTH_9_LSB     = 3u,
    AS5600_FTH_18_LSB    = 4u,
    AS5600_FTH_21_LSB    = 5u,
    AS5600_FTH_24_LSB    = 6u,
    AS5600_FTH_10_LSB    = 7u,
} as5600_fast_filter_threshold_t;

typedef struct {
    as5600_power_mode_t power_mode;
    as5600_hysteresis_t hysteresis;
    as5600_output_stage_t output_stage;
    as5600_pwm_freq_t pwm_freq;
    as5600_slow_filter_t slow_filter;
    as5600_fast_filter_threshold_t fast_filter_threshold;
    bool watchdog_enable;
} as5600_conf_t;

typedef struct {
    bool magnet_detected;
    bool magnet_too_weak;
    bool magnet_too_strong;
    uint8_t raw_status;
} as5600_status_t;

typedef struct {
    void *hi2c;
    uint16_t i2c_addr_8bit;
    uint32_t timeout_ms;

    /* Optional software zero offset for application-level calibration.
     * This does not burn OTP and does not change sensor registers. */
    uint16_t zero_offset_counts;
} as5600_t;

void as5600_init(as5600_t *dev, void *hi2c, uint32_t timeout_ms);
void as5600_set_zero_offset_counts(as5600_t *dev, uint16_t zero_offset_counts);
void as5600_zero_here_from_raw_angle(as5600_t *dev);

bool as5600_ping(const as5600_t *dev);

bool as5600_read_u8(const as5600_t *dev, uint8_t reg, uint8_t *value);
bool as5600_write_u8(const as5600_t *dev, uint8_t reg, uint8_t value);
bool as5600_read_u16(const as5600_t *dev, uint8_t reg_msb, uint16_t *value_12bit);
bool as5600_write_u16(const as5600_t *dev, uint8_t reg_msb, uint16_t value_12bit);

bool as5600_read_status(const as5600_t *dev, as5600_status_t *status);
bool as5600_is_magnet_detected(const as5600_t *dev, bool *detected);
bool as5600_read_agc(const as5600_t *dev, uint8_t *agc);
bool as5600_read_magnitude(const as5600_t *dev, uint16_t *magnitude);

bool as5600_read_raw_angle_counts(const as5600_t *dev, uint16_t *counts);
bool as5600_read_angle_counts(const as5600_t *dev, uint16_t *counts);
bool as5600_read_raw_angle_degrees(const as5600_t *dev, float *degrees);
bool as5600_read_angle_degrees(const as5600_t *dev, float *degrees);

bool as5600_read_raw_angle_counts_zeroed(const as5600_t *dev, uint16_t *counts);
bool as5600_read_raw_angle_degrees_zeroed(const as5600_t *dev, float *degrees);

bool as5600_read_conf(const as5600_t *dev, as5600_conf_t *conf);
bool as5600_write_conf(const as5600_t *dev, const as5600_conf_t *conf);

bool as5600_write_zpos(const as5600_t *dev, uint16_t zpos);
bool as5600_write_mpos(const as5600_t *dev, uint16_t mpos);
bool as5600_write_mang(const as5600_t *dev, uint16_t mang);

bool as5600_program_range_volatile(const as5600_t *dev, uint16_t zpos, uint16_t mpos);
bool as5600_burn_angle(const as5600_t *dev);
bool as5600_burn_settings(const as5600_t *dev);

uint16_t as5600_counts_to_degrees_x100(uint16_t counts);
float as5600_counts_to_degrees_f(uint16_t counts);
uint16_t as5600_degrees_to_counts_f(float degrees);
uint16_t as5600_wrap_counts(uint16_t counts);
float as5600_wrap_degrees_360(float degrees);
int16_t as5600_shortest_delta_counts(uint16_t from_counts, uint16_t to_counts);
float as5600_shortest_delta_degrees(float from_deg, float to_deg);

#ifdef __cplusplus
}
#endif
