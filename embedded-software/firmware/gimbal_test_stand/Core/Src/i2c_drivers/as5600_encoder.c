#include "as5600.h"

#include <stddef.h>

/* Device-layer hooks implemented in as5600_device_stm32_hal.c */
bool as5600_device_is_ready(void *hi2c, uint16_t addr_8bit, uint32_t timeout_ms);
bool as5600_device_mem_read(void *hi2c, uint16_t addr_8bit, uint8_t reg, uint8_t *data, uint16_t len, uint32_t timeout_ms);
bool as5600_device_mem_write(void *hi2c, uint16_t addr_8bit, uint8_t reg, const uint8_t *data, uint16_t len, uint32_t timeout_ms);

// Clamp a 16-bit value to the valid 12-bit range (0 to 4095). This does not perform any I2C communication.
static uint16_t clamp12(uint16_t x) {
    return (uint16_t)(x & 0x0FFFu);
}

// Combine two bytes read from the AS5600 registers into a single 12-bit value. The first byte is the MSB and the second byte is the LSB. This does not perform any I2C communication.
static uint16_t combine_u12(uint8_t msb, uint8_t lsb) {
    return (uint16_t)((((uint16_t)msb << 8) | (uint16_t)lsb) & 0x0FFFu);
}

// Split a 12-bit value into two bytes for writing to the AS5600 registers. The MSB will be in the first byte and the LSB in the second byte. This does not perform any I2C communication.
static void split_u12(uint16_t value, uint8_t *msb, uint8_t *lsb) {
    value = clamp12(value);
    *msb = (uint8_t)((value >> 8) & 0x0Fu);
    *lsb = (uint8_t)(value & 0xFFu);
}

// Pack the as5600_conf_t structure fields into a 16-bit value suitable for writing to the CONF register. This does not perform any I2C communication.
static uint16_t conf_pack(const as5600_conf_t *conf) {
    uint16_t reg = 0u;
    if (conf == NULL) {
        return 0u;
    }

    reg |= ((uint16_t)(conf->power_mode & 0x03u) << 0);
    reg |= ((uint16_t)(conf->hysteresis & 0x03u) << 2);
    reg |= ((uint16_t)(conf->output_stage & 0x03u) << 4);
    reg |= ((uint16_t)(conf->pwm_freq & 0x03u) << 6);
    reg |= ((uint16_t)(conf->slow_filter & 0x03u) << 8);
    reg |= ((uint16_t)(conf->fast_filter_threshold & 0x07u) << 10);
    reg |= ((uint16_t)(conf->watchdog_enable ? 1u : 0u) << 13);

    return reg;
}

// Unpack the 16-bit CONF register value into the as5600_conf_t structure fields. This does not perform any I2C communication.
static void conf_unpack(uint16_t reg, as5600_conf_t *conf) {
    if (conf == NULL) {
        return;
    }

    conf->power_mode = (as5600_power_mode_t)((reg >> 0) & 0x03u);
    conf->hysteresis = (as5600_hysteresis_t)((reg >> 2) & 0x03u);
    conf->output_stage = (as5600_output_stage_t)((reg >> 4) & 0x03u);
    conf->pwm_freq = (as5600_pwm_freq_t)((reg >> 6) & 0x03u);
    conf->slow_filter = (as5600_slow_filter_t)((reg >> 8) & 0x03u);
    conf->fast_filter_threshold = (as5600_fast_filter_threshold_t)((reg >> 10) & 0x07u);
    conf->watchdog_enable = (((reg >> 13) & 0x01u) != 0u);
}

// Initialize the AS5600 device structure with the given I2C handle and timeout. This does not perform any I2C communication or sensor configuration.
void as5600_init(as5600_t *dev, void *hi2c, uint32_t timeout_ms) {
    if (dev == NULL) {
        return;
    }

    dev->hi2c = hi2c;
    dev->i2c_addr_8bit = AS5600_I2C_ADDR_8BIT;
    dev->timeout_ms = timeout_ms;
    dev->zero_offset_counts = 0u;
}

// Set a software zero offset in counts for application-level calibration. This does not burn OTP or change sensor registers.
void as5600_set_zero_offset_counts(as5600_t *dev, uint16_t zero_offset_counts) {
    if (dev == NULL) {
        return;
    }
    dev->zero_offset_counts = as5600_wrap_counts(zero_offset_counts);
}

// Set the current raw angle as the zero reference point for future zeroed readings. This does not burn OTP or change sensor registers.
void as5600_zero_here_from_raw_angle(as5600_t *dev) {
    uint16_t raw = 0u;
    if (dev == NULL) {
        return;
    }
    if (as5600_read_raw_angle_counts(dev, &raw)) {
        dev->zero_offset_counts = raw;
    }
}


// Check if the device is present and responding to I2C commands.
bool as5600_ping(const as5600_t *dev) {
    if (dev == NULL || dev->hi2c == NULL) {
        return false;
    }
    return as5600_device_is_ready(dev->hi2c, dev->i2c_addr_8bit, dev->timeout_ms);
}

bool as5600_read_u8(const as5600_t *dev, uint8_t reg, uint8_t *value) {
    if (dev == NULL || dev->hi2c == NULL || value == NULL) {
        return false;
    }
    return as5600_device_mem_read(dev->hi2c, dev->i2c_addr_8bit, reg, value, 1u, dev->timeout_ms);
}

bool as5600_write_u8(const as5600_t *dev, uint8_t reg, uint8_t value) {
    if (dev == NULL || dev->hi2c == NULL) {
        return false;
    }
    return as5600_device_mem_write(dev->hi2c, dev->i2c_addr_8bit, reg, &value, 1u, dev->timeout_ms);
}

bool as5600_read_u16(const as5600_t *dev, uint8_t reg_msb, uint16_t *value_12bit) {
    uint8_t buf[2];

    if (dev == NULL || dev->hi2c == NULL || value_12bit == NULL) {
        return false;
    }

    if (!as5600_device_mem_read(dev->hi2c, dev->i2c_addr_8bit, reg_msb, buf, 2u, dev->timeout_ms)) {
        return false;
    }

    *value_12bit = combine_u12(buf[0], buf[1]);
    return true;
}

bool as5600_write_u16(const as5600_t *dev, uint8_t reg_msb, uint16_t value_12bit) {
    uint8_t buf[2];

    if (dev == NULL || dev->hi2c == NULL) {
        return false;
    }

    split_u12(value_12bit, &buf[0], &buf[1]);
    return as5600_device_mem_write(dev->hi2c, dev->i2c_addr_8bit, reg_msb, buf, 2u, dev->timeout_ms);
}

bool as5600_read_status(const as5600_t *dev, as5600_status_t *status) {
    uint8_t raw = 0u;

    if (status == NULL) {
        return false;
    }
    if (!as5600_read_u8(dev, AS5600_REG_STATUS, &raw)) {
        return false;
    }

    status->raw_status = raw;
    status->magnet_detected = ((raw & AS5600_STATUS_MD) != 0u);
    status->magnet_too_weak = ((raw & AS5600_STATUS_ML) != 0u);
    status->magnet_too_strong = ((raw & AS5600_STATUS_MH) != 0u);
    return true;
}

bool as5600_is_magnet_detected(const as5600_t *dev, bool *detected) {
    as5600_status_t status;
    if (detected == NULL) {
        return false;
    }
    if (!as5600_read_status(dev, &status)) {
        return false;
    }
    *detected = status.magnet_detected;
    return true;
}

bool as5600_read_agc(const as5600_t *dev, uint8_t *agc) {
    return as5600_read_u8(dev, AS5600_REG_AGC, agc);
}

bool as5600_read_magnitude(const as5600_t *dev, uint16_t *magnitude) {
    return as5600_read_u16(dev, AS5600_REG_MAGNITUDE_H, magnitude);
}

bool as5600_read_raw_angle_counts(const as5600_t *dev, uint16_t *counts) {
    return as5600_read_u16(dev, AS5600_REG_RAW_ANGLE_H, counts);
}

bool as5600_read_angle_counts(const as5600_t *dev, uint16_t *counts) {
    return as5600_read_u16(dev, AS5600_REG_ANGLE_H, counts);
}

bool as5600_read_raw_angle_degrees(const as5600_t *dev, float *degrees) {
    uint16_t counts = 0u;
    if (degrees == NULL) {
        return false;
    }
    if (!as5600_read_raw_angle_counts(dev, &counts)) {
        return false;
    }
    *degrees = as5600_counts_to_degrees_f(counts);
    return true;
}

bool as5600_read_angle_degrees(const as5600_t *dev, float *degrees) {
    uint16_t counts = 0u;
    if (degrees == NULL) {
        return false;
    }
    if (!as5600_read_angle_counts(dev, &counts)) {
        return false;
    }
    *degrees = as5600_counts_to_degrees_f(counts);
    return true;
}

bool as5600_read_raw_angle_counts_zeroed(const as5600_t *dev, uint16_t *counts) {
    uint16_t raw = 0u;
    if (dev == NULL || counts == NULL) {
        return false;
    }
    if (!as5600_read_raw_angle_counts(dev, &raw)) {
        return false;
    }
    *counts = as5600_wrap_counts((uint16_t)(raw - dev->zero_offset_counts));
    return true;
}

bool as5600_read_raw_angle_degrees_zeroed(const as5600_t *dev, float *degrees) {
    uint16_t counts = 0u;
    if (degrees == NULL) {
        return false;
    }
    if (!as5600_read_raw_angle_counts_zeroed(dev, &counts)) {
        return false;
    }
    *degrees = as5600_counts_to_degrees_f(counts);
    return true;
}

bool as5600_read_conf(const as5600_t *dev, as5600_conf_t *conf) {
    uint16_t reg = 0u;
    if (conf == NULL) {
        return false;
    }
    if (!as5600_read_u16(dev, AS5600_REG_CONF_H, &reg)) {
        return false;
    }
    conf_unpack(reg, conf);
    return true;
}

bool as5600_write_conf(const as5600_t *dev, const as5600_conf_t *conf) {
    return as5600_write_u16(dev, AS5600_REG_CONF_H, conf_pack(conf));
}

bool as5600_write_zpos(const as5600_t *dev, uint16_t zpos) {
    return as5600_write_u16(dev, AS5600_REG_ZPOS_H, zpos);
}

bool as5600_write_mpos(const as5600_t *dev, uint16_t mpos) {
    return as5600_write_u16(dev, AS5600_REG_MPOS_H, mpos);
}

bool as5600_write_mang(const as5600_t *dev, uint16_t mang) {
    return as5600_write_u16(dev, AS5600_REG_MANG_H, mang);
}

bool as5600_program_range_volatile(const as5600_t *dev, uint16_t zpos, uint16_t mpos) {
    if (!as5600_write_zpos(dev, zpos)) {
        return false;
    }
    if (!as5600_write_mpos(dev, mpos)) {
        return false;
    }
    return true;
}

bool as5600_burn_angle(const as5600_t *dev) {
    return as5600_write_u8(dev, AS5600_REG_BURN, AS5600_BURN_ANGLE_CMD);
}

bool as5600_burn_settings(const as5600_t *dev) {
    return as5600_write_u8(dev, AS5600_REG_BURN, AS5600_BURN_SETTING_CMD);
}

uint16_t as5600_counts_to_degrees_x100(uint16_t counts) {
    uint32_t x100 = ((uint32_t)as5600_wrap_counts(counts) * 36000u) / 4096u;
    return (uint16_t)x100;
}

float as5600_counts_to_degrees_f(uint16_t counts) {
    return (float)as5600_wrap_counts(counts) * AS5600_DEGREES_PER_COUNT;
}

uint16_t as5600_degrees_to_counts_f(float degrees) {
    float wrapped = as5600_wrap_degrees_360(degrees);
    uint32_t counts = (uint32_t)((wrapped * 4096.0f / 360.0f) + 0.5f);
    return as5600_wrap_counts((uint16_t)counts);
}

uint16_t as5600_wrap_counts(uint16_t counts) {
    return (uint16_t)(counts & 0x0FFFu);
}

float as5600_wrap_degrees_360(float degrees) {
    while (degrees >= 360.0f) {
        degrees -= 360.0f;
    }
    while (degrees < 0.0f) {
        degrees += 360.0f;
    }
    return degrees;
}

int16_t as5600_shortest_delta_counts(uint16_t from_counts, uint16_t to_counts) {
    int16_t delta = (int16_t)(as5600_wrap_counts(to_counts) - as5600_wrap_counts(from_counts));
    if (delta > 2048) {
        delta -= 4096;
    } else if (delta < -2048) {
        delta += 4096;
    }
    return delta;
}

float as5600_shortest_delta_degrees(float from_deg, float to_deg) {
    float delta = as5600_wrap_degrees_360(to_deg) - as5600_wrap_degrees_360(from_deg);
    if (delta > 180.0f) {
        delta -= 360.0f;
    } else if (delta < -180.0f) {
        delta += 360.0f;
    }
    return delta;
}
