#ifndef MAUWB_DEVICE_H
#define MAUWB_DEVICE_H

#include "stm32h5xx_hal.h"
#include "sensors/mauwb.h"
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    MAUWB_DEV_OK = 0,
    MAUWB_DEV_NULL,
    MAUWB_DEV_UART_ERROR,
    MAUWB_DEV_BUILD_ERROR,
    MAUWB_DEV_BAD_RESPONSE
} mauwb_device_status_t;

typedef struct {
    uint8_t device_id;
    mauwb_role_t role;
    mauwb_rate_t rate;
    mauwb_filter_t filter_enable;

    uint8_t tag_capacity;
    uint16_t slot_time_ms;
    mauwb_ext_mode_t ext_mode;
} mauwb_config_t;

typedef struct {
    UART_HandleTypeDef *huart;
    uint32_t timeout_ms;
    mauwb_config_t config;
} mauwb_device_t;


/**
 * @brief Initialize MAUWB UART device wrapper.
 *
 * @param dev        Device wrapper to initialize.
 * @param huart      UART handle used to talk to the module.
 * @param timeout_ms Blocking UART timeout in milliseconds.
 * @param config     Initial MAUWB configuration copied into dev->config.
 */
void mauwb_device_init(mauwb_device_t *dev,
                       UART_HandleTypeDef *huart,
                       uint32_t timeout_ms,
                       const mauwb_config_t *config);


/**
 * @brief Transmit a raw byte buffer to the MAUWB module.
 *
 * @param dev   Device wrapper.
 * @param data  Buffer to transmit.
 * @param len   Number of bytes to transmit.
 * @return MAUWB_DEV_OK on success.
 */
mauwb_device_status_t mauwb_device_write(mauwb_device_t *dev,
                                         const uint8_t *data,
                                         size_t len);


/**
 * @brief Transmit a null-terminated ASCII command string.
 *
 * @param dev  Device wrapper.
 * @param cmd  Null-terminated command string.
 * @return MAUWB_DEV_OK on success.
 */
mauwb_device_status_t mauwb_device_write_string(mauwb_device_t *dev,
                                                const char *cmd);

mauwb_device_status_t mauwb_device_setup(mauwb_device_t *dev);

#endif /* MAUWB_DEVICE_H */
