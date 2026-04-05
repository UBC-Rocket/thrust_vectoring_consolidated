#include "main.h"
#include "as5600.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * STM32 HAL glue layer.
 *
 * Keep the portable driver in as5600.c free from direct HAL dependencies.
 * Only this file knows about HAL_I2C_* symbols and TIMING details.
 */

bool as5600_device_is_ready(void *hi2c, uint16_t addr_8bit, uint32_t timeout_ms) {
    if (hi2c == NULL) {
        return false;
    }

    I2C_HandleTypeDef *h = (I2C_HandleTypeDef *)hi2c;
    return (HAL_I2C_IsDeviceReady(h, addr_8bit, 2u, timeout_ms) == HAL_OK);
}

bool as5600_device_mem_read(void *hi2c,
                            uint16_t addr_8bit,
                            uint8_t reg,
                            uint8_t *data,
                            uint16_t len,
                            uint32_t timeout_ms) {
    if (hi2c == NULL || data == NULL || len == 0u) {
        return false;
    }

    I2C_HandleTypeDef *h = (I2C_HandleTypeDef *)hi2c;
    return (HAL_I2C_Mem_Read(h,
                             addr_8bit,
                             reg,
                             I2C_MEMADD_SIZE_8BIT,
                             data,
                             len,
                             timeout_ms) == HAL_OK);
}

bool as5600_device_mem_write(void *hi2c,
                             uint16_t addr_8bit,
                             uint8_t reg,
                             const uint8_t *data,
                             uint16_t len,
                             uint32_t timeout_ms) {
    if (hi2c == NULL || data == NULL || len == 0u) {
        return false;
    }

    I2C_HandleTypeDef *h = (I2C_HandleTypeDef *)hi2c;
    return (HAL_I2C_Mem_Write(h,
                              addr_8bit,
                              reg,
                              I2C_MEMADD_SIZE_8BIT,
                              (uint8_t *)data,
                              len,
                              timeout_ms) == HAL_OK);
}
