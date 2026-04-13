#include "uwb_drivers/mauwb_device.h"

#include <string.h>

static mauwb_device_status_t mauwb_device_send_and_check(mauwb_device_t *dev,
                                                         const uint8_t *tx_buf,
                                                         size_t tx_len,
                                                         bool expect_ok)
{
    uint8_t rx_buf[MAUWB_MAX_RESP_LEN];
    HAL_StatusTypeDef hal_status;
    size_t rx_len;
    size_t i;

    if ((dev == NULL) || (dev->huart == NULL) || (tx_buf == NULL) || (tx_len == 0U)) {
        return MAUWB_DEV_NULL;
    }

    memset(rx_buf, 0, sizeof(rx_buf));

    hal_status = HAL_UART_Transmit(dev->huart,
                                   (uint8_t *)tx_buf,
                                   (uint16_t)tx_len,
                                   dev->timeout_ms);
    if (hal_status != HAL_OK) {
        return MAUWB_DEV_UART_ERROR;
    }

    if (!expect_ok) {
        return MAUWB_DEV_OK;
    }

    hal_status = HAL_UART_Receive(dev->huart,
                                  rx_buf,
                                  (uint16_t)sizeof(rx_buf),
                                  dev->timeout_ms);
    if ((hal_status != HAL_OK) && (hal_status != HAL_TIMEOUT)) {
        return MAUWB_DEV_UART_ERROR;
    }

    rx_len = sizeof(rx_buf);
    for (i = 0U; i < sizeof(rx_buf); ++i) {
        if (rx_buf[i] == 0U) {
            rx_len = i;
            break;
        }
    }

    if (!mauwb_is_serial_port_ok(rx_buf, rx_len)) {
        return MAUWB_DEV_BAD_RESPONSE;
    }

    return MAUWB_DEV_OK;
}

void mauwb_device_init(mauwb_device_t *dev,
                       UART_HandleTypeDef *huart,
                       uint32_t timeout_ms,
                       const mauwb_config_t *config)
{
    if ((dev == NULL) || (config == NULL)) {
        return;
    }

    dev->huart = huart;
    dev->timeout_ms = timeout_ms;
    dev->config = *config;
}

mauwb_device_status_t mauwb_device_write(mauwb_device_t *dev,
                                         const uint8_t *data,
                                         size_t len)
{
    HAL_StatusTypeDef hal_status;

    if ((dev == NULL) || (dev->huart == NULL) || (data == NULL)) {
        return MAUWB_DEV_NULL;
    }

    hal_status = HAL_UART_Transmit(dev->huart,
                                   (uint8_t *)data,
                                   (uint16_t)len,
                                   dev->timeout_ms);

    if (hal_status != HAL_OK) {
        return MAUWB_DEV_UART_ERROR;
    }

    return MAUWB_DEV_OK;
}

mauwb_device_status_t mauwb_device_write_string(mauwb_device_t *dev,
                                                const char *cmd)
{
    if (cmd == NULL) {
        return MAUWB_DEV_NULL;
    }

    return mauwb_device_write(dev, (const uint8_t *)cmd, strlen(cmd));
}

mauwb_device_status_t mauwb_device_setup(mauwb_device_t *dev)
{
    uint8_t tx_buf[MAUWB_MAX_CMD_LEN];
    size_t tx_len;
    mauwb_device_status_t status;

    if (dev == NULL) {
        return MAUWB_DEV_NULL;
    }
    // send AT?
    tx_len = mauwb_build_at_test(tx_buf, sizeof(tx_buf));
    if (tx_len == 0U) {
        return MAUWB_DEV_BUILD_ERROR;
    }
    status = mauwb_device_send_and_check(dev, tx_buf, tx_len, true);
    if (status != MAUWB_DEV_OK) {
        return status;
    }
    // send AT_RESTORE
    tx_len = mauwb_build_command(MAUWB_CMD_RESTORE, tx_buf, sizeof(tx_buf));
    if (tx_len == 0U) {
        return MAUWB_DEV_BUILD_ERROR;
    }
    status = mauwb_device_send_and_check(dev, tx_buf, tx_len, false);
    if (status != MAUWB_DEV_OK) {
        return status;
    }
    // send AT_SETCFG
    tx_len = mauwb_build_setcfg(tx_buf,
                                sizeof(tx_buf),
                                dev->config.device_id,
                                dev->config.role,
                                dev->config.rate,
                                dev->config.filter_enable);
    if (tx_len == 0U) {
        return MAUWB_DEV_BUILD_ERROR;
    }
    status = mauwb_device_send_and_check(dev, tx_buf, tx_len, false);
    if (status != MAUWB_DEV_OK) {
        return status;
    }
    // send AT_SETCAP
    tx_len = mauwb_build_setcap(tx_buf,
                                sizeof(tx_buf),
                                dev->config.tag_capacity,
                                dev->config.slot_time_ms,
                                dev->config.ext_mode);
    if (tx_len == 0U) {
        return MAUWB_DEV_BUILD_ERROR;
    }
    status = mauwb_device_send_and_check(dev, tx_buf, tx_len, false);
    if (status != MAUWB_DEV_OK) {
        return status;
    }
    // send AT_SETRPT
    tx_len = mauwb_build_setrpt(tx_buf, sizeof(tx_buf), 1U);
    if (tx_len == 0U) {
        return MAUWB_DEV_BUILD_ERROR;
    }
    status = mauwb_device_send_and_check(dev, tx_buf, tx_len, false);
    if (status != MAUWB_DEV_OK) {
        return status;
    }
    // send AT_SAVE
    tx_len = mauwb_build_save(tx_buf, sizeof(tx_buf));
    if (tx_len == 0U) {
        return MAUWB_DEV_BUILD_ERROR;
    }
    status = mauwb_device_send_and_check(dev, tx_buf, tx_len, false);
    if (status != MAUWB_DEV_OK) {
        return status;
    }
    // send AT_RESTART
    tx_len = mauwb_build_restart(tx_buf, sizeof(tx_buf));
    if (tx_len == 0U) {
        return MAUWB_DEV_BUILD_ERROR;
    }
    return mauwb_device_send_and_check(dev, tx_buf, tx_len, false);
    /* ====== MaUWB initilization complete ====== */
}
