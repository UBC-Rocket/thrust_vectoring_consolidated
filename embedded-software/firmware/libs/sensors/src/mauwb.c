#include "sensors/mauwb.h"

#include <stdio.h>
#include <string.h>

size_t mauwb_build_command(const char *cmd, uint8_t *tx_buf, size_t tx_buf_size)
{
    size_t cmd_len;

    if ((cmd == NULL) || (tx_buf == NULL)) {
        return 0U;
    }

    cmd_len = strlen(cmd);

    if ((cmd_len + MAUWB_CMD_TERMINATOR_LEN) > tx_buf_size) {
        return 0U;
    }

    memcpy(tx_buf, cmd, cmd_len);
    memcpy(&tx_buf[cmd_len], MAUWB_CMD_TERMINATOR, MAUWB_CMD_TERMINATOR_LEN);

    return cmd_len + MAUWB_CMD_TERMINATOR_LEN;
}

bool mauwb_response_contains_token(const uint8_t *rx_buf, size_t rx_len, const char *token)
{
    size_t token_len;
    size_t i;

    if ((rx_buf == NULL) || (token == NULL)) {
        return false;
    }

    token_len = strlen(token);

    if ((token_len == 0U) || (token_len > rx_len)) {
        return false;
    }

    for (i = 0U; i + token_len <= rx_len; ++i) {
        if (memcmp(&rx_buf[i], token, token_len) == 0) {
            return true;
        }
    }

    return false;
}

size_t mauwb_build_at_test(uint8_t *tx_buf, size_t tx_buf_size)
{
    return mauwb_build_command(MAUWB_CMD_AT_TEST, tx_buf, tx_buf_size);
}

size_t mauwb_build_restart(uint8_t *tx_buf, size_t tx_buf_size)
{
    return mauwb_build_command(MAUWB_CMD_RESTART, tx_buf, tx_buf_size);
}

size_t mauwb_build_save(uint8_t *tx_buf, size_t tx_buf_size)
{
    return mauwb_build_command(MAUWB_CMD_SAVE, tx_buf, tx_buf_size);
}

size_t mauwb_build_setcfg(uint8_t *tx_buf,
                          size_t tx_buf_size,
                          uint8_t device_id,
                          mauwb_role_t role,
                          mauwb_rate_t rate,
                          mauwb_filter_t filter_enable)
{
    int n;

    if (tx_buf == NULL) {
        return 0U;
    }

    if ((role != MAUWB_ROLE_TAG) && (role != MAUWB_ROLE_ANCHOR)) {
        return 0U;
    }

    if ((rate != MAUWB_RATE_850K) && (rate != MAUWB_RATE_6M8)) {
        return 0U;
    }

    if ((filter_enable != MAUWB_FILTER_DISABLED) &&
        (filter_enable != MAUWB_FILTER_ENABLED)) {
        return 0U;
    }

    n = snprintf((char *)tx_buf,
                 tx_buf_size,
                 "%s=%u,%u,%u,%u%s",
                 MAUWB_CMD_SET_CFG,
                 device_id,
                 (unsigned)role,
                 (unsigned)rate,
                 (unsigned)filter_enable,
                 MAUWB_CMD_TERMINATOR);
    if ((n < 0) || ((size_t)n >= tx_buf_size)) {
        return 0U;
    }

    return (size_t)n;
}

size_t mauwb_build_setcap(uint8_t *tx_buf,
                          size_t tx_buf_size,
                          uint8_t tag_capacity,
                          uint16_t slot_time_ms,
                          mauwb_ext_mode_t ext_mode)
{
    int n;

    if (tx_buf == NULL) {
        return 0U;
    }

    if ((tag_capacity == 0U) || (tag_capacity > 64U)) {
        return 0U;
    }

    if (slot_time_ms == 0U) {
        return 0U;
    }

    if ((ext_mode != MAUWB_EXT_MODE_NORMAL) &&
        (ext_mode != MAUWB_EXT_MODE_EXTENDED)) {
        return 0U;
    }

    n = snprintf((char *)tx_buf,
                 tx_buf_size,
                 "%s=%u,%u,%u%s",
                 MAUWB_CMD_SET_CAPACITY,
                 tag_capacity,
                 slot_time_ms,
                 (unsigned)ext_mode,
                 MAUWB_CMD_TERMINATOR);
    if ((n < 0) || ((size_t)n >= tx_buf_size)) {
        return 0U;
    }

    return (size_t)n;
}

size_t mauwb_build_setrpt(uint8_t *tx_buf,
                          size_t tx_buf_size,
                          uint8_t report_enable)
{
    int n;

    if (tx_buf == NULL) {
        return 0U;
    }

    if (report_enable > 1U) {
        return 0U;
    }

    n = snprintf((char *)tx_buf,
                 tx_buf_size,
                 "%s=%u%s",
                 MAUWB_CMD_SET_REPORT,
                 report_enable,
                 MAUWB_CMD_TERMINATOR);
    if ((n < 0) || ((size_t)n >= tx_buf_size)) {
        return 0U;
    }

    return (size_t)n;
}

bool mauwb_is_serial_port_ok(const uint8_t *rx_buf, size_t rx_len)
{
    if ((rx_buf == NULL) || (rx_len == 0U)) {
        return false;
    }

    if (mauwb_response_contains_token(rx_buf, rx_len, MAUWB_RESP_ERROR)) {
        return false;
    }

    return mauwb_response_contains_token(rx_buf, rx_len, MAUWB_RESP_OK);
}
