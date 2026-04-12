/*
    Driver for the MaUWB_DW3000.
    Bus-agnostic implementation: this driver only builds and parses AT command frames.

    References:
    - https://www.makerfabs.com/mauwb-dw3000-chipset.html

    @ UBC Rocket, Robert Zhao
*/

#ifndef SENSORS_MAUWB_H
#define SENSORS_MAUWB_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* -------------------------------------------------------------------------- */
/* UART AT command protocol                                                   */
/* -------------------------------------------------------------------------- */

#define MAUWB_CMD_TERMINATOR           "\r\n"
#define MAUWB_CMD_TERMINATOR_LEN       2U

#ifndef MAUWB_MAX_CMD_LEN
#define MAUWB_MAX_CMD_LEN              64U
#endif

#ifndef MAUWB_MAX_RESP_LEN
#define MAUWB_MAX_RESP_LEN             128U
#endif

/* -------------------------------------------------------------------------- */
/* Base commands                                                              */
/* -------------------------------------------------------------------------- */

#define MAUWB_CMD_AT_TEST              "AT?"          /* Serial port test */
#define MAUWB_CMD_GET_VERSION          "AT+GETVER?"   /* Get firmware/module version */
#define MAUWB_CMD_RESTART              "AT+RESTART"   /* Restart module */
#define MAUWB_CMD_RESTORE              "AT+RESTORE"   /* Restore configuration */
#define MAUWB_CMD_SAVE                 "AT+SAVE"      /* Save configuration */

/* -------------------------------------------------------------------------- */
/* Configuration commands                                                     */
/* -------------------------------------------------------------------------- */

#define MAUWB_CMD_SET_CFG              "AT+SETCFG"    /* Set role message */
#define MAUWB_CMD_GET_CFG              "AT+GETCFG?"   /* Get role message */
#define MAUWB_CMD_SET_ANT_DELAY        "AT+SETANT"    /* Set antenna delay */
#define MAUWB_CMD_GET_ANT_DELAY        "AT+GETANT?"   /* Get antenna delay */
#define MAUWB_CMD_SET_CAPACITY         "AT+SETCAP"    /* Set anchor/tag capacity of the system */
#define MAUWB_CMD_GET_CAPACITY         "AT+GETCAP?"   /* Get anchor/tag capacity of the system */
#define MAUWB_CMD_SET_REPORT           "AT+SETRPT"    /* Set the automatic reporting status */
#define MAUWB_CMD_GET_REPORT           "AT+GETRPT?"   /* Get the automatic reporting status */
#define MAUWB_CMD_RANGE                "AT+RANGE"     /* Active reporting command */
#define MAUWB_CMD_SLEEP                "AT+SLEEP"     /* Sleep command (tag only) */
#define MAUWB_CMD_SET_POWER            "AT+SETPOW"    /* Configure device transmission power */
#define MAUWB_CMD_GET_POWER            "AT+GETPOW?"   /* Get device transmit power */
#define MAUWB_CMD_DATA                 "AT+DATA"      /* Transparent data transmission */
#define MAUWB_CMD_RDATA                "AT+RDATA"     /* Read received transparent data */
#define MAUWB_CMD_SET_PAN              "AT+SETPAN"    /* Set network differentiation field */
#define MAUWB_CMD_GET_PAN              "AT+GETPAN?"   /* Get network differentiation field */

/* -------------------------------------------------------------------------- */
/* Expected response tokens                                                   */
/* -------------------------------------------------------------------------- */

#define MAUWB_RESP_OK                  "OK"
#define MAUWB_RESP_ERROR               "ERROR"

/* -------------------------------------------------------------------------- */
/* Status                                                                     */
/* -------------------------------------------------------------------------- */

typedef enum {
    UWB_PARSE_OK = 0,
    UWB_PARSE_NULL,
    UWB_PARSE_BUF_TOO_SMALL,
    UWB_PARSE_INVALID_RESPONSE
} mauwb_parse_status_t;

typedef enum {
    MAUWB_ROLE_TAG = 0,
    MAUWB_ROLE_ANCHOR = 1
} mauwb_role_t;

typedef enum {
    MAUWB_RATE_850K = 0,
    MAUWB_RATE_6M8 = 1
} mauwb_rate_t;

typedef enum {
    MAUWB_FILTER_DISABLED = 0,
    MAUWB_FILTER_ENABLED = 1
} mauwb_filter_t;

typedef enum {
    MAUWB_EXT_MODE_NORMAL = 0,
    MAUWB_EXT_MODE_EXTENDED = 1
} mauwb_ext_mode_t;

/* -------------------------------------------------------------------------- */
/* Command builders                                                           */
/* -------------------------------------------------------------------------- */

/**
 * @brief Build a full AT command into a TX buffer.
 *
 * Result format:
 *   <cmd><CR><LF>
 *
 * Example:
 *   "AT?" -> "AT?\r\n"
 *
 * @param cmd         Null-terminated command string macro.
 * @param tx_buf      Output buffer.
 * @param tx_buf_size Size of tx_buf in bytes.
 * @return Number of bytes written, or 0 on failure.
 */
size_t mauwb_build_command(const char *cmd, uint8_t *tx_buf, size_t tx_buf_size);

/**
 * @brief Build the serial-port test command "AT?\r\n".
 *
 * @param tx_buf      Output buffer.
 * @param tx_buf_size Size of tx_buf in bytes.
 * @return Number of bytes written, or 0 on failure.
 */
size_t mauwb_build_at_test(uint8_t *tx_buf, size_t tx_buf_size);
size_t mauwb_build_restart(uint8_t *tx_buf, size_t tx_buf_size);
size_t mauwb_build_save(uint8_t *tx_buf, size_t tx_buf_size);
size_t mauwb_build_setcfg(uint8_t *tx_buf,
                          size_t tx_buf_size,
                          uint8_t device_id,
                          mauwb_role_t role,
                          mauwb_rate_t rate,
                          mauwb_filter_t filter_enable);
size_t mauwb_build_setcap(uint8_t *tx_buf,
                          size_t tx_buf_size,
                          uint8_t tag_capacity,
                          uint16_t slot_time_ms,
                          mauwb_ext_mode_t ext_mode);
size_t mauwb_build_setrpt(uint8_t *tx_buf,
                          size_t tx_buf_size,
                          uint8_t report_enable);

/* -------------------------------------------------------------------------- */
/* Response parsers                                                           */
/* -------------------------------------------------------------------------- */

/**
 * @brief Check whether a token appears anywhere in the received buffer.
 *
 * @param rx_buf  Received bytes.
 * @param rx_len  Number of valid bytes in rx_buf.
 * @param token   Null-terminated token string.
 * @return true if token is found, false otherwise.
 */
bool mauwb_response_contains_token(const uint8_t *rx_buf, size_t rx_len, const char *token);

/**
 * @brief Check whether the module responded with a valid "AT?" success reply.
 *
 * For bring-up, we consider the serial port healthy if:
 * - response contains "OK"
 * - response does not contain "ERROR"
 *
 * @param rx_buf  Received bytes.
 * @param rx_len  Number of valid bytes in rx_buf.
 * @return true if reply looks valid, false otherwise.
 */
bool mauwb_is_serial_port_ok(const uint8_t *rx_buf, size_t rx_len);

#endif /* SENSORS_MAUWB_H */
