#include "uwb_drivers/mauwb_device.h"

#include "collections/spsc_ring.h"
#include <string.h>

/* ============================================================================
 * Receive-side private state
 * ============================================================================ */

SPSC_RING_DECLARE(uwb_ring, uwb_measurement_t, UWB_RING_DEPTH)

/** Circular DMA receive buffer — DMA writes here continuously */
static uint8_t  s_dma_buf[UWB_DMA_BUF_LEN];

/** Last-consumed position in s_dma_buf (tracks the consumer's read head) */
static volatile uint16_t s_last_pos;

/** Line accumulation buffer — bytes appended until '\n' fires */
static char     s_line_buf[UWB_DMA_BUF_LEN];
static uint16_t s_line_len;

/** SPSC ring populated by ISR, drained by StateEstimation task */
static uwb_ring_t s_ring;

/** Count of parse failures and ring-full overruns since init */
static volatile uint32_t s_parse_errors;

/** UART handle stored at start_receive time for the error recovery path */
static UART_HandleTypeDef *s_huart;

/** True after mauwb_device_start_receive() succeeds */
static volatile bool s_initialized;

/* ============================================================================
 * Private helpers
 * ============================================================================ */

/**
 * @brief Parse a decimal integer at *p, advancing *p past the digits.
 * @return Value >= 0, or -1 if no digit is found.
 */
static int32_t parse_decimal(const char **p, const char *end)
{
    if ((*p >= end) || (**p < '0') || (**p > '9')) {
        return -1;
    }
    int32_t val = 0;
    while ((*p < end) && (**p >= '0') && (**p <= '9')) {
        val = val * 10 + (**p - '0');
        (*p)++;
    }
    return val;
}

/**
 * @brief Parse one complete "AN0:1234,AN1:0567,...\r\n" line into *out.
 *
 * Range gate: 10 cm – 10 000 cm (0.10 m – 100 m).
 * Returns false on empty parse or out-of-range value.
 */
static bool parse_line(const char *buf, uint16_t len, uwb_measurement_t *out)
{
    const char *p   = buf;
    const char *end = buf + len;
    uint8_t     count = 0U;

    memset(out, 0, sizeof(*out));

    while ((p < end) && (count < UWB_MAX_ANCHORS)) {
        /* Scan for "AN" prefix */
        if ((end - p) < 4) break;   /* minimum: "ANx:" */
        if ((p[0] != 'A') || (p[1] != 'N')) { p++; continue; }
        p += 2U;

        /* Skip anchor-index digit(s) */
        while ((p < end) && (*p >= '0') && (*p <= '9')) { p++; }

        /* Expect ':' */
        if ((p >= end) || (*p != ':')) break;
        p++;

        /* Parse distance value (centimetres) */
        int32_t val = parse_decimal(&p, end);
        if (val < 0) break;

        /* Range gate */
        if ((val < 10L) || (val > 10000L)) {
            return false;
        }

        out->distance_m[count++] = (float)val * 0.01f;

        /* Skip ',' separator */
        if ((p < end) && (*p == ',')) { p++; }
    }

    if (count == 0U) {
        return false;
    }

    out->anchor_count  = count;
    out->timestamp_ms  = HAL_GetTick();
    return true;
}

/**
 * @brief Feed one byte to the line accumulator.
 *
 * On '\n': attempt parse → push to ring.  '\r' is silently skipped.
 */
static void feed_byte(uint8_t b)
{
    if (b == (uint8_t)'\n') {
        if (s_line_len > 0U) {
            uwb_measurement_t m;
            if (parse_line(s_line_buf, s_line_len, &m)) {
                if (!uwb_ring_push(&s_ring, &m)) {
                    s_parse_errors++;   /* ring full — overrun */
                }
            } else {
                s_parse_errors++;
            }
            s_line_len = 0U;
        }
    } else if (b != (uint8_t)'\r') {
        if (s_line_len < (uint16_t)(sizeof(s_line_buf) - 1U)) {
            s_line_buf[s_line_len++] = (char)b;
        } else {
            /* Line overflow — discard and start fresh */
            s_line_len = 0U;
            s_parse_errors++;
        }
    }
}

/**
 * @brief Process newly arrived bytes from the circular DMA buffer.
 *
 * Handles buffer wrap-around identically to radio_driver.c::process_dma_data().
 *
 * @param new_pos  Current DMA write head (= RxXferSize - BNDT counter).
 */
static void process_dma_data(uint16_t new_pos)
{
    uint16_t last = s_last_pos;
    uint16_t curr = new_pos;

    if (curr == last) {
        return;
    }

    if (curr > last) {
        for (uint16_t i = last; i < curr; i++) {
            feed_byte(s_dma_buf[i]);
        }
    } else {
        /* Wrap-around: consume tail of buffer, then head */
        for (uint16_t i = last; i < UWB_DMA_BUF_LEN; i++) {
            feed_byte(s_dma_buf[i]);
        }
        for (uint16_t i = 0U; i < curr; i++) {
            feed_byte(s_dma_buf[i]);
        }
    }

    s_last_pos = curr;
}

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

/* ============================================================================
 * Receive-side public API
 * ============================================================================ */

mauwb_device_status_t mauwb_device_start_receive(mauwb_device_t *dev)
{
    if ((dev == NULL) || (dev->huart == NULL)) {
        return MAUWB_DEV_NULL;
    }

    s_huart       = dev->huart;
    s_last_pos    = 0U;
    s_line_len    = 0U;
    s_initialized = false;
    memset(s_dma_buf, 0, sizeof(s_dma_buf));

    /* H5 GPDMA has no DMA_CIRCULAR mode constant — the correct STM32H5 API
     * for continuous variable-length UART reception is ReceiveToIdle_DMA.
     * It uses UART Idle line detection and works with the DMA_NORMAL channel
     * already configured by CubeMX (GPDMA2_Channel3, no re-init needed).
     *
     * Callback behaviour:
     *   HAL_UART_RXEVENT_IDLE → Size = bytes received so far → parse new data
     *   HAL_UART_RXEVENT_TC   → Size = UWB_DMA_BUF_LEN (buffer full) → restart
     *   HAL_UART_RXEVENT_HT   → suppressed via __HAL_DMA_DISABLE_IT below
     */
    if (HAL_UARTEx_ReceiveToIdle_DMA(s_huart, s_dma_buf, UWB_DMA_BUF_LEN) != HAL_OK) {
        return MAUWB_DEV_UART_ERROR;
    }

    /* Suppress half-transfer interrupts — only Idle and TC are needed */
    __HAL_DMA_DISABLE_IT(s_huart->hdmarx, DMA_IT_HT);

    s_initialized = true;
    return MAUWB_DEV_OK;
}

bool mauwb_device_dequeue(uwb_measurement_t *out)
{
    if (out == NULL) {
        return false;
    }
    return uwb_ring_pop(&s_ring, out);
}

uint32_t mauwb_parse_error_count(void)
{
    return s_parse_errors;
}

/* ============================================================================
 * HAL weak-function overrides
 *
 * These are the only wiring needed — no changes to stm32h5xx_it.c or main.c.
 * HAL_UART_IRQHandler() calls both of these internally via the UART Idle and
 * error interrupt paths.
 * ============================================================================ */

/**
 * @brief UART Rx Event callback (IDLE / TC events from ReceiveToIdle_DMA).
 *
 * On IDLE: parse newly arrived bytes.
 * On TC (buffer full): parse remaining bytes, reset position, restart DMA.
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if ((huart->Instance != USART2) || !s_initialized) {
        return;
    }

    if (HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_TC) {
        /* Buffer completely filled — process tail then restart immediately.
         * The restart gap at 115200 baud is ~few bit-times, negligible relative
         * to the MaUWB 10 ms line rate. */
        process_dma_data(Size);
        s_last_pos = 0U;
        HAL_UARTEx_ReceiveToIdle_DMA(s_huart, s_dma_buf, UWB_DMA_BUF_LEN);
        __HAL_DMA_DISABLE_IT(s_huart->hdmarx, DMA_IT_HT);
    } else {
        /* IDLE (or HT if not suppressed): process newly arrived bytes */
        process_dma_data(Size);
    }
}

/**
 * @brief UART error callback — reset state and restart DMA reception.
 *
 * Guards on USART2 so other UART handles are unaffected.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if ((huart->Instance != USART2) || !s_initialized) {
        return;
    }

    s_last_pos = 0U;
    s_line_len = 0U;
    HAL_UARTEx_ReceiveToIdle_DMA(s_huart, s_dma_buf, UWB_DMA_BUF_LEN);
    __HAL_DMA_DISABLE_IT(s_huart->hdmarx, DMA_IT_HT);
}
