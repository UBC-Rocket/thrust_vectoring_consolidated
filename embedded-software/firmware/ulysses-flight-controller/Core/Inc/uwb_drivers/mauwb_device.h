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

/* -------------------------------------------------------------------------- */
/* Receive-side types                                                         */
/* -------------------------------------------------------------------------- */

#define UWB_MAX_ANCHORS   4U    /**< Max anchors parsed per output line        */
#define UWB_DMA_BUF_LEN   256U  /**< Circular DMA receive buffer size (bytes)  */
#define UWB_RING_DEPTH    8U    /**< SPSC ring capacity (max occupancy: 7)     */

/**
 * @brief One complete UWB ranging measurement, produced once per '\n'.
 *
 * The MaUWB TAG outputs:  AN0:DDDD,AN1:DDDD,...\r\n  (values in centimetres).
 * distance_m[i] and anchor_ids[i] are paired: anchor_ids[i] is the anchor ID
 * from the line (e.g. 1 for "AN1:"), and distance_m[i] is its distance in
 * metres. anchor_count valid entries. Anchors that did not respond are absent
 * from the line entirely, so anchor_ids must be used to match distances to
 * known anchor positions rather than assuming index == ID.
 */
typedef struct {
    float    distance_m[UWB_MAX_ANCHORS]; /**< Distance to each anchor (m)     */
    uint8_t  anchor_ids[UWB_MAX_ANCHORS]; /**< Anchor ID for each distance      */
    uint8_t  anchor_count;                /**< Number of valid entries          */
    uint32_t timestamp_ms;                /**< HAL_GetTick() at parse time      */
} uwb_measurement_t;


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

/* -------------------------------------------------------------------------- */
/* Receive-side API                                                           */
/* -------------------------------------------------------------------------- */

/**
 * @brief Start continuous DMA reception from the MaUWB TAG.
 *
 * Uses HAL_UARTEx_ReceiveToIdle_DMA on GPDMA2_Channel3 (already configured
 * in DMA_NORMAL mode by CubeMX).  Fires HAL_UARTEx_RxEventCallback (overridden
 * in mauwb_device.c) on Idle and TC events — no changes to stm32h5xx_it.c or
 * main.c are needed.  Call once after mauwb_device_setup() completes.
 *
 * @param dev  Initialised device wrapper (huart must be &huart2).
 * @return MAUWB_DEV_OK on success.
 */
mauwb_device_status_t mauwb_device_start_receive(mauwb_device_t *dev);

/**
 * @brief Pop one measurement from the receive ring buffer (consumer side).
 *
 * Non-blocking.  Call from the StateEstimation task.
 *
 * @param out  Destination struct; written only on true return.
 * @return true if a measurement was dequeued, false if the ring is empty.
 */
bool mauwb_device_dequeue(uwb_measurement_t *out);

/**
 * @brief Return the cumulative count of parse / ring-full errors since init.
 */
uint32_t mauwb_parse_error_count(void);

#endif /* MAUWB_DEVICE_H */
