#include "motor_drivers/dshot/bdshot_dma.h"

#include "stm32h5xx.h"
#include "stm32h563xx.h"
#include "stm32h5xx_hal_def.h"
#include "stm32h5xx_hal_dma.h"
#include "stm32h5xx_hal_tim.h"
#include "stm32h5xx_ll_tim.h"
#include "stm32h5xx_ll_dma.h"

#include "motor_drivers/dshot/dshot.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#define MOTOR_INDEX_INVALID        (UINT32_MAX)
#define PERIPHERAL_CHANNEL_INVALID (UINT32_MAX)

typedef enum bdshot_dma_direction {
    BDSHOT_DMA_DIRECTION_OUTPUT,
    BDSHOT_DMA_DIRECTION_INPUT,
} bdshot_dma_direction_t;

typedef struct bdshot_send_request {
    bool is_dirty;
    bool is_command;
    bdshot_frame_t frame;
} bdshot_send_request_t;

typedef struct bdshot_dma_motor {
    bool is_initialized;
    bool is_first_arm; // TODO: remove for hardware arm/disarm
    bool is_armed;
    bool is_telemetry_valid;

    /// Is the currently sent frame a command frame?
    bool is_command_frame;

    /// Index of the TIM peripheral used for DMA transfers
    /// for this motor
    size_t dma_timer_index;

    bdshot_dma_direction_t direction;
    bdshot_dma_motor_config_t config;
    bdshot_send_request_t send_request;
    bdshot_motor_telemetry_t telemetry;
} bdshot_dma_motor_t;

typedef struct bdshot_dma_timers {
    TIM_HandleTypeDef *tim;
    uint32_t active_dma_sources;
} bdshot_dma_timers_t;

// Timers associated with all initialized motors
static bdshot_dma_timers_t dma_timers[BDSHOT_DMA_MOTOR_COUNT];
static size_t dma_timers_count = 0;

// Configuration of motors
static bdshot_dma_motor_t motors[BDSHOT_DMA_MOTOR_COUNT];

static uint32_t bdshot_dma_tx_buffer[BDSHOT_DMA_MOTOR_COUNT][BDSHOT_DMA_TX_FRAME_SIZE];
static uint32_t bdshot_dma_rx_buffer[BDSHOT_DMA_MOTOR_COUNT][BDSHOT_DMA_RX_FRAME_SIZE];

static volatile uint32_t *get_timer_channel_ccrx_reg(TIM_HandleTypeDef *const tim,
                                                     uint32_t channel);
static volatile uint32_t get_timer_channel_dma_src(TIM_HandleTypeDef *const tim, uint32_t channel);
static uint32_t tim_channel_convert_hal_to_ll(uint32_t hal_channel);
static bool tim_channel_dma_set_enable(TIM_HandleTypeDef *const tim, uint32_t channel, bool enable);

static uint32_t find_motor_index_from_dma(DMA_HandleTypeDef *const dma);
static bool motor_config_is_valid(bdshot_dma_motor_config_t *config);
static void build_dma_tx_buffer(uint32_t *const buffer, bdshot_frame_t frame);
static bool motor_switch_to_rx(bdshot_dma_motor_t *const motor);
static bool motor_switch_to_tx(bdshot_dma_motor_t *const motor);
static bool motor_decode_telemetry(bdshot_dma_motor_t *const motor, uint32_t *const edge_times);
static void dma_transfer_complete_callback(DMA_HandleTypeDef *const dma);
static bool bdshot_decode_telemetry(bdshot_motor_telemetry_t *const telemetry,
                                    const uint32_t *const edge_times, uint32_t edge_count,
                                    uint8_t pole_count);

static volatile uint32_t *get_timer_channel_ccrx_reg(TIM_HandleTypeDef *const tim, uint32_t channel)
{
    switch (channel) {
    case TIM_CHANNEL_1:
        return &tim->Instance->CCR1;
    case TIM_CHANNEL_2:
        return &tim->Instance->CCR2;
    case TIM_CHANNEL_3:
        return &tim->Instance->CCR3;
    case TIM_CHANNEL_4:
        return &tim->Instance->CCR4;
    default:
        return NULL;
    }
}

static volatile uint32_t get_timer_channel_dma_src(TIM_HandleTypeDef *const tim, uint32_t channel)
{
    switch (channel) {
    case TIM_CHANNEL_1:
        return TIM_DMA_CC1;
    case TIM_CHANNEL_2:
        return TIM_DMA_CC2;
    case TIM_CHANNEL_3:
        return TIM_DMA_CC3;
    case TIM_CHANNEL_4:
        return TIM_DMA_CC4;
    default:
        return PERIPHERAL_CHANNEL_INVALID;
    }
}

static uint32_t tim_channel_convert_hal_to_ll(uint32_t hal_channel)
{
    switch (hal_channel) {
    case TIM_CHANNEL_1:
        return LL_TIM_CHANNEL_CH1;
    case TIM_CHANNEL_2:
        return LL_TIM_CHANNEL_CH2;
    case TIM_CHANNEL_3:
        return LL_TIM_CHANNEL_CH3;
    case TIM_CHANNEL_4:
        return LL_TIM_CHANNEL_CH4;
    default:
        return PERIPHERAL_CHANNEL_INVALID;
    }
}

static bool tim_channel_dma_set_enable(TIM_HandleTypeDef *const tim, uint32_t channel, bool enable)
{
    switch (channel) {
    case TIM_CHANNEL_1:
        if (enable) {
            LL_TIM_EnableDMAReq_CC1(tim->Instance);
        } else {
            LL_TIM_DisableDMAReq_CC1(tim->Instance);
        }
        break;
    case TIM_CHANNEL_2:
        if (enable) {
            LL_TIM_EnableDMAReq_CC2(tim->Instance);
        } else {
            LL_TIM_DisableDMAReq_CC2(tim->Instance);
        }
        break;
    case TIM_CHANNEL_3:
        if (enable) {
            LL_TIM_EnableDMAReq_CC3(tim->Instance);
        } else {
            LL_TIM_DisableDMAReq_CC3(tim->Instance);
        }
        break;
    case TIM_CHANNEL_4:
        if (enable) {
            LL_TIM_EnableDMAReq_CC4(tim->Instance);
        } else {
            LL_TIM_DisableDMAReq_CC4(tim->Instance);
        }
        break;
    default:
        return false;
    }

    return true;
}

static uint32_t find_motor_index_from_dma(DMA_HandleTypeDef *const dma)
{
    for (uint32_t index = 0; index < BDSHOT_DMA_MOTOR_COUNT; index++) {
        bdshot_dma_motor_t *motor = &motors[index];

        if (!motor->is_initialized) {
            continue;
        }

        if (motor->config.dma->Instance == dma->Instance) {
            return index;
        }
    }

    return MOTOR_INDEX_INVALID;
}

static bool motor_config_is_valid(bdshot_dma_motor_config_t *config)
{
    return config->tim != NULL && config->dma != NULL && config->pole_count != 0;
}

static void build_dma_tx_buffer(uint32_t *const buffer, bdshot_frame_t frame)
{
    for (uint8_t i = 0; i < BDSHOT_FRAME_BITS; i++) {
        buffer[i] = (frame & 0x8000) != 0 ? BDSHOT_DMA_T1H_TICKS : BDSHOT_DMA_T0H_TICKS;
        frame <<= 1;
    }

    buffer[BDSHOT_FRAME_BITS] = 0;
    buffer[BDSHOT_FRAME_BITS + 1] = 0;
}

static bool motor_switch_to_rx(bdshot_dma_motor_t *const motor)
{
    DMA_HandleTypeDef *dma = motor->config.dma;
    TIM_HandleTypeDef *tim = motor->config.tim;
    uint32_t tim_channel = motor->config.tim_channel;
    uint32_t ll_tim_channel = tim_channel_convert_hal_to_ll(tim_channel);

    if (ll_tim_channel == PERIPHERAL_CHANNEL_INVALID) {
        return false;
    }

    // Disable DMA while we are re-configuring channel to do transfers
    // from peripheral to memory
    __HAL_DMA_DISABLE(dma);

    // Set free running timer for input capture.
    // We want to have ARR preload enabled here since we want to ensure
    // all other motors in the same TIM has finished transferring the last byte.
    LL_TIM_EnableARRPreload(tim->Instance);
    LL_TIM_SetAutoReload(tim->Instance, UINT32_MAX);

    TIM_CCxChannelCmd(tim->Instance, tim_channel, TIM_CCx_DISABLE);

    LL_TIM_IC_Config(tim->Instance, ll_tim_channel,
                     LL_TIM_ACTIVEINPUT_DIRECTTI | LL_TIM_ICPSC_DIV1 | LL_TIM_IC_FILTER_FDIV1_N2 |
                         LL_TIM_IC_POLARITY_BOTHEDGE);

    TIM_CCxChannelCmd(tim->Instance, tim_channel, TIM_CCx_ENABLE);

    // Disable source increment and enable destination increment for DMA
    MODIFY_REG(dma->Instance->CTR1, DMA_CTR1_SINC, LL_DMA_SRC_FIXED);
    MODIFY_REG(dma->Instance->CTR1, DMA_CTR1_DINC, LL_DMA_DEST_INCREMENT);

    // Switch DMA direction to peripheral to memory
    MODIFY_REG(dma->Instance->CTR2, DMA_CTR2_DREQ, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

    return true;
}

static bool motor_switch_to_tx(bdshot_dma_motor_t *const motor)
{
    DMA_HandleTypeDef *dma = motor->config.dma;
    TIM_HandleTypeDef *tim = motor->config.tim;
    uint32_t tim_channel = motor->config.tim_channel;
    uint32_t ll_tim_channel = tim_channel_convert_hal_to_ll(tim_channel);

    if (ll_tim_channel == PERIPHERAL_CHANNEL_INVALID) {
        return false;
    }

    // Disable DMA since since we are re-configuring channel to do transfers
    // from memory to peripheral
    __HAL_DMA_DISABLE(dma);

    TIM_CCxChannelCmd(tim->Instance, tim_channel, TIM_CCx_DISABLE);

    LL_TIM_OC_SetMode(tim->Instance, ll_tim_channel, LL_TIM_OCMODE_PWM2);
    LL_TIM_OC_ConfigOutput(tim->Instance, ll_tim_channel, LL_TIM_OCPOLARITY_HIGH);

    LL_TIM_OC_DisablePreload(tim->Instance, ll_tim_channel);
    __HAL_TIM_SET_COMPARE(tim, tim_channel, 0);
    LL_TIM_OC_EnablePreload(tim->Instance, ll_tim_channel);

    TIM_CCxChannelCmd(tim->Instance, tim_channel, TIM_CCx_ENABLE);

    // Enable source increment and disable destination increment for DMA
    MODIFY_REG(dma->Instance->CTR1, DMA_CTR1_SINC, LL_DMA_SRC_INCREMENT);
    MODIFY_REG(dma->Instance->CTR1, DMA_CTR1_DINC, LL_DMA_DEST_FIXED);

    // Switch DMA direction to memory to peripheral
    MODIFY_REG(dma->Instance->CTR2, DMA_CTR2_DREQ, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

    return true;
}

static bool motor_decode_telemetry(bdshot_dma_motor_t *const motor, uint32_t *const edge_times)
{
    DMA_HandleTypeDef *dma = motor->config.dma;
    TIM_HandleTypeDef *tim = motor->config.tim;
    uint32_t tim_channel = motor->config.tim_channel;

    uint32_t received_edges = BDSHOT_DMA_RX_FRAME_SIZE;

    // Wire bits is designed to be half the edge transitions as the GCR encoded
    // value. So for a fixed sized DMA transfer, it is not always going to complete,
    // so we have to abort it, but the data should still be good.
    if (motor->direction == BDSHOT_DMA_DIRECTION_INPUT) {
        tim_channel_dma_set_enable(tim, tim_channel, false);

        uint32_t unreceived_bytes = __HAL_DMA_GET_COUNTER(dma);

        // DMA peripheral provides the number of unreceived bytes, convert to
        // number of unreceived edge times (since edge times can be more than 1 byte)
        // and calculate the number of received edges
        received_edges = BDSHOT_DMA_RX_FRAME_SIZE - (unreceived_bytes / sizeof(uint32_t));

        // TODO: handle this HAL error properly
        (void)HAL_DMA_Abort(dma);

        (void)motor_switch_to_tx(motor);

        motor->direction = BDSHOT_DMA_DIRECTION_OUTPUT;
    }

    bdshot_motor_telemetry_t telemetry;
    bool success =
        bdshot_decode_telemetry(&telemetry, edge_times, received_edges, motor->config.pole_count);

    if (success) {
        motor->telemetry = telemetry;
        motor->is_telemetry_valid = true;
    }

    return success;
}

static void dma_transfer_complete_callback(DMA_HandleTypeDef *const dma)
{
    size_t motor_index = find_motor_index_from_dma(dma);

    if (motor_index == MOTOR_INDEX_INVALID) {
        return;
    }

    bdshot_dma_motor_t *motor = &motors[motor_index];
    TIM_HandleTypeDef *tim = motor->config.tim;
    uint32_t tim_channel = motor->config.tim_channel;

    // Disable DMA requests from the timer peripheral to prevent
    // accidental transfers while we program the DMA
    (void)tim_channel_dma_set_enable(tim, tim_channel, false);

    // DShot commands don't receive responses from ESC, so we don't need
    // to switch to RX
    if (motor->is_command_frame) {
        return;
    }

    if (motor->direction == BDSHOT_DMA_DIRECTION_OUTPUT) {
        (void)motor_switch_to_rx(motor);

        volatile uint32_t *ccrx = get_timer_channel_ccrx_reg(tim, tim_channel);

        if (ccrx == NULL) {
            return;
        }

        (void)HAL_DMA_Start_IT(dma, (uint32_t)ccrx, (uint32_t)bdshot_dma_rx_buffer[motor_index],
                               sizeof(bdshot_dma_rx_buffer[motor_index]));

        (void)tim_channel_dma_set_enable(tim, tim_channel, true);

        motor->direction = BDSHOT_DMA_DIRECTION_INPUT;
    } else {
        (void)motor_switch_to_tx(motor);

        motor->direction = BDSHOT_DMA_DIRECTION_OUTPUT;
    }
}

static bool bdshot_decode_telemetry(bdshot_motor_telemetry_t *const telemetry,
                                    const uint32_t *const edge_times, uint32_t edge_count,
                                    uint8_t pole_count)
{
    // Wire encoding ensures first bit is 0
    uint8_t bit = 0;

    uint32_t wire_frame = 0x00000000;
    uint32_t wire_bits_written = 0;

    // Error check for null pointers
    if (telemetry == NULL || edge_times == NULL) {
        return false;
    }

    if (edge_count == 0 || edge_count > BDSHOT_DMA_RX_FRAME_SIZE) {
        return false;
    }

    for (uint32_t i = 0; i < (edge_count - 1); i++) {
        // Written enough bits for a full frame
        if (wire_bits_written < BDSHOT_TELEMETRY_WIRE_BITS) {
            break;
        }

        uint32_t width = edge_times[i + 1] - edge_times[i];
        uint32_t bit_count = (uint32_t)roundf((float)width / BDSHOT_DMA_TELEMETRY_BIT_TICKS);

        if (bit_count <= 0 || bit_count > BDSHOT_TELEMETRY_WIRE_BITS ||
            (wire_bits_written + bit_count) > BDSHOT_TELEMETRY_WIRE_BITS) {
            return false;
        }

        for (uint32_t j = 0; j < bit_count; j++) {
            wire_frame = (wire_frame << 1) | bit;
            wire_bits_written++;
        }

        // Every edge we encounter means the bit value we received is flipped
        bit ^= 1;
    }

    // Bits in the rest of the frame is the same as the logic level following the last edge
    for (uint32_t i = wire_bits_written; i < BDSHOT_TELEMETRY_WIRE_BITS; i++) {
        wire_frame = (wire_frame << 1) | bit;
    }

    uint16_t frame;
    bool decode_successful = bdshot_frame_decode_from_wire(&frame, wire_frame);

    if (!decode_successful) {
        return false;
    }

    uint8_t received_crc = (frame & BDSHOT_CHECKSUM_MASK) >> BDSHOT_CHECKSUM_SHIFT;
    uint8_t expected_crc = bdshot_frame_checksum(frame, true);

    if (received_crc != expected_crc) {
        return false;
    }

    // TODO: add support for EDT
    if (bdshot_frame_is_edt(frame)) {
        return false;
    }

    telemetry->rpm = bdshot_frame_calculate_rpm(frame, pole_count);

    return true;
}

bool bdshot_dma_init()
{
    dma_timers_count = 0;

    for (size_t i = 0; i < BDSHOT_DMA_MOTOR_COUNT; i++) {
        (void)memset(bdshot_dma_tx_buffer[i], 0, sizeof(bdshot_dma_tx_buffer[i]));
        (void)memset(bdshot_dma_rx_buffer[i], 0, sizeof(bdshot_dma_rx_buffer[i]));

        dma_timers[i].tim = NULL;
        dma_timers[i].active_dma_sources = 0;

        motors[i].is_initialized = false;
        motors[i].is_armed = false;
        motors[i].is_first_arm = true;
        motors[i].direction = BDSHOT_DMA_DIRECTION_OUTPUT;
    }

    return true;
}

bool bdshot_dma_motor_init(bdshot_motor_index_t index, bdshot_dma_motor_config_t *config)
{
    if (index < 0 || index >= BDSHOT_DMA_MOTOR_COUNT) {
        return false;
    }

    bdshot_dma_motor_t *motor = &motors[index];

    // Don't override an existing motor entry, since we don't have support
    // for deinitialization
    if (motor->is_initialized) {
        return false;
    }

    if (!motor_config_is_valid(config)) {
        return false;
    }

    bool found_timer = false;

    // Keep track of active timers and timer channels associated with initialized motors
    for (size_t i = 0; i < BDSHOT_DMA_MOTOR_COUNT; i++) {
        bdshot_dma_timers_t *dma_timer = &dma_timers[i];
        TIM_HandleTypeDef *tim = dma_timer->tim;

        if (tim == NULL) {
            dma_timer->tim = config->tim;
            dma_timers_count++;

            tim = dma_timer->tim;
        }

        if (tim != config->tim) {
            continue;
        }

        dma_timer->active_dma_sources |=
            get_timer_channel_dma_src(config->tim, config->tim_channel);

        motor->dma_timer_index = i;

        found_timer = true;
    }

    // Should not happen at all, could indicate a mismatched size between the global
    // dma timers array and the global motors array
    if (!found_timer) {
        return false;
    }

    HAL_StatusTypeDef status = HAL_DMA_RegisterCallback(config->dma, HAL_DMA_XFER_CPLT_CB_ID,
                                                        dma_transfer_complete_callback);

    if (status != HAL_OK) {
        return false;
    }

    motor->config = *config;
    motor->is_initialized = true;

    return true;
}

bool bdshot_dma_motor_set_throttle(bdshot_motor_index_t index, uint16_t throttle)
{
    if (index < 0 || index >= BDSHOT_DMA_MOTOR_COUNT) {
        return false;
    }

    bdshot_dma_motor_t *motor = &motors[index];

    if (!motor->is_initialized || !motor->is_armed) {
        return false;
    }

    bool success = bdshot_throttle_frame_pack(&motor->send_request.frame, throttle, true);

    if (!success) {
        return false;
    }

    motor->send_request.is_dirty = true;
    motor->send_request.is_command = false;

    return true;
}

bool bdshot_dma_motor_get_telemetry(bdshot_motor_index_t index, bdshot_motor_telemetry_t *telemetry)
{
    if (index < 0 || index >= BDSHOT_DMA_MOTOR_COUNT) {
        return false;
    }

    bdshot_dma_motor_t *motor = &motors[index];

    if (!motor->is_initialized || !motor->is_telemetry_valid) {
        return false;
    }

    if (telemetry != NULL) {
        *telemetry = motor->telemetry;
    }

    return true;
}

bool bdshot_dma_motor_set_armed(bdshot_motor_index_t index, bool is_armed)
{
    if (index < 0 || index >= BDSHOT_DMA_MOTOR_COUNT) {
        return false;
    }

    bdshot_dma_motor_t *motor = &motors[index];

    if (!motor->is_initialized) {
        return false;
    }

    if (motor->is_armed == is_armed) {
        return true;
    }

    if (is_armed && motor->is_first_arm) {
        HAL_StatusTypeDef status = HAL_TIM_PWM_Start(motor->config.tim, motor->config.tim_channel);

        if (status != HAL_OK) {
            return false;
        }

        motor->is_first_arm = false;
    }

    // ESC needs to receive STOP command frames for a period to arm.
    // For disarm, we just currently do software disarming (i.e. sending a STOP frame and preventing actual
    // throttle values from being set).
    // TODO: implement hardware arm/disarm (i.e. enabling and disabling PWM in its entirety), process is
    // something along the lines of aborting the current DMA transfer, removing the current channel from the
    // active dma timer sources, etc.
    bool success =
        bdshot_command_frame_pack(&motor->send_request.frame, BDSHOT_COMMAND_MOTOR_STOP, false);

    if (!success) {
        return false;
    }

    motor->send_request.is_dirty = true;
    motor->send_request.is_command = true;

    motor->is_telemetry_valid = false;
    motor->is_armed = is_armed;

    return true;
}

bool bdshot_dma_set_armed(bool is_armed)
{
    for (size_t i = 0; i < BDSHOT_DMA_MOTOR_COUNT; i++) {
        bool success = bdshot_dma_motor_set_armed(i, is_armed);

        if (!success) {
            return false;
        }
    }

    return true;
}

bool bdshot_dma_apply()
{
    // Decode telemetry
    for (size_t i = 0; i < BDSHOT_DMA_MOTOR_COUNT; i++) {
        bdshot_dma_motor_t *motor = &motors[i];

        if (!motor->is_initialized || !motor->is_armed) {
            continue;
        }

        // DShot commands don't receive telemetry
        if (motor->is_command_frame) {
            continue;
        }

        (void)motor_decode_telemetry(motor, bdshot_dma_rx_buffer[i]);
    }

    // Starts the DMA controller to begin transfers
    for (size_t i = 0; i < BDSHOT_DMA_MOTOR_COUNT; i++) {
        bdshot_dma_motor_t *motor = &motors[i];

        if (!motor->is_initialized || !motor->is_armed) {
            continue;
        }

        if (motor->direction != BDSHOT_DMA_DIRECTION_OUTPUT) {
            continue;
        }

        if (motor->send_request.is_dirty) {
            motor->send_request.is_dirty = false;

            build_dma_tx_buffer(bdshot_dma_tx_buffer[i], motor->send_request.frame);

            // Keep track of whether the current frame is a command
            // so we know whether we need to receive telemetry data
            motor->is_command_frame = motor->send_request.is_command;
        }

        TIM_HandleTypeDef *tim = motor->config.tim;
        uint32_t tim_channel = motor->config.tim_channel;

        // Write to the compare register for this timer channel to generate DShot PWM signal
        volatile uint32_t *ccrx = get_timer_channel_ccrx_reg(tim, tim_channel);

        if (ccrx == NULL) {
            return false;
        }

        HAL_DMA_Start_IT(motor->config.dma, (uint32_t)bdshot_dma_tx_buffer[i], (uint32_t)ccrx,
                         sizeof(bdshot_dma_tx_buffer[i]));
    }

    // Enable DMA transfers from the timers.
    // Do this separately from the DMA controller configuration
    // to ensure that motors on the same TIM instance is synchronized.
    for (size_t i = 0; i < dma_timers_count; i++) {
        bdshot_dma_timers_t *timer = &dma_timers[i];

        if (timer->tim == NULL || timer->active_dma_sources == 0) {
            continue;
        }

        // Restore the free running timer ARR used for telemetry RX to a value for TX.
        // Must disable ARR preload here since the free running timer use for RX will
        // never have a UEV.
        LL_TIM_DisableARRPreload(timer->tim->Instance);
        LL_TIM_SetAutoReload(timer->tim->Instance, BDSHOT_DMA_BIT_TICKS - 1);
        LL_TIM_EnableARRPreload(timer->tim->Instance);

        LL_TIM_SetCounter(timer->tim->Instance, 0);

        // Enable DMA requests from the timer peripheral
        __HAL_TIM_ENABLE_DMA(timer->tim, timer->active_dma_sources);
    }

    return true;
}
