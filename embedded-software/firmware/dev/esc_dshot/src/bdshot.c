#include "esc_dshot/bdshot.h"

#include "bdshot_ral.h"
#include "esc_dshot/config.h"
#include "dshot/bdshot.h"
#include "dshot/protocol/dshot.h"
#include "dshot/protocol//bdshot.h"
#include "hal_to_ll_translator.h"
#include "hw.h"

#pragma region IO layer migration
#include "stm32h7xx.h"
#include "stm32h745xx.h"
#include "stm32h7xx_hal_def.h"
#include "stm32h7xx_hal_dma.h"
#include "stm32h7xx_hal_tim.h"
#include "stm32h7xx_ll_tim.h"
#include "stm32h7xx_ll_gpio.h"
#pragma endregion IO layer migration

#include "FreeRTOS.h"
#include "task.h"

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

// Idle bits are not necessary, but it holds the line in an idle
// state so the ESC doesn't misinterpret our frame if the switch from
// TX to RX causes noise.
#define BDSHOT_DMA_TX_IDLE_BITS  (2)
#define BDSHOT_DMA_TX_FRAME_SIZE (DSHOT_FRAME_BITS + BDSHOT_DMA_TX_IDLE_BITS)

// Number of rising and falling edges will be at most as the number of
// bits in the longest wire message. This occurs when the wire message
// is alternating bits.
#define BDSHOT_DMA_RX_FRAME_SIZE (BDSHOT_TELEMETRY_WIRE_BITS)

// Currently configured for DSHOT300.
//
// For some version DSHOT[XXX], the bit rate is XXX kb/s (e.g. DSHOT300 is 300 kb/s).
//
// Bit period in seconds is 1/(bit rate). From this, we can calculate the bit ticks
// by using (timer peripheral clock in hertz)*(bit period in seconds).
//
// The high time of a 1 bit is not well documented, but most FCs and ESCs use 75% of the bit
// period/ticks. By the protocol definition, the high time of a 0 bit is always half that of
// a 1 bit.
#define BDSHOT_DMA_T1H_TICKS (600)
#define BDSHOT_DMA_T0H_TICKS (300)
#define BDSHOT_DMA_BIT_TICKS (800)

// Wire telemetry message is always sent at (5/4)*(bit rate).
#define BDSHOT_DMA_TELEMETRY_BIT_TICKS ((BDSHOT_DMA_BIT_TICKS * 4.0f) / 5.0f)

typedef enum bdshot_dma_line_direction {
    BDSHOT_DMA_LINE_DIRECTION_OUTPUT,
    BDSHOT_DMA_LINE_DIRECTION_INPUT,
} bdshot_dma_line_direction;

typedef struct bdshot_set_frame_request {
    bool is_dirty;
    bool is_command;
    dshot_frame_t frame;
} bdshot_set_frame_request_t;

typedef struct bdshot_dma_timers {
    TIM_HandleTypeDef *tim;
    uint32_t active_dma_sources;
} bdshot_dma_timers_t;

typedef struct bdshot_esc_motor {
    bool is_initialized;
    bool is_armed;
    bool is_telemetry_valid;

    /// Is the currently sent frame a command frame?
    bool is_command_frame;

    uint32_t id;

    const io_bdshot_esc_t *io_handle;
    bdshot_dma_line_direction direction;
    bdshot_set_frame_request_t set_frame_request;
    esc_motor_telemetry_t telemetry;
    esc_motor_config_t config;
} bdshot_esc_motor_t;

/// Timers associated with all initialized motors. The number of timers to
/// keep track of is at most equal to the number of motors (i.e. the
/// configuration where there is one timer per motor).
static bdshot_dma_timers_t dma_timers[ESC_MOTOR_ID_COUNT];
static size_t dma_timers_count = 0;

/// Configuration of motors
static bdshot_esc_motor_t motors[ESC_MOTOR_ID_COUNT];

static uint32_t bdshot_dma_tx_buffer[ESC_MOTOR_ID_COUNT][BDSHOT_DMA_TX_FRAME_SIZE];
static uint32_t bdshot_dma_rx_buffer[ESC_MOTOR_ID_COUNT][BDSHOT_DMA_RX_FRAME_SIZE];

static bool motor_config_is_valid(const esc_motor_config_t *config);
static bool is_valid_motor_id(esc_motor_id_t id);
static bdshot_esc_motor_t *find_motor_by_dma_instance(const DMA_HandleTypeDef *dma);

static void build_dma_tx_buffer(uint32_t *buffer, dshot_frame_t frame);

#pragma region IO layer migration
static bool signal_line_set_to_rx(const io_bdshot_esc_t *io_handle);
static bool signal_line_set_to_tx(const io_bdshot_esc_t *io_handle);
static bool signal_line_set_armed(const io_bdshot_esc_t *io_handle, bool is_armed);
#pragma endregion IO layer migration

static bool motor_decode_telemetry(const io_bdshot_esc_t *io_handle, bdshot_esc_motor_t *motor,
                                   const uint32_t *edge_times);
static bool bdshot_decode_telemetry_from_signal(esc_motor_telemetry_t *telemetry,
                                                const uint32_t *edge_times, uint32_t edge_count,
                                                uint8_t pole_count);

static void dma_transfer_complete_callback(DMA_HandleTypeDef *dma);

static bool motor_config_is_valid(const esc_motor_config_t *config)
{
    return config->pole_count != 0;
}

static bool is_valid_motor_id(esc_motor_id_t id)
{
    return id >= 0 && id < ESC_MOTOR_ID_COUNT;
}

static bdshot_esc_motor_t *find_motor_by_dma_instance(const DMA_HandleTypeDef *dma)
{
    bdshot_esc_motor_t *motor = NULL;

    for (uint32_t id = 0; id < ESC_MOTOR_ID_COUNT; id++) {
        bdshot_esc_motor_t *current_motor = &motors[id];

        if (!current_motor->is_initialized) {
            continue;
        }

        if (current_motor->io_handle->dma->Instance == dma->Instance) {
            motor = current_motor;
            break;
        }
    }

    return motor;
}

static void build_dma_tx_buffer(uint32_t *buffer, dshot_frame_t frame)
{
    // Pack the DShot frame
    for (uint8_t i = 0; i < DSHOT_FRAME_BITS; i++) {
        buffer[i] = (frame & 0x8000) != 0 ? BDSHOT_DMA_T1H_TICKS : BDSHOT_DMA_T0H_TICKS;
        frame <<= 1;
    }

    // Pack the idle bits
    for (uint8_t i = 0; i < BDSHOT_DMA_TX_IDLE_BITS; i++) {
        buffer[DSHOT_FRAME_BITS + i] = 0;
    }
}

static bool signal_line_set_to_rx(const io_bdshot_esc_t *io_handle)
{
    DMA_HandleTypeDef *dma = io_handle->dma;
    TIM_HandleTypeDef *tim = io_handle->tim;

    const uint32_t hal_tim_channel = io_handle->tim_channel;
    const uint32_t ll_tim_channel = hal_to_ll_convert_tim_channel(hal_tim_channel);

    if (ll_tim_channel == HAL_TO_LL_CONVERSION_INVALID) {
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

    TIM_CCxChannelCmd(tim->Instance, hal_tim_channel, TIM_CCx_DISABLE);

    LL_TIM_IC_Config(tim->Instance, ll_tim_channel,
                     LL_TIM_ACTIVEINPUT_DIRECTTI | LL_TIM_ICPSC_DIV1 | LL_TIM_IC_FILTER_FDIV1_N2 |
                         LL_TIM_IC_POLARITY_BOTHEDGE);

    TIM_CCxChannelCmd(tim->Instance, hal_tim_channel, TIM_CCx_ENABLE);

    ral_dma_peripheral_increment_set_enabled(dma, false);
    ral_dma_memory_increment_set_enabled(dma, true);

    ral_dma_data_transfer_set_direction(dma, RAL_DMA_TRANSFER_PERIPH_TO_MEMORY);

    return true;
}

static bool signal_line_set_to_tx(const io_bdshot_esc_t *io_handle)
{
    DMA_HandleTypeDef *dma = io_handle->dma;
    TIM_HandleTypeDef *tim = io_handle->tim;

    const uint32_t hal_tim_channel = io_handle->tim_channel;
    const uint32_t ll_tim_channel = hal_to_ll_convert_tim_channel(hal_tim_channel);

    if (ll_tim_channel == HAL_TO_LL_CONVERSION_INVALID) {
        return false;
    }

    // Disable DMA since since we are re-configuring channel to do transfers
    // from memory to peripheral
    __HAL_DMA_DISABLE(dma);

    TIM_CCxChannelCmd(tim->Instance, hal_tim_channel, TIM_CCx_DISABLE);

    LL_TIM_OC_SetMode(tim->Instance, ll_tim_channel, LL_TIM_OCMODE_PWM2);
    LL_TIM_OC_ConfigOutput(tim->Instance, ll_tim_channel, LL_TIM_OCPOLARITY_HIGH);

    LL_TIM_OC_DisablePreload(tim->Instance, ll_tim_channel);
    __HAL_TIM_SET_COMPARE(tim, hal_tim_channel, 0);
    LL_TIM_OC_EnablePreload(tim->Instance, ll_tim_channel);

    TIM_CCxChannelCmd(tim->Instance, hal_tim_channel, TIM_CCx_ENABLE);

    ral_dma_peripheral_increment_set_enabled(dma, true);
    ral_dma_memory_increment_set_enabled(dma, false);

    ral_dma_data_transfer_set_direction(dma, RAL_DMA_TRANSFER_MEMORY_TO_PERIPH);

    return true;
}

static bool signal_line_set_armed(const io_bdshot_esc_t *io_handle, bool is_armed)
{
    TIM_HandleTypeDef *tim = io_handle->tim;
    const uint32_t hal_tim_channel = io_handle->tim_channel;

    GPIO_TypeDef *gpio = io_handle->gpio;
    const uint32_t ll_gpio_pin = hal_to_ll_convert_gpio_pin(io_handle->gpio_pin);

    HAL_StatusTypeDef status = HAL_ERROR;

    if (is_armed) {
        ral_gpio_set_af(gpio, ll_gpio_pin, io_handle->gpio_original_af);
        LL_GPIO_SetPinMode(gpio, ll_gpio_pin, LL_GPIO_MODE_ALTERNATE);

        status = HAL_TIM_PWM_Start(tim, hal_tim_channel);
    } else {
        status = HAL_TIM_PWM_Stop(tim, hal_tim_channel);

        LL_GPIO_SetPinMode(gpio, ll_gpio_pin, LL_GPIO_MODE_OUTPUT);
        LL_GPIO_ResetOutputPin(gpio, ll_gpio_pin);
    }

    return status == HAL_OK;
}

static bool motor_decode_telemetry(const io_bdshot_esc_t *io_handle, bdshot_esc_motor_t *motor,
                                   const uint32_t *edge_times)
{
    DMA_HandleTypeDef *dma = io_handle->dma;
    TIM_HandleTypeDef *tim = io_handle->tim;

    const uint32_t hal_tim_channel = io_handle->tim_channel;

    uint32_t received_edges = BDSHOT_DMA_RX_FRAME_SIZE;

    // Wire bits is designed to be half the edge transitions as the GCR encoded
    // value. So for a fixed sized DMA transfer, it is not always going to complete,
    // so we have to abort it, but the data should still be good.
    if (motor->direction == BDSHOT_DMA_LINE_DIRECTION_INPUT) {
        (void)ral_tim_channel_dma_set_enabled(tim, hal_tim_channel, false);

        uint32_t unreceived_bytes = __HAL_DMA_GET_COUNTER(dma);

        // DMA peripheral provides the number of unreceived bytes, convert to
        // number of unreceived edge times (since edge times can be more than 1 byte)
        // and calculate the number of received edges
        received_edges = BDSHOT_DMA_RX_FRAME_SIZE - (unreceived_bytes / sizeof(uint32_t));

        // TODO: handle this HAL error properly
        (void)HAL_DMA_Abort(dma);

        (void)signal_line_set_to_tx(io_handle);

        motor->direction = BDSHOT_DMA_LINE_DIRECTION_OUTPUT;
    }

    esc_motor_telemetry_t telemetry;
    bool success = bdshot_decode_telemetry_from_signal(&telemetry, edge_times, received_edges,
                                                       motor->config.pole_count);

    if (success) {
        uint32_t saved_interrupt_status = taskENTER_CRITICAL_FROM_ISR();

        motor->telemetry = telemetry;
        motor->is_telemetry_valid = true;

        taskEXIT_CRITICAL_FROM_ISR(saved_interrupt_status);
    }

    return success;
}

static bool bdshot_decode_telemetry_from_signal(esc_motor_telemetry_t *telemetry,
                                                const uint32_t *edge_times, uint32_t edge_count,
                                                uint8_t pole_count)
{
    if (telemetry == NULL || edge_times == NULL) {
        return false;
    }

    if (edge_count == 0 || edge_count > BDSHOT_DMA_RX_FRAME_SIZE) {
        return false;
    }

    // Wire encoding ensures first bit is 0
    uint8_t bit = 0;

    uint32_t wire_frame = 0x00000000;
    uint32_t wire_bits_written = 0;

    for (uint32_t i = 0; i < (edge_count - 1); i++) {
        // Written enough bits for a full frame
        if (wire_bits_written >= BDSHOT_TELEMETRY_WIRE_BITS) {
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

    uint8_t received_crc = (frame & DSHOT_CHECKSUM_MASK) >> DSHOT_CHECKSUM_SHIFT;
    uint8_t expected_crc = bdshot_frame_checksum(frame);

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

static void dma_transfer_complete_callback(DMA_HandleTypeDef *dma)
{
    bdshot_esc_motor_t *motor = find_motor_by_dma_instance(dma);

    if (motor == NULL) {
        return;
    }

    const io_bdshot_esc_t *io_handle = motor->io_handle;

    TIM_HandleTypeDef *tim = io_handle->tim;

    const uint32_t hal_tim_channel = io_handle->tim_channel;

    // Disable DMA requests from the timer peripheral to prevent
    // accidental transfers while we program the DMA
    (void)ral_tim_channel_dma_set_enabled(tim, hal_tim_channel, false);

    // DShot commands don't receive responses from ESC, so we don't need
    // to switch to RX
    if (motor->is_command_frame) {
        return;
    }

    if (motor->direction == BDSHOT_DMA_LINE_DIRECTION_OUTPUT) {
        (void)signal_line_set_to_rx(io_handle);

        volatile uint32_t *ccrx = ral_get_tim_channel_ccrx_reg(tim, hal_tim_channel);

        if (ccrx == NULL) {
            return;
        }

        (void)HAL_DMA_Start_IT(dma, (uint32_t)ccrx, (uint32_t)bdshot_dma_rx_buffer[motor->id],
                               sizeof(bdshot_dma_rx_buffer[motor->id]));

        (void)ral_tim_channel_dma_set_enabled(tim, hal_tim_channel, true);

        motor->direction = BDSHOT_DMA_LINE_DIRECTION_INPUT;
    } else {
        (void)signal_line_set_to_tx(io_handle);

        motor->direction = BDSHOT_DMA_LINE_DIRECTION_OUTPUT;
    }
}

void esc_dshot_init()
{
    dma_timers_count = 0;

    for (uint32_t id = 0; id < ESC_MOTOR_ID_COUNT; id++) {
        (void)memset(bdshot_dma_tx_buffer[id], 0, sizeof(bdshot_dma_tx_buffer[id]));
        (void)memset(bdshot_dma_rx_buffer[id], 0, sizeof(bdshot_dma_rx_buffer[id]));

        dma_timers[id].tim = NULL;
        dma_timers[id].active_dma_sources = 0;

        motors[id].is_initialized = false;
        motors[id].is_armed = false;
        motors[id].id = id;
        motors[id].direction = BDSHOT_DMA_LINE_DIRECTION_OUTPUT;
    }
}

bool esc_dshot_update()
{
    // Decode telemetry
    for (uint32_t id = 0; id < ESC_MOTOR_ID_COUNT; id++) {
        bdshot_esc_motor_t *motor = &motors[id];

        if (!motor->is_initialized || !motor->is_armed) {
            continue;
        }

        // DShot commands don't receive telemetry
        if (motor->is_command_frame) {
            continue;
        }

        (void)motor_decode_telemetry(motor->io_handle, motor, bdshot_dma_rx_buffer[id]);
    }

    // Starts the DMA controller to begin transfers
    for (uint32_t id = 0; id < ESC_MOTOR_ID_COUNT; id++) {
        bdshot_esc_motor_t *motor = &motors[id];

        if (!motor->is_initialized || !motor->is_armed) {
            continue;
        }

        if (motor->direction != BDSHOT_DMA_LINE_DIRECTION_OUTPUT) {
            continue;
        }

        {
            bool is_new_frame_requested = false;
            dshot_frame_t new_frame;

            uint32_t saved_interrupt_status = taskENTER_CRITICAL_FROM_ISR();

            if (motor->set_frame_request.is_dirty) {
                motor->set_frame_request.is_dirty = false;

                // Keep track of whether the current frame is a command
                // so we know whether we need to receive telemetry data
                motor->is_command_frame = motor->set_frame_request.is_command;

                new_frame = motor->set_frame_request.frame;
                is_new_frame_requested = true;
            }

            taskEXIT_CRITICAL_FROM_ISR(saved_interrupt_status);

            if (is_new_frame_requested) {
                build_dma_tx_buffer(bdshot_dma_tx_buffer[id], new_frame);
            }
        }

        const io_bdshot_esc_t *io_handle = motor->io_handle;

        TIM_HandleTypeDef *tim = io_handle->tim;
        const uint32_t hal_tim_channel = io_handle->tim_channel;

        // Write to the compare register for this timer channel to generate DShot PWM signal
        volatile uint32_t *ccrx = ral_get_tim_channel_ccrx_reg(tim, hal_tim_channel);

        if (ccrx == NULL) {
            return false;
        }

        HAL_DMA_Start_IT(io_handle->dma, (uint32_t)bdshot_dma_tx_buffer[id], (uint32_t)ccrx,
                         sizeof(bdshot_dma_tx_buffer[id]));
    }

    // Enable DMA transfers from the timers.
    // Do this separately from the DMA controller configuration
    // to ensure that motors on the same TIM instance is synchronized.
    for (uint32_t i = 0; i < dma_timers_count; i++) {
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

bool esc_dshot_set_armed(bool is_armed)
{
    for (uint32_t id = 0; id < ESC_MOTOR_ID_COUNT; id++) {
        bool success = esc_dshot_motor_set_armed(id, is_armed);

        if (!success) {
            return false;
        }
    }

    return true;
}

bool esc_dshot_motor_init(esc_motor_id_t motor_id, const esc_motor_config_t *config,
                          const io_bdshot_esc_t *io_handle)
{
    if (!is_valid_motor_id(motor_id)) {
        return false;
    }

    bdshot_esc_motor_t *motor = &motors[motor_id];

    // Don't override an existing motor entry, since we don't have support
    // for deinitialization
    if (motor->is_initialized) {
        return false;
    }

    if (!motor_config_is_valid(config)) {
        return false;
    }

    motor->config = *config;
    motor->io_handle = io_handle;

    bool is_disarm_successful = signal_line_set_armed(io_handle, false);

    if (!is_disarm_successful) {
        return false;
    }

    bool found_timer = false;

    // Keep track of active timers and timer channels associated with initialized motors
    for (uint32_t i = 0; i < ESC_MOTOR_ID_COUNT; i++) {
        bdshot_dma_timers_t *dma_timer = &dma_timers[i];
        TIM_HandleTypeDef *tim = dma_timer->tim;

        if (tim == NULL) {
            dma_timer->tim = io_handle->tim;
            dma_timers_count++;

            tim = dma_timer->tim;
        }

        if (tim != io_handle->tim) {
            continue;
        }

        dma_timer->active_dma_sources |=
            ral_get_tim_channel_dma_src(io_handle->tim, io_handle->tim_channel);

        found_timer = true;
    }

    // Should not happen at all, could indicate a mismatched size between the
    // dma timers array and the motors array
    if (!found_timer) {
        return false;
    }

    HAL_StatusTypeDef status = HAL_DMA_RegisterCallback(io_handle->dma, HAL_DMA_XFER_CPLT_CB_ID,
                                                        dma_transfer_complete_callback);

    if (status != HAL_OK) {
        return false;
    }

    motor->is_initialized = true;

    return true;
}

bool esc_dshot_motor_set_armed(esc_motor_id_t motor_id, bool is_armed)
{
    if (!is_valid_motor_id(motor_id)) {
        return false;
    }

    bdshot_esc_motor_t *motor = &motors[motor_id];

    if (!motor->is_initialized) {
        return false;
    }

    bool success = signal_line_set_armed(motor->io_handle, is_armed);

    if (success) {
        // ESC needs to receive STOP command frames for a period to arm.
        // For disarm, we just currently do software disarming (i.e. sending a STOP frame and preventing actual
        // throttle values from being set).
        // TODO: implement hardware arm/disarm (i.e. enabling and disabling PWM in its entirety), process is
        // something along the lines of aborting the current DMA transfer, removing the current channel from the
        // active dma timer sources, etc.
        dshot_frame_t frame;
        success = bdshot_command_frame_pack(&frame, DSHOT_COMMAND_MOTOR_STOP, false);

        if (success) {
            taskENTER_CRITICAL();

            motor->set_frame_request.frame = frame;
            motor->set_frame_request.is_dirty = true;
            motor->set_frame_request.is_command = true;

            motor->is_telemetry_valid = false;
            motor->is_armed = is_armed;

            taskEXIT_CRITICAL();
        }
    }

    return success;
}

bool esc_dshot_motor_set_throttle(esc_motor_id_t motor_id, uint16_t throttle)
{
    if (!is_valid_motor_id(motor_id)) {
        return false;
    }

    bdshot_esc_motor_t *motor = &motors[motor_id];

    if (!motor->is_initialized || !motor->is_armed) {
        return false;
    }

    dshot_frame_t frame;
    bool success = bdshot_throttle_frame_pack(&frame, throttle, true);

    if (success) {
        taskENTER_CRITICAL();

        motor->set_frame_request.frame = frame;
        motor->set_frame_request.is_dirty = true;
        motor->set_frame_request.is_command = false;

        taskEXIT_CRITICAL();
    }

    return success;
}

bool esc_dshot_motor_get_telemetry(esc_motor_id_t motor_id, esc_motor_telemetry_t *telemetry)
{
    if (!is_valid_motor_id(motor_id)) {
        return false;
    }

    bdshot_esc_motor_t *motor = &motors[motor_id];

    if (!motor->is_initialized) {
        return false;
    }

    bool success = false;

    taskENTER_CRITICAL();

    if (motor->is_telemetry_valid) {
        success = true;

        if (telemetry != NULL) {
            *telemetry = motor->telemetry;
        }
    }

    taskEXIT_CRITICAL();

    return success;
}
