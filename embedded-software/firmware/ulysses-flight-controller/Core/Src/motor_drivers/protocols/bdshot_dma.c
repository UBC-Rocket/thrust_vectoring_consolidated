#include "motor_drivers/protocols/bdshot_dma.h"

#include "motor_drivers/protocols/bdshot.h"
#include "stm32h5xx.h"
#include "stm32h563xx.h"
#include "stm32h5xx_hal_def.h"
#include "stm32h5xx_hal_dma.h"
#include "stm32h5xx_hal_gpio.h"
#include "stm32h5xx_hal_tim.h"
#include "stm32h5xx_ll_gpio.h"
#include "stm32h5xx_ll_tim.h"
#include <stdint.h>
#include <string.h>

typedef enum bdshot_dma_direction {
    BDSHOT_DMA_DIRECTION_OUTPUT,
    BDSHOT_DMA_DIRECTION_INPUT,
} bdshot_dma_direction_t;

typedef struct bdshot_dma_motor {
    bool is_initialized;
    bool is_armed;
    bdshot_dma_direction_t direction;

    bdshot_dma_motor_config_t config;

    uint16_t throttle;

    bool is_telemetry_valid;
    bdshot_motor_telemetry_t telemetry;
} bdshot_dma_motor_t;

typedef struct bdshot_dma_timers {
    TIM_HandleTypeDef *tim;
    uint32_t active_dma_sources;
} bdshot_dma_timers_t;

static const uint8_t GCR_DECODE_TABLE[32] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, /* 0x00-0x07 */
    0xFF, 0x09, 0x0A, 0x0B, 0xFF, 0x0D, 0x0E, 0x0F, /* 0x08-0x0F */
    0xFF, 0xFF, 0x02, 0x03, 0xFF, 0x05, 0x06, 0x07, /* 0x10-0x17 */
    0xFF, 0x00, 0x08, 0x01, 0xFF, 0x04, 0x0C, 0xFF, /* 0x18-0x1F */
};

// Timers associated with all initialized motors
static bdshot_dma_timers_t dma_timers[BDSHOT_MOTOR_COUNT];
static size_t dma_timers_count = 0;

// Configuration of motors
static bdshot_dma_motor_t motors[BDSHOT_MOTOR_COUNT];

static uint32_t bdshot_dma_tx_buffer[BDSHOT_MOTOR_COUNT][BDSHOT_DMA_TX_FRAME_SIZE];
static uint32_t bdshot_dma_rx_buffer[BDSHOT_MOTOR_COUNT][BDSHOT_DMA_RX_FRAME_SIZE];

static volatile uint32_t *get_timer_channel_ccrx_reg(TIM_HandleTypeDef *tim, uint32_t channel);
static volatile uint32_t get_timer_channel_dma_src(TIM_HandleTypeDef *tim, uint32_t channel);
static uint32_t tim_channel_convert_hal_to_ll(uint32_t hal_channel);
static uint32_t gpio_pin_convert_hal_to_ll(uint32_t hal_pin);
static size_t find_motor_index_from_dma(DMA_HandleTypeDef *dma);
static bool tim_channel_dma_set_enable(TIM_HandleTypeDef *tim, uint32_t tim_channel, bool enable);
static void bdshot_switch_to_rx(bdshot_dma_motor_t *motor);
static void bdshot_switch_to_tx(bdshot_dma_motor_t *motor);
static void dma_xfer_complete_callback(DMA_HandleTypeDef *const dma);
static void build_dma_tx_buffer(bdshot_frame_t frame, uint32_t *buffer);
static bool bdshot_decode_gcr(const uint32_t *rx_buf, bdshot_motor_telemetry_t *telem,
                              uint8_t pole_count);

static volatile uint32_t *get_timer_channel_ccrx_reg(TIM_HandleTypeDef *tim, uint32_t channel)
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

static volatile uint32_t get_timer_channel_dma_src(TIM_HandleTypeDef *tim, uint32_t channel)
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
        return 0;
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
        return SIZE_MAX;
    }
}

static uint32_t gpio_pin_convert_hal_to_ll(uint32_t hal_pin)
{
    switch (hal_pin) {
    case GPIO_PIN_0:
        return LL_GPIO_PIN_0;
    case GPIO_PIN_1:
        return LL_GPIO_PIN_1;
    case GPIO_PIN_2:
        return LL_GPIO_PIN_2;
    case GPIO_PIN_3:
        return LL_GPIO_PIN_3;
    case GPIO_PIN_4:
        return LL_GPIO_PIN_4;
    case GPIO_PIN_5:
        return LL_GPIO_PIN_5;
    case GPIO_PIN_6:
        return LL_GPIO_PIN_6;
    case GPIO_PIN_7:
        return LL_GPIO_PIN_7;
    case GPIO_PIN_8:
        return LL_GPIO_PIN_8;
    case GPIO_PIN_9:
        return LL_GPIO_PIN_9;
    case GPIO_PIN_10:
        return LL_GPIO_PIN_10;
    case GPIO_PIN_11:
        return LL_GPIO_PIN_11;
    case GPIO_PIN_12:
        return LL_GPIO_PIN_12;
    case GPIO_PIN_13:
        return LL_GPIO_PIN_13;
    case GPIO_PIN_14:
        return LL_GPIO_PIN_14;
    case GPIO_PIN_15:
        return LL_GPIO_PIN_15;
    default:
        return SIZE_MAX;
    }
}

static size_t find_motor_index_from_dma(DMA_HandleTypeDef *dma)
{
    for (size_t index = 0; index < BDSHOT_MOTOR_COUNT; index++) {
        bdshot_dma_motor_t *motor = &motors[index];

        if (!motor->is_initialized) {
            continue;
        }

        if (motor->config.dma->Instance == dma->Instance) {
            return index;
        }
    }

    return SIZE_MAX;
}

static bool tim_channel_dma_set_enable(TIM_HandleTypeDef *tim, uint32_t channel, bool enable)
{
    switch (channel) {
    case TIM_CHANNEL_1:
        LL_TIM_DisableDMAReq_CC1(tim->Instance);
        break;
    case TIM_CHANNEL_2:
        LL_TIM_DisableDMAReq_CC2(tim->Instance);
        break;
    case TIM_CHANNEL_3:
        LL_TIM_DisableDMAReq_CC3(tim->Instance);
        break;
    case TIM_CHANNEL_4:
        LL_TIM_DisableDMAReq_CC4(tim->Instance);
        break;
    default:
        return false;
    }

    return true;
}

static void bdshot_switch_to_rx(bdshot_dma_motor_t *motor)
{
    DMA_HandleTypeDef *dma = motor->config.dma;
    TIM_HandleTypeDef *tim = motor->config.tim;
    uint32_t tim_channel = motor->config.tim_channel;
    uint32_t ll_tim_channel = tim_channel_convert_hal_to_ll(tim_channel);
    GPIO_TypeDef *gpio = motor->config.gpio;
    uint32_t gpio_pin = motor->config.gpio_pin;
    uint32_t ll_gpio_pin = gpio_pin_convert_hal_to_ll(gpio_pin);

    // Disable DMA since since we are re-configuring channel to do transfers
    // from peripheral to memory
    __HAL_DMA_DISABLE(dma);

    // Set free running timer for input capture.
    // We want to have ARR preload enabled here since we want to ensure
    // all other motors in the same TIM has finished transferring the last byte.
    LL_TIM_EnableARRPreload(tim->Instance);
    LL_TIM_SetAutoReload(tim->Instance, 0xFFFFFFFF);

    uint32_t alternate_function;

    if (ll_tim_channel <= LL_GPIO_PIN_7) {
        alternate_function = LL_GPIO_GetAFPin_0_7(gpio, ll_gpio_pin);
    } else {
        alternate_function = LL_GPIO_GetAFPin_8_15(gpio, ll_gpio_pin);
    }

    LL_GPIO_SetPinMode(gpio, ll_gpio_pin, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinSpeed(gpio, ll_gpio_pin, LL_GPIO_SPEED_FREQ_LOW);
    LL_GPIO_SetPinPull(gpio, ll_gpio_pin, LL_GPIO_PULL_NO);
    LL_GPIO_SetPinOutputType(gpio, ll_gpio_pin, LL_GPIO_OUTPUT_PUSHPULL);

    TIM_CCxChannelCmd(tim->Instance, tim_channel, TIM_CCx_DISABLE);

    LL_TIM_IC_Config(tim->Instance, ll_tim_channel,
                     LL_TIM_ACTIVEINPUT_DIRECTTI | LL_TIM_ICPSC_DIV1 | LL_TIM_IC_FILTER_FDIV1_N2 |
                         LL_TIM_IC_POLARITY_BOTHEDGE);

    TIM_CCxChannelCmd(tim->Instance, tim_channel, TIM_CCx_ENABLE);

    LL_GPIO_SetPinMode(gpio, ll_gpio_pin, LL_GPIO_MODE_ALTERNATE);

    if (ll_tim_channel <= LL_GPIO_PIN_7) {
        LL_GPIO_SetAFPin_0_7(gpio, ll_gpio_pin, alternate_function);
    } else {
        LL_GPIO_SetAFPin_8_15(gpio, ll_gpio_pin, alternate_function);
    }

    // TODO: replace direct register access with LL
    dma->Instance->CTR1 &= ~(DMA_CTR1_SINC | DMA_CTR1_DINC);
    dma->Instance->CTR1 |= DMA_DINC_INCREMENTED;
    dma->Instance->CTR2 &= ~DMA_CTR2_DREQ;

    __HAL_DMA_ENABLE(dma);
}

static void bdshot_switch_to_tx(bdshot_dma_motor_t *motor)
{
    DMA_HandleTypeDef *dma = motor->config.dma;
    TIM_HandleTypeDef *tim = motor->config.tim;
    uint32_t tim_channel = motor->config.tim_channel;
    uint32_t ll_tim_channel = tim_channel_convert_hal_to_ll(tim_channel);

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

    // TODO: replace direct register access with LL
    dma->Instance->CTR1 &= ~(DMA_CTR1_SINC | DMA_CTR1_DINC);
    dma->Instance->CTR1 |= DMA_SINC_INCREMENTED;
    dma->Instance->CTR2 |= DMA_CTR2_DREQ;

    __HAL_DMA_ENABLE(dma);
}

static void dma_xfer_complete_callback(DMA_HandleTypeDef *const dma)
{
    size_t motor_index = find_motor_index_from_dma(dma);

    if (motor_index == SIZE_MAX) {
        return;
    }

    bdshot_dma_motor_t *motor = &motors[motor_index];
    TIM_HandleTypeDef *tim = motor->config.tim;
    uint32_t tim_channel = motor->config.tim_channel;

    if (motor->direction == BDSHOT_DMA_DIRECTION_OUTPUT) {
        // Disable DMA requests from the timer peripheral to prevent
        // accidental transfers from reading
        tim_channel_dma_set_enable(tim, tim_channel, false);

        bdshot_switch_to_rx(motor);

        volatile uint32_t *ccrx = get_timer_channel_ccrx_reg(tim, tim_channel);

        if (ccrx == NULL) {
            return;
        }

        HAL_DMA_Start_IT(dma, (uint32_t)ccrx, (uint32_t)bdshot_dma_rx_buffer[motor_index],
                         sizeof(bdshot_dma_tx_buffer[motor_index]));

        tim_channel_dma_set_enable(tim, tim_channel, true);

        motor->direction = BDSHOT_DMA_DIRECTION_INPUT;
    } else {
        tim_channel_dma_set_enable(tim, tim_channel, false);

        bdshot_motor_telemetry_t telemetry;
        bool success = bdshot_decode_gcr(bdshot_dma_rx_buffer[motor_index], &telemetry,
                                         motor->config.pole_count);

        if (success) {
            motor->is_telemetry_valid = true;
            motor->telemetry = telemetry;
        }

        __HAL_TIM_DISABLE_OCxPRELOAD(tim, tim_channel);
        bdshot_switch_to_tx(motor);

        motor->direction = BDSHOT_DMA_DIRECTION_OUTPUT;
    }
}

static void build_dma_tx_buffer(bdshot_frame_t frame, uint32_t *buffer)
{
    for (uint8_t i = 0; i < BDSHOT_FRAME_BITS; i++) {
        buffer[i] = (frame & 0x8000) != 0 ? BDSHOT_DMA_T1H_TICKS : BDSHOT_DMA_T0H_TICKS;
        frame <<= 1;
    }

    buffer[BDSHOT_FRAME_BITS] = 0;
    buffer[BDSHOT_FRAME_BITS + 1] = 0;
}

static bool bdshot_decode_gcr(const uint32_t *rx_buf, bdshot_motor_telemetry_t *telem,
                              uint8_t pole_count)
{
    /* ── Step 1: reconstruct 21-bit GCR bit stream from edge timestamps */
    uint32_t gcr_bits = 0;
    uint32_t bits_written = 0;
    uint8_t level = 1; /* line idles HIGH (inverted bDShot) */

    for (uint32_t i = 0; i + 1 < BDSHOT_TELEMETRY_WIRE_BITS && bits_written < 21; i++) {

        /* Pulse width in timer counts — handle 32-bit counter wraparound */
        uint32_t width = rx_buf[i + 1] - rx_buf[i];

        /* Quantise to number of bit periods.
         * Add half a period for rounding: (width + period/2) / period  */
        uint32_t n_bits = (width + (BDSHOT_DMA_BIT_TICKS / 2)) / BDSHOT_DMA_BIT_TICKS;

        if (n_bits < 1 || n_bits > 4) {
            /* Invalid pulse width — frame is corrupt */
            return false;
        }

        /* Fill n_bits worth of the GCR stream with the current level */
        for (uint32_t b = 0; b < n_bits && bits_written < 21; b++) {
            gcr_bits = (gcr_bits << 1) | level;
            bits_written++;
        }

        /* Toggle level at each edge */
        level ^= 1;
    }

    if (bits_written < 21) {
        return false;
    }

    /* ── Step 2: de-invert the GCR stream
     * ESC output is inverted relative to the bDShot command polarity  */
    gcr_bits ^= 0x1FFFFF; /* 21-bit invert */

    /* ── Step 3: decode 4 × 5-bit GCR symbols → 4 nibbles
     * Bit layout of 21-bit stream (MSB first):
     *   bit 20    = leading framing bit (always 0 after invert, ignored)
     *   bits 19:15 = GCR symbol 0 (most significant nibble)
     *   bits 14:10 = GCR symbol 1
     *   bits  9:5  = GCR symbol 2
     *   bits  4:0  = GCR symbol 3 (least significant nibble)        */
    uint16_t telem_word = 0;

    for (int sym = 0; sym < 4; sym++) {
        /* Extract 5-bit symbol from MSB side, skipping leading bit   */
        uint32_t shift = 15 - (sym * 5);
        uint8_t symbol = (gcr_bits >> shift) & 0x1F;
        uint8_t nibble = GCR_DECODE_TABLE[symbol];

        if (nibble == 0xFF) {
            /* Invalid GCR symbol */
            return false;
        }

        telem_word = (telem_word << 4) | nibble;
    }

    /* ── Step 4: verify CRC
     * CRC covers the upper 12 bits [15:4].
     * Same algorithm as the TX checksum but NOT inverted on the RX side
     * (the ESC inverts it on transmit, our de-invert in step 2 undoes that) */
    uint8_t received_crc = telem_word & BDSHOT_CHECKSUM_MASK;
    uint16_t payload = telem_word >> BDSHOT_CHECKSUM_BITS;
    uint8_t expected_crc = (~(payload ^ (payload >> 4) ^ (payload >> 8))) & BDSHOT_CHECKSUM_MASK;

    if (received_crc != expected_crc) {
        return false;
    }

    /* ── Step 5: decode eRPM from 12-bit payload
     * Format: [11:9] = 3-bit exponent, [8:0] = 9-bit mantissa
     * Period (µs) = mantissa << exponent
     * eRPM = 60,000,000 / period_us
     * mechanical RPM = eRPM / (pole_count / 2)                      */
    uint16_t exponent = (payload >> 9) & 0x7;
    uint16_t mantissa = payload & 0x1FF;

    if (mantissa == 0) {
        /* Motor stopped */
        telem->rpm = 0.0f;
        return true;
    }

    uint32_t period_us = (uint32_t)mantissa << exponent;
    float erpm = 60000000.0f / (float)period_us;
    telem->rpm = erpm / ((float)pole_count / 2.0f);

    return true;
}

bool bdshot_dma_init()
{
    for (size_t i = 0; i < BDSHOT_MOTOR_COUNT; i++) {
        motors[i].is_initialized = false;
        motors[i].is_armed = false;
        motors[i].direction = BDSHOT_DMA_DIRECTION_OUTPUT;

        motors[i].is_telemetry_valid = false;
    }

    return true;
}

bool bdshot_dma_motor_init(bdshot_motor_index_t motor, bdshot_dma_motor_config_t *config)
{
    if (motor < 0 || motor >= BDSHOT_MOTOR_COUNT) {
        return false;
    }

    // Don't override an existing motor entry, since we don't have support
    // for deinitialization
    if (motors[motor].is_initialized) {
        return false;
    }

    motors[motor].is_initialized = true;
    motors[motor].config = *config;

    HAL_DMA_RegisterCallback(config->dma, HAL_DMA_XFER_CPLT_CB_ID, dma_xfer_complete_callback);

    // Keep track of active timers and timer channels associated with initialized motors
    for (size_t i = 0; i < BDSHOT_MOTOR_COUNT; i++) {
        bdshot_dma_timers_t *dma_timer = &dma_timers[i];

        if (dma_timer->tim == config->tim) {
            dma_timer->active_dma_sources |=
                get_timer_channel_dma_src(config->tim, config->tim_channel);
            break;
        } else if (dma_timer->tim == NULL) {
            dma_timer->tim = config->tim;
            dma_timer->active_dma_sources |=
                get_timer_channel_dma_src(config->tim, config->tim_channel);
            dma_timers_count++;
            break;
        }
    }

    return true;
}

bool bdshot_dma_motor_set_throttle(bdshot_motor_index_t motor, uint16_t throttle)
{
    if (motor < 0 || motor >= BDSHOT_MOTOR_COUNT) {
        return false;
    }

    if (!motors[motor].is_initialized) {
        return false;
    }

    bdshot_frame_t frame;
    bool success = bdshot_frame_pack(&frame, throttle, true);

    if (!success) {
        return false;
    }

    motors[motor].throttle = throttle;
    build_dma_tx_buffer(frame, bdshot_dma_tx_buffer[motor]);

    return true;
}

bool bdshot_dma_motor_get_telemetry(bdshot_motor_index_t motor, bdshot_motor_telemetry_t *telemtry)
{
    if (motor < 0 || motor >= BDSHOT_MOTOR_COUNT) {
        return false;
    }

    if (telemtry != NULL) {
        *telemtry = motors[motor].telemetry;
    }

    return true;
}

bool bdshot_dma_set_armed(bool is_armed)
{
    for (size_t i = 0; i < BDSHOT_MOTOR_COUNT; i++) {
        bdshot_dma_motor_t *motor = &motors[i];

        if (!motor->is_initialized) {
            continue;
        }

        HAL_StatusTypeDef status = HAL_ERROR;

        if (is_armed) {
            status = HAL_TIM_PWM_Start(motor->config.tim, motor->config.tim_channel);
        } else {
            status = HAL_TIM_PWM_Stop(motor->config.tim, motor->config.tim_channel);
        }

        if (status == HAL_OK) {
            motor->is_armed = is_armed;
        } else {
            return false;
        }
    }

    return true;
}

bool bdshot_dma_apply()
{
    // Starts the DMA controller to begin transfers
    for (size_t i = 0; i < BDSHOT_MOTOR_COUNT; i++) {
        size_t motor_index = i;
        bdshot_dma_motor_t *motor = &motors[motor_index];

        if (!motor->is_initialized || !motor->is_armed) {
            continue;
        }

        if (motor->direction != BDSHOT_DMA_DIRECTION_OUTPUT) {
            continue;
        }

        TIM_HandleTypeDef *tim = motor->config.tim;
        uint32_t tim_channel = motor->config.tim_channel;

        // Write to the compare register for this timer channel to generate DShot PWM signal
        volatile uint32_t *ccrx = get_timer_channel_ccrx_reg(tim, tim_channel);

        if (ccrx == NULL) {
            return false;
        }

        HAL_DMA_Start_IT(motor->config.dma, (uint32_t)bdshot_dma_tx_buffer[motor_index],
                         (uint32_t)ccrx, sizeof(bdshot_dma_tx_buffer[motor_index]));
    }

    // Enable DMA transfers from the timers.
    // Do this separately from the DMA controller configuration
    // to ensure that motors on the same TIM instance is synchronized.
    for (size_t i = 0; i < dma_timers_count; i++) {
        bdshot_dma_timers_t *timer = &dma_timers[i];

        // Restore the free running timer ARR used for telemetry RX to a value for TX.
        // Must disable ARR preload here since the free running timer use for RX will
        // never have a UEV.
        LL_TIM_EnableARRPreload(timer->tim->Instance);
        LL_TIM_SetAutoReload(timer->tim->Instance, BDSHOT_DMA_BIT_TICKS - 1);
        LL_TIM_DisableARRPreload(timer->tim->Instance);

        LL_TIM_SetCounter(timer->tim->Instance, 0);

        // Enable DMA requests from the timer peripheral
        __HAL_TIM_ENABLE_DMA(timer->tim, timer->active_dma_sources);
    }

    return true;
}
