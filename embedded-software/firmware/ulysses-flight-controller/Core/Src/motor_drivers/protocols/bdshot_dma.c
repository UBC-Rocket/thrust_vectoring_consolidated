#include "motor_drivers/protocols/bdshot_dma.h"

#include "motor_drivers/protocols/bdshot.h"
#include "stm32h563xx.h"
#include "stm32h5xx_hal_def.h"
#include "stm32h5xx_hal_dma.h"
#include "stm32h5xx_hal_tim.h"
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

static const TIM_IC_InitTypeDef TIM_IC_CONFIG = {
    .ICPrescaler = TIM_ICPSC_DIV1,
    .ICSelection = TIM_ICSELECTION_DIRECTTI,
    .ICPolarity = TIM_ICPOLARITY_BOTHEDGE,
    .ICFilter = 2,
};

static const TIM_OC_InitTypeDef TIM_PWM_OC_CONFIG = {
    .OCMode = TIM_OCMODE_PWM2,
    .Pulse = 0,
    .OCPolarity = TIM_OCPOLARITY_HIGH,
    .OCFastMode = TIM_OCFAST_DISABLE,
};

static const uint8_t GCR_DECODE_TABLE[32] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, /* 0x00-0x07 */
    0xFF, 0x09, 0x0A, 0x0B, 0xFF, 0x0D, 0x0E, 0x0F, /* 0x08-0x0F */
    0xFF, 0xFF, 0x02, 0x03, 0xFF, 0x05, 0x06, 0x07, /* 0x10-0x17 */
    0xFF, 0x00, 0x08, 0x01, 0xFF, 0x04, 0x0C, 0xFF, /* 0x18-0x1F */
};

static bdshot_dma_timers_t dma_timers[BDSHOT_MOTOR_COUNT];
static size_t dma_timers_count = 0;

static bdshot_dma_motor_t motors[BDSHOT_MOTOR_COUNT];

static uint32_t bdshot_dma_tx_buffer[BDSHOT_MOTOR_COUNT][BDSHOT_DMA_TX_FRAME_SIZE];
static uint32_t bdshot_dma_rx_buffer[BDSHOT_MOTOR_COUNT][BDSHOT_DMA_RX_FRAME_SIZE];

static volatile uint32_t *get_timer_channel_ccrx_reg(TIM_HandleTypeDef *tim, uint32_t channel);
static volatile uint32_t get_timer_channel_dma_src(TIM_HandleTypeDef *tim, uint32_t channel);
static size_t find_motor_index_from_dma(DMA_HandleTypeDef *dma);
static bool tim_channel_dma_set_enable(TIM_HandleTypeDef *tim, uint32_t tim_channel, bool enable);
static void tim_arr_preload_set_enable(TIM_HandleTypeDef *tim, bool enable);
static void tim_channel_input_capture_init(TIM_TypeDef *TIMx, uint32_t channel,
                                           uint32_t TIM_ICPolarity, uint32_t TIM_ICSelection,
                                           uint32_t TIM_ICFilter);
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

static bool tim_channel_dma_set_enable(TIM_HandleTypeDef *tim, uint32_t tim_channel, bool enable)
{
    uint32_t dma_src = get_timer_channel_dma_src(tim, tim_channel);

    if (dma_src == 0) {
        return false;
    }

    if (enable) {
        __HAL_TIM_ENABLE_DMA(tim, dma_src);
    } else {
        __HAL_TIM_DISABLE_DMA(tim, dma_src);
    }

    return true;
}

static void tim_arr_preload_set_enable(TIM_HandleTypeDef *tim, bool enable)
{
    if (enable) {
        tim->Instance->CR1 |= TIM_CR1_ARPE;
    } else {
        tim->Instance->CR1 &= ~TIM_CR1_ARPE;
    }
}

static void tim_channel_input_capture_init(TIM_TypeDef *TIMx, uint32_t channel,
                                           uint32_t TIM_ICPolarity, uint32_t TIM_ICSelection,
                                           uint32_t TIM_ICFilter)
{
    uint32_t tmpccmrx;
    uint32_t tmpccer;

    /* Disable the Channel: Reset the CCxE Bit */
    tmpccer = TIMx->CCER;

    switch (channel) {
    case TIM_CHANNEL_1:
        TIMx->CCER &= ~TIM_CCER_CC1E;
        break;
    case TIM_CHANNEL_2:
        TIMx->CCER &= ~TIM_CCER_CC2E;
        break;
    case TIM_CHANNEL_3:
        TIMx->CCER &= ~TIM_CCER_CC3E;
        break;
    case TIM_CHANNEL_4:
        TIMx->CCER &= ~TIM_CCER_CC4E;
        break;
    }

    switch (channel) {
    case TIM_CHANNEL_1:
    case TIM_CHANNEL_2:
        tmpccmrx = TIMx->CCMR1;
        break;
    case TIM_CHANNEL_3:
    case TIM_CHANNEL_4:
        tmpccmrx = TIMx->CCMR2;
        break;
    }

    /* Select the Input */
    switch (channel) {
    case TIM_CHANNEL_1:
        tmpccmrx &= ~TIM_CCMR1_CC1S;
        break;
    case TIM_CHANNEL_2:
        tmpccmrx &= ~TIM_CCMR1_CC2S;
        break;
    case TIM_CHANNEL_3:
        tmpccmrx &= ~TIM_CCMR2_CC3S;
        break;
    case TIM_CHANNEL_4:
        tmpccmrx &= ~TIM_CCMR2_CC4S;
        break;
    }

    switch (channel) {
    case TIM_CHANNEL_1:
    case TIM_CHANNEL_3:
        tmpccmrx = TIMx->CCMR1;
        tmpccmrx |= TIM_ICSelection;
        break;
    case TIM_CHANNEL_2:
    case TIM_CHANNEL_4:
        tmpccmrx = TIMx->CCMR2;
        tmpccmrx |= (TIM_ICSelection << 8U);
        break;
    }

    /* Set the filter */
    switch (channel) {
    case TIM_CHANNEL_1:
        tmpccmrx &= ~TIM_CCMR1_IC1F;
        tmpccmrx |= ((TIM_ICFilter << 4U) & TIM_CCMR1_IC1F);
        break;
    case TIM_CHANNEL_2:
        tmpccmrx &= ~TIM_CCMR1_IC2F;
        tmpccmrx |= ((TIM_ICFilter << 12U) & TIM_CCMR1_IC2F);
        break;
    case TIM_CHANNEL_3:
        tmpccmrx &= ~TIM_CCMR2_IC3F;
        tmpccmrx |= ((TIM_ICFilter << 4U) & TIM_CCMR2_IC3F);
        break;
    case TIM_CHANNEL_4:
        tmpccmrx &= ~TIM_CCMR2_IC4F;
        tmpccmrx |= ((TIM_ICFilter << 12U) & TIM_CCMR2_IC4F);
        break;
    }

    /* Select the Polarity and set the CCxE Bit */
    switch (channel) {
    case TIM_CHANNEL_1:
        tmpccer &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP);
        tmpccer |= (TIM_ICPolarity & (TIM_CCER_CC1P | TIM_CCER_CC1NP));
        break;
    case TIM_CHANNEL_2:
        tmpccer &= ~(TIM_CCER_CC2P | TIM_CCER_CC2NP);
        tmpccer |= ((TIM_ICPolarity << 4U) & (TIM_CCER_CC2P | TIM_CCER_CC2NP));
        break;
    case TIM_CHANNEL_3:
        tmpccer &= ~(TIM_CCER_CC3P | TIM_CCER_CC3NP);
        tmpccer |= ((TIM_ICPolarity << 8U) & (TIM_CCER_CC3P | TIM_CCER_CC3NP));
        break;
    case TIM_CHANNEL_4:
        tmpccer &= ~(TIM_CCER_CC4P | TIM_CCER_CC4NP);
        tmpccer |= ((TIM_ICPolarity << 12U) & (TIM_CCER_CC4P | TIM_CCER_CC4NP));
        break;
    }

    /* Write to TIMx CCMRx and CCER registers */
    switch (channel) {
    case TIM_CHANNEL_1:
    case TIM_CHANNEL_2:
        TIMx->CCMR1 = tmpccmrx;
        break;
    case TIM_CHANNEL_3:
    case TIM_CHANNEL_4:
        TIMx->CCMR2 = tmpccmrx;
        break;
    }

    TIMx->CCER = tmpccer;
}

static void bdshot_switch_to_rx(bdshot_dma_motor_t *motor)
{
    DMA_HandleTypeDef *dma = motor->config.dma;
    TIM_HandleTypeDef *tim = motor->config.tim;
    uint32_t tim_channel = motor->config.tim_channel;

    // Disable DMA since since we are re-configuring channel to do transfers
    // from peripheral to memory
    __HAL_DMA_DISABLE(dma);

    // Set free running timer for input capture.
    // We want to have ARR preload enabled here since we want to ensure
    // all other motors in the same TIM has finished transferring the last byte.
    tim_arr_preload_set_enable(tim, true);
    __HAL_TIM_SET_AUTORELOAD(tim, 0xFFFFFFFF);

    TIM_CCxChannelCmd(tim->Instance, tim_channel, TIM_CCx_DISABLE);

    HAL_TIM_IC_ConfigChannel(tim, &TIM_IC_CONFIG, tim_channel);

    TIM_CCxChannelCmd(tim->Instance, tim_channel, TIM_CCx_ENABLE);

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

    // Disable DMA since since we are re-configuring channel to do transfers
    // from memory to peripheral
    __HAL_DMA_DISABLE(dma);

    TIM_CCxChannelCmd(tim->Instance, tim_channel, TIM_CCx_DISABLE);

    HAL_TIM_PWM_ConfigChannel(tim, &TIM_PWM_OC_CONFIG, tim_channel);

    TIM_CCxChannelCmd(tim->Instance, tim_channel, TIM_CCx_ENABLE);

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
    // Start DMA transfer
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

    for (size_t i = 0; i < dma_timers_count; i++) {
        bdshot_dma_timers_t *timer = &dma_timers[i];

        // Restore the free running timer ARR used for telemetry RX to a value for TX.
        // Must disable ARR preload here since the free running timer use for RX will
        // never have a UEV.
        tim_arr_preload_set_enable(timer->tim, false);
        __HAL_TIM_SET_AUTORELOAD(timer->tim, (BDSHOT_DMA_BIT_TICKS - 1));
        tim_arr_preload_set_enable(timer->tim, true);

        __HAL_TIM_SET_COUNTER(timer->tim, 0);

        // Enable DMA requests from the timer peripheral
        __HAL_TIM_ENABLE_DMA(timer->tim, timer->active_dma_sources);
    }

    return true;
}
