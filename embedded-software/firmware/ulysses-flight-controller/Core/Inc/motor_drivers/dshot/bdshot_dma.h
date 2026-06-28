#ifndef ULYSSES_MOTOR_DRIVER_PROTOCOLS_BDSHOT_DMA_H
#define ULYSSES_MOTOR_DRIVER_PROTOCOLS_BDSHOT_DMA_H

#include <assert.h>
#include <stdint.h>

#include "stm32h5xx.h"
#include "stm32h563xx.h"
#include "stm32h5xx_hal_dma.h"
#include "stm32h5xx_hal_tim.h"

#include "motor_drivers/dshot/dshot.h"

#define BDSHOT_DMA_MOTOR_COUNT (2)

// Idle bits are not necessary, but it holds the line in an idle
// state so the ESC doesn't misinterpret our frame if the switch from
// TX to RX causes noise.
#define BDSHOT_DMA_TX_IDLE_BITS  (2)
#define BDSHOT_DMA_TX_FRAME_SIZE (BDSHOT_FRAME_BITS + BDSHOT_DMA_TX_IDLE_BITS)

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
#define BDSHOT_DMA_T1H_TICKS (625)
#define BDSHOT_DMA_T0H_TICKS (312)
#define BDSHOT_DMA_BIT_TICKS (833)

// Wire telemetry message is always sent at (5/4)*(bit rate).
#define BDSHOT_DMA_TELEMETRY_BIT_TICKS ((BDSHOT_DMA_BIT_TICKS * 4.0f) / 5.0f)

// Reserve one motor index value to denote an invalid motor
#define BDSHOT_DMA_MAX_MOTOR_COUNT (UINT32_MAX - 1)

static_assert(BDSHOT_DMA_MOTOR_COUNT <= BDSHOT_DMA_MAX_MOTOR_COUNT,
              "Exceeded maximum number of supported motors");

typedef struct bdshot_dma_motor_config {
    TIM_HandleTypeDef *tim;
    uint32_t tim_channel;

    DMA_HandleTypeDef *dma;

    GPIO_TypeDef *gpio;
    uint32_t gpio_pin;

    uint8_t pole_count;
} bdshot_dma_motor_config_t;

bool bdshot_dma_init();
bool bdshot_dma_apply();
bool bdshot_dma_set_armed(bool is_armed);

bool bdshot_dma_motor_init(bdshot_motor_index_t motor, bdshot_dma_motor_config_t *config);
bool bdshot_dma_motor_set_armed(bdshot_motor_index_t index, bool is_armed);
bool bdshot_dma_motor_set_throttle(bdshot_motor_index_t motor, uint16_t throttle);
bool bdshot_dma_motor_get_telemetry(bdshot_motor_index_t motor,
                                    bdshot_motor_telemetry_t *telemetry);

#endif // ULYSSES_MOTOR_DRIVER_PROTOCOLS_BDSHOT_DMA_H
