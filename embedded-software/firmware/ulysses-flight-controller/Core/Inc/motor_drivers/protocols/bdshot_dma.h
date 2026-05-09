#ifndef ULYSSES_MOTOR_DRIVER_PROTOCOLS_BDSHOT_DMA_H
#define ULYSSES_MOTOR_DRIVER_PROTOCOLS_BDSHOT_DMA_H

#include <stdint.h>

#include "stm32h5xx.h"
#include "stm32h563xx.h"
#include "stm32h5xx_hal_dma.h"
#include "stm32h5xx_hal_tim.h"

#include "motor_drivers/protocols/bdshot.h"

#define BDSHOT_TX_IDLE_BITS      (2)
#define BDSHOT_DMA_TX_FRAME_SIZE (BDSHOT_FRAME_BITS + BDSHOT_TX_IDLE_BITS)

#define BDSHOT_DMA_RX_FRAME_SIZE (BDSHOT_TELEMETRY_WIRE_BITS)

#define BDSHOT_DMA_T1H_TICKS (312)
#define BDSHOT_DMA_T0H_TICKS (138)
#define BDSHOT_DMA_BIT_TICKS (416)

typedef struct bdshot_dma_motor_config {
    TIM_HandleTypeDef *tim;
    uint32_t tim_channel;

    DMA_HandleTypeDef *dma;

    GPIO_TypeDef *gpio;
    uint32_t gpio_pin;

    uint8_t pole_count;
} bdshot_dma_motor_config_t;

bool bdshot_dma_init();

bool bdshot_dma_motor_init(bdshot_motor_index_t motor, bdshot_dma_motor_config_t *config);
bool bdshot_dma_motor_set_throttle(bdshot_motor_index_t motor, uint16_t throttle);
bool bdshot_dma_motor_get_telemetry(bdshot_motor_index_t motor, bdshot_motor_telemetry_t *telemtry);

bool bdshot_dma_set_armed(bool is_armed);
bool bdshot_dma_apply();

#endif // ULYSSES_MOTOR_DRIVER_PROTOCOLS_BDSHOT_DMA_H
