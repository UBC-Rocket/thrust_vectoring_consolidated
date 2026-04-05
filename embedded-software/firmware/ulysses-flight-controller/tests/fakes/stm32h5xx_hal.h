#ifndef STM32H5XX_HAL_H
#define STM32H5XX_HAL_H

#include <stdint.h>

#define __IO volatile
#define __NOP() ((void)0)
#define __DMB() ((void)0)
#define __DSB() ((void)0)
#define __ISB() ((void)0)

#define HAL_MAX_DELAY 0xFFFFFFFFu

#define GPIO_PIN_0  ((uint16_t)0x0001u)
#define GPIO_PIN_1  ((uint16_t)0x0002u)
#define GPIO_PIN_2  ((uint16_t)0x0004u)
#define GPIO_PIN_3  ((uint16_t)0x0008u)
#define GPIO_PIN_4  ((uint16_t)0x0010u)
#define GPIO_PIN_5  ((uint16_t)0x0020u)
#define GPIO_PIN_6  ((uint16_t)0x0040u)
#define GPIO_PIN_7  ((uint16_t)0x0080u)
#define GPIO_PIN_8  ((uint16_t)0x0100u)
#define GPIO_PIN_9  ((uint16_t)0x0200u)
#define GPIO_PIN_10 ((uint16_t)0x0400u)
#define GPIO_PIN_11 ((uint16_t)0x0800u)
#define GPIO_PIN_12 ((uint16_t)0x1000u)
#define GPIO_PIN_13 ((uint16_t)0x2000u)
#define GPIO_PIN_14 ((uint16_t)0x4000u)
#define GPIO_PIN_15 ((uint16_t)0x8000u)

typedef enum {
    HAL_OK = 0,
    HAL_ERROR = 1
} HAL_StatusTypeDef;

typedef enum {
    GPIO_PIN_RESET = 0,
    GPIO_PIN_SET = 1
} GPIO_PinState;

typedef struct {
    uint32_t BSRR;
} GPIO_TypeDef;

typedef struct {
    uint32_t instance_id;
} SPI_HandleTypeDef;

typedef struct {
    uint32_t dummy;
} TIM_HandleTypeDef;

void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi,
                                   uint8_t *pData,
                                   uint16_t Size,
                                   uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi,
                                          uint8_t *pTxData,
                                          uint8_t *pRxData,
                                          uint16_t Size,
                                          uint32_t Timeout);

extern GPIO_TypeDef fake_gpiod_port;

#define GPIOD (&fake_gpiod_port)

#endif
