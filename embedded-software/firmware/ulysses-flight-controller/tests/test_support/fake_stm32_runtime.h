#ifndef FAKE_STM32_RUNTIME_H
#define FAKE_STM32_RUNTIME_H

#include "SPI_queue.h"

#include <stddef.h>
#include <stdint.h>

#define FAKE_MAX_SPI_CALLS 32
#define FAKE_MAX_GPIO_CALLS 64
#define FAKE_MAX_DELAY_CALLS 64
#define FAKE_MAX_SPI_RESPONSES 16
#define FAKE_MAX_SPI_BYTES 32

typedef struct {
    SPI_HandleTypeDef *hspi;
    uint8_t data[FAKE_MAX_SPI_BYTES];
    uint16_t len;
    uint32_t timeout;
} fake_spi_tx_call_t;

typedef struct {
    SPI_HandleTypeDef *hspi;
    uint8_t tx[FAKE_MAX_SPI_BYTES];
    uint8_t rx[FAKE_MAX_SPI_BYTES];
    uint16_t len;
    uint32_t timeout;
} fake_spi_txrx_call_t;

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
    GPIO_PinState state;
} fake_gpio_call_t;

typedef struct {
    uint32_t us;
} fake_delay_call_t;

extern SPI_HandleTypeDef fake_spi_handle;
extern GPIO_TypeDef fake_gpiod_port;

extern size_t fake_spi_tx_count;
extern size_t fake_spi_txrx_count;
extern size_t fake_gpio_count;
extern size_t fake_delay_count;
extern size_t fake_spi_submit_count;

extern fake_spi_tx_call_t fake_spi_tx_calls[FAKE_MAX_SPI_CALLS];
extern fake_spi_txrx_call_t fake_spi_txrx_calls[FAKE_MAX_SPI_CALLS];
extern fake_gpio_call_t fake_gpio_calls[FAKE_MAX_GPIO_CALLS];
extern fake_delay_call_t fake_delay_calls[FAKE_MAX_DELAY_CALLS];
extern spi_job_t fake_last_submitted_job;
extern spi_job_queue_t *fake_last_submit_queue;

void fake_stm32_reset(void);
void fake_spi_queue_response(const uint8_t *data, size_t len);
void fake_set_timestamp_us(uint64_t now_us);

#endif
