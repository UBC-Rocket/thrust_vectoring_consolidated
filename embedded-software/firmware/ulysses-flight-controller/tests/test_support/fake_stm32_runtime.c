#include "fake_stm32_runtime.h"

#include <string.h>

SPI_HandleTypeDef fake_spi_handle = {0};
GPIO_TypeDef fake_gpiod_port = {0};
bool g_sd_card_initialized = false;

size_t fake_spi_tx_count = 0;
size_t fake_spi_txrx_count = 0;
size_t fake_gpio_count = 0;
size_t fake_delay_count = 0;
size_t fake_spi_submit_count = 0;

fake_spi_tx_call_t fake_spi_tx_calls[FAKE_MAX_SPI_CALLS];
fake_spi_txrx_call_t fake_spi_txrx_calls[FAKE_MAX_SPI_CALLS];
fake_gpio_call_t fake_gpio_calls[FAKE_MAX_GPIO_CALLS];
fake_delay_call_t fake_delay_calls[FAKE_MAX_DELAY_CALLS];
spi_job_t fake_last_submitted_job = {0};
spi_job_queue_t *fake_last_submit_queue = NULL;

static uint8_t fake_spi_responses[FAKE_MAX_SPI_RESPONSES][FAKE_MAX_SPI_BYTES];
static size_t fake_spi_response_lengths[FAKE_MAX_SPI_RESPONSES];
static size_t fake_spi_response_count = 0;
static size_t fake_spi_response_index = 0;
static uint64_t fake_now_us = 0;

void fake_stm32_reset(void)
{
    fake_spi_tx_count = 0;
    fake_spi_txrx_count = 0;
    fake_gpio_count = 0;
    fake_delay_count = 0;
    fake_spi_submit_count = 0;
    fake_spi_response_count = 0;
    fake_spi_response_index = 0;
    fake_now_us = 0;
    fake_last_submitted_job = (spi_job_t){0};
    fake_last_submit_queue = NULL;
    memset(fake_spi_tx_calls, 0, sizeof(fake_spi_tx_calls));
    memset(fake_spi_txrx_calls, 0, sizeof(fake_spi_txrx_calls));
    memset(fake_gpio_calls, 0, sizeof(fake_gpio_calls));
    memset(fake_delay_calls, 0, sizeof(fake_delay_calls));
    memset(fake_spi_responses, 0, sizeof(fake_spi_responses));
    memset(fake_spi_response_lengths, 0, sizeof(fake_spi_response_lengths));
}

void fake_spi_queue_response(const uint8_t *data, size_t len)
{
    if (fake_spi_response_count >= FAKE_MAX_SPI_RESPONSES) {
        return;
    }

    if (len > FAKE_MAX_SPI_BYTES) {
        len = FAKE_MAX_SPI_BYTES;
    }

    memcpy(fake_spi_responses[fake_spi_response_count], data, len);
    fake_spi_response_lengths[fake_spi_response_count] = len;
    fake_spi_response_count++;
}

void fake_set_timestamp_us(uint64_t now_us)
{
    fake_now_us = now_us;
}

void fake_delay_us(uint32_t us)
{
    if (fake_delay_count >= FAKE_MAX_DELAY_CALLS) {
        return;
    }

    fake_delay_calls[fake_delay_count].us = us;
    fake_delay_count++;
}

void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
    if (fake_gpio_count >= FAKE_MAX_GPIO_CALLS) {
        return;
    }

    fake_gpio_calls[fake_gpio_count].port = GPIOx;
    fake_gpio_calls[fake_gpio_count].pin = GPIO_Pin;
    fake_gpio_calls[fake_gpio_count].state = PinState;
    fake_gpio_count++;
}

HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi,
                                   uint8_t *pData,
                                   uint16_t Size,
                                   uint32_t Timeout)
{
    if (fake_spi_tx_count < FAKE_MAX_SPI_CALLS) {
        fake_spi_tx_calls[fake_spi_tx_count].hspi = hspi;
        fake_spi_tx_calls[fake_spi_tx_count].len = Size;
        fake_spi_tx_calls[fake_spi_tx_count].timeout = Timeout;
        if (Size > FAKE_MAX_SPI_BYTES) {
            Size = FAKE_MAX_SPI_BYTES;
        }
        memcpy(fake_spi_tx_calls[fake_spi_tx_count].data, pData, Size);
        fake_spi_tx_count++;
    }

    return HAL_OK;
}

HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi,
                                          uint8_t *pTxData,
                                          uint8_t *pRxData,
                                          uint16_t Size,
                                          uint32_t Timeout)
{
    size_t copy_len = Size;

    if (fake_spi_txrx_count < FAKE_MAX_SPI_CALLS) {
        fake_spi_txrx_calls[fake_spi_txrx_count].hspi = hspi;
        fake_spi_txrx_calls[fake_spi_txrx_count].len = Size;
        fake_spi_txrx_calls[fake_spi_txrx_count].timeout = Timeout;
        if (copy_len > FAKE_MAX_SPI_BYTES) {
            copy_len = FAKE_MAX_SPI_BYTES;
        }
        memcpy(fake_spi_txrx_calls[fake_spi_txrx_count].tx, pTxData, copy_len);
    }

    memset(pRxData, 0, Size);
    if (fake_spi_response_index < fake_spi_response_count) {
        size_t scripted_len = fake_spi_response_lengths[fake_spi_response_index];
        if (scripted_len > Size) {
            scripted_len = Size;
        }
        memcpy(pRxData, fake_spi_responses[fake_spi_response_index], scripted_len);
        fake_spi_response_index++;
    }

    if (fake_spi_txrx_count < FAKE_MAX_SPI_CALLS) {
        if (copy_len > FAKE_MAX_SPI_BYTES) {
            copy_len = FAKE_MAX_SPI_BYTES;
        }
        memcpy(fake_spi_txrx_calls[fake_spi_txrx_count].rx, pRxData, copy_len);
        fake_spi_txrx_count++;
    }

    return HAL_OK;
}

bool spi_submit_job(spi_job_t job, spi_job_queue_t *q)
{
    fake_last_submitted_job = job;
    fake_last_submit_queue = q;
    fake_spi_submit_count++;
    return true;
}

uint64_t timestamp_us64(void)
{
    return fake_now_us;
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim)
{
    (void)htim;
}

void Error_Handler(void)
{
}
