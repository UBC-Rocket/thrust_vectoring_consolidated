/**
 * @file uart_callbacks.c
 * @brief HAL UART callback implementations + Character Match handling
 *
 * Routes UART events to appropriate driver modules:
 * - USART1: Debug console (DMA circular + Character Match on '\n')
 * - USART5: Radio transceiver (DMA circular + Character Match on 0x00)
 * - USART6: GPS module (DMA circular + Character Match on '\n')
 *
 * Character Match (CM) provides deterministic message framing:
 * - USART1 fires only when '\n' arrives (complete line ready)
 * - USART5 fires only when 0x00 arrives (complete radio message ready)
 * - USART6 fires only when '\n' arrives (complete NMEA sentence ready)
 * - HAL doesn't implement CM handling, so we do it in the ISR layer
 */

#include "uart_callbacks.h"
#include "gps.h"
#include "radio_driver.h"
#include "stm32g0xx_hal.h"
#include <string.h>
#ifdef DEBUG
#include "debug_uart.h"
#endif

/* ============================================================================
 * External UART Handles
 * ============================================================================ */

extern UART_HandleTypeDef huart1;  /* Debug console (defined in main.c) */
extern UART_HandleTypeDef huart5;  /* Radio UART (defined in main.c) */
extern UART_HandleTypeDef huart6;  /* GPS UART (defined in main.c) */

/* ============================================================================
 * Private Variables - USART1 Line Buffering
 * ============================================================================ */

/** DMA RX buffer for USART1 (circular mode - DMA writes here continuously) */
#define UART1_DMA_BUF_SIZE 256
static uint8_t uart1_dma_buf[UART1_DMA_BUF_SIZE];

/** Line buffer - data extracted from DMA buffer on CM event */
static uint8_t uart1_line_buf[UART1_DMA_BUF_SIZE];

/** Last processed position in circular DMA buffer */
static volatile uint16_t uart1_last_pos;

/* ============================================================================
 * Private Function Prototypes
 * ============================================================================ */

static void uart1_line_received(const uint8_t *line, uint16_t len);

/* ============================================================================
 * Initialization
 * ============================================================================ */

void uart_callbacks_init(UART_HandleTypeDef *huart1_handle)
{
    uart1_last_pos = 0;

    /* ADD[7:0] can only be written when UE=0 or RE=0 (RM0444) */
    __HAL_UART_DISABLE(huart1_handle);
    MODIFY_REG(huart1_handle->Instance->CR2, USART_CR2_ADD,
               ((uint32_t)'\n' << USART_CR2_ADD_Pos));
    __HAL_UART_ENABLE(huart1_handle);

    /* USART1: Start DMA circular reception (no IDLE — CM is the only trigger) */
    HAL_UART_Receive_DMA(huart1_handle, uart1_dma_buf, UART1_DMA_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(huart1_handle->hdmarx, DMA_IT_HT | DMA_IT_TC);
    __HAL_UART_ENABLE_IT(huart1_handle, UART_IT_CM);
}

/* ============================================================================
 * Character Match Handler
 * ============================================================================ */

void uart_cm_handler(UART_HandleTypeDef *huart)
{
    /* Get current DMA position (bytes received so far) */
    uint16_t pos = huart->RxXferSize - __HAL_DMA_GET_COUNTER(huart->hdmarx);

    if (huart->Instance == USART1)
    {
        /* Extract line from circular DMA buffer: last_pos → pos */
        uint16_t line_len = 0;
        uint16_t last = uart1_last_pos;
        uint16_t curr = pos;

        if (curr > last)
        {
            line_len = curr - last;
            memcpy(uart1_line_buf, &uart1_dma_buf[last], line_len);
        }
        else if (curr < last)
        {
            /* Wrap-around: copy tail then head */
            uint16_t tail = UART1_DMA_BUF_SIZE - last;
            memcpy(uart1_line_buf, &uart1_dma_buf[last], tail);
            memcpy(&uart1_line_buf[tail], uart1_dma_buf, curr);
            line_len = tail + curr;
        }
        uart1_last_pos = curr;

        /* Strip trailing \r\n */
        while (line_len > 0 &&
               (uart1_line_buf[line_len - 1] == '\n' ||
                uart1_line_buf[line_len - 1] == '\r'))
        {
            line_len--;
        }

        /* Null-terminate so downstream %s doesn't read stale data */
        uart1_line_buf[line_len] = '\0';

        if (line_len > 0)
        {
            uart1_line_received(uart1_line_buf, line_len);
        }
    }
    else if (huart->Instance == USART5)
    {
        /*
         * Radio CM on 0x00: call existing radio callback with DMA position.
         * process_dma_data() uses s_last_pos tracking so IDLE events
         * that fire afterwards won't double-process.
         */
        radio_rx_event_callback(huart, pos);
    }
    else if (huart->Instance == USART6)
    {
        /* GPS CM on '\n': complete NMEA sentence ready */
        gps_rx_event_callback(huart, pos);
    }
}

/* ============================================================================
 * USART1 Line Processing
 * ============================================================================ */

/**
 * @brief Process a complete line received on USART1
 *
 * Called when Character Match fires on '\n'. The line has been extracted
 * from the circular DMA buffer with \r\n stripped.
 */
static void uart1_line_received(const uint8_t *line, uint16_t len)
{
#ifdef DEBUG
    /* Debug mode: check for injection prefix, otherwise forward to radio */
    if (len >= 2 && line[0] == 'R')
    {
        /* Radio injection: "R<payload>\n" → inject to radio queue */
        debug_uart_rx_line_callback(&line[1], len - 1, DEBUG_UART_TYPE_RADIO);
    }
    else if (len >= 2 && line[0] == 'G')
    {
        /* GPS injection: "G$GPGGA,...\n" → inject to GPS queue */
        debug_uart_rx_line_callback(&line[1], len - 1, DEBUG_UART_TYPE_GPS);
    }
    else
    {
        /* Forward line to Radio UART5 as a message */
        HAL_UART_Transmit(&huart5, line, len, 100);
    }
#else
    /* Release: forward complete line to Radio UART5 */
    HAL_UART_Transmit(&huart5, line, len, 100);
#endif
}

/* ============================================================================
 * HAL UART Callbacks (weak redefinitions)
 * ============================================================================ */

/**
 * @brief DMA RX Event callback for IDLE/HT/TC events
 *
 * All three UARTs now use Character Match as their primary trigger.
 * This callback is kept for HAL compatibility but no longer routes GPS events.
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    (void)huart;
    (void)Size;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    gps_uart_tx_cplt_callback(huart);
#ifdef DEBUG
    if (huart->Instance == USART1) {
        debug_uart_dma_tx_cplt();
    }
#endif
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6)
    {
        gps_uart_error_callback(huart);
    }
    else if (huart->Instance == USART5)
    {
        radio_uart_error_callback(huart);
    }
}
