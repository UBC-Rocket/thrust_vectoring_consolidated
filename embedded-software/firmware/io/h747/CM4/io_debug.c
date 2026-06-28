/**
 * @file    io_debug.c
 * @brief   H745 CM4 IO impl: debug-console transport.
 *
 * Routes io_debug_write() bytes to LPUART1 (PA9 TX / PA10 RX, 115200 8-N-1),
 * which the board wires to the STLINK-V3 USB Virtual COM Port — so output
 * lands directly in a host serial terminal (CoolTerm) with no extra adapter.
 *
 * Transmit is blocking HAL UART (io_uart_send), serialised by a mutex once the
 * scheduler runs so concurrent task-context callers never interleave mid-line.
 * Pre-scheduler callers (boot banners from app_init_cm4) skip the lock, which
 * is correct: nothing else can race them before vTaskStartScheduler.
 *
 * UBC Rocket, 2026
 */
#include "io_sys/io_debug.h"
#include "io/io_uart.h"      /* IO_UART_DEBUG — io-impl files may see transports */

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

/* Bounded blocking transmit. A full debug line at 115200 baud is ~14 ms for
 * 160 bytes; 100 ms is generous headroom without wedging a caller forever on
 * a wedged UART. */
#define IO_DEBUG_TX_TIMEOUT_MS  100U

static SemaphoreHandle_t s_tx_mutex;
static StaticSemaphore_t s_tx_mutex_buf;
static bool              s_ready;

io_status_t io_debug_init(void) {
    if (s_ready) {
        return IO_OK;            /* idempotent */
    }

    s_tx_mutex = xSemaphoreCreateMutexStatic(&s_tx_mutex_buf);

    /* Open the debug UART. RX callback is NULL: this is a TX-only text
     * console, and the io_uart_dma_cm RX path is NULL-cb-safe (it guards
     * `msg_len > 0 && u->cb` before dispatching). */
    if (io_uart_open(&IO_UART_DEBUG, NULL, NULL) != IO_OK) {
        return IO_ERR_BUS;
    }

    s_ready = true;
    return IO_OK;
}

io_status_t io_debug_write(const uint8_t *data, size_t len) {
    if (data == NULL || len == 0U) {
        return IO_ERR_PARAM;
    }
    if (!s_ready) {
        return IO_ERR_NOT_READY;
    }

    /* Only take the mutex once the scheduler owns it; taking a mutex before
     * vTaskStartScheduler is undefined in FreeRTOS, and there is no concurrency
     * to guard against pre-scheduler anyway. */
    const bool lock = (s_tx_mutex != NULL) &&
                      (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED);
    if (lock) {
        (void)xSemaphoreTake(s_tx_mutex, portMAX_DELAY);
    }

    io_status_t st = io_uart_send(&IO_UART_DEBUG, data, len, IO_DEBUG_TX_TIMEOUT_MS);

    if (lock) {
        (void)xSemaphoreGive(s_tx_mutex);
    }
    return st;
}
