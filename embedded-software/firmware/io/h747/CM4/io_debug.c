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
#include "app/shared_memory.h"   /* CM7→CM4 console ring layout */

#include "stm32h7xx.h"       /* __DMB */

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

/* ------------------------------------------------------------------------- *
 * CM7 → CM4 console forwarding (consumer side).
 *
 * CM7 pushes its io_debug_printf text into a SPSC byte ring in uncached SRAM4
 * (see shared_memory.h / io/h747/CM7/io_debug.c). io_debug_pump_remote() drains
 * that ring and emits the bytes to LPUART1 via the same mutex-serialised path
 * as CM4's own console writes, so the two cores' text never interleaves mid-
 * line. A CM4 task calls this periodically (see app_init_cm4.c).
 * ------------------------------------------------------------------------- */
#define DBG_RING_BYTES  APP_SLOT_DEBUG_CONSOLE_RING_BYTES
#define DBG_RING_MASK   (DBG_RING_BYTES - 1U)

typedef struct {
    volatile uint32_t head;
    volatile uint32_t tail;
    volatile uint32_t drops;
    volatile uint32_t magic;
    uint32_t          _pad[4];
} dbg_ring_hdr_t;

static dbg_ring_hdr_t   *s_rhdr;
static volatile uint8_t *s_rdata;

void io_debug_pump_remote(void) {
    if (!s_ready) {
        return;                        /* CM4's own LPUART console not up yet */
    }
    if (s_rhdr == NULL) {
        s_rhdr  = (dbg_ring_hdr_t *)app_shared_slot(
                      APP_SLOT_DEBUG_CONSOLE_RING_OFFSET);
        s_rdata = (volatile uint8_t *)app_shared_slot(
                      APP_SLOT_DEBUG_CONSOLE_RING_OFFSET + APP_SLOT_DEBUG_CONSOLE_RING_HDR);
    }
    if (s_rhdr->magic != APP_DEBUG_CONSOLE_RING_MAGIC) {
        return;                        /* CM7 hasn't initialised the ring yet */
    }

    uint32_t head  = s_rhdr->head;
    uint32_t tail  = s_rhdr->tail;
    uint32_t avail = head - tail;      /* wrap-safe (free-running counters)   */
    if (avail == 0U) {
        return;
    }
    __DMB();                           /* see data written before head advance */

    uint8_t buf[128];
    while (avail > 0U) {
        uint32_t n = (avail > sizeof(buf)) ? (uint32_t)sizeof(buf) : avail;
        for (uint32_t i = 0; i < n; ++i) {
            buf[i] = s_rdata[(tail + i) & DBG_RING_MASK];
        }
        (void)io_debug_write(buf, n);  /* mutex-serialised LPUART emit */
        tail  += n;
        avail -= n;
    }
    __DMB();                           /* finish reads before publishing tail  */
    s_rhdr->tail = tail;
}
