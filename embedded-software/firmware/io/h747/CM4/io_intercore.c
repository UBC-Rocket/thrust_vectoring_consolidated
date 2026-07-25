/**
 * @file    io_intercore.c
 * @brief   CM4 side of the HSEM-backed intercore notifier.
 *
 * Each @ref io_ic_event_t maps 1:1 to an HSEM ID (0..31). The receiver side
 * (CM4) enables the HSEM notification interrupt for the other core and
 * routes completed HSEM interrupts back through the registered handler.
 *
 * UBC Rocket, 2026
 */

#include "io_common.h"
#include "io_sys/io_intercore.h"

/* HSEM IDs — CM4 listens to events the CM7 will signal and signals events
 * consumed by CM7. Allocate a distinct HSEM id per event. */
#define HSEM_ID_BASE 0U

static io_ic_handler_t s_handlers[IO_IC_EVENT_COUNT];

static inline uint32_t hsem_mask(io_ic_event_t evt) {
    return 1UL << (HSEM_ID_BASE + (uint32_t)evt);
}

void io_intercore_init(void) {
    __HAL_RCC_HSEM_CLK_ENABLE();
    /* Enable HSEM notification interrupts for all events meant for CM4.
     * The interrupt fires when *another* core releases the semaphore. */
    HAL_HSEM_ActivateNotification(
        hsem_mask(IO_IC_CONTROL_OUTPUT_READY) |
        hsem_mask(IO_IC_LOG_BUFFER_CM7_READY));

    /* HSEM2_IRQn is the CM4-side notification interrupt. */
    HAL_NVIC_SetPriority(HSEM2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(HSEM2_IRQn);
}

void io_intercore_signal(io_ic_event_t evt) {
    if (evt >= IO_IC_EVENT_COUNT) return;
    uint32_t id = HSEM_ID_BASE + (uint32_t)evt;
    HAL_HSEM_FastTake(id);
    HAL_HSEM_Release(id, 0);
}

void io_intercore_register_handler(io_ic_event_t evt, io_ic_handler_t cb) {
    if (evt >= IO_IC_EVENT_COUNT) return;
    s_handlers[evt] = cb;
}

/* HAL weak callback — called from HSEM1_IRQHandler (MUST be forwarded from
 * stm32h7xx_it.c's HSEM1_IRQHandler via HAL_HSEM_IRQHandler). */
void HAL_HSEM_FreeCallback(uint32_t SemMask) {
    for (uint32_t e = 0; e < IO_IC_EVENT_COUNT; ++e) {
        if (SemMask & hsem_mask((io_ic_event_t)e)) {
            if (s_handlers[e]) s_handlers[e]();
            /* Re-arm the notification for the next signal. */
            HAL_HSEM_ActivateNotification(hsem_mask((io_ic_event_t)e));
        }
    }
}
