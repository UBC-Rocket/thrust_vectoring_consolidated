/**
 * @file    io_intercore.c
 * @brief   CM7 side of the HSEM-backed intercore notifier.
 *
 * UBC Rocket, 2026
 */

#include "io_common.h"
#include "io_sys/io_intercore.h"

#define HSEM_ID_BASE 0U

static io_ic_handler_t s_handlers[IO_IC_EVENT_COUNT];

static inline uint32_t hsem_mask(io_ic_event_t evt) {
    return 1UL << (HSEM_ID_BASE + (uint32_t)evt);
}

void io_intercore_init(void) {
    __HAL_RCC_HSEM_CLK_ENABLE();
    HAL_HSEM_ActivateNotification(
        hsem_mask(IO_IC_STATE_UPDATED) |
        hsem_mask(IO_IC_FLIGHT_STATE_UPDATED) |
        hsem_mask(IO_IC_LOG_BUFFER_CM4_FREE));

    /* HSEM1_IRQn is the CM7-side notification interrupt. */
    HAL_NVIC_SetPriority(HSEM1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(HSEM1_IRQn);
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

void HAL_HSEM_FreeCallback(uint32_t SemMask) {
    for (uint32_t e = 0; e < IO_IC_EVENT_COUNT; ++e) {
        if (SemMask & hsem_mask((io_ic_event_t)e)) {
            if (s_handlers[e]) s_handlers[e]();
            HAL_HSEM_ActivateNotification(hsem_mask((io_ic_event_t)e));
        }
    }
}
