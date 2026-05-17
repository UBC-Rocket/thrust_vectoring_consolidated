/**
 * @file    io_common.h
 * @brief   CM4 IO impl private helpers shared between the per-peripheral files.
 *
 * UBC Rocket, 2026
 */
#ifndef IO_COMMON_H
#define IO_COMMON_H

#include "main.h"
#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* FreeRTOS notify helper — stored per-peripheral to wake a task from ISR. */
#include "FreeRTOS.h"
#include "task.h"

typedef struct {
    TaskHandle_t task;
    uint32_t     bit;
} io_notify_t;

static inline void io_notify_from_isr(const io_notify_t *n) {
    if (n && n->task && n->bit) {
        BaseType_t woken = pdFALSE;
        xTaskNotifyFromISR(n->task, n->bit, eSetBits, &woken);
        portYIELD_FROM_ISR(woken);
    }
}

#endif /* IO_COMMON_H */
