/**
 * @file    io_timestamp.c
 * @brief   CM7-side timestamp: read-only attach to TIM13 + the shared
 *          software upper word that CM4's update IRQ bumps.
 *
 * UBC Rocket, 2026
 */
#include "io_common.h"
#include "io_sys/io_timestamp.h"
#include "app/shared_memory.h"

/* TIM13 register access — its base address is the same from both cores. */
#include "stm32h7xx.h"

static volatile uint32_t *s_upper;

void io_timestamp_init(void) {
    /* CM4 brings TIM13 up before releasing CM7 from boot, so the timer is
     * already running. We just attach to the shared upper word. */
    s_upper = (volatile uint32_t *)app_shared_slot(APP_TIMESTAMP_UPPER_OFFSET);
}

static uint64_t read_ticks(void) {
    uint32_t up1, up2, cnt;
    do {
        up1 = *s_upper;
        __DMB();
        cnt = TIM13->CNT;
        __DMB();
        up2 = *s_upper;
    } while (up1 != up2);
    return ((uint64_t)up1 << 16) | (uint64_t)cnt;
}

uint64_t io_timestamp_ticks_100ns(void) {
    return read_ticks();
}

uint64_t io_timestamp_us(void) {
    return read_ticks() / 10ULL;
}
