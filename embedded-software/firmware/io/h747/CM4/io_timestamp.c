/**
 * @file    io_timestamp.c
 * @brief   CM4-side timestamp: configures TIM13 at 10 MHz, owns the overflow
 *          IRQ that bumps the shared upper word.
 *
 * TIM13 is a 16-bit basic timer in APB1 (D2 domain) — accessible from both
 * cores. CubeMX must set:
 *   - clock source for TIM13 such that the timer tick is 10 MHz
 *     (e.g. APB1 timer-clock = 200 MHz, PSC = 19, ARR = 0xFFFF)
 *   - update interrupt enabled
 *
 * UBC Rocket, 2026
 */
#include "io_common.h"
#include "io_sys/io_timestamp.h"
#include "app/shared_memory.h"

#include "tim.h"   /* CubeMX-generated; declares htim13 */

/* Shared software upper word. Lives in SRAM4 so CM7 can also read. */
static volatile uint32_t *s_upper;

/* Called from the CubeMX HAL_TIM_PeriodElapsedCallback dispatcher in
 * Core/Src/main.c (USER CODE BEGIN Callback 1). We can't override the
 * weak symbol directly because main.c already defines it for the HAL
 * timebase (TIM5). TIM13 wraps every 6.5536 ms at 10 MHz. */
void io_timestamp_on_tim_period_elapsed(void *htim) {
    TIM_HandleTypeDef *t = (TIM_HandleTypeDef *)htim;
    if (t->Instance == TIM13) {
        if (s_upper) (*s_upper)++;
    }
}

void io_timestamp_init(void) {
    s_upper = (volatile uint32_t *)app_shared_slot(APP_TIMESTAMP_UPPER_OFFSET);
    *s_upper = 0;

    __HAL_TIM_SET_COUNTER(&htim13, 0);
    HAL_TIM_Base_Start_IT(&htim13);
}

static uint64_t read_ticks(void) {
    /* Seqlock-style read: re-poll if upper word changes mid-read. */
    uint32_t up1, up2, cnt;
    do {
        up1 = *s_upper;
        __DMB();
        cnt = __HAL_TIM_GET_COUNTER(&htim13);
        __DMB();
        up2 = *s_upper;
    } while (up1 != up2);
    return ((uint64_t)up1 << 16) | (uint64_t)cnt;
}

uint64_t io_timestamp_ticks_100ns(void) {
    return read_ticks();
}

uint64_t io_timestamp_us(void) {
    /* 1 tick = 0.1 µs → 10 ticks per µs. */
    return read_ticks() / 10ULL;
}
