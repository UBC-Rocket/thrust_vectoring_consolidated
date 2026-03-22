/**
 * @file timestamp.c
 * @brief Implementation of high-resolution timestamp utilities.
 *
 * Uses the ARM DWT (Data Watchpoint and Trace) cycle counter for
 * microsecond-resolution timestamps. The CMSIS CoreDebug and DWT
 * registers are standard across Cortex-M3/M4/M7/M33 cores.
 *
 * @ UBC Rocket, Benedikt Howard, 2025
 */

#include "timestamp/timestamp.h"

/*
 * CMSIS core registers are included transitively via the board's HAL.
 * The consuming CMakeLists.txt must ensure the appropriate HAL include
 * paths are available (e.g., stm32h5xx.h or stm32h7xx.h).
 *
 * We rely on these CMSIS-standard definitions:
 *   - CoreDebug->DEMCR, CoreDebug_DEMCR_TRCENA_Msk
 *   - DWT->CYCCNT, DWT->CTRL, DWT_CTRL_CYCCNTENA_Msk
 *   - SystemCoreClock (extern uint32_t)
 */
#if defined(STM32H5)
  #include "stm32h5xx.h"
#elif defined(STM32H7)
  #include "stm32h7xx.h"
#elif defined(STM32F4)
  #include "stm32f4xx.h"
#else
  /* Fallback: consumer must provide CMSIS headers via include paths */
  #include "cmsis_compiler.h"
  extern uint32_t SystemCoreClock;
#endif

/* -------------------------------------------------------------------------- */
/* Private state                                                              */
/* -------------------------------------------------------------------------- */

/**
 * @brief Precomputed divisor for cycle-to-microsecond conversion.
 *
 * Precomputing avoids expensive division in the hot path.
 * cycles_per_us = SystemCoreClock / 1,000,000
 */
static uint32_t g_cycles_per_us = 250;  /* Default for 250MHz, updated in init */

/**
 * @brief High 32 bits of the 64-bit timestamp.
 *
 * Incremented each time CYCCNT wraps around.
 */
static volatile uint32_t g_timestamp_high = 0;

/**
 * @brief Previous CYCCNT value for detecting overflow.
 */
static volatile uint32_t g_last_cyccnt = 0;

/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */

void timestamp_init(void)
{
    /* Enable DWT and CYCCNT if not already enabled */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* Unlock DWT access (required on some Cortex-M cores) */
#if defined(DWT_LAR_KEY)
    DWT->LAR = 0xC5ACCE55U;  /* Unlock key */
#endif

    /* Reset and enable the cycle counter */
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /* Precompute divisor for microsecond conversion */
    g_cycles_per_us = SystemCoreClock / 1000000UL;
    if (g_cycles_per_us == 0) {
        g_cycles_per_us = 1;  /* Prevent divide-by-zero */
    }

    /* Initialize overflow tracking */
    g_timestamp_high = 0;
    g_last_cyccnt = DWT->CYCCNT;
}

void timestamp_update(void)
{
    uint32_t current = DWT->CYCCNT;
    uint32_t last = g_last_cyccnt;

    /* Detect overflow: current < last means CYCCNT wrapped */
    if (current < last) {
        g_timestamp_high++;
    }

    g_last_cyccnt = current;
}

uint64_t timestamp_us64(void)
{
    uint32_t high1, high2, low;

    /*
     * Double-read pattern to handle race with timestamp_update():
     * Read high, read low, read high again. If high changed, retry.
     */
    do {
        high1 = g_timestamp_high;
        low = DWT->CYCCNT;
        high2 = g_timestamp_high;
    } while (high1 != high2);

    /* Convert to microseconds:
     * total_cycles = (high << 32) | low
     * total_us = total_cycles / cycles_per_us
     *
     * We compute this in parts to avoid 64-bit division:
     * us_from_high = (high * (2^32 / cycles_per_us))
     * us_from_low = low / cycles_per_us
     */
    uint32_t cycles_per_us = g_cycles_per_us;

    /* Microseconds from overflow count:
     * Each overflow is 2^32 cycles = (2^32 / cycles_per_us) microseconds
     */
    uint64_t overflow_us = (uint64_t)high1 * (0xFFFFFFFFUL / cycles_per_us + 1);

    /* Microseconds from current cycle count */
    uint32_t current_us = low / cycles_per_us;

    return overflow_us + current_us;
}
