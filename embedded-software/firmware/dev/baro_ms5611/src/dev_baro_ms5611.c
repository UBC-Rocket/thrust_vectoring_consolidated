/**
 * @file    dev_baro_ms5611.c
 * @brief   MS5611 barometer driver — TIM7-paced, non-blocking conversion
 *          state machine.
 *
 * The MS5611 has no data-ready pin, so sampling is time-paced. A 100 Hz
 * hardware timer (TIM7, see MX_TIM7_Init) drives the conversion state
 * machine one step per tick — nothing ever blocks:
 *
 *   IDLE       → kick D1 (pressure) conversion            → PENDING_D1
 *   PENDING_D1 → read D1, kick D2 (temperature)           → PENDING_D2
 *   PENDING_D2 → read D2, compute, push sample, kick D1   → PENDING_D1
 *
 * Because the 10 ms tick period exceeds the ~9 ms OSR=4096 conversion time,
 * each tick's read is guaranteed to see a completed conversion — the timer
 * period *is* the conversion wait, so there are no busy-waits or vTaskDelays.
 * One sample lands every two ticks → ~50 Hz. OSR=4096 gives the best
 * resolution (~0.012 mbar RMS ≈ 10 cm).
 *
 * Threading: the TIM7 update IRQ (via HAL_TIM_PeriodElapsedCallback in
 * main.c → baro_ms5611_on_tim_period_elapsed) only *notifies* a small worker
 * task — it does NO SPI in the ISR. The worker runs one state-machine step
 * per notification, so the blocking SPI reads happen in task context (where
 * a HAL timeout can actually fire). The state-estimation task drains the
 * ring. The worker blocks on its notification, never on a delay, so it can't
 * wedge the way a vTaskDelay-loop task could.
 *
 * Task creation + timer start live in baro_ms5611_start(), called from
 * app_init — NOT from baro_ms5611_init(). Creating a task before the
 * scheduler starts leaves BASEPRI raised (uxCriticalNesting is 0xaaaaaaaa
 * until the scheduler runs), which masks the HAL timebase IRQ and freezes
 * HAL_GetTick(); see the same note in dev_mag_mmc5983.c.
 *
 * SPI: IO_SPI_MS5611 (SPI2) with hardware NSS, so io_spi_xfer_blocking frames
 * CS for us. Init runs blocking transfers pre-scheduler (fine, like the other
 * sensor inits).
 *
 * UBC Rocket, 2026
 */
#include "dev_baro_ms5611.h"

#include "io/io_spi.h"
#include "io_sys/io_timestamp.h"
#include "io_sys/io_test_hooks.h"

#include "sensors/ms5611_baro.h"

#include "stm32h7xx_hal.h"   /* HAL_Delay (init) + HAL_TIM_Base_Start_IT (start) */

#include <string.h>

#if defined(__has_include)
#  if __has_include("FreeRTOS.h")
#    include "FreeRTOS.h"
#    include "task.h"
#    define BARO_HAVE_FREERTOS 1
#  endif
#endif

#ifdef BARO_HAVE_FREERTOS
/* TIM7 is the 100 Hz conversion pacer (declared in the CubeMX tim.c). We
 * take it by extern rather than including tim.h to avoid pulling main.h into
 * a device driver — TIM7 / TIM_HandleTypeDef / HAL_TIM_* all come from the
 * HAL headers above. */
extern TIM_HandleTypeDef htim7;
#endif

/* -------------------------------------------------------------------------- */
/* Tunables                                                                    */
/* -------------------------------------------------------------------------- */

#define MS5611_RING_SIZE       8

/* OSR=4096: best resolution. Datasheet max conversion time is 9.04 ms, under
 * the 10 ms TIM7 period — so each tick's read always sees a finished
 * conversion. */
#define MS5611_OSR             MS5611_OSR_4096

/* Reset triggers a PROM reload; datasheet requires >= 2.8 ms settle. */
#define MS5611_RESET_DELAY_MS  10U

#define MS5611_SPI_TIMEOUT_MS  10U

/* -------------------------------------------------------------------------- */
/* Driver state                                                                */
/* -------------------------------------------------------------------------- */

typedef enum {
    BARO_CONV_IDLE = 0,      /* nothing in flight; next tick kicks D1        */
    BARO_CONV_PENDING_D1,    /* D1 (pressure) conversion running             */
    BARO_CONV_PENDING_D2,    /* D2 (temperature) conversion running          */
} baro_conv_state_t;

struct baro_ms5611 {
    ms5611_t          dev;             /* PROM coeffs + raw/compensated values */
    io_task_handle_t  notify_task;     /* downstream (state estimation)        */
    uint32_t          notify_bit;
    baro_sample_t     ring[MS5611_RING_SIZE];
    volatile uint8_t  head, tail;

    baro_conv_state_t conv_state;

#ifdef BARO_HAVE_FREERTOS
    TaskHandle_t      worker_task;     /* woken by the TIM7 IRQ, runs tick()  */
    StaticTask_t      worker_tcb;
    StackType_t       worker_stack[512];    /* 2 KB — one tick step: quick SPI
                                            * + ms5611_compute, no printf. */
#endif

    bool              ready;
};

static struct baro_ms5611 s_self;

/* Test-only access to module-private state. No-op in production builds. */
IO_TEST_HOOK_RW(s_self, struct baro_ms5611, dev_baro_ms5611_self)

/* -------------------------------------------------------------------------- */
/* SPI helpers (init / worker-task context, never the timer ISR)               */
/* -------------------------------------------------------------------------- */

/* One-byte command, no readback (reset / convert). */
static bool spi_cmd(uint8_t cmd) {
    return io_spi_xfer_blocking(&IO_SPI_MS5611, &cmd, NULL, 1,
                                MS5611_SPI_TIMEOUT_MS) == IO_OK;
}

/* Read one 16-bit PROM word: TX [cmd,0,0], data lands in rx[1..2]. */
static bool prom_read(uint8_t index, uint16_t *out) {
    uint8_t tx[3];
    uint8_t rx[3] = {0};
    (void)ms5611_build_prom_read(index, tx);
    if (io_spi_xfer_blocking(&IO_SPI_MS5611, tx, rx, 3,
                             MS5611_SPI_TIMEOUT_MS) != IO_OK) {
        return false;
    }
    return ms5611_parse_prom_word(&rx[1], out);
}

/* Read the 24-bit ADC result: TX [0x00,0,0,0], data lands in rx[1..3]. */
static bool adc_read(uint32_t *out) {
    uint8_t tx[4];
    uint8_t rx[4] = {0};
    (void)ms5611_build_adc_read(tx);
    if (io_spi_xfer_blocking(&IO_SPI_MS5611, tx, rx, 4,
                             MS5611_SPI_TIMEOUT_MS) != IO_OK) {
        return false;
    }
    return ms5611_parse_adc_result(&rx[1], out);
}

/* -------------------------------------------------------------------------- */
/* Sample plumbing                                                             */
/* -------------------------------------------------------------------------- */

static inline bool ring_full(const baro_ms5611_t *d) {
    return (uint8_t)((d->head + 1U) % MS5611_RING_SIZE) == d->tail;
}

static void notify_downstream(baro_ms5611_t *d) {
#ifdef BARO_HAVE_FREERTOS
    /* Runs in the worker task (not the ISR), so plain xTaskNotify is fine. */
    if (d->notify_task && d->notify_bit) {
        xTaskNotify((TaskHandle_t)d->notify_task, d->notify_bit, eSetBits);
    }
#else
    (void)d;
#endif
}

static void push_sample(baro_ms5611_t *d) {
    if (ring_full(d)) return;
    baro_sample_t *s = &d->ring[d->head];
    s->t_us = io_timestamp_us();
    /* P_centi_mbar is pressure in 0.01 mbar; 0.01 mbar == 1 Pa exactly, so
     * the value in P_centi_mbar equals the pressure in pascals. */
    s->pressure_pa = (float)d->dev.P_centi_mbar;
    s->temp_c      = (float)d->dev.TEMP_centi * 0.01f;
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    d->head = (uint8_t)((d->head + 1U) % MS5611_RING_SIZE);
    notify_downstream(d);
}

/* -------------------------------------------------------------------------- */
/* Conversion state machine — one step per TIM7 tick (worker-task context)     */
/* -------------------------------------------------------------------------- */

static void baro_do_tick(baro_ms5611_t *d) {
    const uint8_t d1_cmd = (uint8_t)(MS5611_CMD_CONVERT_D1 | (uint8_t)MS5611_OSR);
    const uint8_t d2_cmd = (uint8_t)(MS5611_CMD_CONVERT_D2 | (uint8_t)MS5611_OSR);
    uint32_t raw;

    switch (d->conv_state) {
    case BARO_CONV_IDLE:
        (void)spi_cmd(d1_cmd);
        d->conv_state = BARO_CONV_PENDING_D1;
        break;

    case BARO_CONV_PENDING_D1:
        /* D1 conversion started a full tick ago → done. Read it, kick D2. */
        if (adc_read(&raw)) d->dev.D1_raw = raw;
        (void)spi_cmd(d2_cmd);
        d->conv_state = BARO_CONV_PENDING_D2;
        break;

    case BARO_CONV_PENDING_D2:
        /* D2 done. Read it, compute the compensated pair, publish, kick D1. */
        if (adc_read(&raw)) {
            d->dev.D2_raw = raw;
            if (ms5611_compute(&d->dev)) push_sample(d);
        }
        (void)spi_cmd(d1_cmd);
        d->conv_state = BARO_CONV_PENDING_D1;
        break;
    }
}

#ifdef BARO_HAVE_FREERTOS
static void baro_worker(void *arg) {
    baro_ms5611_t *d = (baro_ms5611_t *)arg;
    for (;;) {
        /* Block until the TIM7 IRQ ticks us. No timeout, no delay-loop — the
         * hardware timer is the sole pacer. */
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        baro_do_tick(d);
    }
}
#endif

/* -------------------------------------------------------------------------- */
/* Public API                                                                  */
/* -------------------------------------------------------------------------- */

/* C1..C6 are the only PROM words ms5611_compute() uses. A dead / absent bus
 * reads them all 0x0000 or all 0xFFFF; a genuine calibration is neither.
 * Fallback sanity gate when BARO_RELAX_CRC bypasses the strict CRC. */
static bool coeffs_plausible(const ms5611_t *dev) {
    for (int i = 1; i <= 6; ++i) {
        if (dev->C[i] == 0x0000 || dev->C[i] == 0xFFFF) return false;
    }
    return true;
}

bool baro_ms5611_init(void) {
    memset(&s_self, 0, sizeof(s_self));

    /* 1. Reset → PROM reload. Blocking + HAL_Delay are fine here: init runs
     *    in the DEV phase before the scheduler, where the HAL tick is live. */
    if (!spi_cmd(MS5611_CMD_RESET)) return false;
    HAL_Delay(MS5611_RESET_DELAY_MS);

    /* 2. Read the 8 calibration PROM words (C0..C7). */
    for (uint8_t i = 0; i < 8; ++i) {
        if (!prom_read(i, &s_self.dev.C[i])) return false;
    }

    /* 3. Verify the factory CRC-4. A bad CRC normally means a garbage read
     *    (wrong bus config / wrong device) → fail so the handle stays NULL
     *    and consumers skip the baro.
     *
     *    Known bring-up wrinkle on the current board: PROM word 0 (factory /
     *    reserved — NOT used by ms5611_compute, which only needs C1..C6)
     *    reads back 0x0000, which fails the CRC even though C1..C6 are all
     *    valid. Suspected cause (unconfirmed, deferred): NSS setup time —
     *    the gap between hardware NSS going active and the first SPI clock
     *    edge (H7 MasterSSIdleness) may be too short for the first word.
     *
     *    BARO_RELAX_CRC (compile-time, default off) lets bring-up proceed
     *    when the CRC fails but C1..C6 look sane. Strict by default. Do NOT
     *    rely on this for flight without resolving the underlying read. */
    if (!ms5611_check_crc(s_self.dev.C)) {
#if BARO_RELAX_CRC
        if (!coeffs_plausible(&s_self.dev)) return false;
#else
        (void)coeffs_plausible;
        return false;
#endif
    }

    s_self.conv_state = BARO_CONV_IDLE;
    s_self.ready = true;
    return true;
}

bool baro_ms5611_start(void) {
#ifdef BARO_HAVE_FREERTOS
    if (!s_self.ready) return false;              /* init must have run */
    if (s_self.worker_task != NULL) return true;  /* idempotent */

    /* Priority 5 — sensor-producer tier, same as the mag service task. The
     * worker must exist before TIM7 fires so the first tick has a target. */
    s_self.worker_task = xTaskCreateStatic(
        baro_worker, "baro", 512, &s_self, 5,
        s_self.worker_stack, &s_self.worker_tcb);
    if (s_self.worker_task == NULL) return false;

    /* Release the 100 Hz conversion pacer. */
    return HAL_TIM_Base_Start_IT(&htim7) == HAL_OK;
#else
    return true;
#endif
}

void baro_ms5611_on_tim_period_elapsed(void *htim) {
#ifdef BARO_HAVE_FREERTOS
    TIM_HandleTypeDef *t = (TIM_HandleTypeDef *)htim;
    if (t == NULL || t->Instance != TIM7) return;
    if (s_self.worker_task == NULL) return;
    /* ISR context: notify only, no SPI here. */
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(s_self.worker_task, &woken);
    portYIELD_FROM_ISR(woken);
#else
    (void)htim;
#endif
}

baro_ms5611_t *baro_ms5611_get(void) {
    return s_self.ready ? &s_self : NULL;
}

size_t baro_ms5611_drain(baro_ms5611_t *d, baro_sample_t *out, size_t max) {
    if (!d || !out) return 0;
    size_t n = 0;
    while (n < max && d->tail != d->head) {
        out[n++] = d->ring[d->tail];
        d->tail = (uint8_t)((d->tail + 1U) % MS5611_RING_SIZE);
    }
    return n;
}

void baro_ms5611_set_notify(baro_ms5611_t *d, io_task_handle_t t, uint32_t bit) {
    if (!d) return;
    d->notify_task = t;
    d->notify_bit  = bit;
}
