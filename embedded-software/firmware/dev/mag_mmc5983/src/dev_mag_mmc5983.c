/**
 * @file    dev_mag_mmc5983.c
 * @brief   MMC5983MA driver — single-shot SET/RESET differencing.
 *
 * Why single-shot, not continuous mode:
 *   The MMC5983MA has a non-zero magnetic offset that drifts with
 *   temperature + nearby field history. The chip provides SET and
 *   RESET coil pulses that flip the sensor's magnetic-domain
 *   orientation; SET and RESET produce equal-magnitude, OPPOSITE-sign
 *   readings of the same external field. Therefore:
 *       field  = (M_set - M_reset) / 2   ← offset cancels
 *       offset = (M_set + M_reset) / 2   ← live offset estimate
 *   Continuous mode auto-fires measurements at the configured ODR
 *   with periodic SET pulses (PRD_SET), but that's a degauss schedule
 *   for domain alignment — NOT offset cancellation. Continuous mode
 *   gives a stable but offset-biased reading; single-shot lets us do
 *   the SET/RESET differencing. We pay 2× the chip's ODR per output
 *   sample, so BW=200 Hz (~4 ms per measurement) → 100 Hz field rate.
 *
 * Pipeline:
 *   1. EXTI on DRDY notifies mag_service_task.
 *   2. Service task reads the measurement, advances its 2-step state
 *      machine (waiting_for_set → waiting_for_reset), and:
 *        - kicks SET + TM_M, waits for DRDY → captures M_set
 *        - kicks RESET + TM_M, waits for DRDY → captures M_reset
 *        - computes (M_set - M_reset)/2 = field, (M_set + M_reset)/2 = offset
 *        - pushes field sample to the ring, optionally exposes offset
 *          via a getter for diagnostic
 *   3. The drain API is unchanged: state_estimation_task drains samples
 *      as before.
 *
 * Init delay uses HAL_Delay (HAL tick / SysTick are up before the
 * FreeRTOS scheduler starts; mag_mmc5983_init is called from
 * dev_init_cm4 in that window).
 *
 * UBC Rocket, 2026
 */
#include "dev_mag_mmc5983.h"

#include "io/io_spi.h"
#include "io/io_exti.h"
#include "io_sys/io_timestamp.h"
#include "io_sys/io_test_hooks.h"

#include "stm32h7xx_hal.h"   /* HAL_Delay during init */

#include <string.h>

#if defined(__has_include)
#  if __has_include("FreeRTOS.h")
#    include "FreeRTOS.h"
#    include "task.h"
#    define MAG_HAVE_FREERTOS 1
#  endif
#endif

/* -------------------------------------------------------------------------- */
/* Register map (MEMSIC MMC5983MA datasheet rev D)                              */
/* -------------------------------------------------------------------------- */

#define MMC_REG_XOUT_0          0x00    /* X[17:10] */
#define MMC_REG_XOUT_1          0x01    /* X[9:2]   */
#define MMC_REG_YOUT_0          0x02
#define MMC_REG_YOUT_1          0x03
#define MMC_REG_ZOUT_0          0x04
#define MMC_REG_ZOUT_1          0x05
#define MMC_REG_XYZ_OUT_2       0x06    /* X[1:0]:Y[1:0]:Z[1:0]:0:0 */
#define MMC_REG_TOUT            0x07    /* 0.8 °C/LSB, offset -75 °C */
#define MMC_REG_STATUS          0x08
#define MMC_REG_INTERNAL_CTRL_0 0x09
#define MMC_REG_INTERNAL_CTRL_1 0x0A
#define MMC_REG_INTERNAL_CTRL_2 0x0B
#define MMC_REG_INTERNAL_CTRL_3 0x0C
#define MMC_REG_PRODUCT_ID      0x2F

#define MMC_PRODUCT_ID_VALUE    0x30

/* STATUS bits */
#define MMC_STATUS_MEAS_M_DONE   (1U << 0)
#define MMC_STATUS_MEAS_T_DONE   (1U << 1)
#define MMC_STATUS_OTP_READ_DONE (1U << 4)

/* INTERNAL_CTRL_0 bits */
#define MMC_CTRL0_TM_M             (1U << 0)
#define MMC_CTRL0_TM_T             (1U << 1)
#define MMC_CTRL0_INT_MEAS_DONE_EN (1U << 2)
#define MMC_CTRL0_SET              (1U << 3)
#define MMC_CTRL0_RESET            (1U << 4)
#define MMC_CTRL0_AUTO_SR_EN       (1U << 5)
#define MMC_CTRL0_OTP_READ         (1U << 6)

/* INTERNAL_CTRL_1 — bandwidth + soft-reset */
#define MMC_CTRL1_BW_100HZ      0x00    /* 8 ms / sample */
#define MMC_CTRL1_BW_200HZ      0x01    /* 4 ms — what we use */
#define MMC_CTRL1_BW_400HZ      0x02
#define MMC_CTRL1_BW_800HZ      0x03    /* 0.5 ms */
#define MMC_CTRL1_SW_RST        (1U << 7)

/* INTERNAL_CTRL_2 — continuous-mode bits (we leave OFF for single-shot) */
#define MMC_CTRL2_CMM_EN        (1U << 3)

/* Scale: 18-bit unsigned, centered at 2^17, full-scale ±8 gauss
 *        → 16 G / 2^18 = 6.1035e-5 G/LSB ≈ 0.0610 mG/LSB.
 * Datasheet sensitivity 16384 LSB/G → 1/16384 G/LSB = 6.1035e-5. ✓ */
#define MMC_CENTER_CODE         (1L << 17)
#define MMC_LSB_TO_GAUSS        (16.0f / 262144.0f)

/* SPI read framing: top bit of address = 1 for read. */
#define MMC_SPI_READ_BIT        0x80

/* Datasheet timings:
 *   - SW_RST settle: 10 ms
 *   - SET / RESET coil pulse + settling: ~500 ns; we wait 1 measurement
 *     period anyway between the pulse and the kick of TM_M, so no extra
 *     delay needed.
 *   - At BW=200 Hz a measurement completes in ~4 ms; we conservatively
 *     bound the per-half wait at 20 ms before declaring a fault. */
#define MMC_SOFT_RESET_DELAY_MS   10U
#define MMC_MEAS_TIMEOUT_TICKS    pdMS_TO_TICKS(20U)

/* -------------------------------------------------------------------------- */
/* Driver state                                                                */
/* -------------------------------------------------------------------------- */

#define MAG_RING_SIZE 8

/* Settle time between a SET/RESET degauss pulse and the measurement trigger.
 * The magnetization must complete before TM_M or the two halves read the same
 * state and the differenced field collapses to ~0. SparkFun's driver uses
 * ~1 ms; we match that. */
#ifndef MAG_SETRESET_SETTLE_MS
#define MAG_SETRESET_SETTLE_MS 1U
#endif

typedef enum {
    MAG_PHASE_AWAIT_SET = 0,    /* next DRDY captures M_set    */
    MAG_PHASE_AWAIT_RESET,      /* next DRDY captures M_reset  */
} mag_phase_t;

struct mag_mmc5983 {
    io_task_handle_t notify_task;   /* downstream task (state est) */
    uint32_t         notify_bit;

    /* Live SET/RESET state machine. */
    mag_phase_t      phase;
    int32_t          m_set[3];      /* last raw SET measurement (centered) */

    /* Most recent computed offset, exposed for diagnostic / EKF init. */
    float            offset_g[3];
    bool             offset_valid;

    /* Output ring + task plumbing. */
    mag_sample_t     ring[MAG_RING_SIZE];
    volatile uint8_t head, tail;

#ifdef MAG_HAVE_FREERTOS
    TaskHandle_t     service_task;  /* the mag worker we spawn */
    StaticTask_t     service_tcb;
    StackType_t      service_stack[512];  /* 2 KB — plenty for SPI + math */
#endif

    bool             ready;
};

static struct mag_mmc5983 s_self;

IO_TEST_HOOK_RW(s_self, struct mag_mmc5983, dev_mag_mmc5983_self)

/* -------------------------------------------------------------------------- */
/* SPI helpers (always called from task context, never ISR)                    */
/* -------------------------------------------------------------------------- */

static bool reg_write(uint8_t addr, uint8_t value) {
    uint8_t tx[2] = { (uint8_t)(addr & 0x7F), value };
    return io_spi_xfer_blocking(&IO_SPI_MMC, tx, NULL, sizeof(tx), 10) == IO_OK;
}

static bool reg_read(uint8_t addr, uint8_t *out, uint16_t n) {
    uint8_t tx[1 + 8] = { (uint8_t)(addr | MMC_SPI_READ_BIT) };
    uint8_t rx[1 + 8];
    if (n + 1U > sizeof(rx)) return false;
    if (io_spi_xfer_blocking(&IO_SPI_MMC, tx, rx, (uint16_t)(n + 1U), 10) != IO_OK) {
        return false;
    }
    memcpy(out, &rx[1], n);
    return true;
}

/* -------------------------------------------------------------------------- */
/* Sample plumbing                                                             */
/* -------------------------------------------------------------------------- */

static inline bool ring_full(const mag_mmc5983_t *d) {
    return (uint8_t)((d->head + 1U) % MAG_RING_SIZE) == d->tail;
}

static void notify_downstream(mag_mmc5983_t *d) {
#ifdef MAG_HAVE_FREERTOS
    if (d->notify_task && d->notify_bit) {
        xTaskNotify((TaskHandle_t)d->notify_task, d->notify_bit, eSetBits);
    }
#else
    (void)d;
#endif
}

/* Assemble an 18-bit unsigned measurement from bytes 0..6 of the output
 * block, return signed (centered at 2^17 → subtract). */
static int32_t assemble_axis(uint8_t out0, uint8_t out1, uint8_t lo2_bits) {
    uint32_t raw = ((uint32_t)out0 << 10) |
                   ((uint32_t)out1 << 2)  |
                   ((uint32_t)lo2_bits & 0x3U);
    return (int32_t)raw - (int32_t)MMC_CENTER_CODE;
}

static bool read_xyzt(int32_t *rx_out, int32_t *ry_out, int32_t *rz_out,
                      uint8_t *temp_out) {
    uint8_t buf[8];
    if (!reg_read(MMC_REG_XOUT_0, buf, sizeof(buf))) return false;
    *rx_out = assemble_axis(buf[0], buf[1], (uint8_t)((buf[6] >> 6) & 0x3));
    *ry_out = assemble_axis(buf[2], buf[3], (uint8_t)((buf[6] >> 4) & 0x3));
    *rz_out = assemble_axis(buf[4], buf[5], (uint8_t)((buf[6] >> 2) & 0x3));
    *temp_out = buf[7];
    return true;
}

/* Issue a SET/RESET degauss pulse, then start a magnetic measurement with the
 * measurement-done interrupt enabled.
 *
 * These MUST be two separate CTRL_0 writes. Combining the SET/RESET pulse bit
 * with TM_M in one write fires the degauss coil but the measurement never
 * runs — so the chip never asserts INT and DRDY never fires (observed: the
 * service task stalls in AWAIT_SET, no samples). CTRL_0 action bits (SET,
 * RESET, TM_M) are one-shot / self-clearing, so each is written fresh.
 *
 * A settle delay between the pulse and TM_M is REQUIRED: firing the
 * measurement immediately reads the pre-magnetization state, so the SET and
 * RESET halves come back with the same polarity and the differenced field
 * collapses to ~0 (verified on hardware: without the delay M_set ~= M_reset
 * and measurements also intermittently time out; with a 1 ms delay the halves
 * flip polarity and the field is real). */
static bool trigger_with_pulse(uint8_t pulse_bit) {
    /* 0. Clear the previous Meas_M_Done. Per the datasheet TM_M already turns
     *    this bit to 0 on the next measurement, so this write-1 is belt-and-
     *    suspenders — it also deasserts the INT-pin latch, keeping things
     *    clean if the DRDY interrupt is ever re-enabled. Cheap insurance. */
    (void)reg_write(MMC_REG_STATUS, MMC_STATUS_MEAS_M_DONE);
    /* 1. Degauss pulse alone. */
    if (!reg_write(MMC_REG_INTERNAL_CTRL_0, pulse_bit)) {
        return false;
    }
    /* 1b. Let the SET/RESET coil operation complete BEFORE measuring. The
     *     magnetization must settle or the measurement reads the pre-pulse
     *     state and SET/RESET come back identical (field differences to ~0).
     *     Known-good drivers (SparkFun) wait ~1 ms here. */
    vTaskDelay(pdMS_TO_TICKS(MAG_SETRESET_SETTLE_MS));
    /* 2. Start the measurement + enable the done-interrupt → DRDY on complete. */
    return reg_write(MMC_REG_INTERNAL_CTRL_0,
                     (uint8_t)(MMC_CTRL0_INT_MEAS_DONE_EN | MMC_CTRL0_TM_M));
}

/* -------------------------------------------------------------------------- */
/* EXTI handler — runs in ISR context, just nudges the service task            */
/* -------------------------------------------------------------------------- */

#define MAG_NOTIFY_DRDY  (1U << 0)

static void on_drdy(void *user) {
#ifdef MAG_HAVE_FREERTOS
    mag_mmc5983_t *d = user;
    if (d->service_task == NULL) return;
    BaseType_t woken = pdFALSE;
    xTaskNotifyFromISR(d->service_task, MAG_NOTIFY_DRDY, eSetBits, &woken);
    portYIELD_FROM_ISR(woken);
#else
    (void)user;
#endif
}

/* -------------------------------------------------------------------------- */
/* Service task — owns the SET/RESET state machine                             */
/* -------------------------------------------------------------------------- */

#ifdef MAG_HAVE_FREERTOS
/* Wait for the magnetic measurement to finish by polling STATUS.Meas_M_Done.
 * The MMC5983's DRDY/INT pin was not producing edges on this board (the
 * service task stalled forever waiting on the EXTI), so we poll the status
 * bit instead — the part is slow (~4 ms/measurement at BW=200 Hz), so a 1 ms
 * poll costs little. Returns true once done, false on timeout. */
static bool wait_meas_done(uint32_t timeout_ms) {
    for (uint32_t i = 0; i <= timeout_ms; ++i) {
        uint8_t st = 0;
        if (reg_read(MMC_REG_STATUS, &st, 1) && (st & MMC_STATUS_MEAS_M_DONE)) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return false;
}

static void mag_service_task(void *arg) {
    mag_mmc5983_t *d = (mag_mmc5983_t *)arg;

    /* Start the state machine on the SET half. */
    d->phase = MAG_PHASE_AWAIT_SET;
    if (!trigger_with_pulse(MMC_CTRL0_SET)) {
        /* Init-time SPI failure: stall here so an external watchdog can
         * recover. */
        for (;;) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }

    for (;;) {
        if (!wait_meas_done(20U)) {
            /* Measurement didn't finish in the budget. Retry from the SET
             * half so we don't get stuck mid-cycle. */
            d->phase = MAG_PHASE_AWAIT_SET;
            (void)trigger_with_pulse(MMC_CTRL0_SET);
            continue;
        }

        int32_t rx, ry, rz;
        uint8_t tempb;
        if (!read_xyzt(&rx, &ry, &rz, &tempb)) {
            /* Bad SPI — reset state machine and try again. */
            d->phase = MAG_PHASE_AWAIT_SET;
            (void)trigger_with_pulse(MMC_CTRL0_SET);
            continue;
        }

        if (d->phase == MAG_PHASE_AWAIT_SET) {
            d->m_set[0] = rx;
            d->m_set[1] = ry;
            d->m_set[2] = rz;
            d->phase    = MAG_PHASE_AWAIT_RESET;
            (void)trigger_with_pulse(MMC_CTRL0_RESET);
            continue;
        }

        /* AWAIT_RESET — we have both halves. Difference cancels offset. */
        int32_t diff_x = (d->m_set[0] - rx) / 2;
        int32_t diff_y = (d->m_set[1] - ry) / 2;
        int32_t diff_z = (d->m_set[2] - rz) / 2;
        int32_t off_x  = (d->m_set[0] + rx) / 2;
        int32_t off_y  = (d->m_set[1] + ry) / 2;
        int32_t off_z  = (d->m_set[2] + rz) / 2;

        d->offset_g[0]  = (float)off_x * MMC_LSB_TO_GAUSS;
        d->offset_g[1]  = (float)off_y * MMC_LSB_TO_GAUSS;
        d->offset_g[2]  = (float)off_z * MMC_LSB_TO_GAUSS;
        d->offset_valid = true;

        if (!ring_full(d)) {
            mag_sample_t *s = &d->ring[d->head];
            s->t_us   = io_timestamp_us();
            s->mx     = (float)diff_x * MMC_LSB_TO_GAUSS;
            s->my     = (float)diff_y * MMC_LSB_TO_GAUSS;
            s->mz     = (float)diff_z * MMC_LSB_TO_GAUSS;
            s->temp_c = ((float)tempb * 0.8f) - 75.0f;
            __atomic_thread_fence(__ATOMIC_SEQ_CST);
            d->head = (uint8_t)((d->head + 1U) % MAG_RING_SIZE);
            notify_downstream(d);
        }

        /* Loop back: next pair starts with SET again. */
        d->phase = MAG_PHASE_AWAIT_SET;
        (void)trigger_with_pulse(MMC_CTRL0_SET);
    }
}
#endif /* MAG_HAVE_FREERTOS */

/* -------------------------------------------------------------------------- */
/* Public API                                                                  */
/* -------------------------------------------------------------------------- */

bool mag_mmc5983_init(void) {
    memset(&s_self, 0, sizeof(s_self));

    /* 1. Software reset. Datasheet specifies 10 ms settle. HAL_Delay
     *    is the right call here: it uses SysTick (set up by HAL_Init
     *    before any of our code runs), works pre-scheduler, blocks
     *    accurately. */
    if (!reg_write(MMC_REG_INTERNAL_CTRL_1, MMC_CTRL1_SW_RST)) return false;
    HAL_Delay(MMC_SOFT_RESET_DELAY_MS);

    /* 2. Product ID sanity. */
    uint8_t pid = 0;
    if (!reg_read(MMC_REG_PRODUCT_ID, &pid, 1)) return false;
    if (pid != MMC_PRODUCT_ID_VALUE) return false;

    /* 3. Bandwidth → 200 Hz (4 ms per measurement). With SET/RESET
     *    differencing this gives 100 Hz field samples — the sweet spot
     *    for inertial heading correction. */
    if (!reg_write(MMC_REG_INTERNAL_CTRL_1, MMC_CTRL1_BW_200HZ)) return false;

    /* 4. CTRL_2 stays at 0 — continuous mode OFF, no auto periodic SET.
     *    The service task drives SET / RESET / TM_M manually. */
    if (!reg_write(MMC_REG_INTERNAL_CTRL_2, 0)) return false;

    /* 5. EXTI hookup. We register the callback now; no DRDY can fire yet
     *    because single-shot mode issues no measurement until the service
     *    task kicks the first SET pulse (see mag_mmc5983_start). The
     *    on_drdy ISR also guards on service_task == NULL, so an early edge
     *    would be ignored regardless. */
    io_exti_register(&IO_EXTI_MMC_INT, on_drdy, &s_self);
    io_exti_enable  (&IO_EXTI_MMC_INT, true);

    /* NOTE: the SET/RESET service task is deliberately NOT created here.
     * mag_mmc5983_init() runs in the DEV phase, before the FreeRTOS
     * scheduler starts, and xTaskCreateStatic() takes a critical section.
     * FreeRTOS's uxCriticalNesting is 0xaaaaaaaa until the scheduler
     * starts, so that critical section raises BASEPRI to
     * configMAX_SYSCALL_INTERRUPT_PRIORITY and never lowers it — which
     * masks the TIM5 HAL-timebase IRQ and freezes HAL_GetTick(). Any
     * later pre-scheduler code that blocks on a HAL timeout (e.g. the
     * Dynamixel bus init's HAL_UART_Receive) would then hang forever.
     * The task is created by mag_mmc5983_start() from app_init, in the
     * same window as every other task, right before the scheduler runs. */

    s_self.ready = true;
    return true;
}

bool mag_mmc5983_start(void) {
#ifdef MAG_HAVE_FREERTOS
    if (!s_self.ready) return false;              /* init must have run */
    if (s_self.service_task != NULL) return true; /* idempotent */
    /* Priority 5 — same neighborhood as state estimation, so DRDY
     * handling doesn't get starved by lower-priority work. */
    s_self.service_task = xTaskCreateStatic(
        mag_service_task, "mag", 512, &s_self, 5,
        s_self.service_stack, &s_self.service_tcb);
    return s_self.service_task != NULL;
#else
    return true;
#endif
}

mag_mmc5983_t *mag_mmc5983_get(void) {
    return s_self.ready ? &s_self : NULL;
}

size_t mag_mmc5983_drain(mag_mmc5983_t *d, mag_sample_t *out, size_t max) {
    if (!d || !out) return 0;
    size_t n = 0;
    while (n < max && d->tail != d->head) {
        out[n++] = d->ring[d->tail];
        d->tail = (uint8_t)((d->tail + 1U) % MAG_RING_SIZE);
    }
    return n;
}

void mag_mmc5983_set_notify(mag_mmc5983_t *d, io_task_handle_t t, uint32_t bit) {
    if (!d) return;
    d->notify_task = t;
    d->notify_bit  = bit;
}

bool mag_mmc5983_get_offset(mag_mmc5983_t *d, float out_g[3]) {
    if (!d || !out_g || !d->offset_valid) return false;
    /* offset_g is updated atomically from the service task — three scalar
     * float writes; the consumer can see a transiently mixed-axis snapshot.
     * Acceptable for the cal-complete persistence use case (we save once
     * per boot, off the critical path). */
    out_g[0] = d->offset_g[0];
    out_g[1] = d->offset_g[1];
    out_g[2] = d->offset_g[2];
    return true;
}
