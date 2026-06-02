/**
 * @file    dev_mag_mmc5983.c
 * @brief   MMC5983MA driver implementation.
 *
 * Register map + SPI framing taken from the MEMSIC MMC5983MA datasheet
 * (rev D, 2019). Numbers are quoted in the comments at point-of-use.
 *
 * SPI framing:
 *   - 4-wire (CTRL_3.Spi_3wire = 0 — chip default).
 *   - 1st byte: { RW:1, ADDR:7 } where RW=1 is read.
 *   - Subsequent bytes are data; address auto-increments on read.
 *
 * Sampling strategy:
 *   - Continuous Measurement Mode (CMM) at 100 Hz.
 *   - Auto SET/RESET enabled (CTRL_2.PRD_SET = 0b001 = every measurement)
 *     so the demagnetisation pulse fires before each measurement and
 *     keeps offset bounded — the chip's recommended config for inertial
 *     applications.
 *   - DRDY (INT pin, level-high when conversion complete) is wired
 *     through IO_EXTI_MMC_INT; ISR submits an async SPI read and pushes
 *     the parsed sample to the ring.
 *
 * UBC Rocket, 2026
 */
#include "dev_mag_mmc5983.h"

#include "io/io_spi.h"
#include "io/io_exti.h"
#include "io_sys/io_timestamp.h"
#include "io_sys/io_test_hooks.h"

#include <string.h>

#if defined(__has_include)
#  if __has_include("FreeRTOS.h")
#    include "FreeRTOS.h"
#    include "task.h"
#    define MAG_HAVE_FREERTOS 1
#  endif
#endif

/* -------------------------------------------------------------------------- */
/* Register map                                                               */
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
#define MMC_REG_PRODUCT_ID      0x2F    /* must read 0x30 */

#define MMC_PRODUCT_ID_VALUE    0x30

/* STATUS bits */
#define MMC_STATUS_MEAS_T_DONE   (1U << 1)
#define MMC_STATUS_MEAS_M_DONE   (1U << 0)
#define MMC_STATUS_OTP_READ_DONE (1U << 4)

/* INTERNAL_CTRL_0 bits */
#define MMC_CTRL0_TM_M          (1U << 0)
#define MMC_CTRL0_TM_T          (1U << 1)
#define MMC_CTRL0_INT_MEAS_DONE_EN (1U << 2)
#define MMC_CTRL0_SET           (1U << 3)
#define MMC_CTRL0_RESET         (1U << 4)
#define MMC_CTRL0_AUTO_SR_EN    (1U << 5)
#define MMC_CTRL0_OTP_READ      (1U << 6)

/* INTERNAL_CTRL_1 — bandwidth + soft-reset */
#define MMC_CTRL1_BW_100HZ      0x00    /* 8 ms measurement */
#define MMC_CTRL1_BW_200HZ      0x01
#define MMC_CTRL1_BW_400HZ      0x02
#define MMC_CTRL1_BW_800HZ      0x03    /* 0.5 ms measurement */
#define MMC_CTRL1_SW_RST        (1U << 7)

/* INTERNAL_CTRL_2 — CMM + PRD_SET */
#define MMC_CTRL2_CM_FREQ_OFF   0x00
#define MMC_CTRL2_CM_FREQ_1HZ   0x01
#define MMC_CTRL2_CM_FREQ_10HZ  0x02
#define MMC_CTRL2_CM_FREQ_20HZ  0x03
#define MMC_CTRL2_CM_FREQ_50HZ  0x04
#define MMC_CTRL2_CM_FREQ_100HZ 0x05
#define MMC_CTRL2_CM_FREQ_200HZ 0x06
#define MMC_CTRL2_CM_FREQ_1000HZ 0x07
#define MMC_CTRL2_CMM_EN        (1U << 3)
#define MMC_CTRL2_PRD_SET_1     (0U << 4)  /* every meas */
#define MMC_CTRL2_PRD_SET_25    (1U << 4)
#define MMC_CTRL2_PRD_SET_75    (2U << 4)
#define MMC_CTRL2_PRD_SET_100   (3U << 4)
#define MMC_CTRL2_PRD_SET_250   (4U << 4)
#define MMC_CTRL2_PRD_SET_500   (5U << 4)
#define MMC_CTRL2_PRD_SET_1000  (6U << 4)
#define MMC_CTRL2_PRD_SET_2000  (7U << 4)
#define MMC_CTRL2_EN_PRD_SET    (1U << 7)

/* Scale: 18-bit signed centered at 0x20000 (= 1 << 17), full-scale ±8 G.
 * 1 LSB = 16 G / (2^18) = 6.1035e-5 G ≈ 0.0625 milligauss. */
#define MMC_CENTER_CODE         (1L << 17)
#define MMC_LSB_TO_GAUSS        (16.0f / 262144.0f)   /* 6.103515625e-5 */

/* SPI write/read framing. */
#define MMC_SPI_READ_BIT        0x80

/* -------------------------------------------------------------------------- */
/* Driver state                                                                */
/* -------------------------------------------------------------------------- */

#define MAG_RING_SIZE 8

struct mag_mmc5983 {
    io_task_handle_t notify_task;
    uint32_t         notify_bit;
    mag_sample_t     ring[MAG_RING_SIZE];
    volatile uint8_t head, tail;
    bool             ready;
};

static struct mag_mmc5983 s_self;

IO_TEST_HOOK_RW(s_self, struct mag_mmc5983, dev_mag_mmc5983_self)

/* -------------------------------------------------------------------------- */
/* SPI helpers (blocking — only used at init or from ISR-deferred work)        */
/* -------------------------------------------------------------------------- */

static bool reg_write(uint8_t addr, uint8_t value) {
    uint8_t tx[2] = { addr & 0x7F, value };
    return io_spi_xfer_blocking(&IO_SPI_MMC, tx, NULL, sizeof(tx), 10) == IO_OK;
}

static bool reg_read(uint8_t addr, uint8_t *out, uint16_t n) {
    /* Read framing: header byte then n dummy clocks for the response.
     * io_spi_xfer_blocking is full-duplex; we tx the header in the first
     * byte and ignore the corresponding rx; the device drives valid data
     * in slots 1..n. */
    uint8_t tx[1 + 7] = { (uint8_t)(addr | MMC_SPI_READ_BIT) };
    uint8_t rx[1 + 7];
    if (n + 1U > sizeof(rx)) return false;     /* covers 0x00..0x07 read */
    if (io_spi_xfer_blocking(&IO_SPI_MMC, tx, rx, (uint16_t)(n + 1U), 10) != IO_OK) {
        return false;
    }
    memcpy(out, &rx[1], n);
    return true;
}

/* -------------------------------------------------------------------------- */
/* Sample assembly                                                             */
/* -------------------------------------------------------------------------- */

static inline bool ring_full(const mag_mmc5983_t *d) {
    return ((uint8_t)(d->head + 1U) % MAG_RING_SIZE) == d->tail;
}

static void notify_isr(mag_mmc5983_t *d) {
#ifdef MAG_HAVE_FREERTOS
    if (d->notify_task && d->notify_bit) {
        BaseType_t woken = pdFALSE;
        xTaskNotifyFromISR((TaskHandle_t)d->notify_task, d->notify_bit,
                           eSetBits, &woken);
        portYIELD_FROM_ISR(woken);
    }
#else
    (void)d;
#endif
}

/* Assemble an 18-bit signed measurement from bytes 0..6 of the output
 * register block. Layout (per datasheet table 5):
 *   OUT_0 = bits [17:10]
 *   OUT_1 = bits [9:2]
 *   XYZ_OUT_2 = { X[1:0], Y[1:0], Z[1:0], 0, 0 }
 * Code is unsigned, centered at 2^17 → subtract the center for signed. */
static int32_t assemble_axis(uint8_t out0, uint8_t out1, uint8_t lo2) {
    uint32_t raw = ((uint32_t)out0 << 10) |
                   ((uint32_t)out1 << 2)  |
                   ((uint32_t)lo2  & 0x3U);
    return (int32_t)raw - (int32_t)MMC_CENTER_CODE;
}

/* Called from the EXTI ISR. Reads 8 registers (0x00..0x07) and pushes
 * one mag_sample into the ring. Blocking SPI inside an ISR is normally
 * a no-no, but at SPI1 max clock (~50 MHz) the 8-byte read takes ~2 µs,
 * which is acceptable for a 100 Hz EXTI. If we ever clock SPI1 slower
 * or run at higher rates this should be re-cast as an async submit
 * with a TxRx-done callback. */
static void on_drdy(void *user) {
    mag_mmc5983_t *d = user;
    uint8_t buf[8];
    if (!reg_read(MMC_REG_XOUT_0, buf, sizeof(buf))) return;

    int32_t rx = assemble_axis(buf[0], buf[1], (uint8_t)((buf[6] >> 6) & 0x3));
    int32_t ry = assemble_axis(buf[2], buf[3], (uint8_t)((buf[6] >> 4) & 0x3));
    int32_t rz = assemble_axis(buf[4], buf[5], (uint8_t)((buf[6] >> 2) & 0x3));

    if (ring_full(d)) return;
    mag_sample_t *s = &d->ring[d->head];
    s->t_us   = io_timestamp_us();
    s->mx     = (float)rx * MMC_LSB_TO_GAUSS;
    s->my     = (float)ry * MMC_LSB_TO_GAUSS;
    s->mz     = (float)rz * MMC_LSB_TO_GAUSS;
    s->temp_c = ((float)buf[7] * 0.8f) - 75.0f;

    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    d->head = (uint8_t)((d->head + 1U) % MAG_RING_SIZE);
    notify_isr(d);
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                  */
/* -------------------------------------------------------------------------- */

bool mag_mmc5983_init(void) {
    memset(&s_self, 0, sizeof(s_self));

    /* 1. Software reset (CTRL_1.SW_RST). Datasheet specifies a 10 ms
     *    wait after the bit goes high before the chip is responsive. */
    if (!reg_write(MMC_REG_INTERNAL_CTRL_1, MMC_CTRL1_SW_RST)) return false;
    for (volatile int i = 0; i < 200000; ++i) { __asm__ volatile (""); }

    /* 2. Verify product ID — cheap sanity that SPI is wired correctly. */
    uint8_t pid = 0;
    if (!reg_read(MMC_REG_PRODUCT_ID, &pid, 1)) return false;
    if (pid != MMC_PRODUCT_ID_VALUE) return false;

    /* 3. Bandwidth → 100 Hz family (8 ms measurement window). Trades
     *    measurement noise for max ODR. Matches the 100 Hz CMM below. */
    if (!reg_write(MMC_REG_INTERNAL_CTRL_1, MMC_CTRL1_BW_100HZ)) return false;

    /* 4. Continuous mode @ 100 Hz with auto SET every measurement.
     *    PRD_SET = 1 + EN_PRD_SET keeps the chip degaussed in flight. */
    if (!reg_write(MMC_REG_INTERNAL_CTRL_2,
                   MMC_CTRL2_CM_FREQ_100HZ
                 | MMC_CTRL2_CMM_EN
                 | MMC_CTRL2_PRD_SET_1
                 | MMC_CTRL2_EN_PRD_SET)) return false;

    /* 5. Arm the EXTI dispatch. CTRL_0.INT_MEAS_DONE_EN drives the INT
     *    pin high when a measurement is ready. */
    io_exti_register(&IO_EXTI_MMC_INT, on_drdy, &s_self);
    io_exti_enable  (&IO_EXTI_MMC_INT, true);

    if (!reg_write(MMC_REG_INTERNAL_CTRL_0, MMC_CTRL0_INT_MEAS_DONE_EN)) return false;

    s_self.ready = true;
    return true;
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
