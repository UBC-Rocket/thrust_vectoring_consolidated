/**
 * @file    dev_imu_icm40609.c
 * @brief   ICM-40609 driver. Opaque singleton.
 *
 * UBC Rocket, 2026
 */
#include "dev_imu_icm40609.h"

#include "io/io_spi.h"
#include "io/io_exti.h"
#include "io_sys/io_timestamp.h"
#include "io_sys/io_test_hooks.h"

#include "sensors/icm40609.h"

#include <string.h>

#if defined(__has_include)
#  if __has_include("FreeRTOS.h")
#    include "FreeRTOS.h"
#    include "task.h"
#    define ICM_HAVE_FREERTOS 1
#  endif
#endif

#define ICM_DATA_BYTES 12

/* Bring-up configuration. Flight default: ±16 g / ±2000 dps, both at 1 kHz in
 * low-noise mode. 1 kHz < 4 kHz, so the INT_CONFIG1 default path applies. */
#define ICM_ACCEL_FS        ICM40609_ACCEL_FS_16G
#define ICM_GYRO_FS         ICM40609_GYRO_FS_2000DPS
#define ICM_ODR             ICM40609_ODR_1000_HZ
#define ICM_SPI_TIMEOUT_MS  100u

struct imu_icm40609 {
    icm40609_t              dev;
    icm40609_sample_queue_t q;
    io_task_handle_t        notify_task;
    uint32_t                notify_bit;
    bool                    ready;
};

static struct imu_icm40609 s_self;

/* Test-only access to module-private state. No-op in production builds. */
IO_TEST_HOOK_RW(s_self, struct imu_icm40609, dev_imu_icm40609_self)
static uint8_t s_tx[1 + ICM_DATA_BYTES];
static uint8_t s_rx[1 + ICM_DATA_BYTES];

static void notify_isr(imu_icm40609_t *d) {
#ifdef ICM_HAVE_FREERTOS
    if (d->notify_task && d->notify_bit) {
        BaseType_t woken = pdFALSE;
        xTaskNotifyFromISR((TaskHandle_t)d->notify_task, d->notify_bit, eSetBits, &woken);
        portYIELD_FROM_ISR(woken);
    }
#else
    (void)d;
#endif
}

static void on_done(const io_spi_xfer_t *x, void *user, io_status_t st) {
    if (st != IO_OK) return;
    imu_icm40609_t *d = user;
    icm40609_sample_t s = {0};
    if (!icm40609_parse_accel_gyro(x->rx, &s, &d->dev)) return;
    s.t_us = x->t_sample;
    icm40609_queue(&d->q, &s);
    notify_isr(d);
}

static void on_drdy(void *user) {
    imu_icm40609_t *d = user;
    s_tx[0] = (uint8_t)(ICM40609_ACCEL_DATA_X1 | 0x80);
    io_spi_xfer_t x = {
        .tx = s_tx, .rx = s_rx, .len = (uint16_t)(1 + ICM_DATA_BYTES),
        .done = on_done, .user = d,
        .t_sample = io_timestamp_us(),
    };
    io_spi_submit(&IO_SPI_ICM40609, &x);
}

/* ---- Blocking-init SPI helpers (CS framed by the IO layer / HW NSS) ---- */

/* Busy-wait on the shared monotonic timer. init() runs once at boot, before
 * this core's scheduler is loaded, so a spin is acceptable — and it keeps the
 * driver free of HAL_Delay / SysTick assumptions. */
static void icm_delay_us(uint32_t us) {
    uint64_t start = io_timestamp_us();
    while ((io_timestamp_us() - start) < (uint64_t)us) { /* spin */ }
}

static bool icm_write(const uint8_t *tx, uint16_t len) {
    return io_spi_xfer_blocking(&IO_SPI_ICM40609, tx, NULL, len,
                                ICM_SPI_TIMEOUT_MS) == IO_OK;
}

/* Single-register read: address byte + one dummy; data lands in rx[1]. */
static bool icm_read_reg(uint8_t reg, uint8_t *out) {
    uint8_t tx[2] = { (uint8_t)(reg | 0x80), 0x00 };
    uint8_t rx[2] = { 0 };
    if (io_spi_xfer_blocking(&IO_SPI_ICM40609, tx, rx, 2,
                             ICM_SPI_TIMEOUT_MS) != IO_OK) {
        return false;
    }
    *out = rx[1];
    return true;
}

bool imu_icm40609_init(void) {
    memset(&s_self, 0, sizeof(s_self));
    icm40609_t *dev = &s_self.dev;
    uint8_t tx[4];
    uint8_t reg;

    /* 1. Soft reset — DS-000330 §10.1.1 needs >=1 ms before the part responds. */
    icm40609_build_softreset(tx);
    if (!icm_write(tx, 2)) return false;
    icm_delay_us(1000);

    /* 2. Force Bank 0 (soft reset already selects it; belt-and-suspenders). */
    icm40609_build_bank_select(0, tx);
    if (!icm_write(tx, 2)) return false;
    icm_delay_us(100);

    /* 3. SPI slew-rate (DRIVE_CONFIG, §14.4). */
    icm40609_build_write_reg(ICM40609_DRIVE_CONFIG, ICM40609_DRIVE_CONFIG_SLEW, tx);
    if (!icm_write(tx, 2)) return false;
    icm_delay_us(100);

    /* 4. Identity check — bail rather than configure an unknown part. */
    if (!icm_read_reg(ICM40609_WHO_AM_I, &reg)) return false;
    if (reg != ICM40609_WHO_AM_I_VAL)           return false;

    /* 5. Lock big-endian sensor data: the lib parsers assume be16(). */
    icm40609_build_write_reg(ICM40609_INTF_CONFIG0, ICM40609_INTF_CONFIG0_BE, tx);
    if (!icm_write(tx, 2)) return false;
    icm_delay_us(100);

    /* 6. Clear INT_ASYNC_RESET (reset value 1) — required for INT1 (§13.42).
     *    ODR is 1 kHz (< 4 kHz), so the default 100µs pulse path is fine. */
    icm40609_build_write_reg(ICM40609_INT_CONFIG1, ICM40609_INT_CONFIG1_DEFAULT, tx);
    if (!icm_write(tx, 2)) return false;
    icm_delay_us(100);

    /* 7. Accelerometer FS + ODR, then read back to confirm it latched. */
    icm40609_build_accel_conf(ICM_ACCEL_FS, ICM_ODR, tx, dev);
    if (!icm_write(tx, 2)) return false;
    icm_delay_us(100);
    if (!icm_read_reg(ICM40609_ACCEL_CONFIG0, &reg) || reg != dev->accel_config0)
        return false;

    /* 8. Gyroscope FS + ODR, then read back. */
    icm40609_build_gyro_conf(ICM_GYRO_FS, ICM_ODR, tx, dev);
    if (!icm_write(tx, 2)) return false;
    icm_delay_us(100);
    if (!icm_read_reg(ICM40609_GYRO_CONFIG0, &reg) || reg != dev->gyro_config0)
        return false;

    /* 9. Power on accel+gyro (low-noise), verify, then wait out gyro startup
     *    (>=45 ms OFF->LN, plus >=200 µs settle after the PWR_MGMT0 write). */
    icm40609_build_power(ICM40609_PWR_LOW_NOISE, tx, dev);
    if (!icm_write(tx, 2)) return false;
    icm_delay_us(300);
    if (!icm_read_reg(ICM40609_PWR_MGMT0, &reg)
        || (reg & 0x0F) != (uint8_t)(ICM40609_PWR_LOW_NOISE & 0x0F))
        return false;
    icm_delay_us(45000);

    /* 10. Route DATA_READY to INT1. The builder packs INT_CONFIG and
     *     INT_SOURCE0 as two back-to-back 2-byte writes; each register write
     *     needs its own CS frame, so submit them one at a time. */
    {
        uint8_t intcfg[4];
        size_t  n = icm40609_build_int1_data_ready(intcfg);
        for (size_t i = 0; i + 1 < n; i += 2) {
            if (!icm_write(&intcfg[i], 2)) return false;
            icm_delay_us(100);
        }
    }

    /* 11. Hand the DATA_READY line to the EXTI dispatcher; runtime path is now
     *     interrupt-driven (on_drdy -> async burst read -> queue -> notify). */
    io_exti_register(&IO_EXTI_ICM40609_INT1, on_drdy, &s_self);
    io_exti_enable(&IO_EXTI_ICM40609_INT1, true);

    s_self.ready = true;
    return true;
}

imu_icm40609_t *imu_icm40609_get(void) {
    return s_self.ready ? &s_self : NULL;
}

size_t imu_icm40609_drain(imu_icm40609_t *d, icm_sample_t *out, size_t max) {
    size_t n = 0;
    icm40609_sample_t s;
    while (n < max && icm40609_dequeue(&d->q, &s)) {
        out[n].t_us   = s.t_us;
        out[n].ax     = s.ax;  out[n].ay = s.ay;  out[n].az = s.az;
        out[n].gx     = s.gx;  out[n].gy = s.gy;  out[n].gz = s.gz;
        out[n].temp_c = s.temp_c;
        ++n;
    }
    return n;
}

void imu_icm40609_set_notify(imu_icm40609_t *d, io_task_handle_t t, uint32_t bit) {
    d->notify_task = t;
    d->notify_bit  = bit;
}
