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

bool imu_icm40609_init(void) {
    memset(&s_self, 0, sizeof(s_self));
    /* TODO: bring-up via lib/sensors/icm40609 builders +
     *       io_spi_xfer_blocking(&IO_SPI_ICM40609, …). */
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
