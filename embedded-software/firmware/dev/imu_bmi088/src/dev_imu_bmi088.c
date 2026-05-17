/**
 * @file    dev_imu_bmi088.c
 * @brief   BMI088 driver. Opaque singleton; IO bindings baked in.
 *
 * UBC Rocket, 2026
 */
#include "dev_imu_bmi088.h"

#include "io/io_spi.h"
#include "io/io_exti.h"
#include "io_sys/io_timestamp.h"
#include "io_sys/io_test_hooks.h"

#include "sensors/bmi088_accel.h"
#include "sensors/bmi088_gyro.h"

#include <string.h>

#if defined(__has_include)
#  if __has_include("FreeRTOS.h")
#    include "FreeRTOS.h"
#    include "task.h"
#    define IMU_BMI088_HAVE_FREERTOS 1
#  endif
#endif

#define BMI_RD(reg)   ((uint8_t)((reg) | 0x80))

struct imu_bmi088 {
    bmi088_accel_t              acc_dev;
    bmi088_gyro_t               gyro_dev;
    bmi088_accel_sample_queue_t acc_q;
    bmi088_gyro_sample_queue_t  gyro_q;
    io_task_handle_t            notify_task;
    uint32_t                    notify_bit;
    bool                        ready;
};

/* Singleton storage + scratch. */
static struct imu_bmi088 s_self;
static uint8_t s_acc_tx[8];
static uint8_t s_acc_rx[8];
static uint8_t s_gyro_tx[7];
static uint8_t s_gyro_rx[7];

/* Test-only access to module-private state. No-op in production builds. */
IO_TEST_HOOK_RW   (s_self,    struct imu_bmi088, dev_imu_bmi088_self)
IO_TEST_HOOK_ARRAY(s_acc_tx,  uint8_t,           dev_imu_bmi088_acc_tx)
IO_TEST_HOOK_ARRAY(s_acc_rx,  uint8_t,           dev_imu_bmi088_acc_rx)
IO_TEST_HOOK_ARRAY(s_gyro_tx, uint8_t,           dev_imu_bmi088_gyro_tx)
IO_TEST_HOOK_ARRAY(s_gyro_rx, uint8_t,           dev_imu_bmi088_gyro_rx)

static void notify_isr(imu_bmi088_t *d) {
#ifdef IMU_BMI088_HAVE_FREERTOS
    if (d->notify_task && d->notify_bit) {
        BaseType_t woken = pdFALSE;
        xTaskNotifyFromISR((TaskHandle_t)d->notify_task, d->notify_bit, eSetBits, &woken);
        portYIELD_FROM_ISR(woken);
    }
#else
    (void)d;
#endif
}

static void on_acc_done(const io_spi_xfer_t *x, void *user, io_status_t st) {
    if (st != IO_OK) return;
    imu_bmi088_t *d = user;
    bmi088_accel_sample_t s = {0};
    if (!bmi088_accel_parse_data_xyz(&x->rx[2], &s, &d->acc_dev)) return;
    s.t_us = x->t_sample;
    bmi088_acc_sample_queue(&d->acc_q, &s);
    notify_isr(d);
}

static void on_gyro_done(const io_spi_xfer_t *x, void *user, io_status_t st) {
    if (st != IO_OK) return;
    imu_bmi088_t *d = user;
    bmi088_gyro_sample_t s = {0};
    if (!bmi088_gyro_parse_data_xyz(&x->rx[1], &s, &d->gyro_dev)) return;
    s.t_us = x->t_sample;
    bmi088_gyro_sample_queue(&d->gyro_q, &s);
    notify_isr(d);
}

static void on_acc_drdy(void *user) {
    imu_bmi088_t *d = user;
    s_acc_tx[0] = BMI_RD(BMI088_ACC_DATA_START);
    io_spi_xfer_t x = {
        .tx = s_acc_tx, .rx = s_acc_rx, .len = 8,
        .done = on_acc_done, .user = d,
        .t_sample = io_timestamp_us(),
    };
    io_spi_submit(&IO_SPI_BMI088_ACC, &x);
}

static void on_gyro_drdy(void *user) {
    imu_bmi088_t *d = user;
    s_gyro_tx[0] = BMI_RD(BMI088_GYRO_RATE_X_LSB);
    io_spi_xfer_t x = {
        .tx = s_gyro_tx, .rx = s_gyro_rx, .len = 7,
        .done = on_gyro_done, .user = d,
        .t_sample = io_timestamp_us(),
    };
    io_spi_submit(&IO_SPI_BMI088_GYRO, &x);
}

bool imu_bmi088_init(void) {
    memset(&s_self, 0, sizeof(s_self));

    /* TODO: bring-up via lib/sensors/ command builders +
     *       io_spi_xfer_blocking(&IO_SPI_BMI088_{ACC,GYRO}, …). */

    io_exti_register(&IO_EXTI_BMI088_ACC_INT1,  on_acc_drdy,  &s_self);
    io_exti_register(&IO_EXTI_BMI088_GYRO_INT1, on_gyro_drdy, &s_self);
    io_exti_enable(&IO_EXTI_BMI088_ACC_INT1,  true);
    io_exti_enable(&IO_EXTI_BMI088_GYRO_INT1, true);

    s_self.ready = true;
    return true;
}

imu_bmi088_t *imu_bmi088_get(void) {
    return s_self.ready ? &s_self : NULL;
}

size_t imu_bmi088_drain(imu_bmi088_t *d, imu_sample_t *out, size_t max) {
    size_t n = 0;
    bmi088_accel_sample_t a;
    bmi088_gyro_sample_t  g;
    while (n < max
           && bmi088_acc_sample_dequeue(&d->acc_q, &a)
           && bmi088_gyro_sample_dequeue(&d->gyro_q, &g)) {
        out[n].t_us = a.t_us;
        out[n].ax = a.ax; out[n].ay = a.ay; out[n].az = a.az;
        out[n].gx = g.gx; out[n].gy = g.gy; out[n].gz = g.gz;
        out[n].temp_c = 0.0f;
        ++n;
    }
    return n;
}

void imu_bmi088_set_notify(imu_bmi088_t *d, io_task_handle_t task, uint32_t bit) {
    d->notify_task = task;
    d->notify_bit  = bit;
}
