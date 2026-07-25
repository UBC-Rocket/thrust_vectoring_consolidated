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
#include "io_sys/io_status_led.h"

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

/* Must outlive on_*_drdy(): io_spi_submit's completion callback fires
 * asynchronously from the DMA ISR, well after the *_drdy ISR (and any
 * stack-local xfer descriptor) has returned. */
static io_spi_xfer_t s_acc_xfer;
static io_spi_xfer_t s_gyro_xfer;

static void on_acc_drdy(void *user) {
    imu_bmi088_t *d = user;
    s_acc_tx[0] = BMI_RD(BMI088_ACC_DATA_START);
    s_acc_xfer = (io_spi_xfer_t){
        .tx = s_acc_tx, .rx = s_acc_rx, .len = 8,
        .done = on_acc_done, .user = d,
        .t_sample = io_timestamp_us(),
    };
    io_spi_submit(&IO_SPI_BMI088_ACC, &s_acc_xfer);
}

static void on_gyro_drdy(void *user) {
    imu_bmi088_t *d = user;
    s_gyro_tx[0] = BMI_RD(BMI088_GYRO_RATE_X_LSB);
    s_gyro_xfer = (io_spi_xfer_t){
        .tx = s_gyro_tx, .rx = s_gyro_rx, .len = 7,
        .done = on_gyro_done, .user = d,
        .t_sample = io_timestamp_us(),
    };
    io_spi_submit(&IO_SPI_BMI088_GYRO, &s_gyro_xfer);
}

/* ---- Bring-up configuration -------------------------------------------- *
 * Flight defaults: accel ±24 g @ 800 Hz (normal BW), gyro ±2000 dps @ 1 kHz.
 * Mirrors the ICM-40609's full-scale/ODR intent; both stream into the same
 * state-estimation drain. */
#define BMI_ACC_CHIP_ID_VAL   0x1Eu   /* BMI088 accel CHIP_ID (reg 0x00)      */
#define BMI_SPI_TIMEOUT_MS    100u

/* init() runs once at boot before this core's scheduler, so a spin on the
 * shared monotonic timer is acceptable and keeps the driver free of
 * HAL_Delay / SysTick assumptions (same approach as the ICM driver). */
static void bmi_delay_us(uint32_t us) {
    uint64_t start = io_timestamp_us();
    while ((io_timestamp_us() - start) < (uint64_t)us) { /* spin */ }
}

static bool acc_write(const uint8_t *tx, uint16_t len) {
    return io_spi_xfer_blocking(&IO_SPI_BMI088_ACC, tx, NULL, len,
                                BMI_SPI_TIMEOUT_MS) == IO_OK;
}
static bool gyro_write(const uint8_t *tx, uint16_t len) {
    return io_spi_xfer_blocking(&IO_SPI_BMI088_GYRO, tx, NULL, len,
                                BMI_SPI_TIMEOUT_MS) == IO_OK;
}

/* The build_*conf() helpers may emit several back-to-back 2-byte register
 * writes in one buffer (e.g. power = PWR_CONF + PWR_CTRL). Each register write
 * needs its own CS frame (no SPI auto-increment assumed), so submit one pair
 * at a time — same split the ICM driver does for its INT1 config. */
static bool acc_write_split(const uint8_t *tx, size_t n) {
    for (size_t i = 0; i + 1 < n; i += 2) {
        if (!acc_write(&tx[i], 2)) return false;
        bmi_delay_us(1000);
    }
    return true;
}
static bool gyro_write_split(const uint8_t *tx, size_t n) {
    for (size_t i = 0; i + 1 < n; i += 2) {
        if (!gyro_write(&tx[i], 2)) return false;
        bmi_delay_us(1000);
    }
    return true;
}

/* Accel single-register read: addr|0x80 + 1 dummy + 1 data → data in rx[2]. */
static bool acc_read_reg(uint8_t reg, uint8_t *out) {
    uint8_t tx[3] = { (uint8_t)(reg | 0x80), 0x00, 0x00 };
    uint8_t rx[3] = { 0 };
    if (io_spi_xfer_blocking(&IO_SPI_BMI088_ACC, tx, rx, 3,
                             BMI_SPI_TIMEOUT_MS) != IO_OK) return false;
    *out = rx[2];
    return true;
}
/* Gyro single-register read: addr|0x80 + 1 data → data in rx[1] (no dummy). */
static bool gyro_read_reg(uint8_t reg, uint8_t *out) {
    uint8_t tx[2] = { (uint8_t)(reg | 0x80), 0x00 };
    uint8_t rx[2] = { 0 };
    if (io_spi_xfer_blocking(&IO_SPI_BMI088_GYRO, tx, rx, 2,
                             BMI_SPI_TIMEOUT_MS) != IO_OK) return false;
    
    *out = rx[1];
    return true;
}

static bool bmi088_accel_bringup(bmi088_accel_t *dev) {
    uint8_t tx[8];
    uint8_t id = 0;
    
    /* BMI088 accel powers up in I2C mode; the first SPI access (a rising CS
     * edge) switches it to SPI but returns invalid data, and a soft reset
     * returns it to I2C — so issue a dummy read, reset, then dummy read again
     * before trusting any value. */
    (void)acc_read_reg(BMI088_ACC_CHIP_ID_REG, &id);   /* dummy: enter SPI    */
    bmi_delay_us(1000);
    
    bmi088_accel_build_softreset(tx);
    if (!acc_write(tx, 2)) return false;
    bmi_delay_us(2000);                                /* >=1 ms post-reset   */

    (void)acc_read_reg(BMI088_ACC_CHIP_ID_REG, &id);   /* dummy: re-enter SPI */
    bmi_delay_us(1000);
    if (!acc_read_reg(BMI088_ACC_CHIP_ID_REG, &id)) return false;
    if (id != BMI_ACC_CHIP_ID_VAL) return false;

    /* Power on: PWR_CONF=0x00 (active) then PWR_CTRL=0x04 (accel enable). */
    if (!acc_write_split(tx,
            bmi088_accel_build_power_conf(BMI088_ACC_PWR_ACTIVE, tx, dev)))
        return false;
    bmi_delay_us(50000);                               /* accel start-up      */

    bmi088_accel_build_range(BMI088_ACC_RANGE_24G, tx, dev);
    if (!acc_write(tx, 2)) return false;
    bmi_delay_us(1000);

    bmi088_accel_build_odr_bw(BMI088_ACC_ODR_800_HZ, BMI088_ACC_BWP_NORMAL,
                              tx, dev);
    if (!acc_write(tx, 2)) return false;
    bmi_delay_us(1000);

    /* DATA_READY out on INT1: push-pull, active-high, PULSED (not latched, so
     * the data burst-read need not also clear an int-status reg to re-arm).
     * Matches the MCU EXTI on PC13 = rising edge / no-pull (gpio.c). */
    bmi088_accel_build_int_pin_conf(BMI088_ACC_INT1, BMI088_ACC_INT_PUSH_PULL,
                                    BMI088_ACC_INT_ACTIVE_HIGH, false, tx);
    if (!acc_write(tx, 2)) return false;
    bmi_delay_us(1000);

    bmi088_accel_build_int_event_map(BMI088_ACC_INT1,
                                     BMI088_ACC_INT_EVENT_DATA_READY, tx);
    if (!acc_write(tx, 2)) return false;
    bmi_delay_us(1000);
    return true;
}

static bool bmi088_gyro_bringup(bmi088_gyro_t *dev) {
    uint8_t tx[8];
    uint8_t id = 0;

    /* Gyro is native-SPI (no I2C/SPI mode switch like the accel). */
    bmi088_gyro_build_softreset(tx);
    if (!gyro_write(tx, 2)) return false;
    bmi_delay_us(30000);                               /* >=30 ms post-reset  */

    if (!gyro_read_reg(BMI088_GYRO_CHIP_ID_REG, &id)) return false;
    if (id != BMI088_GYRO_CHIP_ID_VAL) return false;

    bmi088_gyro_build_range(BMI088_GYRO_RANGE_2000DPS, tx, dev);
    if (!gyro_write(tx, 2)) return false;
    bmi_delay_us(1000);

    bmi088_gyro_build_odr(BMI088_GYRO_ODR_1000_HZ_BW_230, tx, dev);
    if (!gyro_write(tx, 2)) return false;
    bmi_delay_us(1000);

    bmi088_gyro_build_power_conf(BMI088_GYRO_PWR_NORMAL, tx, dev);
    if (!gyro_write(tx, 2)) return false;
    bmi_delay_us(30000);

    /* DATA_READY out on INT3, push-pull / active-high. NOTE: the board net
     * "BMI_GYRO_INT1" (PF10) is ASSUMED to be the gyro's physical INT3 pin.
     * The hardware repo ships only the .ioc (no schematic/netlist), so this
     * could not be confirmed. If the gyro IRQ never fires on hardware, its
     * DRDY is wired to INT4 — change BMI088_GYRO_INT3 -> BMI088_GYRO_INT4 in
     * the two calls below. */
    bmi088_gyro_build_int_pin_conf(BMI088_GYRO_INT3, BMI088_GYRO_INT_PUSH_PULL,
                                   BMI088_GYRO_INT_ACTIVE_HIGH, tx);
    if (!gyro_write(tx, 2)) return false;
    bmi_delay_us(1000);

    if (!gyro_write_split(tx,
            bmi088_gyro_build_int_event_map(
                BMI088_GYRO_INT3, BMI088_GYRO_INT_EVENT_DATA_READY, tx)))
        return false;
    bmi_delay_us(1000);
    return true;
}

bool imu_bmi088_init(void) {
    memset(&s_self, 0, sizeof(s_self));

    if (!bmi088_accel_bringup(&s_self.acc_dev)) return false;
    if (!bmi088_gyro_bringup(&s_self.gyro_dev)) return false;

    /* Hand both DATA_READY lines to the EXTI dispatcher; the runtime path is
     * now interrupt-driven (on_*_drdy -> async SPI burst read -> queue ->
     * notify), no polling. */
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
    if (!d || !out) return 0;
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
    if (!d) return;
    d->notify_task = task;
    d->notify_bit  = bit;
}
