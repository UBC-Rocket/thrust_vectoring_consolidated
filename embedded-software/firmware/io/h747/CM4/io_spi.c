/**
 * @file    io_spi.c
 * @brief   H747 CM4 IO impl: SPI transport. Defines struct io_spi_dev plus
 *          the const IO_SPI_<NAME> instances that drivers extern-reference.
 *
 * UBC Rocket, 2026
 */
#include "io_common.h"
#include "io/io_spi.h"

#include "spi.h"
#include "io_spi_queue/io_spi_queue.h"

#include <string.h>

/* ------------------------------------------------------------------------- */
/* Full struct definition lives here (opaque from drivers' perspective).     */
/* ------------------------------------------------------------------------- */
struct io_spi_dev {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *cs_port;   /* NULL = use peripheral hardware NSS */
    uint16_t           cs_pin;
};

/* ------------------------------------------------------------------------- */
/* Per-physical-bus queues (private state)                                   */
/* ------------------------------------------------------------------------- */
#define SPI_DEPTH 16
static io_spi_queue_t s_q[6];
static io_spi_queue_job_t   s_q_jobs[6][SPI_DEPTH];
static bool        s_q_init[6];

static int bus_index(SPI_HandleTypeDef *hspi) {
    if (hspi == &hspi1) return 0;
    if (hspi == &hspi2) return 1;
    if (hspi == &hspi3) return 2;
    if (hspi == &hspi4) return 3;
    if (hspi == &hspi5) return 4;
    if (hspi == &hspi6) return 5;
    return -1;
}

static void ensure_queue(SPI_HandleTypeDef *hspi) {
    int idx = bus_index(hspi);
    if (idx < 0 || s_q_init[idx]) return;
    io_spi_queue_init(&s_q[idx], hspi, s_q_jobs[idx], SPI_DEPTH);
    s_q_init[idx] = true;
}

/* ------------------------------------------------------------------------- */
/* Public board-wiring symbols (referenced extern-const by drivers).         */
/* ------------------------------------------------------------------------- */
const io_spi_dev_t IO_SPI_ICM40609    = { .hspi = &hspi3, .cs_port = NULL,                       .cs_pin = 0 };
const io_spi_dev_t IO_SPI_BMI088_ACC  = { .hspi = &hspi4, .cs_port = BMI_ACC_CS_GPIO_Port,       .cs_pin = BMI_ACC_CS_Pin };
const io_spi_dev_t IO_SPI_BMI088_GYRO = { .hspi = &hspi4, .cs_port = BMI_GYRO_CS_GPIO_Port,      .cs_pin = BMI_GYRO_CS_Pin };
const io_spi_dev_t IO_SPI_MS5611      = { .hspi = &hspi2, .cs_port = NULL,                       .cs_pin = 0 };
const io_spi_dev_t IO_SPI_MMC         = { .hspi = &hspi1, .cs_port = NULL,                       .cs_pin = 0 };

/* Called from io_init_cm4(). */
void io_spi_init(void) {
    ensure_queue(IO_SPI_ICM40609.hspi);
    ensure_queue(IO_SPI_BMI088_ACC.hspi);
    ensure_queue(IO_SPI_MS5611.hspi);
    ensure_queue(IO_SPI_MMC.hspi);
}

/* ------------------------------------------------------------------------- */
/* Async submit                                                              */
/* ------------------------------------------------------------------------- */
static void async_done_shim(const io_spi_queue_job_t *job, void *user, bool ok) {
    (void)job;
    io_spi_xfer_t *x = user;
    if (x->done) x->done(x, x->user, ok ? IO_OK : IO_ERR_BUS);
}

io_status_t io_spi_submit(const io_spi_dev_t *dev, const io_spi_xfer_t *x) {
    if (!dev || !x) return IO_ERR_PARAM;
    int idx = bus_index(dev->hspi);
    if (idx < 0) return IO_ERR_NO_DEVICE;

    io_spi_queue_job_t job = {
        .cs_port  = dev->cs_port,
        .cs_pin   = dev->cs_pin,
        .tx_buf   = x->tx,
        .rx_buf   = x->rx,
        .len      = x->len,
        .type     = (x->tx && x->rx) ? IO_SPI_QUEUE_XFER_TXRX
                  : (x->tx)          ? IO_SPI_QUEUE_XFER_TX
                                     : IO_SPI_QUEUE_XFER_RX,
        .done     = async_done_shim,
        .user     = (void *)x,
        .t_sample = x->t_sample,
    };
    return io_spi_queue_submit(&s_q[idx], &job);
}

/* ------------------------------------------------------------------------- */
/* Blocking xfer                                                             */
/* ------------------------------------------------------------------------- */
static inline void cs_low (GPIO_TypeDef *p, uint16_t pin) { if (p) p->BSRR = (uint32_t)pin << 16; }
static inline void cs_high(GPIO_TypeDef *p, uint16_t pin) { if (p) p->BSRR = (uint32_t)pin;       }

io_status_t io_spi_xfer_blocking(const io_spi_dev_t *dev,
                                 const uint8_t *tx, uint8_t *rx,
                                 uint16_t len, uint32_t timeout_ms) {
    if (!dev) return IO_ERR_PARAM;
    HAL_StatusTypeDef st;
    cs_low(dev->cs_port, dev->cs_pin);
    if (tx && rx)      st = HAL_SPI_TransmitReceive(dev->hspi, (uint8_t *)tx, rx, len, timeout_ms);
    else if (tx)       st = HAL_SPI_Transmit       (dev->hspi, (uint8_t *)tx,    len, timeout_ms);
    else if (rx)       st = HAL_SPI_Receive        (dev->hspi,                rx, len, timeout_ms);
    else               st = HAL_OK;
    cs_high(dev->cs_port, dev->cs_pin);
    if (st == HAL_OK)      return IO_OK;
    if (st == HAL_TIMEOUT) return IO_ERR_TIMEOUT;
    if (st == HAL_BUSY)    return IO_ERR_BUSY;
    return IO_ERR_BUS;
}

/* ------------------------------------------------------------------------- *
 * HAL weak-callback overrides: route to the matching queue.
 * ------------------------------------------------------------------------- */
static io_spi_queue_t *queue_for(SPI_HandleTypeDef *hspi) {
    int idx = bus_index(hspi);
    if (idx < 0 || !s_q_init[idx]) return NULL;
    return &s_q[idx];
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) { io_spi_queue_on_complete(queue_for(hspi)); }
void HAL_SPI_TxCpltCallback  (SPI_HandleTypeDef *hspi) { io_spi_queue_on_complete(queue_for(hspi)); }
void HAL_SPI_RxCpltCallback  (SPI_HandleTypeDef *hspi) { io_spi_queue_on_complete(queue_for(hspi)); }
void HAL_SPI_ErrorCallback   (SPI_HandleTypeDef *hspi) { io_spi_queue_on_error   (queue_for(hspi)); }
