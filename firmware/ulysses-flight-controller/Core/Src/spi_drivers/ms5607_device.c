/**
 * @file ms5607_device.c
 * @brief MS5607 barometer device layer implementation.
 *
 * Hardware interaction, poller state machine, DMA job submission, and callbacks.
 * Uses SPI4 (jobq_spi_4) for the second barometer.
 *
 * @ UBC Rocket, Jan 2026
 */

#include "SPI_device_interactions.h"
#include "ms5607_poller.h"
#include "SPI_queue.h"
#include "main.h"
#include "timestamp.h"
#include <string.h>

/* -------------------------------------------------------------------------- */
/* Static state for DMA callbacks                                             */
/* -------------------------------------------------------------------------- */

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *cs_port;
    uint16_t           cs_pin;
} ms5607_hw_t;

static ms5607_hw_t g_baro2_hw;
static ms5607_poller_t *g_baro2 = NULL;

/* Conversion time lookup table (max values from datasheet page 3) */
static const uint32_t ms5607_osr_delay_us_tbl[] = {
    [MS5607_OSR_256]  = 600,
    [MS5607_OSR_512]  = 1170,
    [MS5607_OSR_1024] = 2280,
    [MS5607_OSR_2048] = 4540,
    [MS5607_OSR_4096] = 9040
};

/* Forward declarations for DMA callbacks */
static void ms5607_cb_read_d1(spi_job_t *job, const uint8_t *rx, void *arg);
static void ms5607_cb_read_d2(spi_job_t *job, const uint8_t *rx, void *arg);

/* -------------------------------------------------------------------------- */
/* Blocking initialization                                                    */
/* -------------------------------------------------------------------------- */

uint8_t ms5607_init(SPI_HandleTypeDef *hspi,
                    GPIO_TypeDef *cs_port,
                    uint16_t cs_pin,
                    ms5607_t *dev)
{
    uint8_t tx[4], rx[4];

    /* --- 1. Send reset command --- */
    size_t n = ms5607_build_reset(tx);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    delay_us(1000);
    HAL_SPI_Transmit(hspi, tx, n, HAL_MAX_DELAY);
    delay_us(1000);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);

    /* --- 2. Wait ≥2.8 ms for PROM reload --- */
    delay_us(3000);

    /* --- 3. Read PROM[0..7] --- */
    for (uint8_t i = 0; i < 8; i++) {
        n = ms5607_build_prom_read(i, tx);

        HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
        delay_us(1000);
        HAL_SPI_TransmitReceive(hspi, tx, rx, n, HAL_MAX_DELAY);
        delay_us(1000);
        HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
        delay_us(1000);

        if (!ms5607_parse_prom_word(&rx[1], &dev->C[i])) {
            return 1; /* parse error */
        }
    }

    /* --- 4. Validate PROM CRC --- */
    if (!ms5607_check_crc(dev->C)) {
        return 2; /* CRC mismatch (invalid PROM) */
    }

    /* --- 5. Clear previous raw readings --- */
    dev->D1_raw = 0;
    dev->D2_raw = 0;
    dev->TEMP_centi = 0;
    dev->P_centi_mbar = 0;

    return 0; /* success */
}

/* -------------------------------------------------------------------------- */
/* Poller initialization and configuration                                    */
/* -------------------------------------------------------------------------- */

void ms5607_poller_init(ms5607_poller_t *p,
                        SPI_HandleTypeDef *hspi,
                        GPIO_TypeDef *cs_port, uint16_t cs_pin,
                        ms5607_osr_t osr, uint32_t odr_hz)
{
    memset(p, 0, sizeof(*p));

    /* Initialize ring buffer */
    ms5607_sample_ring.head = 0;
    ms5607_sample_ring.tail = 0;

    /* Store hardware handles for DMA callbacks */
    g_baro2_hw.hspi = hspi;
    g_baro2_hw.cs_port = cs_port;
    g_baro2_hw.cs_pin = cs_pin;

    /* 1) Reset device */
    uint8_t tx[4], rx[4];
    size_t n = ms5607_build_reset(tx);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    delay_us(1000);
    HAL_SPI_Transmit(hspi, tx, n, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    delay_us(3000); /* >= 2.8 ms */

    /* 2) PROM read + CRC */
    for (uint8_t i = 0; i < 8; ++i) {
        n = ms5607_build_prom_read(i, tx);
        HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
        delay_us(1000);
        HAL_SPI_TransmitReceive(hspi, tx, rx, n, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
        ms5607_parse_prom_word(&rx[1], &p->dev.C[i]);
        delay_us(200);
    }
    (void)ms5607_check_crc(p->dev.C);

    /* Configure timing */
    ms5607_poller_set_rate(p, osr, odr_hz);

    /* Initialize state machine */
    p->state            = MS5607_POLLER_IDLE;
    p->t_cycle_start_us = timestamp_us();
    p->t_next_action_us = p->t_cycle_start_us; /* ready to start anytime */
    p->fresh            = false;
    p->seq              = 0;

    /* Keep global pointer for ISR/callback access */
    g_baro2 = p;
}

void ms5607_poller_set_rate(ms5607_poller_t *p, ms5607_osr_t osr, uint32_t odr_hz)
{
    p->osr       = osr;
    p->conv_us   = ms5607_osr_delay_us_tbl[osr];
    p->odr_hz    = odr_hz ? odr_hz : 50;
    p->period_us = (1000000UL + p->odr_hz / 2) / p->odr_hz; /* rounded */
}

/* -------------------------------------------------------------------------- */
/* DMA job submission helpers                                                 */
/* -------------------------------------------------------------------------- */

static void ms5607_submit_convert_d1(void)
{
    ms5607_poller_t *p = g_baro2;
    if (!p) return;

    spi_job_t j = {0};
    j.cs_port = g_baro2_hw.cs_port;
    j.cs_pin  = g_baro2_hw.cs_pin;
    j.len     = ms5607_build_convert_d1(p->osr, j.tx);
    j.type    = SPI_XFER_TX;
    j.sensor  = SENSOR_ID_BARO2;
    spi_submit_job(j, &jobq_spi_4);
}

static void ms5607_submit_convert_d2(void)
{
    ms5607_poller_t *p = g_baro2;
    if (!p) return;

    spi_job_t j = {0};
    j.cs_port = g_baro2_hw.cs_port;
    j.cs_pin  = g_baro2_hw.cs_pin;
    j.len     = ms5607_build_convert_d2(p->osr, j.tx);
    j.type    = SPI_XFER_TX;
    j.sensor  = SENSOR_ID_BARO2;
    spi_submit_job(j, &jobq_spi_4);
}

static void ms5607_submit_read_d1(void)
{
    if (!g_baro2) return;

    spi_job_t j = {0};
    j.cs_port = g_baro2_hw.cs_port;
    j.cs_pin  = g_baro2_hw.cs_pin;
    j.len     = ms5607_build_adc_read(j.tx);
    j.type    = SPI_XFER_TXRX;
    j.sensor  = SENSOR_ID_BARO2;
    j.done    = ms5607_cb_read_d1;
    j.task_notification_flag = MS5607_BARO2_SAMPLE_FLAG;
    spi_submit_job(j, &jobq_spi_4);
}

static void ms5607_submit_read_d2(void)
{
    if (!g_baro2) return;

    spi_job_t j = {0};
    j.cs_port = g_baro2_hw.cs_port;
    j.cs_pin  = g_baro2_hw.cs_pin;
    j.len     = ms5607_build_adc_read(j.tx);
    j.t_sample = timestamp_us();
    j.type    = SPI_XFER_TXRX;
    j.sensor  = SENSOR_ID_BARO2;
    j.done    = ms5607_cb_read_d2;
    j.task_notification_flag = MS5607_BARO2_SAMPLE_FLAG;
    spi_submit_job(j, &jobq_spi_4);
}

/* -------------------------------------------------------------------------- */
/* DMA completion callbacks (ISR context)                                     */
/* -------------------------------------------------------------------------- */

static void ms5607_cb_read_d1(spi_job_t *job, const uint8_t *rx, void *arg)
{
    (void)job;
    (void)arg;
    ms5607_poller_t *p = g_baro2;
    if (!p) return;

    ms5607_parse_adc_result(&rx[1], &p->dev.D1_raw);
}

static void ms5607_cb_read_d2(spi_job_t *job, const uint8_t *rx, void *arg)
{
    (void)arg;
    ms5607_poller_t *p = g_baro2;
    if (!p) return;

    ms5607_parse_adc_result(&rx[1], &p->dev.D2_raw);
    ms5607_compute(&p->dev);
    p->fresh = true;
    p->seq++;

    ms5607_sample_t sample = {
        .t_us           = job ? job->t_sample : 0U,
        .d1_raw         = p->dev.D1_raw,
        .d2_raw         = p->dev.D2_raw,
        .temp_centi     = p->dev.TEMP_centi,
        .pressure_centi = p->dev.P_centi_mbar,
        .seq            = p->seq
    };
    ms5607_sample_queue(&ms5607_sample_ring, &sample);
}

/* -------------------------------------------------------------------------- */
/* Poller state machine                                                       */
/* -------------------------------------------------------------------------- */

void ms5607_poller_tick(ms5607_poller_t *p)
{
    const uint32_t now = timestamp_us();

    switch (p->state) {
        case MS5607_POLLER_IDLE:
            if ((int32_t)(now - p->t_next_action_us) >= 0) {
                p->t_cycle_start_us = now;
                ms5607_submit_convert_d1();
                p->t_next_action_us = now + p->conv_us;  /* when D1 is ready */
                p->state = MS5607_POLLER_WAIT_D1;
            }
            break;

        case MS5607_POLLER_WAIT_D1:
            if ((int32_t)(now - p->t_next_action_us) >= 0) {
                ms5607_submit_read_d1();
                p->t_next_action_us = now;
                p->state = MS5607_POLLER_READ_D1;
            }
            break;

        case MS5607_POLLER_READ_D1:
            if ((int32_t)(now - p->t_next_action_us) >= 0) {
                ms5607_submit_convert_d2();
                p->t_next_action_us = now + p->conv_us;
                p->state = MS5607_POLLER_WAIT_D2;
            }
            break;

        case MS5607_POLLER_WAIT_D2:
            if ((int32_t)(now - p->t_next_action_us) >= 0) {
                ms5607_submit_read_d2();
                p->t_next_action_us = p->t_cycle_start_us + p->period_us;
                p->state = MS5607_POLLER_READ_D2;
            }
            break;

        case MS5607_POLLER_READ_D2:
            if ((int32_t)(now - p->t_next_action_us) >= 0) {
                p->state = MS5607_POLLER_IDLE;
            }
            break;

        default:
            /* Unknown state - reset to idle */
            p->state = MS5607_POLLER_IDLE;
            break;
    }
}

bool ms5607_fetch_latest(ms5607_poller_t *p, ms5607_t *out, uint32_t *seq)
{
    if (!p->fresh) return false;
    if (out) *out = p->dev;
    if (seq) *seq = p->seq;
    p->fresh = false;
    return true;
}
