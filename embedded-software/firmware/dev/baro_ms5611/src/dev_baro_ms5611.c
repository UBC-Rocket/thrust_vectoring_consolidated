/**
 * @file    dev_baro_ms5611.c
 * @brief   MS5611 driver. Opaque singleton.
 *
 * UBC Rocket, 2026
 */
#include "dev_baro_ms5611.h"

#include "io/io_spi.h"
#include "io_sys/io_timestamp.h"
#include "io_sys/io_test_hooks.h"

#include "sensors/ms5611_baro.h"

#include <string.h>

#define MS5611_RING_SIZE 8

struct baro_ms5611 {
    ms5611_t          dev;
    io_task_handle_t  notify_task;
    uint32_t          notify_bit;
    baro_sample_t     ring[MS5611_RING_SIZE];
    volatile uint8_t  head, tail;
    bool              ready;
};

static struct baro_ms5611 s_self;

/* Test-only access to module-private state. No-op in production builds. */
IO_TEST_HOOK_RW(s_self, struct baro_ms5611, dev_baro_ms5611_self)

bool baro_ms5611_init(void) {
    memset(&s_self, 0, sizeof(s_self));
    /* TODO: reset + PROM read via io_spi_xfer_blocking(&IO_SPI_MS5611, …). */
    s_self.ready = true;
    return true;
}

baro_ms5611_t *baro_ms5611_get(void) {
    return s_self.ready ? &s_self : NULL;
}

void baro_ms5611_tick(baro_ms5611_t *d) {
    (void)d;
    /* TODO: conv state machine, enqueue (t_us, pressure_pa, temp_c), notify. */
}

size_t baro_ms5611_drain(baro_ms5611_t *d, baro_sample_t *out, size_t max) {
    size_t n = 0;
    while (n < max && d->head != d->tail) {
        out[n++] = d->ring[d->tail];
        d->tail = (d->tail + 1) % MS5611_RING_SIZE;
    }
    return n;
}

void baro_ms5611_set_notify(baro_ms5611_t *d, io_task_handle_t t, uint32_t bit) {
    d->notify_task = t;
    d->notify_bit  = bit;
}
