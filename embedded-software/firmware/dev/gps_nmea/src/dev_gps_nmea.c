/**
 * @file    dev_gps_nmea.c
 * @brief   NMEA GPS driver. Opaque singleton.
 *
 * UBC Rocket, 2026
 */
#include "dev_gps_nmea.h"

#include "io/io_uart.h"
#include "io_sys/io_timestamp.h"
#include "io_sys/io_test_hooks.h"

#include "lwgps/lwgps.h"

#include <string.h>

#if defined(__has_include)
#  if __has_include("FreeRTOS.h")
#    include "FreeRTOS.h"
#    include "task.h"
#    define GPS_HAVE_FREERTOS 1
#  endif
#endif

#define GPS_FIX_RING_SIZE 4

struct gps_nmea {
    lwgps_t           gps;
    io_task_handle_t  notify_task;
    uint32_t          notify_bit;
    gps_fix_t         ring[GPS_FIX_RING_SIZE];
    volatile uint8_t  head, tail;
    bool              ready;
};

static struct gps_nmea s_self;

/* Test-only access to module-private state. No-op in production builds. */
IO_TEST_HOOK_RW(s_self, struct gps_nmea, dev_gps_nmea_self)

static inline bool ring_full(const gps_nmea_t *d) {
    return ((d->head + 1) % GPS_FIX_RING_SIZE) == d->tail;
}

static void notify_isr(gps_nmea_t *d) {
#ifdef GPS_HAVE_FREERTOS
    if (d->notify_task && d->notify_bit) {
        BaseType_t woken = pdFALSE;
        xTaskNotifyFromISR((TaskHandle_t)d->notify_task, d->notify_bit, eSetBits, &woken);
        portYIELD_FROM_ISR(woken);
    }
#else
    (void)d;
#endif
}

static void on_sentence(const uint8_t *data, size_t len, void *user) {
    gps_nmea_t *d = user;
    if (lwgps_process(&d->gps, data, len) == 0) return;
    if (ring_full(d)) return;
    gps_fix_t *f = &d->ring[d->head];
    f->t_us             = io_timestamp_us();
    f->latitude_deg     = d->gps.latitude;
    f->longitude_deg    = d->gps.longitude;
    f->altitude_m       = d->gps.altitude;
    f->ground_speed_mps = d->gps.speed;
    f->course_deg       = d->gps.course;
    f->hdop             = d->gps.dop_h;
    f->fix_type         = d->gps.fix;
    f->sats_used        = d->gps.sats_in_use;
    f->valid            = d->gps.is_valid;
    __DMB();
    d->head = (d->head + 1) % GPS_FIX_RING_SIZE;
    notify_isr(d);
}

bool gps_nmea_init(void) {
    memset(&s_self, 0, sizeof(s_self));
    lwgps_init(&s_self.gps);
    if (io_uart_open(&IO_UART_GPS, on_sentence, &s_self) != IO_OK) return false;
    s_self.ready = true;
    return true;
}

gps_nmea_t *gps_nmea_get(void) {
    return s_self.ready ? &s_self : NULL;
}

size_t gps_nmea_drain_fixes(gps_nmea_t *d, gps_fix_t *out, size_t max) {
    size_t n = 0;
    while (n < max && d->head != d->tail) {
        __DMB();
        out[n++] = d->ring[d->tail];
        d->tail = (d->tail + 1) % GPS_FIX_RING_SIZE;
    }
    return n;
}

void gps_nmea_set_notify(gps_nmea_t *d, io_task_handle_t t, uint32_t bit) {
    d->notify_task = t;
    d->notify_bit  = bit;
}
