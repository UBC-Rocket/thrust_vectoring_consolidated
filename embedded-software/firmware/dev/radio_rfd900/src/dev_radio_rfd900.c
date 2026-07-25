/**
 * @file    dev_radio_rfd900.c
 * @brief   RFD900x driver. Opaque singleton.
 *
 * UBC Rocket, 2026
 */
#include "dev_radio_rfd900.h"

#include "io/io_uart.h"
#include "io_sys/io_timestamp.h"
#include "io_sys/io_test_hooks.h"

#include <string.h>

#if defined(__has_include)
#  if __has_include("FreeRTOS.h")
#    include "FreeRTOS.h"
#    include "task.h"
#    define RADIO_HAVE_FREERTOS 1
#  endif
#endif

#define RADIO_RING_SIZE 8

struct radio_rfd900 {
    io_task_handle_t notify_task;
    uint32_t         notify_bit;
    radio_frame_t    ring[RADIO_RING_SIZE];
    volatile uint8_t head, tail;
    bool             ready;
};

static struct radio_rfd900 s_self;

/* Test-only access to module-private state. No-op in production builds. */
IO_TEST_HOOK_RW(s_self, struct radio_rfd900, dev_radio_rfd900_self)

static inline bool ring_full(const radio_rfd900_t *d) {
    return ((d->head + 1) % RADIO_RING_SIZE) == d->tail;
}

static void notify_isr(radio_rfd900_t *d) {
#ifdef RADIO_HAVE_FREERTOS
    if (d->notify_task && d->notify_bit) {
        BaseType_t woken = pdFALSE;
        xTaskNotifyFromISR((TaskHandle_t)d->notify_task, d->notify_bit, eSetBits, &woken);
        portYIELD_FROM_ISR(woken);
    }
#else
    (void)d;
#endif
}

static void ring_push(radio_rfd900_t *d, const uint8_t *data, size_t len) {
    if (ring_full(d)) return;
    if (len > RADIO_RFD900_MAX_FRAME) len = RADIO_RFD900_MAX_FRAME;
    radio_frame_t *f = &d->ring[d->head];
    f->t_us = io_timestamp_us();
    f->len  = (uint16_t)len;
    memcpy(f->payload, data, len);
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    d->head = (d->head + 1) % RADIO_RING_SIZE;
}

/* ISR path (character-match on 0x00): push + wake the task. */
static void on_frame(const uint8_t *data, size_t len, void *user) {
    radio_rfd900_t *d = user;
    ring_push(d, data, len);
    notify_isr(d);
}

bool radio_rfd900_init(void) {
    memset(&s_self, 0, sizeof(s_self));
    if (io_uart_open(&IO_UART_RADIO, on_frame, &s_self) != IO_OK) return false;
    s_self.ready = true;
    return true;
}

radio_rfd900_t *radio_rfd900_get(void) {
    return s_self.ready ? &s_self : NULL;
}

size_t radio_rfd900_drain_frames(radio_rfd900_t *d, radio_frame_t *out, size_t max) {
    size_t n = 0;
    while (n < max && d->head != d->tail) {
        __atomic_thread_fence(__ATOMIC_SEQ_CST);
        out[n++] = d->ring[d->tail];
        d->tail = (d->tail + 1) % RADIO_RING_SIZE;
    }
    return n;
}

void radio_rfd900_set_notify(radio_rfd900_t *d, io_task_handle_t t, uint32_t bit) {
    d->notify_task = t;
    d->notify_bit  = bit;
}

bool radio_rfd900_send(radio_rfd900_t *d, const uint8_t *data, size_t len) {
    (void)d;
    return io_uart_send(&IO_UART_RADIO, data, len, 200) == IO_OK;
}

uint32_t radio_rfd900_diag_rx_index(void) {
    return io_uart_diag_rx_index(&IO_UART_RADIO);
}

size_t radio_rfd900_diag_rx_copy(uint32_t from, uint32_t to, uint8_t *out, size_t max) {
    return io_uart_diag_rx_copy(&IO_UART_RADIO, from, to, out, max);
}

void radio_rfd900_poll(void) {
    if (!s_self.ready) {
        return;
    }
    /* Persist across calls: our read cursor into the RX DMA ring and a partial
     * frame accumulator (a frame may span two poll intervals). */
    static uint32_t scan_idx;
    static uint8_t  acc[RADIO_RFD900_MAX_FRAME];
    static size_t   acc_len;
    /* Snapshot buffer: >= the RX DMA ring so a full [scan_idx, cur) span always
     * fits in one copy (static → off the caller's stack). */
    static uint8_t  buf[1024];

    const uint32_t cur = io_uart_diag_rx_index(&IO_UART_RADIO);
    if (cur == scan_idx) {
        return;                       /* no new bytes since last poll */
    }
    const size_t got = io_uart_diag_rx_copy(&IO_UART_RADIO, scan_idx, cur,
                                            buf, sizeof(buf));
    scan_idx = cur;

    /* Reassemble COBS frames: accumulate until the 0x00 delimiter, then push
     * the whole frame (incl. delimiter — rp_packet_decode expects it) to the
     * ring for mission_manager to decode. Self-synchronising: the first
     * (possibly partial) frame is dropped, subsequent ones align on 0x00. */
    for (size_t i = 0; i < got; ++i) {
        const uint8_t b = buf[i];
        if (acc_len < sizeof(acc)) {
            acc[acc_len++] = b;
        }
        if (b == 0x00) {
            if (acc_len > 1) {        /* payload + delimiter present */
                ring_push(&s_self, acc, acc_len);
            }
            acc_len = 0;
        }
    }
}
