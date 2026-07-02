/**
 * @file    io_debug.c
 * @brief   Portable debug-console formatter (io_debug_printf / io_debug_puts).
 *
 * Target-independent: it only knows how to turn a format string into bytes
 * and hand them to io_debug_write(), which is provided per-target (see
 * io/h747/CM{4,7}/io_debug.c) and faked in the host unit tests.
 *
 * UBC Rocket, 2026
 */
#include "io_sys/io_debug.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

int io_debug_printf(const char *fmt, ...) {
    if (fmt == NULL) {
        return -1;
    }

    /* Caller-stack buffer → reentrant: each concurrent call formats into its
     * own storage. Only the transmit (io_debug_write) needs serialising. */
    char buf[IO_DEBUG_LINE_MAX];

    va_list ap;
    va_start(ap, fmt);
    int written = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (written < 0) {
        return -1;          /* encoding error */
    }

    /* vsnprintf returns the length it *would* have written; clamp to what
     * actually fit (buffer minus the NUL) so a long line truncates cleanly
     * instead of reporting a length we never sent. */
    size_t len = ((size_t)written < sizeof(buf)) ? (size_t)written
                                                 : (sizeof(buf) - 1U);
    if (len == 0U) {
        return 0;           /* nothing to send (e.g. "" ) */
    }

    return (io_debug_write((const uint8_t *)buf, len) == IO_OK) ? (int)len : -1;
}

io_status_t io_debug_puts(const char *s) {
    if (s == NULL) {
        return IO_ERR_PARAM;
    }
    size_t len = strlen(s);
    if (len == 0U) {
        return IO_OK;       /* empty string is a successful no-op */
    }
    return io_debug_write((const uint8_t *)s, len);
}
