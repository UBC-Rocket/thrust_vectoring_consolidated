/**
 * @file    io_debug.c
 * @brief   H745 CM7 IO impl: debug-console transport — stub.
 *
 * CM7 has no debug UART of its own: the VCP (LPUART1) is owned by CM4. So
 * io_debug on CM7 is a no-op — io_debug_printf (the portable formatter, which
 * is linked into both cores) calls io_debug_write here, gets IO_ERR_NOT_READY,
 * and returns -1 to the caller without side effects.
 *
 * Follow-up (documented, not in scope here): forward CM7 debug text over the
 * SRAM4 log-handoff ring to CM4 so CM7 code can log to the same console.
 *
 * UBC Rocket, 2026
 */
#include "io_sys/io_debug.h"

io_status_t io_debug_init(void) {
    return IO_ERR_NOT_READY;
}

io_status_t io_debug_write(const uint8_t *data, size_t len) {
    (void)data;
    (void)len;
    return IO_ERR_NOT_READY;
}
