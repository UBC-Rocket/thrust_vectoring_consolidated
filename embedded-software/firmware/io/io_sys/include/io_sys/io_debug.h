/**
 * @file    io_debug.h
 * @brief   Official plain-text debug console — a system service (io_sys).
 *
 * Any layer (APP or DEV) may call these to print human-readable text to the
 * board's debug serial port. On the H745 that port is LPUART1 (PA9/PA10),
 * which is wired to the STLINK-V3 USB Virtual COM Port — so the output shows
 * up directly in a serial terminal (CoolTerm, screen, minicom) at 115200 8-N-1.
 *
 * Why this lives in io_sys (not app, not io/): the debug UART is an io/
 * transport, which the APP layer is forbidden to include by the CMake link
 * rules (see docs/architecture_layers.md, Move 5/6). A debug console is a
 * cross-cutting *system service*, like io_timestamp / io_status_led, so it
 * belongs in io_sys where APP and DEV can both reach it. The portable
 * formatter (io_debug_printf) lives in io/io_debug/; the transport
 * (io_debug_init / io_debug_write) is provided per-target in io/h747/CM{4,7}/.
 *
 * Threading: io_debug_printf formats into a caller-stack buffer, so it is
 * reentrant. The per-target io_debug_write serialises the actual UART transmit
 * with a mutex once the scheduler is running, so concurrent callers do not
 * interleave bytes mid-line. Pre-scheduler calls (during init) skip the lock.
 *
 * Core note: only CM4 owns the debug UART. CM7's implementation is a no-op
 * stub (returns IO_ERR_NOT_READY); io_debug_printf there is a safe no-op.
 * Forwarding CM7 text over the SRAM4 log ring to CM4 is a documented follow-up.
 *
 * UBC Rocket, 2026
 */
#ifndef IO_DEBUG_H
#define IO_DEBUG_H

#include "io_sys/io_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Max formatted bytes (including the implicit NUL) a single
 *        io_debug_printf call may emit. Longer output is truncated, never
 *        overflows. Sized for a comfortable log line; lives on the caller's
 *        stack, so keep it modest.
 */
#ifndef IO_DEBUG_LINE_MAX
#define IO_DEBUG_LINE_MAX 160U
#endif

/**
 * @brief Bring up the debug console transport (opens the debug UART).
 *        Idempotent — safe to call more than once. Target-provided.
 *
 * @return IO_OK on success, IO_ERR_NOT_READY on a core with no debug UART.
 */
io_status_t io_debug_init(void);

/**
 * @brief Write raw bytes to the debug console. Blocking, serialised once the
 *        scheduler is running. Target-provided.
 *
 * @param data  Bytes to send (not copied; sent synchronously).
 * @param len   Number of bytes.
 * @return IO_OK on success; IO_ERR_PARAM on NULL/zero; IO_ERR_NOT_READY if
 *         io_debug_init has not run (or no debug UART on this core);
 *         IO_ERR_TIMEOUT if the UART did not drain in time.
 */
io_status_t io_debug_write(const uint8_t *data, size_t len);

/**
 * @brief printf-style debug print. Formats into a caller-stack buffer
 *        (reentrant) and hands the result to io_debug_write.
 *
 * Supported conversions are whatever the linked C library's vsnprintf
 * provides. NOTE: floating-point (%f/%g) requires the float-enabled printf
 * (link with `-u _printf_float`); integer/string conversions always work.
 *
 * @param fmt  printf format string (must not be NULL).
 * @return Number of bytes handed to the transport (>= 0), clamped to
 *         IO_DEBUG_LINE_MAX-1 on truncation; or -1 on a NULL/encoding error
 *         or if the transport rejected the write.
 */
int io_debug_printf(const char *fmt, ...)
#if defined(__GNUC__)
    __attribute__((format(printf, 1, 2)))
#endif
    ;

/**
 * @brief Convenience: write a NUL-terminated string verbatim (no formatting).
 *
 * @param s  NUL-terminated string (must not be NULL).
 * @return Same status semantics as io_debug_write; IO_ERR_PARAM if @p s NULL.
 */
io_status_t io_debug_puts(const char *s);

#ifdef __cplusplus
}
#endif

#endif /* IO_DEBUG_H */
