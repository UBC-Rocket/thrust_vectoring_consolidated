/**
 * @file io_intercore.h
 * @brief Inter-core event-notification interface.
 *
 * On the H747 this is backed by HSEM. On a future SIL build the same
 * interface is implemented in-process (handlers fire synchronously from
 * @ref io_intercore_signal). Shared-memory data exchange itself lives in
 * the @c io_intercore_slot library; this header is only for the
 * wakeup / event channel.
 *
 * UBC Rocket, 2026
 */

#ifndef IO_INTERCORE_H
#define IO_INTERCORE_H

#include "io_sys/io_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    IO_IC_STATE_UPDATED = 0,        ///< CM4 → CM7: new state estimate published
    IO_IC_CONTROL_OUTPUT_READY,     ///< CM7 → CM4: new control output published
    IO_IC_FLIGHT_STATE_UPDATED,     ///< CM4 → CM7: mission/flight state changed
    IO_IC_LOG_BUFFER_CM7_READY,     ///< CM7 → CM4: log staging buffer ready to flush
    IO_IC_LOG_BUFFER_CM4_FREE,      ///< CM4 → CM7: log staging buffer free again
    IO_IC_EVENT_COUNT
} io_ic_event_t;

typedef void (*io_ic_handler_t)(void);

void io_intercore_init(void);

/**
 * @brief Signal an event to the other core.
 */
void io_intercore_signal(io_ic_event_t evt);

/**
 * @brief Register a handler invoked from interrupt context when the other
 *        core signals @p evt. Pass NULL to unregister.
 */
void io_intercore_register_handler(io_ic_event_t evt, io_ic_handler_t cb);

#ifdef __cplusplus
}
#endif

#endif /* IO_INTERCORE_H */
