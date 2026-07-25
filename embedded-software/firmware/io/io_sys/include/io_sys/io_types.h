/**
 * @file io_types.h
 * @brief Common types shared across the IO contract.
 *
 * The IO layer is the boundary between target-specific transports and
 * hardware-agnostic drivers / app. No STM32 types leak through.
 *
 * UBC Rocket, 2026
 */
#ifndef IO_TYPES_H
#define IO_TYPES_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declaration to avoid pulling FreeRTOS headers into the IO interface.
 * The full definition (struct tskTaskControlBlock *) lives in FreeRTOS itself. */
struct tskTaskControlBlock;
typedef struct tskTaskControlBlock *io_task_handle_t;

/**
 * @brief Result type for every IO API call.
 *
 * Drivers can react meaningfully — retry on @c IO_ERR_BUSY, escalate on
 * persistent @c IO_ERR_BUS, etc. A simple "did it work?" check is
 * `if (io_call(...) != IO_OK)`.
 */
typedef enum {
    IO_OK = 0,
    IO_ERR_BUSY,         ///< queue full / channel still transmitting (try again)
    IO_ERR_TIMEOUT,      ///< blocking call did not complete in time
    IO_ERR_NACK,         ///< target did not acknowledge
    IO_ERR_BUS,          ///< bus-level fault (DMA error, framing, etc.)
    IO_ERR_NO_DEVICE,    ///< @p dev not present / @p id out of range
    IO_ERR_PARAM,        ///< bad argument
    IO_ERR_NOT_READY,    ///< called before init or in the wrong mode
} io_status_t;

#ifdef __cplusplus
}
#endif

#endif /* IO_TYPES_H */
